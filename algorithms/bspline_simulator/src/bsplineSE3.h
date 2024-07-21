#pragma once

#include <Eigen/Eigen>
#include <map>
#include <vector>

inline Eigen::Matrix<double, 3, 3> skew_x(const Eigen::Matrix<double, 3, 1> &w) {
  Eigen::Matrix<double, 3, 3> w_x;
  w_x << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return w_x;
}

inline Eigen::Matrix<double, 3, 1> vee(const Eigen::Matrix<double, 3, 3> &w_x) {
  Eigen::Matrix<double, 3, 1> w;
  w << w_x(2, 1), w_x(0, 2), w_x(1, 0);
  return w;
}

inline Eigen::Matrix4d hat_se3(const Eigen::Matrix<double, 6, 1> &vec) {
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = skew_x(vec.head(3));
  mat.block(0, 3, 3, 1) = vec.tail(3);
  return mat;
}

inline Eigen::Matrix<double, 3, 1> log_so3(const Eigen::Matrix<double, 3, 3> &R) {

  // note switch to base 1
  double R11 = R(0, 0), R12 = R(0, 1), R13 = R(0, 2);
  double R21 = R(1, 0), R22 = R(1, 1), R23 = R(1, 2);
  double R31 = R(2, 0), R32 = R(2, 1), R33 = R(2, 2);

  // Get trace(R)
  const double tr = R.trace();
  Eigen::Vector3d omega;

  // when trace == -1, i.e., when theta = +-pi, +-3pi, +-5pi, etc.
  // we do something special
  if (tr + 1.0 < 1e-10) {
    if (std::abs(R33 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R33)) * Eigen::Vector3d(R13, R23, 1.0 + R33);
    else if (std::abs(R22 + 1.0) > 1e-5)
      omega = (M_PI / sqrt(2.0 + 2.0 * R22)) * Eigen::Vector3d(R12, 1.0 + R22, R32);
    else
      // if(std::abs(R.r1_.x()+1.0) > 1e-5)  This is implicit
      omega = (M_PI / sqrt(2.0 + 2.0 * R11)) * Eigen::Vector3d(1.0 + R11, R21, R31);
  } else {
    double magnitude;
    const double tr_3 = tr - 3.0; // always negative
    if (tr_3 < -1e-7) {
      double theta = acos((tr - 1.0) / 2.0);
      magnitude = theta / (2.0 * sin(theta));
    } else {
      // when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
      // use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
      // see https://github.com/borglab/gtsam/issues/746 for details
      magnitude = 0.5 - tr_3 / 12.0;
    }
    omega = magnitude * Eigen::Vector3d(R32 - R23, R13 - R31, R21 - R12);
  }

  return omega;
}

inline Eigen::Matrix4d exp_se3(Eigen::Matrix<double, 6, 1> vec) {

  // Precompute our values
  Eigen::Vector3d w = vec.head(3);
  Eigen::Vector3d u = vec.tail(3);
  double theta = sqrt(w.dot(w));
  Eigen::Matrix3d wskew;
  wskew << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

  // Handle small angle values
  double A, B, C;
  if (theta < 1e-7) {
    A = 1;
    B = 0.5;
    C = 1.0 / 6.0;
  } else {
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / (theta * theta);
    C = (1 - A) / (theta * theta);
  }

  // Matrices we need V and Identity
  Eigen::Matrix3d I_33 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = I_33 + B * wskew + C * wskew * wskew;

  // Get the final matrix to return
  Eigen::Matrix4d mat = Eigen::Matrix4d::Zero();
  mat.block(0, 0, 3, 3) = I_33 + A * wskew + B * wskew * wskew;
  mat.block(0, 3, 3, 1) = V * u;
  mat(3, 3) = 1;
  return mat;
}

inline Eigen::Matrix4d Inv_se3(const Eigen::Matrix4d &T) {
  Eigen::Matrix4d Tinv = Eigen::Matrix4d::Identity();
  Tinv.block(0, 0, 3, 3) = T.block(0, 0, 3, 3).transpose();
  Tinv.block(0, 3, 3, 1) = -Tinv.block(0, 0, 3, 3) * T.block(0, 3, 3, 1);
  return Tinv;
}

inline Eigen::Matrix<double, 6, 1> log_se3(Eigen::Matrix4d mat) {
  Eigen::Vector3d w = log_so3(mat.block<3, 3>(0, 0));
  Eigen::Vector3d T = mat.block<3, 1>(0, 3);
  const double t = w.norm();
  if (t < 1e-10) {
    Eigen::Matrix<double, 6, 1> log;
    log << w, T;
    return log;
  } else {
    Eigen::Matrix3d W = skew_x(w / t);
    // Formula from Agrawal06iros, equation (14)
    // simplified with Mathematica, and multiplying in T to avoid matrix math
    double Tan = tan(0.5 * t);
    Eigen::Vector3d WT = W * T;
    Eigen::Vector3d u = T - (0.5 * t) * WT + (1 - t / (2. * Tan)) * (W * WT);
    Eigen::Matrix<double, 6, 1> log;
    log << w, u;
    return log;
  }
}

class BsplineSE3 {

public:
  /**
   * @brief Default constructor
   */
  BsplineSE3() {}

  /**
   * @brief Will feed in a series of poses that we will then convert into control points.
   *
   * Our control points need to be uniformly spaced over the trajectory, thus given a trajectory we will
   * uniformly sample based on the average spacing between the pose points specified.
   *
   * @param traj_points Trajectory poses that we will convert into control points (timestamp(s), q_GtoI, p_IinG)
   */
  void feed_trajectory(std::vector<Eigen::VectorXd> traj_points);

  /**
   * @brief Gets the orientation and position at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @return False if we can't find it
   */
  bool get_pose(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG);

  /**
   * @brief Gets the angular and linear velocity at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @param w_IinI Angular velocity in the inertial frame
   * @param v_IinG Linear velocity in the global frame
   * @return False if we can't find it
   */
  bool get_velocity(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI, Eigen::Vector3d &v_IinG);

  /**
   * @brief Gets the angular and linear acceleration at a given timestamp
   * @param timestamp Desired time to get the pose at
   * @param R_GtoI SO(3) orientation of the pose in the global frame
   * @param p_IinG Position of the pose in the global
   * @param w_IinI Angular velocity in the inertial frame
   * @param v_IinG Linear velocity in the global frame
   * @param alpha_IinI Angular acceleration in the inertial frame
   * @param a_IinG Linear acceleration in the global frame
   * @return False if we can't find it
   */
  bool get_acceleration(double timestamp, Eigen::Matrix3d &R_GtoI, Eigen::Vector3d &p_IinG, Eigen::Vector3d &w_IinI,
                        Eigen::Vector3d &v_IinG, Eigen::Vector3d &alpha_IinI, Eigen::Vector3d &a_IinG);

  /// Returns the simulation start time that we should start simulating from
  double get_start_time() { return timestamp_start; }

protected:
  /// Uniform sampling time for our control points
  double dt;

  /// Start time of the system
  double timestamp_start;

  /// Type defintion of our aligned eigen4d matrix: https://eigen.tuxfamily.org/dox/group__TopicStlContainers.html
  typedef std::map<double, Eigen::Matrix4d, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix4d>>>
      AlignedEigenMat4d;

  /// Our control SE3 control poses (R_ItoG, p_IinG)
  AlignedEigenMat4d control_points;

  /**
   * @brief Will find the two bounding poses for a given timestamp.
   *
   * This will loop through the passed map of poses and find two bounding poses.
   * If there are no bounding poses then this will return false.
   *
   * @param timestamp Desired timestamp we want to get two bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_poses(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0, double &t1,
                                  Eigen::Matrix4d &pose1);

  /**
   * @brief Will find two older poses and two newer poses for the current timestamp
   *
   * @param timestamp Desired timestamp we want to get four bounding poses of
   * @param poses Map of poses and timestamps
   * @param t0 Timestamp of the first pose
   * @param pose0 SE(3) pose of the first pose
   * @param t1 Timestamp of the second pose
   * @param pose1 SE(3) pose of the second pose
   * @param t2 Timestamp of the third pose
   * @param pose2 SE(3) pose of the third pose
   * @param t3 Timestamp of the fourth pose
   * @param pose3 SE(3) pose of the fourth pose
   * @return False if we are unable to find bounding poses
   */
  static bool find_bounding_control_points(const double timestamp, const AlignedEigenMat4d &poses, double &t0, Eigen::Matrix4d &pose0,
                                           double &t1, Eigen::Matrix4d &pose1, double &t2, Eigen::Matrix4d &pose2, double &t3,
                                           Eigen::Matrix4d &pose3);
};
