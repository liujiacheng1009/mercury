#include "se3_spline.h"

#include <iostream>

#include "gtest/gtest.h"
#include "test_utils.h"

template <int N>
void testGyroRes(const Se3Spline<N> &s, int64_t t_ns) {
  typename Se3Spline<N>::SO3JacobianStruct J_spline;
  Eigen::Matrix<double, 3, 12> J_bias;

  CalibGyroBias<double> bias;

  bias.setRandom();

  // bias << 0.01, -0.02, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  Eigen::Vector3d measurement = s.rotVelBody(t_ns);

  s.gyroResidual(t_ns, measurement, bias, &J_spline, &J_bias);

  for (size_t i = 0; i < s.numKnots(); i++) {
    Sophus::Vector3d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_gyro_res_d_knot" << i << " time " << t_ns;

    Sophus::Matrix3d J_a;

    if (i >= J_spline.start_idx && i < J_spline.start_idx + N) {
      J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
    } else {
      J_a.setZero();
    }

    test_jacobian(
        ss.str(), J_a,
        [&](const Sophus::Vector3d &x) {
          Se3Spline<N> s1 = s;
          s1.getKnotSO3(i) = Sophus::SO3d::exp(x) * s.getKnotSO3(i);

          return s1.gyroResidual(t_ns, measurement, bias);
        },
        x0);
  }

  {
    Eigen::Matrix<double, 12, 1> x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_gyro_res_d_bias";

    test_jacobian(
        ss.str(), J_bias,
        [&](const Eigen::Matrix<double, 12, 1> &x) {
          auto b1 = bias;
          b1 += x;
          return s.gyroResidual(t_ns, measurement, b1);
        },
        x0);
  }
}

template <int N>
void testAccelRes(const Se3Spline<N> &s, int64_t t_ns) {
  typename Se3Spline<N>::AccelPosSO3JacobianStruct J_spline;
  Eigen::Matrix3d J_g;

  Eigen::Matrix<double, 3, 9> J_bias;

  CalibAccelBias<double> bias;
  bias.setRandom();
  // bias << -0.4, 0.5, -0.6, 0, 0, 0, 0, 0, 0;

  Eigen::Vector3d g(0, 0, -9.81);
  Eigen::Vector3d measurement = s.transAccelWorld(t_ns) + g;

  s.accelResidual(t_ns, measurement, bias, g, &J_spline, &J_bias, &J_g);

  for (size_t i = 0; i < s.numKnots(); i++) {
    Sophus::Vector6d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_knot" << i;

    typename Se3Spline<N>::Mat36 J_a;

    if (i >= J_spline.start_idx && i < J_spline.start_idx + N) {
      J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
    } else {
      J_a.setZero();
    }

    test_jacobian(
        ss.str(), J_a,
        [&](const Sophus::Vector6d &x) {
          Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.accelResidual(t_ns, measurement, bias, g);
        },
        x0);
  }

  {
    Eigen::Matrix<double, 9, 1> x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_bias";

    test_jacobian(
        ss.str(), J_bias,
        [&](const Eigen::Matrix<double, 9, 1> &x) {
          auto b1 = bias;
          b1 += x;
          return s.accelResidual(t_ns, measurement, b1, g);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();

    std::stringstream ss;
    ss << "Spline order " << N << " d_accel_res_d_g";

    test_jacobian(
        ss.str(), J_g,
        [&](const Sophus::Vector3d &x) {
          return s.accelResidual(t_ns, measurement, bias, g + x);
        },
        x0);
  }
}

template <int N>
void testOrientationRes(const Se3Spline<N> &s, int64_t t_ns) {
  typename Se3Spline<N>::SO3JacobianStruct J_spline;

  Sophus::SO3d measurement =
      s.pose(t_ns).so3();  // * Sophus::expd(Sophus::Vector6d::Random() / 10);

  s.orientationResidual(t_ns, measurement, &J_spline);

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_rot_res_d_knot" << i;

    Sophus::Matrix3d J_a;

    if (i >= J_spline.start_idx && i < J_spline.start_idx + N) {
      J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
    } else {
      J_a.setZero();
    }

    Sophus::Vector3d x0;
    x0.setZero();

    test_jacobian(
        ss.str(), J_a,
        [&](const Sophus::Vector3d &x_rot) {
          Sophus::Vector6d x;
          x.setZero();
          x.tail<3>() = x_rot;

          Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.orientationResidual(t_ns, measurement);
        },
        x0);
  }
}

template <int N>
void testPositionRes(const Se3Spline<N> &s, int64_t t_ns) {
  typename Se3Spline<N>::PosJacobianStruct J_spline;

  Eigen::Vector3d measurement =
      s.pose(t_ns)
          .translation();  // * Sophus::expd(Sophus::Vector6d::Random() / 10);

  s.positionResidual(t_ns, measurement, &J_spline);

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_pos_res_d_knot" << i;

    Sophus::Matrix3d J_a;
    J_a.setZero();

    if (i >= J_spline.start_idx && i < J_spline.start_idx + N) {
      J_a.diagonal().setConstant(J_spline.d_val_d_knot[i - J_spline.start_idx]);
    }

    Sophus::Vector3d x0;
    x0.setZero();

    test_jacobian(
        ss.str(), J_a,
        [&](const Sophus::Vector3d &x_rot) {
          Sophus::Vector6d x;
          x.setZero();
          x.head<3>() = x_rot;

          Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return s1.positionResidual(t_ns, measurement);
        },
        x0);
  }
}

template <int N>
void testPose(const Se3Spline<N> &s, int64_t t_ns) {
  typename Se3Spline<N>::PosePosSO3JacobianStruct J_spline;

  Sophus::SE3d res = s.pose(t_ns, &J_spline);

  Sophus::Vector6d x0;
  x0.setZero();

  for (size_t i = 0; i < s.numKnots(); i++) {
    std::stringstream ss;
    ss << "Spline order " << N << " d_pose_d_knot" << i;

    typename Se3Spline<N>::Mat6 J_a;

    if (i >= J_spline.start_idx && i < J_spline.start_idx + N) {
      J_a = J_spline.d_val_d_knot[i - J_spline.start_idx];
    } else {
      J_a.setZero();
    }

    test_jacobian(
        ss.str(), J_a,
        [&](const Sophus::Vector6d &x) {
          Se3Spline<N> s1 = s;
          s1.applyInc(i, x);

          return Sophus::se3_logd(res.inverse() * s1.pose(t_ns));
        },
        x0);
  }

  {
    Eigen::Matrix<double, 1, 1> x0;
    x0[0] = 0;

    typename Se3Spline<N>::Vec6 J_a;

    //    J.template head<3>() = res.so3().inverse() * s.transVelWorld(t_ns);
    //    J.template tail<3>() = s.rotVelBody(t_ns);

    s.d_pose_d_t(t_ns, J_a);

    test_jacobian(
        "J_pose_time", J_a,
        [&](const Eigen::Matrix<double, 1, 1> &x) {
          int64_t t_ns_new = t_ns;
          t_ns_new += x[0] * 1e9;

          return Sophus::se3_logd(res.inverse() * s.pose(t_ns_new));
        },
        x0);
  }
}

TEST(SplineSE3, GyroResidualTest) {
  static constexpr int N = 5;

  const int num_knots = 3 * N;
  Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    testGyroRes<N>(s, t_ns);
  }
}

TEST(SplineSE3, AccelResidualTest) {
  static constexpr int N = 5;

  const int num_knots = 3 * N;
  Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    testAccelRes<N>(s, t_ns);
  }
}

TEST(SplineSE3, PositionResidualTest) {
  static constexpr int N = 5;

  const int num_knots = 3 * N;
  Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    testPositionRes<N>(s, t_ns);
  }
}

TEST(SplineSE3, OrientationResidualTest) {
  static constexpr int N = 5;

  const int num_knots = 3 * N;
  Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  for (int64_t t_ns = 0; t_ns < s.maxTimeNs(); t_ns += 1e8) {
    testOrientationRes<N>(s, t_ns);
  }
}

TEST(SplineSE3, PoseTest) {
  static constexpr int N = 5;

  const int num_knots = 3 * N;
  Se3Spline<N> s(int64_t(2e9));
  s.genRandomTrajectory(num_knots);

  int64_t offset = 100;

  for (int64_t t_ns = offset; t_ns < s.maxTimeNs() - offset; t_ns += 1e8) {
    testPose<N>(s, t_ns);
  }
}
