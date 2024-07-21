#include "vio_simulator.h"
#include "bsplineSE3.h"

VioSimulator::VioSimulator(SimulatorOptions &params) {
  this->params = params;
  spline = std::make_shared<BsplineSE3>();
  spline->feed_trajectory(params.traj_data);
  gen_meas_imu.seed(0);
  gen_map_feat.seed(0);
  double time_synth = spline->get_start_time();
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline->get_pose(time_synth, R_GtoI, p_IinG);
  generate_points(R_GtoI, p_IinG, featmap, params.num_feats);
}

bool VioSimulator::get_imu_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate) {
  // Set to default state
  imustate.setZero();
  imustate(4) = 1;
  // Current state values
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG;
  // Get the pose, velocity, and acceleration
  bool success_vel = spline->get_velocity(desired_time, R_GtoI, p_IinG, w_IinI, v_IinG);
  if (!success_vel) {
    return false;
  }
  // Finally lets create the current state
  imustate(0, 0) = desired_time;
  Eigen::Quaterniond quat(R_GtoI);
  imustate.block(1, 0, 4, 1) = Eigen::Vector4d(quat.x(), quat.y(), quat.z(), quat.w());
  imustate.block(5, 0, 3, 1) = p_IinG;
  imustate.block(8, 0, 3, 1) = v_IinG;
  imustate.block(11, 0, 3, 1) = Eigen::MatrixXd::Zero(3,1);
  imustate.block(14, 0, 3, 1) = Eigen::MatrixXd::Zero(3,1);
  return true;
}

bool VioSimulator::get_next_imu(double time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am)
{
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;
  bool success_accel = spline->get_acceleration(time_imu, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

  if (!success_accel)
    return false;

  // Transform omega and linear acceleration into imu frame
  Eigen::Vector3d omega_inI = w_IinI;
  Eigen::Vector3d gravity;
  gravity << 0.0, 0.0, params.gravity_norm;
  Eigen::Vector3d accel_inI = R_GtoI * (a_IinG + gravity);

  double dt = 1.0 / params.sim_freq_imu;
  std::normal_distribution<double> w(0., 1.);
  true_bias_gyro(0) += params.gyro_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_gyro(1) += params.gyro_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_gyro(2) += params.gyro_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(0) += params.acc_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(1) += params.acc_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  true_bias_accel(2) += params.acc_bias_noise * std::sqrt(dt) * w(gen_meas_imu);
  // Now add noise to these measurements
  wm(0) = omega_inI(0) + true_bias_gyro(0) + params.gyro_noise / std::sqrt(dt) * w(gen_meas_imu);
  wm(1) = omega_inI(1) + true_bias_gyro(1) + params.gyro_noise / std::sqrt(dt) * w(gen_meas_imu);
  wm(2) = omega_inI(2) + true_bias_gyro(2) + params.gyro_noise / std::sqrt(dt) * w(gen_meas_imu);
  am(0) = accel_inI(0) + true_bias_accel(0) + params.acc_noise / std::sqrt(dt) * w(gen_meas_imu);
  am(1) = accel_inI(1) + true_bias_accel(1) + params.acc_noise / std::sqrt(dt) * w(gen_meas_imu);
  am(2) = accel_inI(2) + true_bias_accel(2) + params.acc_noise / std::sqrt(dt) * w(gen_meas_imu);
  // Return success
  return true;
}

bool VioSimulator::get_next_cam(double time_cam, std::vector<std::pair<size_t, Eigen::Vector2d>> &feats)
{
  Eigen::Matrix3d R_GtoI;
  Eigen::Vector3d p_IinG;
  bool success_pose = spline->get_pose(time_cam, R_GtoI, p_IinG);
  if (!success_pose)
    return false;
  feats = project_pointcloud(R_GtoI, p_IinG, featmap);
  return true;
}

std::vector<std::pair<size_t, Eigen::Vector2d>> VioSimulator::project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG, std::unordered_map<size_t, Eigen::Vector3d> &featmap)
{
  std::vector<std::pair<size_t, Eigen::Vector2d>> uvs;
  for (auto it = featmap.begin(); it != featmap.end();)
  {
    Eigen::Vector3d p_FinI0 = R_GtoI * (it->second - p_IinG);
    Eigen::Vector3d p_FinC0 = p_FinI0;
    if (p_FinC0(2) > params.sim_max_feature_gen_dist || p_FinC0(2) < 0.1)
    {
      it = featmap.erase(it);
      continue;
    }

    Eigen::Vector2d uv_norm0;
    uv_norm0 << (float)(p_FinC0(0) / p_FinC0(2)), (float)(p_FinC0(1) / p_FinC0(2));
    if (uv_norm0.norm() > 1.)
    {
      it = featmap.erase(it);
      continue;
    }
    uvs.push_back({it->first, uv_norm0});
    it++;
  }
  if (uvs.size() < params.num_feats)
  {
    generate_points(R_GtoI, p_IinG, featmap, params.num_feats - (int)uvs.size());
  }
  return uvs;
}

void VioSimulator::generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                                std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts)
{
  for (int i = 0; i < numpts; i++)
  {
    std::uniform_real_distribution<double> gen_u(0, 1);
    std::uniform_real_distribution<double> gen_v(0, 1);
    double u_dist = gen_u(gen_map_feat);
    double v_dist = gen_v(gen_map_feat);
    cv::Point2f uv_norm(gen_u(gen_map_feat), gen_v(gen_map_feat));
    std::uniform_real_distribution<double> gen_depth(0.1, params.sim_max_feature_gen_dist);
    double depth = gen_depth(gen_map_feat);

    // Get the 3d point
    Eigen::Vector3d bearing;
    bearing << uv_norm.x, uv_norm.y, 1;
    Eigen::Vector3d p_FinC;
    p_FinC = depth * bearing;
    Eigen::Vector3d p_FinI = p_FinC;
    Eigen::Vector3d p_FinG = R_GtoI.transpose() * p_FinI + p_IinG;
    // Append this as a new feature
    featmap.insert({id_map, p_FinG});
    id_map++;
  }
}
