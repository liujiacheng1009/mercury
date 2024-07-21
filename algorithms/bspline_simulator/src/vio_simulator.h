#pragma once
#include "bsplineSE3.h"

#include <Eigen/Eigen>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

class BsplineSE3;

class SimulatorOptions
{
public:
  std::vector<Eigen::VectorXd> traj_data;
  int num_cameras{1};
  double gravity_norm{9.81};
  double gyro_bias_noise{0.001};
  double gyro_noise{0.005};
  double acc_bias_noise{0.01};
  double acc_noise{0.05};
  int sim_freq_imu{200};
  int num_feats{100};
  double sim_max_feature_gen_dist{10};
};

class VioSimulator {

public:
  VioSimulator(SimulatorOptions &params_);

  bool get_imu_state(double desired_time, Eigen::Matrix<double, 17, 1> &imustate);

  bool get_next_imu(double time_imu, Eigen::Vector3d &wm, Eigen::Vector3d &am);

  bool get_next_cam(double time_cam, std::vector<std::pair<size_t, Eigen::Vector2d>> &feats);

  std::unordered_map<size_t, Eigen::Vector3d> get_map() { return featmap; }

  double get_start_time(){return  this->spline->get_start_time();}

protected:
  std::vector<std::pair<size_t, Eigen::Vector2d>> project_pointcloud(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,std::unordered_map<size_t, Eigen::Vector3d> &feats);

  void generate_points(const Eigen::Matrix3d &R_GtoI, const Eigen::Vector3d &p_IinG,
                       std::unordered_map<size_t, Eigen::Vector3d> &feats, int numpts);

  SimulatorOptions params;

  std::shared_ptr<BsplineSE3> spline;

  size_t id_map = 0;
  std::unordered_map<size_t, Eigen::Vector3d> featmap;

  std::mt19937 gen_meas_imu;
  std::mt19937 gen_map_feat;

  Eigen::Vector3d true_bias_gyro{0., 0., 0};
  Eigen::Vector3d true_bias_accel{0., 0., 0};
};
