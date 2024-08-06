#pragma once

#include <sophus/se3.hpp>

#include "imu_types.h"


inline Sophus::Vector6d relPoseError(
    const Sophus::SE3d& T_i_j, const Sophus::SE3d& T_w_i,
    const Sophus::SE3d& T_w_j, Sophus::Matrix6d* d_res_d_T_w_i = nullptr,
    Sophus::Matrix6d* d_res_d_T_w_j = nullptr) {
  Sophus::SE3d T_j_i = T_w_j.inverse() * T_w_i;
  Sophus::Vector6d res = Sophus::se3_logd(T_i_j * T_j_i);

  if (d_res_d_T_w_i || d_res_d_T_w_j) {
    Sophus::Matrix6d J;
    Sophus::rightJacobianInvSE3Decoupled(res, J);

    Eigen::Matrix3d R = T_w_i.so3().inverse().matrix();

    Sophus::Matrix6d Adj;
    Adj.setZero();
    Adj.topLeftCorner<3, 3>() = R;
    Adj.bottomRightCorner<3, 3>() = R;

    if (d_res_d_T_w_i) {
      *d_res_d_T_w_i = J * Adj;
    }

    if (d_res_d_T_w_j) {
      Adj.topRightCorner<3, 3>() =
          Sophus::SO3d::hat(T_j_i.inverse().translation()) * R;
      *d_res_d_T_w_j = -J * Adj;
    }
  }

  return res;
}

inline Sophus::Vector3d absPositionError(
    const Sophus::SE3d& T_w_i, const Eigen::Vector3d pos,
    Eigen::Matrix<double, 3, 6>* d_res_d_T_w_i = nullptr) {
  if (d_res_d_T_w_i) {
    d_res_d_T_w_i->topLeftCorner<3, 3>().setIdentity();
    d_res_d_T_w_i->topRightCorner<3, 3>().setZero();
  }

  return T_w_i.translation() - pos;
}

inline double yawError(const Sophus::SE3d& T_w_i,
                       const Eigen::Vector3d yaw_dir_body,
                       Eigen::Matrix<double, 1, 6>* d_res_d_T_w_i = nullptr) {
  Eigen::Matrix3d curr_R_w_i = T_w_i.so3().matrix();
  Eigen::Vector3d tmp = curr_R_w_i * yaw_dir_body;
  double res_yaw = tmp[1];

  if (d_res_d_T_w_i) {
    d_res_d_T_w_i->setZero();
    (*d_res_d_T_w_i)[3] = -tmp[2];
    (*d_res_d_T_w_i)[5] = tmp[0];
  }

  return res_yaw;
}

inline Sophus::Vector2d rollPitchError(
    const Sophus::SE3d& T_w_i, const Sophus::SO3d& R_w_i_meas,
    Eigen::Matrix<double, 2, 6>* d_res_d_T_w_i = nullptr) {
  // Assumes g direction is negative Z in world coordinate frame

  Eigen::Matrix3d R = (R_w_i_meas * T_w_i.so3().inverse()).matrix();
  Eigen::Vector3d res = R * (-Eigen::Vector3d::UnitZ());

  if (d_res_d_T_w_i) {
    d_res_d_T_w_i->setZero();
    (*d_res_d_T_w_i)(0, 3) = -R(0, 1);
    (*d_res_d_T_w_i)(1, 3) = -R(1, 1);
    (*d_res_d_T_w_i)(0, 4) = R(0, 0);
    (*d_res_d_T_w_i)(1, 4) = R(1, 0);
  }

  return res.head<2>();
}
