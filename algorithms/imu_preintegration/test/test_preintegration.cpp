#include "preintegration.h"
#include "se3_spline.h"
#include "test_utils.h"

#include <iostream>
#include <gtest/gtest.h>

static const Eigen::Vector3d G(0, 0, -9.81);
static const double ACCEL_STD_DEV = 0.23;
static const double GYRO_STD_DEV = 0.0027;

// Smaller noise for testing
// static const double accel_std_dev = 0.00023;
// static const double gyro_std_dev = 0.0000027;

std::random_device rd{};
std::mt19937 gen{rd()};

std::normal_distribution<> gyro_noise_dist{0, GYRO_STD_DEV};
std::normal_distribution<> accel_noise_dist{0, ACCEL_STD_DEV};

TEST(ImuPreintegrationTestCase, PredictTestGT) {
  int num_knots = 15;

  IntegratedImuMeasurement<double> imu_meas(0, Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  PoseVelState<double> state0;
  PoseVelState<double> state1;
  PoseVelState<double> state1_gt;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(20e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    ImuData<double> data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  state1_gt.T_w_i = gt_spline.pose(imu_meas.get_dt_ns());
  state1_gt.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns());

  imu_meas.predictState(state0, G, state1);

  EXPECT_TRUE(state1_gt.vel_w_i.isApprox(state1.vel_w_i, 1e-4))
      << "vel1_gt " << state1_gt.vel_w_i.transpose() << " vel1 "
      << state1.vel_w_i.transpose();

  EXPECT_LE(state1_gt.T_w_i.unit_quaternion().angularDistance(
                state1.T_w_i.unit_quaternion()),
            1e-6);

  EXPECT_TRUE(
      state1_gt.T_w_i.translation().isApprox(state1.T_w_i.translation(), 1e-4))
      << "pose1_gt p " << state1_gt.T_w_i.translation().transpose()
      << " pose1 p " << state1.T_w_i.translation().transpose();
}

TEST(ImuPreintegrationTestCase, PredictTest) {
  int num_knots = 15;

  Se3Spline<5> gt_spline(int64_t(2e9));
  gt_spline.genRandomTrajectory(num_knots);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2; t_ns < gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    ImuData<double> data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    PoseVelState<double> next_state;

    int64_t curr_state_t_ns = t_ns - dt_ns / 2;
    PoseVelState<double> curr_state(
        curr_state_t_ns, gt_spline.pose(curr_state_t_ns),
        gt_spline.transVelWorld(curr_state_t_ns));

    IntegratedImuMeasurement<double>::MatNN d_next_d_curr;
    IntegratedImuMeasurement<double>::MatN3 d_next_d_accel;
    IntegratedImuMeasurement<double>::MatN3 d_next_d_gyro;

    IntegratedImuMeasurement<double>::propagateState(
        curr_state, data, next_state, &d_next_d_curr, &d_next_d_accel,
        &d_next_d_gyro);

    {
      PoseVelState<double>::VecN x0;
      x0.setZero();
      test_jacobian(
          "F_TEST", d_next_d_curr,
          [&](const PoseVelState<double>::VecN& x) {
            PoseVelState<double> curr_state1 = curr_state;
            curr_state1.applyInc(x);
            PoseVelState<double> next_state1;

            IntegratedImuMeasurement<double>::propagateState(
                curr_state1, data, next_state1);

            return next_state.diff(next_state1);
          },
          x0);
    }

    {
      Eigen::Vector3d x0;
      x0.setZero();
      test_jacobian(
          "A_TEST", d_next_d_accel,
          [&](const Eigen::Vector3d& x) {
            ImuData<double> data1 = data;
            data1.accel += x;
            PoseVelState<double> next_state1;

            IntegratedImuMeasurement<double>::propagateState(
                curr_state, data1, next_state1);

            return next_state.diff(next_state1);
          },
          x0);
    }

    {
      Eigen::Vector3d x0;
      x0.setZero();
      test_jacobian(
          "G_TEST", d_next_d_gyro,
          [&](const Eigen::Vector3d& x) {
            ImuData<double> data1 = data;
            data1.gyro += x;
            PoseVelState<double> next_state1;

            IntegratedImuMeasurement<double>::propagateState(
                curr_state, data1, next_state1);

            return next_state.diff(next_state1);
          },
          x0, 1e-8);
    }
  }
}

TEST(ImuPreintegrationTestCase, ResidualTest) {
  int num_knots = 15;

  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  PoseVelState<double> state0;
  PoseVelState<double> state1;
  PoseVelState<double> state1_gt;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e8);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    ImuData<double> data;
    data.accel = accel_body + ba;
    data.gyro = rot_vel_body + bg;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  state1_gt.T_w_i = gt_spline.pose(imu_meas.get_dt_ns());
  state1_gt.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns());

  PoseVelState<double>::VecN res_gt =
      imu_meas.residual(state0, G, state1_gt, bg, ba);

  EXPECT_LE(res_gt.array().abs().maxCoeff(), 1e-6)
      << "res_gt " << res_gt.transpose();

  state1.T_w_i = gt_spline.pose(imu_meas.get_dt_ns()) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns()) +
                   Sophus::Vector3d::Random() / 10;

  IntegratedImuMeasurement<double>::MatNN d_res_d_state0;
  IntegratedImuMeasurement<double>::MatNN d_res_d_state1;
  IntegratedImuMeasurement<double>::MatN3 d_res_d_bg;
  IntegratedImuMeasurement<double>::MatN3 d_res_d_ba;

  imu_meas.residual(state0, G, state1, bg, ba,
                    &d_res_d_state0, &d_res_d_state1, &d_res_d_bg, &d_res_d_ba);

  {
    PoseVelState<double>::VecN x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_state0", d_res_d_state0,
        [&](const PoseVelState<double>::VecN& x) {
          PoseVelState<double> state0_new = state0;
          state0_new.applyInc(x);

          return imu_meas.residual(state0_new, G, state1, bg,
                                   ba);
        },
        x0);
  }

  {
    PoseVelState<double>::VecN x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_state1", d_res_d_state1,
        [&](const PoseVelState<double>::VecN& x) {
          PoseVelState<double> state1_new = state1;
          state1_new.applyInc(x);

          return imu_meas.residual(state0, G, state1_new, bg,
                                   ba);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", d_res_d_bg,
        [&](const Sophus::Vector3d& x) {
          return imu_meas.residual(state0, G, state1, bg + x,
                                   ba);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", d_res_d_ba,
        [&](const Sophus::Vector3d& x) {
          return imu_meas.residual(state0, G, state1, bg,
                                   ba + x);
        },
        x0);
  }
}

TEST(ImuPreintegrationTestCase, BiasTest) {
  int num_knots = 15;

  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body + ba);
    gyro_data_vec.emplace_back(rot_vel_body + bg);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    ImuData<double> data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  IntegratedImuMeasurement<double>::MatN3 d_res_d_ba;
  IntegratedImuMeasurement<double>::MatN3 d_res_d_bg;

  PoseVelState<double> delta_state = imu_meas.getDeltaState();

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", imu_meas.get_d_state_d_bg(),
        [&](const Sophus::Vector3d& x) {
          IntegratedImuMeasurement<double> imu_meas1(0, bg + x, ba);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            ImuData<double> data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          PoseVelState<double> delta_state1 = imu_meas1.getDeltaState();
          return delta_state.diff(delta_state1);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", imu_meas.get_d_state_d_ba(),
        [&](const Sophus::Vector3d& x) {
          IntegratedImuMeasurement<double> imu_meas1(0, bg, ba + x);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            ImuData<double> data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          PoseVelState<double> delta_state1 = imu_meas1.getDeltaState();
          return delta_state.diff(delta_state1);
        },
        x0);
  }
}

TEST(ImuPreintegrationTestCase, ResidualBiasTest) {
  int num_knots = 15;

  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  bg = Eigen::Vector3d::Random() / 100;
  ba = Eigen::Vector3d::Random() / 10;

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body + ba);
    gyro_data_vec.emplace_back(rot_vel_body + bg);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  IntegratedImuMeasurement<double> imu_meas(0, bg, ba);

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    ImuData<double> data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    imu_meas.integrate(data, Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones());
  }

  PoseVelState<double> state0;
  PoseVelState<double> state1;

  state0.T_w_i = gt_spline.pose(int64_t(0));
  state0.vel_w_i = gt_spline.transVelWorld(int64_t(0));

  state1.T_w_i = gt_spline.pose(imu_meas.get_dt_ns()) *
                 Sophus::se3_expd(Sophus::Vector6d::Random() / 10);
  state1.vel_w_i = gt_spline.transVelWorld(imu_meas.get_dt_ns()) +
                   Sophus::Vector3d::Random() / 10;

  Eigen::Vector3d bg_test = bg + Eigen::Vector3d::Random() / 1000;
  Eigen::Vector3d ba_test = ba + Eigen::Vector3d::Random() / 100;

  IntegratedImuMeasurement<double>::MatN3 d_res_d_ba;
  IntegratedImuMeasurement<double>::MatN3 d_res_d_bg;

  PoseVelState<double>::VecN res =
      imu_meas.residual(state0, G, state1, bg_test, ba_test,
                        nullptr, nullptr, &d_res_d_bg, &d_res_d_ba);

  {
    IntegratedImuMeasurement<double> imu_meas1(0, bg_test, ba_test);

    for (size_t i = 0; i < timestamps_vec.size(); i++) {
      ImuData<double> data;
      data.accel = accel_data_vec[i];
      data.gyro = gyro_data_vec[i];
      data.t_ns = timestamps_vec[i];

      imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                          Eigen::Vector3d::Ones());
    }

    PoseVelState<double>::VecN res1 = imu_meas1.residual(
        state0, G, state1, bg_test, ba_test);

    EXPECT_TRUE(res.isApprox(res1, 1e-4))
        << "res\n"
        << res.transpose() << "\nres1\n"
        << res1.transpose() << "\ndiff\n"
        << (res - res1).transpose() << std::endl;
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_ba", d_res_d_ba,
        [&](const Sophus::Vector3d& x) {
          IntegratedImuMeasurement<double> imu_meas1(0, bg_test,
                                                             ba_test + x);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            ImuData<double> data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          return imu_meas1.residual(state0, G, state1,
                                    bg_test, ba_test + x);
        },
        x0);
  }

  {
    Sophus::Vector3d x0;
    x0.setZero();
    test_jacobian(
        "d_res_d_bg", d_res_d_bg,
        [&](const Sophus::Vector3d& x) {
          IntegratedImuMeasurement<double> imu_meas1(0, bg_test + x,
                                                             ba_test);

          for (size_t i = 0; i < timestamps_vec.size(); i++) {
            ImuData<double> data;
            data.accel = accel_data_vec[i];
            data.gyro = gyro_data_vec[i];
            data.t_ns = timestamps_vec[i];

            imu_meas1.integrate(data, Eigen::Vector3d::Ones(),
                                Eigen::Vector3d::Ones());
          }

          return imu_meas1.residual(state0, G, state1,
                                    bg_test + x, ba_test);
        },
        x0, 1e-8, 1e-2);
  }
}

TEST(ImuPreintegrationTestCase, CovarianceTest) {
  int num_knots = 15;

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  Eigen::aligned_vector<Eigen::Vector3d> accel_data_vec;
  Eigen::aligned_vector<Eigen::Vector3d> gyro_data_vec;
  Eigen::aligned_vector<int64_t> timestamps_vec;

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(1e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    accel_data_vec.emplace_back(accel_body);
    gyro_data_vec.emplace_back(rot_vel_body);
    timestamps_vec.emplace_back(t_ns + dt_ns / 2);
  }

  Eigen::Vector3d accel_cov;
  Eigen::Vector3d gyro_cov;
  accel_cov.setConstant(ACCEL_STD_DEV * ACCEL_STD_DEV);
  gyro_cov.setConstant(GYRO_STD_DEV * GYRO_STD_DEV);

  IntegratedImuMeasurement<double> imu_meas(0, Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());

  for (size_t i = 0; i < timestamps_vec.size(); i++) {
    ImuData<double> data;
    data.accel = accel_data_vec[i];
    data.gyro = gyro_data_vec[i];
    data.t_ns = timestamps_vec[i];

    // std::cerr << "data.accel " << data.accel.transpose() << std::endl;

    // std::cerr << "cov " << i << "\n" << imu_meas.get_cov() << std::endl;
    imu_meas.integrate(data, accel_cov, gyro_cov);
  }

  // std::cerr << "cov\n" << imu_meas.get_cov() << std::endl;

  PoseVelState<double> delta_state = imu_meas.getDeltaState();

  IntegratedImuMeasurement<double>::MatNN cov_computed;
  cov_computed.setZero();

  const int num_samples = 1000;
  for (int i = 0; i < num_samples; i++) {
    IntegratedImuMeasurement<double> imu_meas1(
        0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < timestamps_vec.size(); i++) {
      ImuData<double> data;
      data.accel = accel_data_vec[i];
      data.gyro = gyro_data_vec[i];
      data.t_ns = timestamps_vec[i];

      data.accel[0] += accel_noise_dist(gen);
      data.accel[1] += accel_noise_dist(gen);
      data.accel[2] += accel_noise_dist(gen);

      data.gyro[0] += gyro_noise_dist(gen);
      data.gyro[1] += gyro_noise_dist(gen);
      data.gyro[2] += gyro_noise_dist(gen);

      imu_meas1.integrate(data, accel_cov, gyro_cov);
    }

    PoseVelState<double> delta_state1 = imu_meas1.getDeltaState();

    PoseVelState<double>::VecN diff = delta_state.diff(delta_state1);

    cov_computed += diff * diff.transpose();
  }

  cov_computed /= num_samples;
  // std::cerr << "cov_computed\n" << cov_computed << std::endl;

  double kl =
      (imu_meas.get_cov_inv() * cov_computed).trace() - 9 +
      std::log(imu_meas.get_cov().determinant() / cov_computed.determinant());

  // std::cerr << "kl " << kl << std::endl;
  EXPECT_LE(kl, 0.08);

  Eigen::VectorXd test_vec(num_samples);
  for (int i = 0; i < num_samples; i++) {
    test_vec[i] = accel_noise_dist(gen);
  }

  double mean = test_vec.mean();
  double var = (test_vec.array() - mean).square().sum() / num_samples;

  // Small test for rangdom generator
  EXPECT_LE(std::abs(std::sqrt(var) - ACCEL_STD_DEV), 0.03);
}

TEST(ImuPreintegrationTestCase, RandomWalkTest) {
  double dt = 0.005;

  double period = 200;
  double period_dt = period * dt;

  int num_samples = 10000;

  Eigen::VectorXd test_vec(num_samples);
  for (int j = 0; j < num_samples; j++) {
    double test = 0;
    for (int i = 0; i < period; i++) {
      test += gyro_noise_dist(gen) * std::sqrt(dt);
    }
    test_vec[j] = test;
  }

  double mean = test_vec.mean();
  double var = (test_vec.array() - mean).square().sum() / num_samples;
  double std = std::sqrt(var);

  EXPECT_NEAR(GYRO_STD_DEV * std::sqrt(period_dt), std, 1e-4);
  EXPECT_NEAR(GYRO_STD_DEV * GYRO_STD_DEV * period_dt, var, 1e-6);
}

TEST(ImuPreintegrationTestCase, ComputeCovInv) {
  using MatNN = IntegratedImuMeasurement<double>::MatNN;

  int num_knots = 15;

  IntegratedImuMeasurement<double> imu_meas(0, Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(20e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    ImuData<double> data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, 0.1 * Eigen::Vector3d::Ones(),
                       0.01 * Eigen::Vector3d::Ones());
  }

  MatNN cov_inv_computed = imu_meas.get_cov_inv();
  MatNN cov_inv_gt = imu_meas.get_cov().inverse();

  EXPECT_TRUE(cov_inv_computed.isApprox(cov_inv_gt, 1e-12))
      << "cov_inv_computed\n"
      << cov_inv_computed << "\ncov_inv_gt\n"
      << cov_inv_gt;
}

TEST(ImuPreintegrationTestCase, ComputeSqrtCovInv) {
  using MatNN = IntegratedImuMeasurement<double>::MatNN;

  int num_knots = 15;

  IntegratedImuMeasurement<double> imu_meas(0, Eigen::Vector3d::Zero(),
                                                    Eigen::Vector3d::Zero());

  Se3Spline<5> gt_spline(int64_t(10e9));
  gt_spline.genRandomTrajectory(num_knots);

  int64_t dt_ns = 1e7;
  for (int64_t t_ns = dt_ns / 2;
       t_ns < int64_t(20e9);  //  gt_spline.maxTimeNs() - int64_t(1e9);
       t_ns += dt_ns) {
    Sophus::SE3d pose = gt_spline.pose(t_ns);
    Eigen::Vector3d accel_body =
        pose.so3().inverse() *
        (gt_spline.transAccelWorld(t_ns) - G);
    Eigen::Vector3d rot_vel_body = gt_spline.rotVelBody(t_ns);

    ImuData<double> data;
    data.accel = accel_body;
    data.gyro = rot_vel_body;
    data.t_ns = t_ns + dt_ns / 2;  // measurement in the middle of the interval;

    imu_meas.integrate(data, 0.1 * Eigen::Vector3d::Ones(),
                       0.01 * Eigen::Vector3d::Ones());
  }

  MatNN sqrt_cov_inv_computed = imu_meas.get_sqrt_cov_inv();
  MatNN cov_inv_computed =
      sqrt_cov_inv_computed.transpose() * sqrt_cov_inv_computed;

  MatNN cov_inv_gt = imu_meas.get_cov().inverse();

  EXPECT_TRUE(cov_inv_computed.isApprox(cov_inv_gt, 1e-12))
      << "cov_inv_computed\n"
      << cov_inv_computed << "\ncov_inv_gt\n"
      << cov_inv_gt;
}
