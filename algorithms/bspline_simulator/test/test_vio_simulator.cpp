#include "vio_simulator.h"
#include "gtest/gtest.h"
#include <sophus/so3.hpp>

class VioSimulatorTest : public ::testing::Test
{
protected:
    VioSimulatorTest() {}
    ~VioSimulatorTest() {}

    virtual void SetUp()
    {
        SimulatorOptions opt;
        opt.traj_data = gen_traj_data();
        sim = std::shared_ptr<VioSimulator>(new VioSimulator(opt));
    }
    virtual void TearDown() {}

    std::vector<Eigen::VectorXd> gen_traj_data(){
        double timestamp = 0;
        double dt = 0.05;
        int N = 20;
        std::vector<Eigen::VectorXd> traj_data;
        Eigen::Vector3d pos{0,0,0};
        Eigen::Quaterniond quat{1,0,0,0};
        for(size_t i = 0; i< N ; ++i){
            Eigen::Vector3d dpos = 0.01 * Eigen::Vector3d::Random();
            Eigen::Vector3d dquat = 0.005 * Eigen::Vector3d::Random();
            pos += dpos;
            Sophus::SO3d delta_rot = Sophus::SO3d::exp(dquat);
            quat *= delta_rot.unit_quaternion();
            Eigen::VectorXd data(8);
            data << timestamp,pos(0), pos(1), pos(2),quat.w(),quat.x(), quat.y(), quat.z();
            traj_data.emplace_back(data);
            timestamp += dt;
        }
        return traj_data;
    }

public:
    std::shared_ptr<VioSimulator> sim;
};

void predictImu2(double dt, Eigen::Vector3d wm, Eigen::Vector3d am, 
Eigen::Vector3d& curr_pos, Eigen::Vector3d& curr_vel, Eigen::Quaterniond& curr_rot
){
    Eigen::Vector3d gravity{0,0,9.81};
    curr_pos += curr_vel * dt + 0.5 * (curr_rot.toRotationMatrix() * am - gravity) * (dt * dt);
    curr_vel += (curr_rot.toRotationMatrix() * am - gravity) * dt;
    auto drot = wm * dt;
    curr_rot *= Eigen::Quaterniond(1, 0.5 * drot(0), 0.5 * drot(1), 0.5 * drot(2));
}

void predictImu(double dt, Eigen::Vector3d wm, Eigen::Vector3d am, 
Eigen::Vector3d& curr_pos, Eigen::Vector3d& curr_vel, Eigen::Quaterniond& curr_rot
){
    Eigen::Vector3d gravity{0,0,9.81};
    curr_pos += curr_vel * dt + 0.5 * (curr_rot.toRotationMatrix() * am - gravity) * (dt * dt);
    curr_vel += (curr_rot.toRotationMatrix() * am - gravity) * dt;
    Sophus::SO3d delta_rot = Sophus::SO3d::exp(wm * dt);
    curr_rot *= delta_rot.unit_quaternion();
}

TEST_F(VioSimulatorTest, GenIMUMeas)
{
    double timestamp0 = sim->get_start_time();
    double timestamp1 = timestamp0 + 0.5;
    Eigen::Matrix<double, 17, 1> imu_state0, imu_state1;
    sim->get_imu_state(timestamp0, imu_state0);
    sim->get_imu_state(timestamp1, imu_state1);
    // std::cout<< "pos0 : " << imu_state0.block(5, 0, 3, 1).transpose() << std::endl;
    // std::cout<< "pos1 : " << imu_state1.block(5, 0, 3, 1).transpose() << std::endl;

    Eigen::Vector3d gravity{0, 0,9.81};
    Eigen::Vector3d curr_pos = imu_state0.block(5, 0, 3, 1);
    Eigen::Quaterniond curr_rot = Eigen::Quaterniond(imu_state0(4,0),imu_state0(1,0),imu_state0(2,0),imu_state0(3,0)).inverse();
    // std::cout << "curr_rot0 :" << curr_rot.toRotationMatrix().eulerAngles(0,1,2).transpose() << std::endl;
    Eigen::Quaterniond curr_rot1 = Eigen::Quaterniond(imu_state1(4,0),imu_state1(1,0),imu_state1(2,0),imu_state1(3,0)).inverse();
    // std::cout << "curr_rot1 :" << curr_rot1.toRotationMatrix().eulerAngles(0,1,2).transpose() << std::endl;
    Eigen::Vector3d curr_vel = imu_state0.block(8,0,3,1);
    double dt = 1.0/ 200.;
    for(size_t i = 0; i < 100;i++){
        double curr_timestamp = timestamp0 + i * dt;
        if(curr_timestamp > timestamp1) break;
        Eigen::Vector3d wm, am;
        if(!sim->get_next_imu(timestamp0 + i * dt, wm,am)) break;
        predictImu2(dt,wm, am,curr_pos, curr_vel, curr_rot);
        if(i % 10 == 0){
            std::vector<std::pair<size_t, Eigen::Vector2d>> feats;
            if(!sim->get_next_cam(timestamp0 + i * dt,feats)) break;
            EXPECT_GT(feats.size(), 0); // hhh
        }
    }
    // std::cout<< "curr_pos : " << curr_pos.transpose() << std::endl;
    EXPECT_NEAR((curr_pos-imu_state1.block(5, 0, 3, 1)).norm(),0,5.e-3);
    // std::cout<< "curr_rot : " << curr_rot.toRotationMatrix().eulerAngles(0,1,2).transpose() << std::endl;
    EXPECT_NEAR((curr_rot.inverse()*curr_rot1).toRotationMatrix().eulerAngles(0,1,2).norm(),0,1.e-2);
    // std::cout << (curr_rot.inverse()*curr_rot1).toRotationMatrix().eulerAngles(0,1,2) << std::endl;
}

