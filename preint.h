#pragma once

#include <Eigen/Dense>

class Preint
{
private:
    Eigen::Matrix3d att_pre;
    Eigen::Vector3d vel_pre;
    Eigen::Vector3d pos_pre;

public:
    Preint();

    void accum(const Eigen::VectorXd& imu_m1, const Eigen::VectorXd& imu_m);

    Eigen::VectorXd mech(const Eigen::VectorXd& ins_k1, const Eigen::VectorXd& imu_k);

    static Eigen::MatrixXd pure(const Eigen::MatrixXd& gt, const Eigen::MatrixXd& imu, const double& ts);
};
