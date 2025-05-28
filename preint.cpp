#include "preint.h"
#include "earth.h"
#include "ins.h"
#include "mkl.h"
#include "trans.h"

Preint::Preint()
{
    att_pre = mkl::eye(3);
    vel_pre = Eigen::Vector3d::Zero();
    pos_pre = Eigen::Vector3d::Zero();
}

void Preint::accum(const Eigen::VectorXd& imu_m1, const Eigen::VectorXd& imu_m)
{
    Eigen::Vector3d wm_m1 = imu_m1.segment(0, 3);
    Eigen::Vector3d vm_m1 = imu_m1.segment(3, 3);
    Eigen::Vector3d wm_m = imu_m.segment(0, 3);
    Eigen::Vector3d vm_m = imu_m.segment(3, 3);
    double ts = imu_m(6) - imu_m1(6);

    Eigen::Vector3d dv_rot = 0.5 * wm_m.cross(vm_m);
    Eigen::Vector3d dv_scull = (wm_m1.cross(vm_m) + vm_m1.cross(wm_m)) / 12;
    Eigen::Vector3d dv_sfb = vm_m + dv_rot + dv_scull;
    pos_pre += vel_pre * ts + 0.5 * att_pre * dv_sfb * ts;

    vel_pre += att_pre * dv_sfb;

    Eigen::Vector3d phib = wm_m + wm_m1.cross(wm_m) / 12;
    Eigen::Matrix3d C_bk_bk1 = trans::rv2rm(phib);
    att_pre *= C_bk_bk1;
}

Eigen::VectorXd Preint::mech(const Eigen::VectorXd& ins_k1, const Eigen::VectorXd& imu_k)
{
    // time interval
    double t_k1 = ins_k1(10);
    double t_k = imu_k(6);
    double dt = t_k - t_k1;

    // position change due to velocity
    Eigen::Vector3d ve_k1 = ins_k1.segment(4, 3);
    Eigen::Vector3d dp_vel = ve_k1 * dt;

    // position change due to Gravity and Coriolis effects
    Eigen::Vector3d xyz_k1 = ins_k1.segment(7, 3);
    Eigen::Vector3d ge = earth::ge(xyz_k1);
    Eigen::Vector3d wiee = {0, 0, earth::wie};
    Eigen::Vector3d phie = wiee * dt;
    Eigen::Vector3d dp_gcor = 0.5 * ge * pow(dt, 2) - phie.cross(ve_k1 * dt);

    // position change due to specific force effect
    Eigen::Quaterniond q_bk1_ek1(Eigen::Vector4d(ins_k1.segment(0, 4)));
    Eigen::Matrix3d C_bk1_ek1(q_bk1_ek1.normalized());
    Eigen::Matrix3d I33 = mkl::eye(3);
    Eigen::Vector3d dp_sfe = (I33 - 0.5 * mkl::skew(phie)) * C_bk1_ek1 * pos_pre;

    // position update
    Eigen::Vector3d xyz_k = xyz_k1 + dp_vel + dp_gcor + dp_sfe;

    // Velocity update
    Eigen::Vector3d dv_gcor = ge * dt - 2 * wiee.cross(xyz_k - xyz_k1);
    Eigen::Vector3d dv_sfe = (I33 - 0.5 * mkl::skew(phie)) * C_bk1_ek1 * vel_pre;
    Eigen::Vector3d ve_k = ve_k1 + dv_gcor + dv_sfe;

    // attitude update
    Eigen::Matrix3d C_ek1_ek = trans::rv2rm(-phie);
    Eigen::Matrix3d C_bk_ek = mkl::orth(C_ek1_ek * C_bk1_ek1 * att_pre);
    Eigen::Quaterniond q_bk_ek(C_bk_ek);

    // ins_k
    Eigen::VectorXd ins_k = (Eigen::VectorXd(11) << q_bk_ek.coeffs(), ve_k, xyz_k, t_k).finished();

    // reset
    att_pre = I33;
    vel_pre = Eigen::Vector3d::Zero();
    pos_pre = Eigen::Vector3d::Zero();

    return ins_k;
}

Eigen::MatrixXd Preint::pure(const Eigen::MatrixXd& gt, const Eigen::MatrixXd& imu, const double& ts)
{
    Preint preint;
    Eigen::Index nrow = imu.rows();
    Eigen::MatrixXd ins = Eigen::MatrixXd::Constant(nrow, 11, mkl::NaN);
    Eigen::VectorXd ins_k1 = Eigen::VectorXd::Zero(11);

    for (Eigen::Index k = 0; k < nrow; k++)
    {
        if (k == 0)
        {
            ins.row(k) = gt.row(k);
            ins_k1 = gt.row(k);
            continue;
        }

        Eigen::VectorXd imu_k1 = mkl::toCol(imu.row(k - 1));
        Eigen::VectorXd imu_k = mkl::toCol(imu.row(k));
        preint.accum(imu_k1, imu_k);

        double t_k = imu_k(6);
        double t_k1 = ins_k1(10);
        if (mkl::geq(t_k - t_k1, ts))
        {
            Eigen::VectorXd ins_k = preint.mech(ins_k1, imu_k);
            ins::damp(ins_k, gt.row(k));
            ins.row(k) = mkl::toRow(ins_k);
            ins_k1 = ins_k;
        }
    }
    mkl::rmNaN(ins);

    return ins;
}
