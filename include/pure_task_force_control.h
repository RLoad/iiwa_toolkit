//|
//|  Cartesian wrench regulation with optional velocity damping / feedforward.
//|  F_lin = F_des + Kv_lin * (v_des - v_ee), same for angular part.
//|
#ifndef IIWA_TOOLKIT_PURE_TASK_FORCE_CONTROL_H
#define IIWA_TOOLKIT_PURE_TASK_FORCE_CONTROL_H

#include <Eigen/Dense>
#include <iiwa_tools/iiwa_tools.h>
#include "ros/ros.h"
#include "pure_task_pseudo_inverse.hpp"

struct PureTaskForceRobotState
{
    static constexpr unsigned int no_joints = 7;
    Eigen::VectorXd jnt_position = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(no_joints);
    Eigen::VectorXd jnt_torque = Eigen::VectorXd(no_joints);
    Eigen::VectorXd nulljnt_position = Eigen::VectorXd(no_joints);

    Eigen::Vector3d ee_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_angVel = Eigen::Vector3d::Zero();
    Eigen::Vector4d ee_quat = Eigen::Vector4d::Zero();

    Eigen::MatrixXd jacob = Eigen::MatrixXd(6, 7);
    Eigen::MatrixXd jacobPos = Eigen::MatrixXd(3, 7);
    Eigen::MatrixXd jacobAng = Eigen::MatrixXd(3, 7);
    Eigen::MatrixXd pseudo_inv_jacob = Eigen::MatrixXd(6, 6);
};

class PureTaskForceControl
{
public:
    explicit PureTaskForceControl(const std::string& urdf_string, const std::string& end_effector);

    void updateRobot(const Eigen::VectorXd& jnt_p, const Eigen::VectorXd& jnt_v, const Eigen::VectorXd& jnt_t);

    void set_desired_wrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque);
    void set_desired_twist_for_damping(const Eigen::Vector3d& lin_vel, const Eigen::Vector3d& ang_vel);
    void set_null_pos(const Eigen::VectorXd& nullPosition);

    void set_velocity_coupling_gains(double Kv_lin, double Kv_ang);
    void set_load_mass(double mass);

    Eigen::VectorXd getCmd()
    {
        computeTorqueCmd();
        return _trq_cmd;
    }
    Eigen::Vector3d getEEpos() const { return _robot.ee_pos; }
    Eigen::Vector4d getEEquat() const { return _robot.ee_quat; }
    Eigen::Vector3d getEEVel() const { return _robot.ee_vel; }
    Eigen::Vector3d getEEAngVel() const { return _robot.ee_angVel; }

private:
    void computeTorqueCmd();

    PureTaskForceRobotState _robot;
    iiwa_tools::IiwaTools _tools;
    bool first_ = true;

    Eigen::Vector3d F_des_lin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d F_des_ang_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_des_lin_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_des_ang_ = Eigen::Vector3d::Zero();

    double Kv_lin_ = 40.0;
    double Kv_ang_ = 20.0;
    double load_mass_ = 0.0;

    Eigen::VectorXd _trq_cmd = Eigen::VectorXd::Zero(7);
};

#endif
