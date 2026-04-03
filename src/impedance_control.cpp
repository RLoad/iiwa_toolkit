//|
//|    Copyright (C) 2020 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Farshad Khadivr (maintainer)
//|    email:   farshad.khadivar@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of iiwa_toolkit.
//|
//|    iiwa_toolkit is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_toolkit is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|

#include "impedance_control.h"

// ===========================================================================
// ImpedanceController
// ===========================================================================

ImpedanceController::ImpedanceController(const Eigen::VectorXd& Kp,
                                         const Eigen::VectorXd& Kd)
    : _Kp(Kp), _Kd(Kd)
{
}

void ImpedanceController::set_gains(const Eigen::VectorXd& Kp,
                                    const Eigen::VectorXd& Kd)
{
    _Kp = Kp;
    _Kd = Kd;
}

Eigen::VectorXd ImpedanceController::update(const Eigen::VectorXd& q,
                                             const Eigen::VectorXd& q_dot,
                                             const Eigen::VectorXd& q_des,
                                             const Eigen::VectorXd& q_dot_des)
{
    // τ = Kp ⊙ (q_des − q) + Kd ⊙ (q̇_des − q̇)
    return _Kp.cwiseProduct(q_des - q) + _Kd.cwiseProduct(q_dot_des - q_dot);
}

// ===========================================================================
// ImpedanceControl
// ===========================================================================

ImpedanceControl::ImpedanceControl(const std::string& urdf_string,
                                   const std::string& end_effector,
                                   double dt)
    : _dt(dt)
{
    _tools.init_rbdyn(urdf_string, end_effector);

    // Default impedance gains
    Eigen::VectorXd Kp(7), Kd(7);
    Kp << 100., 100., 100., 80., 50., 30., 10.;
    Kd <<  10., 10., 10., 8., 5., 3., 1.;
    _impedanceCont = std::make_unique<ImpedanceController>(Kp, Kd);

    _robot.name += std::to_string(0);
    _robot.jnt_position.setZero();
    _robot.jnt_velocity.setZero();
    _robot.jnt_torque.setZero();
    _robot.nulljnt_position.setZero();
    _robot.ee_pos.setZero();
    _robot.ee_vel.setZero();
    _robot.ee_acc.setZero();

    _robot.Measure = 0.0;

    double angle0 = 0.25 * M_PI;
    _robot.ee_quat[0] = std::cos(angle0 / 2);
    _robot.ee_quat.segment(1, 3) = std::sin(angle0 / 2) * Eigen::Vector3d::UnitZ();

    _robot.ee_angVel.setZero();
    _robot.ee_angAcc.setZero();

    // Desired state defaults
    _robot.ee_des_vel.setZero();
    _robot.ee_des_acc.setZero();
    _robot.ee_des_angVel.setZero();
    _robot.ee_des_angAcc.setZero();

    double angled = 1.0 * M_PI;
    _robot.ee_des_quat[0] = std::cos(angled / 2);
    _robot.ee_des_quat.segment(1, 3) = std::sin(angled / 2) * Eigen::Vector3d::UnitX();

    _robot.jacob.setZero();
    _robot.jacob_drv.setZero();
    _robot.jacob_t_pinv.setZero();
    _robot.jacobPos.setZero();
    _robot.jacobAng.setZero();
    _robot.pseudo_inv_jacob.setZero();
    _robot.pseudo_inv_jacobPos.setZero();
    _pseudo_inv_jacobAng.setZero();

    _robot.nulljnt_position << 0.0, 0.0, 0.0, -.75, 0., 0.0, 0.0;
}

ImpedanceControl::~ImpedanceControl() {}

void ImpedanceControl::updateRobot(const Eigen::VectorXd& jnt_p,
                                   const Eigen::VectorXd& jnt_v,
                                   const Eigen::VectorXd& jnt_t)
{
    _robot.jnt_position = jnt_p;
    _robot.jnt_velocity = jnt_v;
    _robot.jnt_torque   = jnt_t;

    iiwa_tools::RobotState robot_state;
    robot_state.position.resize(jnt_p.size());
    robot_state.velocity.resize(jnt_p.size());
    for (size_t i = 0; i < jnt_p.size(); i++) {
        robot_state.position[i] = _robot.jnt_position[i];
        robot_state.velocity[i] = _robot.jnt_velocity[i];
    }

    std::tie(_robot.jacob, _robot.jacob_drv) = _tools.jacobians(robot_state);
    _robot.jacobPos = _robot.jacob.bottomRows(3);
    _robot.jacobAng = _robot.jacob.topRows(3);

    _robot.pseudo_inv_jacob    = pseudo_inverse(Eigen::MatrixXd(_robot.jacob    * _robot.jacob.transpose()));
    _robot.pseudo_inv_jacobPos = pseudo_inverse(Eigen::MatrixXd(_robot.jacobPos * _robot.jacobPos.transpose()));
    _pseudo_inv_jacobAng       = pseudo_inverse(Eigen::MatrixXd(_robot.jacobAng * _robot.jacobAng.transpose()));
    _robot.pseudo_inv_jacobJnt = pseudo_inverse(Eigen::MatrixXd(_robot.jacob.transpose() * _robot.jacob));

    auto ee_state = _tools.perform_fk(robot_state);
    _robot.ee_pos      = ee_state.translation;
    _robot.ee_quat[0]  = ee_state.orientation.w();
    _robot.ee_quat.segment(1, 3) = ee_state.orientation.vec();

    Eigen::VectorXd vel = _robot.jacob * _robot.jnt_velocity;
    _robot.ee_vel    = vel.tail(3);
    _robot.ee_angVel = vel.head(3);

    Eigen::MatrixXd matJacob = _robot.jacob * _robot.jacob.transpose();
    double manipulability = std::sqrt(matJacob.determinant());
    _robot.Measure = manipulability;

    Eigen::EigenSolver<Eigen::MatrixXd> solver(matJacob);
    (void)solver;  // eigenvalues computed for completeness; unused here
}

void ImpedanceControl::set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat)
{
    _robot.ee_des_pos  = pos;
    _robot.ee_des_quat = quat;
    is_just_velocity   = false;
}

void ImpedanceControl::set_desired_position(const Eigen::Vector3d& pos)
{
    _robot.ee_des_pos = pos;
    is_just_velocity  = false;
}

void ImpedanceControl::set_desired_velocity(const Eigen::Vector3d& vel)
{
    _robot.ee_des_vel = vel;
    is_just_velocity  = true;
}

void ImpedanceControl::set_desired_quat(const Eigen::Vector4d& quat)
{
    _robot.ee_des_quat = quat;
}

void ImpedanceControl::set_pos_gain(const double& ds)
{
    dsGain_pos = ds;
}

void ImpedanceControl::set_dt_scale(const double& scale)
{
    _dt_scale = scale;
}

void ImpedanceControl::set_impedance_gains(const Eigen::VectorXd& Kp,
                                           const Eigen::VectorXd& Kd)
{
    _impedanceCont->set_gains(Kp, Kd);
}

void ImpedanceControl::set_null_pos(const Eigen::VectorXd& nullPosition)
{
    if (nullPosition.size() == _robot.nulljnt_position.size()) {
        _robot.nulljnt_position = nullPosition;
    } else {
        ROS_ERROR("wrong size for the null joint position");
    }
}

Eigen::Vector3d ImpedanceControl::getEEpos()    { return _robot.ee_pos; }
Eigen::Vector4d ImpedanceControl::getEEquat()   { return _robot.ee_quat; }
Eigen::Vector3d ImpedanceControl::getEEVel()    { return _robot.ee_vel; }
Eigen::Vector3d ImpedanceControl::getEEAngVel() { return _robot.ee_angVel; }
double          ImpedanceControl::getMeasure()  { return _robot.Measure; }

void ImpedanceControl::computeTorqueCmd()
{
    // ------------------------------------------------------------------
    // 0. Position DS attractor → ee_des_vel  (mirrors PassiveControl)
    //    Skipped when in pure velocity-command mode.
    // ------------------------------------------------------------------
    if (!is_just_velocity) {
        Eigen::Vector3d deltaX = _robot.ee_des_pos - _robot.ee_pos;
        double maxDx = 0.1;
        if (deltaX.norm() > maxDx)
            deltaX = maxDx * deltaX.normalized();
        double theta_g = (-.5 / (4 * maxDx * maxDx)) * deltaX.transpose() * deltaX;
        _robot.ee_des_vel = dsGain_pos * (1.0 + std::exp(theta_g)) * deltaX;
    }

    // ------------------------------------------------------------------
    // 1. Derive desired angular velocity from quaternion error
    //    (identical to PassiveControl::computeTorqueCmd())
    // ------------------------------------------------------------------
    Eigen::Vector4d dqd     = Utils<double>::slerpQuaternion(_robot.ee_quat, _robot.ee_des_quat, 0.5);
    Eigen::Vector4d deltaQ  = dqd - _robot.ee_quat;

    Eigen::Vector4d qconj = _robot.ee_quat;
    qconj.segment(1, 3) = -1 * qconj.segment(1, 3);
    Eigen::Vector4d temp_angVel = Utils<double>::quaternionProduct(deltaQ, qconj);

    Eigen::Vector3d tmp_angular_vel = temp_angVel.segment(1, 3);
    double maxDq = 0.2;
    if (tmp_angular_vel.norm() > maxDq)
        tmp_angular_vel = maxDq * tmp_angular_vel.normalized();

    double theta_gq = (-.5 / (4 * maxDq * maxDq)) * tmp_angular_vel.transpose() * tmp_angular_vel;
    _robot.ee_des_angVel = 2.0 * dsGain_ori * (1.0 + std::exp(theta_gq)) * tmp_angular_vel;

    // ------------------------------------------------------------------
    // 2. Convert desired Cartesian velocity to joint space
    //    q̇_des = J_pos^T · (J_pos · J_pos^T)^{-1} · v_des
    //           + J_ang^T · (J_ang · J_ang^T)^{-1} · ω_des
    // ------------------------------------------------------------------
    Eigen::VectorXd q_dot_des =
        _robot.jacobPos.transpose() * _robot.pseudo_inv_jacobPos * _robot.ee_des_vel
      + _robot.jacobAng.transpose() * _pseudo_inv_jacobAng        * _robot.ee_des_angVel;

    // ------------------------------------------------------------------
    // 3. Integrate for desired joint position
    // ------------------------------------------------------------------
    const double dt_eff = _dt * _dt_scale;
    Eigen::VectorXd q_des = _robot.jnt_position + q_dot_des * dt_eff;
    
    // ------------------------------------------------------------------
    // 4. Joint-space impedance torque
    // ------------------------------------------------------------------
    Eigen::VectorXd tau_task = _impedanceCont->update(
        _robot.jnt_position, _robot.jnt_velocity, q_des, q_dot_des);

    // ------------------------------------------------------------------
    // 5. Gravity compensation for attached load
    // ------------------------------------------------------------------
    tau_task += _robot.jacobPos.transpose() * (load_added * 9.8 * Eigen::Vector3d::UnitZ());

    // ------------------------------------------------------------------
    // 6. Null-space control  (identical to PassiveControl::computeTorqueCmd())
    // ------------------------------------------------------------------
    Eigen::MatrixXd tempMat2 =
        Eigen::MatrixXd::Identity(7, 7)
        - _robot.jacob.transpose() * _robot.pseudo_inv_jacob * _robot.jacob;

    Eigen::VectorXd nullgains = Eigen::VectorXd::Zero(7);
    nullgains << 5., 80, 10., 30, 5., 2., 2.;

    Eigen::VectorXd er_null = _robot.jnt_position - _robot.nulljnt_position;
    if (er_null.norm() < 1.5) {
        first = false;
    }
    if (er_null.norm() > 2e-1) {
        er_null = 0.2 * er_null.normalized();
    }

    Eigen::VectorXd tmp_null_trq = Eigen::VectorXd::Zero(7);
    for (int i = 0; i < 7; i++) {
        tmp_null_trq[i]  = -nullgains[i] * er_null[i];
        tmp_null_trq[i] += -1. * _robot.jnt_velocity[i];
    }

    if (first) {
        _trq_cmd = tmp_null_trq;
        ROS_INFO_ONCE("going to the first pose");
    } else {
        ROS_INFO_ONCE("Impedance tracking in process");
        _trq_cmd = tau_task + 10. * tempMat2 * tmp_null_trq;
    }
}
