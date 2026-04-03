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
#ifndef __IMPEDANCE_CONTROL__
#define __IMPEDANCE_CONTROL__

#include "passive_control.h"  // Robot struct, pseudo_inverse, Utils

// ---------------------------------------------------------------------------
// ImpedanceController — joint-space PD torque law
//   τ = Kp ⊙ (q_des − q) + Kd ⊙ (q̇_des − q̇)
// ---------------------------------------------------------------------------
class ImpedanceController
{
private:
    Eigen::VectorXd _Kp;
    Eigen::VectorXd _Kd;

public:
    ImpedanceController(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd);
    ~ImpedanceController() = default;

    Eigen::VectorXd update(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& q_dot,
                           const Eigen::VectorXd& q_des,
                           const Eigen::VectorXd& q_dot_des);

    void set_gains(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd);
};

// ---------------------------------------------------------------------------
// ImpedanceControl — full robot controller (mirrors PassiveControl)
//   Accepts a desired Cartesian velocity + desired quaternion, converts to
//   joint-space via Jacobian pseudo-inverse, and runs ImpedanceController.
// ---------------------------------------------------------------------------
class ImpedanceControl
{
private:
    Robot _robot;
    iiwa_tools::IiwaTools _tools;

    // Extra pseudo-inverse for the angular Jacobian (not in Robot struct)
    Eigen::MatrixXd _pseudo_inv_jacobAng = Eigen::MatrixXd(3, 3);

    bool   first            = true;
    bool   is_just_velocity = false;
    double dsGain_pos   = 5.0;
    double dsGain_ori   = 2.5;
    double load_added   = 0.;
    double _dt;
    double _dt_scale    = 30.0;

    Eigen::VectorXd _trq_cmd = Eigen::VectorXd::Zero(7);

    std::unique_ptr<ImpedanceController> _impedanceCont;

    void computeTorqueCmd();

public:
    ImpedanceControl(const std::string& urdf_string,
                     const std::string& end_effector,
                     double dt = 0.001);
    ~ImpedanceControl();

    void updateRobot(const Eigen::VectorXd& jnt_p,
                     const Eigen::VectorXd& jnt_v,
                     const Eigen::VectorXd& jnt_t);

    // Command inputs
    void set_desired_pose(const Eigen::Vector3d& pos, const Eigen::Vector4d& quat);
    void set_desired_position(const Eigen::Vector3d& pos);
    void set_desired_velocity(const Eigen::Vector3d& vel);
    void set_desired_quat(const Eigen::Vector4d& quat);

    // Gain / configuration setters
    void set_pos_gain(const double& ds);
    void set_dt_scale(const double& scale);
    void set_impedance_gains(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd);
    void set_null_pos(const Eigen::VectorXd& nullPosition);

    // Returns the torque command computed from the latest robot state
    Eigen::VectorXd getCmd()
    {
        computeTorqueCmd();
        return _trq_cmd;
    }

    // State accessors
    Eigen::Vector3d getEEpos();
    Eigen::Vector3d getEEVel();
    Eigen::Vector3d getEEAngVel();
    Eigen::Vector4d getEEquat();
    double getMeasure();
};

#endif  // __IMPEDANCE_CONTROL__
