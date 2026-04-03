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
#include <mutex>
#include <fstream>
#include <pthread.h>
#include <memory>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>

#include "impedance_control.h"
#include "iiwa_toolkit/passive_cfg_paramsConfig.h"
#include "dynamic_reconfigure/server.h"

#define No_JOINTS 7
#define No_Robots 1
#define TOTAL_No_MARKERS 2

struct Options
{
    std::string control_mode;
    bool is_optitrack_on;
    double filter_gain = 0.;
};

struct feedback
{
    Eigen::VectorXd jnt_position = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd jnt_velocity = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd jnt_torque   = Eigen::VectorXd(No_JOINTS);
};


class IiwaRosMaster
{
  public:
    IiwaRosMaster(ros::NodeHandle& n, double frequency, Options options)
        : _n(n), _loopRate(frequency), _dt(1.0f / frequency), _options(options)
    {
        _stop = false;
    }

    ~IiwaRosMaster() {}

    bool init()
    {
        std::string ns = _n.getNamespace();
        std::string robot_name;
        if (ns.substr(0, 1) == "/")
            robot_name = ns.substr(1, ns.size() - 1);
        else {
            robot_name = ns;
            ns = "/" + robot_name;
        }
        std::cout << "the namespace is: " + ns << std::endl;

        _feedback.jnt_position.setZero();
        _feedback.jnt_velocity.setZero();
        _feedback.jnt_torque.setZero();
        command_trq.setZero();
        command_plt.setZero();

        // Joint state subscriber
        _subRobotStates[0] = _n.subscribe<sensor_msgs::JointState>(
            ns + "/joint_states", 1,
            boost::bind(&IiwaRosMaster::updateRobotStates, this, _1, 0),
            ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

        // Velocity + quaternion command (only subscriber for impedance control)
        _subControl = _n.subscribe<geometry_msgs::Pose>(
            "/passive_control/vel_quat", 1,
            boost::bind(&IiwaRosMaster::updateControlVel, this, _1),
            ros::VoidPtr(), ros::TransportHints().reliable().tcpNoDelay());

        _TrqCmdPublisher  = _n.advertise<std_msgs::Float64MultiArray>(ns + "/TorqueController/command", 1);
        _EEPosePublisher  = _n.advertise<geometry_msgs::Pose>(ns + "/ee_info/Pose", 1);
        _EEVelPublisher   = _n.advertise<geometry_msgs::Twist>(ns + "/ee_info/Vel", 1);
        _MeasurePublisher = _n.advertise<geometry_msgs::Twist>(ns + "/measure", 1);

        // URDF
        std::string urdf_string, full_param;
        std::string robot_description = ns + "/robot_description";
        std::string end_effector;

        if (!_n.searchParam(robot_description, full_param)) {
            ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
            return false;
        }
        while (urdf_string.empty()) {
            ROS_INFO_ONCE_NAMED("Controller", "Controller is waiting for model"
                                " URDF in parameter [%s] on the ROS param server.",
                                robot_description.c_str());
            _n.getParam(full_param, urdf_string);
            usleep(100000);
        }
        ROS_INFO_STREAM_NAMED("Controller", "Received urdf from param server, parsing...");

        _n.param<std::string>("params/end_effector", end_effector, robot_name + "_link_ee");

        _controller = std::make_unique<ImpedanceControl>(urdf_string, end_effector, _dt);

        // Load impedance gains
        std::vector<double> kp_vec, kd_vec;
        while (!_n.getParam("control/impedance_Kp", kp_vec)) {
            ROS_INFO("Waiting for parameter impedance_Kp");
        }
        while (!_n.getParam("control/impedance_Kd", kd_vec)) {
            ROS_INFO("Waiting for parameter impedance_Kd");
        }
        Eigen::VectorXd Kp(7), Kd(7);
        for (int i = 0; i < 7; i++) { Kp[i] = kp_vec[i]; Kd[i] = kd_vec[i]; }
        _controller->set_impedance_gains(Kp, Kd);

        // Load DS position gain
        double ds_gain_pos;
        while (!_n.getParam("control/dsGainPos", ds_gain_pos)) {
            ROS_INFO("Waiting for parameter dsGainPos");
        }
        _controller->set_pos_gain(ds_gain_pos);

        // Load dt scale
        double dt_scale;
        while (!_n.getParam("control/dt_scale", dt_scale)) {
            ROS_INFO("Waiting for parameter dt_scale");
        }
        _controller->set_dt_scale(dt_scale);

        // Load target pose and set as initial desired pose
        std::vector<double> dpos, dquat;
        while (!_n.getParam("target/pos",  dpos))  { ROS_INFO("Waiting for parameter target/pos"); }
        while (!_n.getParam("target/quat", dquat)) { ROS_INFO("Waiting for parameter target/quat"); }
        Eigen::Vector3d des_pos;
        Eigen::Vector4d des_quat;
        for (size_t i = 0; i < 3; i++) des_pos(i)  = dpos[i];
        for (size_t i = 0; i < 4; i++) des_quat(i) = dquat[i];
        _controller->set_desired_pose(des_pos, des_quat);

        // Plot publisher
        _plotPublisher = _n.advertise<std_msgs::Float64MultiArray>(ns + "/plotvar", 1);

        // Dynamic reconfigure
        _dynRecCallback = boost::bind(&IiwaRosMaster::param_cfg_callback, this, _1, _2);
        _dynRecServer.setCallback(_dynRecCallback);

        return true;
    }

    void run()
    {
        while (!_stop && ros::ok()) {
            _mutex.lock();
                _controller->updateRobot(
                    _feedback.jnt_position,
                    _feedback.jnt_velocity,
                    _feedback.jnt_torque);
                publishCommandTorque(_controller->getCmd());
                publishPlotVariable(command_plt);
                publishEEInfo();
            _mutex.unlock();

            ros::spinOnce();
            _loopRate.sleep();
        }
        publishCommandTorque(Eigen::VectorXd::Zero(No_JOINTS));
        ros::spinOnce();
        _loopRate.sleep();
        ros::shutdown();
    }

  protected:
    double  _dt;
    Options _options;

    ros::NodeHandle _n;
    ros::Rate       _loopRate;

    ros::Subscriber _subRobotStates[No_Robots];
    ros::Subscriber _subControl;

    ros::Publisher _TrqCmdPublisher;
    ros::Publisher _EEPosePublisher;
    ros::Publisher _EEVelPublisher;
    ros::Publisher _MeasurePublisher;
    ros::Publisher _plotPublisher;

    dynamic_reconfigure::Server<iiwa_toolkit::passive_cfg_paramsConfig> _dynRecServer;
    dynamic_reconfigure::Server<iiwa_toolkit::passive_cfg_paramsConfig>::CallbackType _dynRecCallback;

    feedback _feedback;

    Eigen::VectorXd command_trq = Eigen::VectorXd(No_JOINTS);
    Eigen::VectorXd command_plt = Eigen::VectorXd(3);

    std::unique_ptr<ImpedanceControl> _controller;

    bool _stop;
    std::mutex _mutex;

  private:

    void updateRobotStates(const sensor_msgs::JointState::ConstPtr& msg, int /*k*/)
    {
        for (int i = 0; i < No_JOINTS; i++) {
            _feedback.jnt_position[i] = (double)msg->position[i];
            _feedback.jnt_velocity[i] = (double)msg->velocity[i];
            _feedback.jnt_torque[i]   = (double)msg->effort[i];
        }
    }

    void publishCommandTorque(const Eigen::VectorXd& cmdTrq)
    {
        std_msgs::Float64MultiArray _cmd_jnt_torque;
        _cmd_jnt_torque.data.resize(No_JOINTS);
        if (cmdTrq.size() == No_JOINTS) {
            for (int i = 0; i < No_JOINTS; i++)
                _cmd_jnt_torque.data[i] = cmdTrq[i];
            _TrqCmdPublisher.publish(_cmd_jnt_torque);
        }
    }

    void publishPlotVariable(const Eigen::VectorXd& pltVar)
    {
        std_msgs::Float64MultiArray _plotVar;
        _plotVar.data.resize(pltVar.size());
        for (size_t i = 0; i < (size_t)pltVar.size(); i++)
            _plotVar.data[i] = pltVar[i];
        _plotPublisher.publish(_plotVar);
    }

    void publishEEInfo()
    {
        geometry_msgs::Pose  msg1;
        geometry_msgs::Twist msg2;
        geometry_msgs::Twist msg3;

        Eigen::Vector3d pos    = _controller->getEEpos();
        Eigen::Vector4d quat   = _controller->getEEquat();
        Eigen::Vector3d vel    = _controller->getEEVel();
        Eigen::Vector3d angVel = _controller->getEEAngVel();

        msg1.position.x    = pos[0];  msg1.position.y    = pos[1];  msg1.position.z    = pos[2];
        msg1.orientation.w = quat[0]; msg1.orientation.x = quat[1]; msg1.orientation.y = quat[2]; msg1.orientation.z = quat[3];

        msg2.linear.x  = vel[0];    msg2.linear.y  = vel[1];    msg2.linear.z  = vel[2];
        msg2.angular.x = angVel[0]; msg2.angular.y = angVel[1]; msg2.angular.z = angVel[2];

        double measure = _controller->getMeasure();
        msg3.linear.x  = measure;   msg3.linear.y  = vel[1];    msg3.linear.z  = vel[2];
        msg3.angular.x = angVel[0]; msg3.angular.y = angVel[1]; msg3.angular.z = angVel[2];

        _EEPosePublisher.publish(msg1);
        _EEVelPublisher.publish(msg2);
        _MeasurePublisher.publish(msg3);
    }

    void updateControlVel(const geometry_msgs::Pose::ConstPtr& msg)
    {
        Eigen::Vector3d vel;
        Eigen::Vector4d quat;
        vel  << (double)msg->position.x, (double)msg->position.y, (double)msg->position.z;
        quat << (double)msg->orientation.w, (double)msg->orientation.x,
                (double)msg->orientation.y, (double)msg->orientation.z;
        if (vel.norm() < 1.) {
            _controller->set_desired_velocity(vel);
            if ((quat.norm() > 0) && (quat.norm() < 1.1)) {
                quat.normalize();
                _controller->set_desired_quat(quat);
            }
        } else {
            ROS_WARN("VELOCITY OUT OF BOUND");
        }
    }

    void param_cfg_callback(iiwa_toolkit::passive_cfg_paramsConfig& config, uint32_t /*level*/)
    {
        ROS_INFO("Reconfigure request received (impedance_track). Dynamic gain scaling not applied.");
        (void)config;
    }
};

//****************************************************
int main(int argc, char** argv)
{
    float frequency = 1000.0f;
    ros::init(argc, argv, "iiwa_impedance_track");
    ros::NodeHandle n;

    Options options;
    while (!n.getParam("options/filter_gain", options.filter_gain)) {
        ROS_INFO("Waiting for the option setting");
    }

    std::unique_ptr<IiwaRosMaster> IiwaTrack = std::make_unique<IiwaRosMaster>(n, frequency, options);

    if (!IiwaTrack->init()) {
        return -1;
    } else {
        IiwaTrack->run();
    }
    return 0;
}
