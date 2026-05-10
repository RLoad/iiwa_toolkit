//|
//| Kinematic Cartesian controller using iiwa_ros PositionController.
//| - If latest input is pose: track Cartesian pose with proportional law.
//| - If latest input is twist: integrate twist through Jacobian pseudo-inverse.
//| - If latest input is vel_quat (ds_motion_generator filtered output, a Pose where
//|   .position is desired linear velocity and .orientation is target quaternion):
//|   feed linear velocity directly and derive angular velocity from quaternion error.
//|

#include <algorithm>
#include <memory>
#include <mutex>
#include <tuple>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64MultiArray.h"

#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iiwa_tools/iiwa_tools.h>

#define No_JOINTS 7

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudo_inverse(const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-5})
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto& singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance) {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

enum InputMode {
    INPUT_NONE = 0,
    INPUT_POSE = 1,
    INPUT_TWIST = 2,
    // ds_motion_generator's /passive_control/vel_quat: geometry_msgs::Pose where
    // .position is desired linear velocity (m/s) and .orientation is target quaternion.
    INPUT_VEL_QUAT = 3
};

class PureVelocityKinematicNode {
public:
    PureVelocityKinematicNode(ros::NodeHandle& n, double frequency) : n_(n), dt_(1.0 / frequency), loop_rate_(frequency) {}

    bool init()
    {
        std::string ns = n_.getNamespace();
        if (ns.empty() || ns == "/")
            ns = "/iiwa";

        std::string robot_name = ns;
        if (!robot_name.empty() && robot_name[0] == '/')
            robot_name = robot_name.substr(1);

        sub_joint_state_ = n_.subscribe<sensor_msgs::JointState>(
            ns + "/joint_states", 1, &PureVelocityKinematicNode::onJointState, this,
            ros::TransportHints().reliable().tcpNoDelay());

        std::string cmd_pose_topic;
        std::string cmd_twist_topic;
        std::string cmd_vel_quat_topic;
        std::string cmd_pos_quat_topic;
        n_.param<std::string>("topics/cmd_pose", cmd_pose_topic, std::string("/pure_kinematic/cmd_pose"));
        n_.param<std::string>("topics/cmd_twist", cmd_twist_topic, std::string("/pure_kinematic/cmd_twist"));
        // DS-compatible inputs from ds_motion_generator passive-track convention.
        n_.param<std::string>("topics/cmd_vel_quat", cmd_vel_quat_topic, std::string("/passive_control/vel_quat"));
        n_.param<std::string>("topics/cmd_pos_quat", cmd_pos_quat_topic, std::string("/passive_control/pos_quat"));
        sub_cmd_pose_ = n_.subscribe<geometry_msgs::Pose>(cmd_pose_topic, 1, &PureVelocityKinematicNode::onPoseCmd, this);
        sub_cmd_twist_ = n_.subscribe<geometry_msgs::Twist>(cmd_twist_topic, 1, &PureVelocityKinematicNode::onTwistCmd, this);
        sub_cmd_vel_quat_ = n_.subscribe<geometry_msgs::Pose>(cmd_vel_quat_topic, 1, &PureVelocityKinematicNode::onVelQuatCmd, this);
        sub_cmd_pos_quat_ = n_.subscribe<geometry_msgs::Pose>(cmd_pos_quat_topic, 1, &PureVelocityKinematicNode::onPosQuatCmd, this);

        pub_joint_pos_cmd_ = n_.advertise<std_msgs::Float64MultiArray>(ns + "/PositionController/command", 1);
        pub_ee_pose_ = n_.advertise<geometry_msgs::Pose>(ns + "/ee_info/Pose", 1);
        pub_ee_vel_ = n_.advertise<geometry_msgs::Twist>(ns + "/ee_info/Vel", 1);

        n_.param("control/kp_pos", kp_pos_, 2.5);
        n_.param("control/kp_ori", kp_ori_, 2.0);
        n_.param("control/max_lin_speed", max_lin_speed_, 0.25);
        n_.param("control/max_ang_speed", max_ang_speed_, 0.7);
        n_.param("control/max_qdot", max_qdot_, 0.8);
        n_.param("control/command_timeout", command_timeout_, 0.5);
        n_.param("control/max_step_per_cycle", max_step_per_cycle_, 0.02);
        n_.param("control/nullspace_gain", nullspace_gain_, 1.5);
        n_.param("control/nullspace_damping", nullspace_damping_, 0.1);
        n_.param("control/nullspace_max_qdot", nullspace_max_qdot_, 0.4);

        // OFF by default: when the velocity stream stops, the controller holds the
        // last commanded joint position instead of snapping back to the YAML target.
        // Set to true to restore the legacy snap-on-boot behavior.
        n_.param("behavior/use_startup_target", use_startup_target_, false);

        // Keep null-space posture fixed to match passive_control.cpp behavior.
        q_null_ << 0.0, 0.0, 0.0, -0.75, 0.0, 0.0, 0.0;

        // Optional startup target: only auto-activated if behavior/use_startup_target is true.
        std::vector<double> target_pos;
        std::vector<double> target_quat;
        if (n_.getParam("target/pos", target_pos) && target_pos.size() == 3) {
            pose_cmd_.position.x = target_pos[0];
            pose_cmd_.position.y = target_pos[1];
            pose_cmd_.position.z = target_pos[2];
            has_startup_pose_target_ = true;
        }
        if (n_.getParam("target/quat", target_quat) && target_quat.size() == 4) {
            // Keep the same convention as existing toolkit YAMLs: [w, x, y, z].
            pose_cmd_.orientation.w = target_quat[0];
            pose_cmd_.orientation.x = target_quat[1];
            pose_cmd_.orientation.y = target_quat[2];
            pose_cmd_.orientation.z = target_quat[3];
            has_startup_pose_target_ = true;
        }

        std::string urdf_string, full_param;
        std::string robot_description = ns + "/robot_description";
        std::string end_effector;
        if (!n_.searchParam(robot_description, full_param)) {
            ROS_ERROR("Could not find parameter %s on parameter server", robot_description.c_str());
            return false;
        }
        while (urdf_string.empty() && ros::ok()) {
            n_.getParam(full_param, urdf_string);
            usleep(100000);
        }
        n_.param<std::string>("params/end_effector", end_effector, robot_name + "_link_ee");
        tools_.init_rbdyn(urdf_string, end_effector);
        ROS_INFO("pure_velocity_track_kinematic initialized on namespace %s", ns.c_str());
        return true;
    }

    void run()
    {
        while (ros::ok()) {
            step();
            ros::spinOnce();
            loop_rate_.sleep();
        }
    }

private:
    void onJointState(const sensor_msgs::JointState::ConstPtr& msg)
    {
        if (msg->position.size() < No_JOINTS || msg->velocity.size() < No_JOINTS || msg->effort.size() < No_JOINTS)
            return;
        std::lock_guard<std::mutex> lock(mtx_);
        for (int i = 0; i < No_JOINTS; ++i) {
            q_(i) = msg->position[i];
            dq_(i) = msg->velocity[i];
            tau_(i) = msg->effort[i];
        }
        has_joint_state_ = true;
        if (!q_cmd_initialized_) {
            q_cmd_ = q_;
            q_cmd_initialized_ = true;
            if (has_startup_pose_target_ && use_startup_target_) {
                last_pose_cmd_time_ = ros::Time::now().toSec();
                has_pose_target_ = true;
                last_input_mode_ = INPUT_POSE;
                ROS_INFO("pure_velocity_track_kinematic: using startup target/pose from params");
            } else {
                ROS_INFO("pure_velocity_track_kinematic: holding initial joint pose; "
                         "no startup target (behavior/use_startup_target=false)");
            }
        }
    }

    void onPoseCmd(const geometry_msgs::Pose::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        pose_cmd_.position.x = msg->position.x;
        pose_cmd_.position.y = msg->position.y;
        pose_cmd_.position.z = msg->position.z;
        pose_cmd_.orientation = msg->orientation;
        last_pose_cmd_time_ = ros::Time::now().toSec();
        has_pose_target_ = true;
        last_input_mode_ = INPUT_POSE;
        explicit_pose_cmd_received_ = true;
    }

    void onTwistCmd(const geometry_msgs::Twist::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        twist_cmd_ = *msg;
        last_twist_cmd_time_ = ros::Time::now().toSec();
        last_input_mode_ = INPUT_TWIST;
        velocity_cmd_received_ = true;
    }

    // ds_motion_generator filtered output: .position is linear velocity, .orientation is
    // absolute target quaternion. Matches passive_track / damping_track semantics.
    void onVelQuatCmd(const geometry_msgs::Pose::ConstPtr& msg)
    {
        const Eigen::Vector3d v(msg->position.x, msg->position.y, msg->position.z);
        if (v.norm() >= 1.0) {
            ROS_WARN_THROTTLE(1.0, "pure_velocity_track_kinematic: vel_quat |v|=%.3f out of bound, ignored", v.norm());
            return;
        }
        const Eigen::Vector4d q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        std::lock_guard<std::mutex> lock(mtx_);
        lin_vel_cmd_ = v;
        if (q.norm() > 1e-6 && q.norm() < 1.1) {
            Eigen::Quaterniond qd(q(0), q(1), q(2), q(3));
            qd.normalize();
            ori_target_ = qd;
            has_ori_target_ = true;
        }
        last_vel_quat_cmd_time_ = ros::Time::now().toSec();
        last_input_mode_ = INPUT_VEL_QUAT;
        velocity_cmd_received_ = true;
    }

    // /passive_control/pos_quat: full Pose target (used as INPUT_POSE).
    void onPosQuatCmd(const geometry_msgs::Pose::ConstPtr& msg)
    {
        const Eigen::Vector3d p(msg->position.x, msg->position.y, msg->position.z);
        if (p.norm() <= 0.0 || p.norm() >= 1.5) {
            ROS_WARN_THROTTLE(1.0, "pure_velocity_track_kinematic: pos_quat |p|=%.3f out of bound, ignored", p.norm());
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        pose_cmd_.position.x = p.x();
        pose_cmd_.position.y = p.y();
        pose_cmd_.position.z = p.z();
        const Eigen::Vector4d q(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (q.norm() > 1e-6 && q.norm() < 1.1) {
            pose_cmd_.orientation.w = q(0);
            pose_cmd_.orientation.x = q(1);
            pose_cmd_.orientation.y = q(2);
            pose_cmd_.orientation.z = q(3);
        }
        last_pose_cmd_time_ = ros::Time::now().toSec();
        has_pose_target_ = true;
        last_input_mode_ = INPUT_POSE;
        explicit_pose_cmd_received_ = true;
    }

    Eigen::Vector3d quatErrorOmega(const Eigen::Quaterniond& q_des, const Eigen::Quaterniond& q_cur) const
    {
        Eigen::Quaterniond qe = q_des * q_cur.conjugate();
        if (qe.w() < 0.0) {
            qe.coeffs() *= -1.0;
        }
        return 2.0 * qe.vec();
    }

    void step()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_joint_state_ || !q_cmd_initialized_)
            return;

        const double now = ros::Time::now().toSec();
        InputMode active_mode = INPUT_NONE;
        if (last_input_mode_ == INPUT_VEL_QUAT && (now - last_vel_quat_cmd_time_) < command_timeout_)
            active_mode = INPUT_VEL_QUAT;
        else if (last_input_mode_ == INPUT_TWIST && (now - last_twist_cmd_time_) < command_timeout_)
            active_mode = INPUT_TWIST;
        else if (has_pose_target_) {
            // Only fall back to POSE if the user actually sent an explicit pose command
            // (onPoseCmd / onPosQuatCmd), or if no velocity command has ever arrived.
            // After velocity input, the YAML startup target is permanently consumed and
            // the controller HOLDs the last commanded joint position on timeout, instead
            // of snapping back to it.
            if (explicit_pose_cmd_received_ || !velocity_cmd_received_)
                active_mode = INPUT_POSE;
        }

        if (active_mode == INPUT_NONE)
            return;

        iiwa_tools::RobotState st;
        st.position.resize(No_JOINTS);
        st.velocity.resize(No_JOINTS);
        for (int i = 0; i < No_JOINTS; ++i) {
            st.position[i] = q_(i);
            st.velocity[i] = dq_(i);
        }

        Eigen::MatrixXd J, Jdot;
        std::tie(J, Jdot) = tools_.jacobians(st);
        (void)Jdot;
        const auto ee_state = tools_.perform_fk(st);
        const Eigen::Vector3d p_cur = ee_state.translation.cast<double>();
        const Eigen::Quaterniond q_cur(ee_state.orientation.w(), ee_state.orientation.x(), ee_state.orientation.y(),
                                       ee_state.orientation.z());
        const Eigen::VectorXd xdot_cur = J * dq_;
        const Eigen::Vector3d w_cur = xdot_cur.head(3);
        const Eigen::Vector3d v_cur = xdot_cur.tail(3);

        geometry_msgs::Pose ee_pose_msg;
        ee_pose_msg.position.x = p_cur.x();
        ee_pose_msg.position.y = p_cur.y();
        ee_pose_msg.position.z = p_cur.z();
        ee_pose_msg.orientation.w = q_cur.w();
        ee_pose_msg.orientation.x = q_cur.x();
        ee_pose_msg.orientation.y = q_cur.y();
        ee_pose_msg.orientation.z = q_cur.z();
        pub_ee_pose_.publish(ee_pose_msg);

        geometry_msgs::Twist ee_vel_msg;
        ee_vel_msg.linear.x = v_cur.x();
        ee_vel_msg.linear.y = v_cur.y();
        ee_vel_msg.linear.z = v_cur.z();
        ee_vel_msg.angular.x = w_cur.x();
        ee_vel_msg.angular.y = w_cur.y();
        ee_vel_msg.angular.z = w_cur.z();
        pub_ee_vel_.publish(ee_vel_msg);

        Eigen::Vector3d p_target = p_cur;
        Eigen::Quaterniond q_target = q_cur;

        Eigen::VectorXd xdot_des(6);
        xdot_des.setZero();
        if (active_mode == INPUT_POSE) {
            p_target = Eigen::Vector3d(pose_cmd_.position.x, pose_cmd_.position.y, pose_cmd_.position.z);
            q_target = Eigen::Quaterniond(pose_cmd_.orientation.w, pose_cmd_.orientation.x, pose_cmd_.orientation.y,
                                          pose_cmd_.orientation.z);
            if (q_target.norm() < 1e-6)
                q_target = q_cur;
            q_target.normalize();

            Eigen::Vector3d v = kp_pos_ * (p_target - p_cur);
            if (v.norm() > max_lin_speed_)
                v = v.normalized() * max_lin_speed_;
            Eigen::Vector3d w = kp_ori_ * quatErrorOmega(q_target, q_cur);
            if (w.norm() > max_ang_speed_)
                w = w.normalized() * max_ang_speed_;
            xdot_des.head(3) = w;
            xdot_des.tail(3) = v;
        } else if (active_mode == INPUT_VEL_QUAT) {
            // ds_motion_generator passive-track convention: lin velocity from .position,
            // absolute target orientation from .orientation. Angular velocity is derived
            // from quaternion error so we converge to the target attitude.
            Eigen::Vector3d v = lin_vel_cmd_;
            if (v.norm() > max_lin_speed_)
                v = v.normalized() * max_lin_speed_;
            Eigen::Vector3d w = Eigen::Vector3d::Zero();
            if (has_ori_target_) {
                q_target = ori_target_;
                w = kp_ori_ * quatErrorOmega(q_target, q_cur);
                if (w.norm() > max_ang_speed_)
                    w = w.normalized() * max_ang_speed_;
            }
            xdot_des.head(3) = w;
            xdot_des.tail(3) = v;
            p_target = p_cur + v * dt_;
        } else {
            Eigen::Vector3d v(twist_cmd_.linear.x, twist_cmd_.linear.y, twist_cmd_.linear.z);
            Eigen::Vector3d w(twist_cmd_.angular.x, twist_cmd_.angular.y, twist_cmd_.angular.z);
            if (v.norm() > max_lin_speed_)
                v = v.normalized() * max_lin_speed_;
            if (w.norm() > max_ang_speed_)
                w = w.normalized() * max_ang_speed_;
            xdot_des.head(3) = w;
            xdot_des.tail(3) = v;

            // Twist mode has no absolute pose command; integrate one cycle for logging purposes.
            p_target = p_cur + v * dt_;
            const double angle = w.norm() * dt_;
            if (angle > 1e-9) {
                const Eigen::AngleAxisd aa(angle, w.normalized());
                q_target = (q_cur * Eigen::Quaterniond(aa)).normalized();
            }
        }

        const Eigen::MatrixXd J_pinv = pseudo_inverse(J);
        Eigen::VectorXd qdot_cmd = J_pinv * xdot_des;

        // Null-space posture stabilization: qdot += (I - J+J) * (K(q0-q) - D dq)
        const Eigen::MatrixXd N = Eigen::MatrixXd::Identity(No_JOINTS, No_JOINTS) - J_pinv * J;
        Eigen::VectorXd qdot_null = nullspace_gain_ * (q_null_ - q_) - nullspace_damping_ * dq_;
        for (int i = 0; i < No_JOINTS; ++i) {
            qdot_null(i) = std::max(-nullspace_max_qdot_, std::min(nullspace_max_qdot_, qdot_null(i)));
        }
        qdot_cmd += N * qdot_null;

        for (int i = 0; i < No_JOINTS; ++i) {
            qdot_cmd(i) = std::max(-max_qdot_, std::min(max_qdot_, qdot_cmd(i)));
        }

        Eigen::VectorXd q_next = q_cmd_ + qdot_cmd * dt_;
        for (int i = 0; i < No_JOINTS; ++i) {
            const double step = q_next(i) - q_(i);
            const double step_clamped = std::max(-max_step_per_cycle_, std::min(max_step_per_cycle_, step));
            q_next(i) = q_(i) + step_clamped;
        }
        q_cmd_ = q_next;

        iiwa_tools::RobotState st_cmd;
        st_cmd.position.resize(No_JOINTS);
        st_cmd.velocity.resize(No_JOINTS);
        for (int i = 0; i < No_JOINTS; ++i) {
            st_cmd.position[i] = q_cmd_(i);
            st_cmd.velocity[i] = 0.0;
        }
        const auto ee_cmd_state = tools_.perform_fk(st_cmd);
        const Eigen::Vector3d p_cmd = ee_cmd_state.translation.cast<double>();
        const Eigen::Quaterniond q_cmd_cart(ee_cmd_state.orientation.w(), ee_cmd_state.orientation.x(),
                                            ee_cmd_state.orientation.y(), ee_cmd_state.orientation.z());

        ROS_INFO_STREAM_THROTTLE(1.0, "pure_velocity_track_kinematic\n"
                                           << "  cartesian command sent: p=[" << p_cmd.transpose() << "], q=["
                                           << q_cmd_cart.w() << ", " << q_cmd_cart.x() << ", " << q_cmd_cart.y() << ", "
                                           << q_cmd_cart.z() << "]\n"
                                           << "  cartesian target      : p=[" << p_target.transpose() << "], q=["
                                           << q_target.w() << ", " << q_target.x() << ", " << q_target.y() << ", "
                                           << q_target.z() << "]");

        std_msgs::Float64MultiArray cmd_msg;
        cmd_msg.data.resize(No_JOINTS);
        for (int i = 0; i < No_JOINTS; ++i)
            cmd_msg.data[i] = q_cmd_(i);
        pub_joint_pos_cmd_.publish(cmd_msg);
    }

private:
    ros::NodeHandle n_;
    double dt_;
    ros::Rate loop_rate_;
    std::mutex mtx_;

    ros::Subscriber sub_joint_state_;
    ros::Subscriber sub_cmd_pose_;
    ros::Subscriber sub_cmd_twist_;
    ros::Subscriber sub_cmd_vel_quat_;
    ros::Subscriber sub_cmd_pos_quat_;
    ros::Publisher pub_joint_pos_cmd_;
    ros::Publisher pub_ee_pose_;
    ros::Publisher pub_ee_vel_;

    iiwa_tools::IiwaTools tools_;
    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(No_JOINTS);
    Eigen::VectorXd dq_ = Eigen::VectorXd::Zero(No_JOINTS);
    Eigen::VectorXd tau_ = Eigen::VectorXd::Zero(No_JOINTS);
    Eigen::VectorXd q_cmd_ = Eigen::VectorXd::Zero(No_JOINTS);

    geometry_msgs::Pose pose_cmd_;
    geometry_msgs::Twist twist_cmd_;
    Eigen::Vector3d lin_vel_cmd_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond ori_target_ = Eigen::Quaterniond::Identity();

    bool has_joint_state_ = false;
    bool q_cmd_initialized_ = false;
    bool has_pose_target_ = false;
    bool has_ori_target_ = false;
    double last_pose_cmd_time_ = -1.0;
    double last_twist_cmd_time_ = -1.0;
    double last_vel_quat_cmd_time_ = -1.0;
    InputMode last_input_mode_ = INPUT_NONE;
    bool has_startup_pose_target_ = false;
    bool use_startup_target_ = false;
    bool velocity_cmd_received_ = false;
    bool explicit_pose_cmd_received_ = false;

    double kp_pos_ = 2.5;
    double kp_ori_ = 2.0;
    double max_lin_speed_ = 0.25;
    double max_ang_speed_ = 0.7;
    double max_qdot_ = 0.8;
    double command_timeout_ = 0.5;
    double max_step_per_cycle_ = 0.02;
    Eigen::VectorXd q_null_ = Eigen::VectorXd::Zero(No_JOINTS);
    double nullspace_gain_ = 1.5;
    double nullspace_damping_ = 0.1;
    double nullspace_max_qdot_ = 0.4;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pure_velocity_track_kinematic");
    ros::NodeHandle n;
    PureVelocityKinematicNode node(n, 200.0);
    if (!node.init())
        return -1;
    node.run();
    return 0;
}
