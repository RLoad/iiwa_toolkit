#include <mutex>
#include <memory>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

#include <Eigen/Dense>

#include <iiwa_tools/iiwa_tools.h>
#include "thirdparty/Utils.h"

namespace {
constexpr int kNumJoints = 7;
}

class VelocityIKTrackNode {
 public:
  VelocityIKTrackNode(ros::NodeHandle& nh, const double frequency_hz)
      : nh_(nh),
        loop_rate_(frequency_hz),
        dt_(1.0 / frequency_hz),
        q_(Eigen::VectorXd::Zero(kNumJoints)),
        qd_(Eigen::VectorXd::Zero(kNumJoints)),
        desired_linear_vel_(Eigen::Vector3d::Zero()),
        desired_quat_(Eigen::Vector4d::Zero()),
        qdot_cmd_(Eigen::VectorXd::Zero(kNumJoints)),
        q_cmd_(Eigen::VectorXd::Zero(kNumJoints)) {}

  bool init() {
    std::string ns = nh_.getNamespace();
    if (ns.empty() || ns == "/") ns = "/iiwa";
    if (ns[0] != '/') ns = "/" + ns;
    ns_ = ns;

    std::string urdf_string;
    std::string full_param;
    const std::string robot_description = ns_ + "/robot_description";
    if (!nh_.searchParam(robot_description, full_param)) {
      ROS_ERROR_STREAM("Could not find parameter: " << robot_description);
      return false;
    }
    while (ros::ok() && urdf_string.empty()) {
      nh_.getParam(full_param, urdf_string);
      if (urdf_string.empty()) {
        ROS_INFO_THROTTLE(1.0, "Waiting for robot URDF...");
        ros::Duration(0.1).sleep();
      }
    }

    std::string end_effector;
    nh_.param<std::string>("params/end_effector", end_effector, "iiwa_link_ee");
    nh_.param<double>("velocity_ik/orientation_gain", orientation_gain_, 4.0);
    nh_.param<double>("velocity_ik/dls_lambda", dls_lambda_, 0.03);
    nh_.param<double>("velocity_ik/max_joint_velocity", max_joint_velocity_, 0.6);
    nh_.param<bool>("velocity_ik/publish_position_command", publish_position_command_, true);

    tools_.init_rbdyn(urdf_string, end_effector);

    sub_joint_states_ = nh_.subscribe<sensor_msgs::JointState>(
        ns_ + "/joint_states", 1, &VelocityIKTrackNode::jointStateCb, this,
        ros::TransportHints().reliable().tcpNoDelay());
    sub_vel_quat_ = nh_.subscribe<geometry_msgs::Pose>(
        "/passive_control/vel_quat", 1, &VelocityIKTrackNode::velQuatCb, this,
        ros::TransportHints().reliable().tcpNoDelay());

    pub_joint_velocity_cmd_ =
        nh_.advertise<std_msgs::Float64MultiArray>(ns_ + "/joint_velocity_cmd", 1);
    pub_ee_pose_ = nh_.advertise<geometry_msgs::Pose>(ns_ + "/ee_info/Pose", 1);
    pub_ee_vel_ = nh_.advertise<geometry_msgs::Twist>(ns_ + "/ee_info/Vel", 1);
    if (publish_position_command_) {
      pub_position_cmd_ =
          nh_.advertise<std_msgs::Float64MultiArray>(ns_ + "/PositionController/command", 1);
    }

    ROS_INFO_STREAM("[velocity_ik_track] namespace: " << ns_
                    << ", publish_position_command=" << publish_position_command_);
    return true;
  }

  void run() {
    while (ros::ok()) {
      step();
      ros::spinOnce();
      loop_rate_.sleep();
    }
  }

 private:
  void jointStateCb(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() < kNumJoints || msg->velocity.size() < kNumJoints) return;
    std::lock_guard<std::mutex> lock(mutex_);
    for (int i = 0; i < kNumJoints; ++i) {
      q_(i) = msg->position[i];
      qd_(i) = msg->velocity[i];
    }
    if (!q_cmd_initialized_) {
      q_cmd_ = q_;
      q_cmd_initialized_ = true;
    }
    has_joint_state_ = true;
  }

  void velQuatCb(const geometry_msgs::Pose::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    desired_linear_vel_ << msg->position.x, msg->position.y, msg->position.z;
    desired_quat_ << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
    if (desired_quat_.norm() > 1e-8) desired_quat_.normalize();
    has_command_ = true;
  }

  void publishEeInfo(const Eigen::Vector3d& p, const Eigen::Vector4d& q_wxyz,
                     const Eigen::Vector3d& v, const Eigen::Vector3d& w) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = p.x();
    pose_msg.position.y = p.y();
    pose_msg.position.z = p.z();
    pose_msg.orientation.w = q_wxyz(0);
    pose_msg.orientation.x = q_wxyz(1);
    pose_msg.orientation.y = q_wxyz(2);
    pose_msg.orientation.z = q_wxyz(3);
    pub_ee_pose_.publish(pose_msg);

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = v.x();
    vel_msg.linear.y = v.y();
    vel_msg.linear.z = v.z();
    vel_msg.angular.x = w.x();
    vel_msg.angular.y = w.y();
    vel_msg.angular.z = w.z();
    pub_ee_vel_.publish(vel_msg);
  }

  void step() {
    Eigen::VectorXd q_local(kNumJoints), qd_local(kNumJoints), q_cmd_local(kNumJoints);
    Eigen::Vector3d v_des;
    Eigen::Vector4d q_des_wxyz;
    bool has_js = false;
    bool has_cmd = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      q_local = q_;
      qd_local = qd_;
      q_cmd_local = q_cmd_;
      v_des = desired_linear_vel_;
      q_des_wxyz = desired_quat_;
      has_js = has_joint_state_;
      has_cmd = has_command_;
    }
    if (!has_js) return;

    iiwa_tools::RobotState robot_state;
    robot_state.position.resize(kNumJoints);
    robot_state.velocity.resize(kNumJoints);
    for (int i = 0; i < kNumJoints; ++i) {
      robot_state.position[i] = q_local(i);
      robot_state.velocity[i] = qd_local(i);
    }

    Eigen::MatrixXd J(6, kNumJoints), Jdot(6, kNumJoints);
    std::tie(J, Jdot) = tools_.jacobians(robot_state);
    const auto ee_state = tools_.perform_fk(robot_state);

    Eigen::Vector4d q_cur_wxyz;
    q_cur_wxyz << ee_state.orientation.w(), ee_state.orientation.x(), ee_state.orientation.y(),
        ee_state.orientation.z();

    Eigen::VectorXd ee_twist = J * qd_local;
    Eigen::Vector3d w_cur = ee_twist.head(3);
    Eigen::Vector3d v_cur = ee_twist.tail(3);

    // Desired angular velocity from quaternion error.
    Eigen::Vector3d w_des = Eigen::Vector3d::Zero();
    if (has_cmd && q_des_wxyz.norm() > 1e-6) {
      Eigen::Vector4d q_conj = q_cur_wxyz;
      q_conj.segment(1, 3) = -q_conj.segment(1, 3);
      Eigen::Vector4d q_err = Utils<double>::quaternionProduct(q_des_wxyz, q_conj);
      if (q_err(0) < 0.0) q_err *= -1.0;
      w_des = 2.0 * orientation_gain_ * q_err.segment(1, 3);
    }

    Eigen::VectorXd twist_des = Eigen::VectorXd::Zero(6);  // [angular; linear]
    if (has_cmd) {
      twist_des.head(3) = w_des;
      twist_des.tail(3) = v_des;
    }

    const Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6, 6);
    const Eigen::MatrixXd damped_inv =
        J.transpose() * (J * J.transpose() + dls_lambda_ * dls_lambda_ * I6).inverse();
    Eigen::VectorXd qdot_cmd = damped_inv * twist_des;

    const double qdot_norm = qdot_cmd.norm();
    if (qdot_norm > max_joint_velocity_ && qdot_norm > 1e-9) {
      qdot_cmd *= (max_joint_velocity_ / qdot_norm);
    }

    std_msgs::Float64MultiArray qdot_msg;
    qdot_msg.data.resize(kNumJoints);
    for (int i = 0; i < kNumJoints; ++i) qdot_msg.data[i] = qdot_cmd(i);
    pub_joint_velocity_cmd_.publish(qdot_msg);

    if (publish_position_command_) {
      q_cmd_local += qdot_cmd * dt_;
      std_msgs::Float64MultiArray q_cmd_msg;
      q_cmd_msg.data.resize(kNumJoints);
      for (int i = 0; i < kNumJoints; ++i) q_cmd_msg.data[i] = q_cmd_local(i);
      pub_position_cmd_.publish(q_cmd_msg);

      std::lock_guard<std::mutex> lock(mutex_);
      q_cmd_ = q_cmd_local;
    }

    publishEeInfo(ee_state.translation, q_cur_wxyz, v_cur, w_cur);
  }

 private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;
  double dt_;

  std::string ns_;
  iiwa_tools::IiwaTools tools_;

  ros::Subscriber sub_joint_states_;
  ros::Subscriber sub_vel_quat_;
  ros::Publisher pub_joint_velocity_cmd_;
  ros::Publisher pub_position_cmd_;
  ros::Publisher pub_ee_pose_;
  ros::Publisher pub_ee_vel_;

  std::mutex mutex_;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::Vector3d desired_linear_vel_;
  Eigen::Vector4d desired_quat_;
  Eigen::VectorXd qdot_cmd_;
  Eigen::VectorXd q_cmd_;

  bool has_joint_state_ = false;
  bool has_command_ = false;
  bool q_cmd_initialized_ = false;

  double orientation_gain_ = 4.0;
  double dls_lambda_ = 0.03;
  double max_joint_velocity_ = 0.6;
  bool publish_position_command_ = true;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "iiwa_velocity_ik_track");
  ros::NodeHandle nh;
  double frequency = 200.0;
  nh.param("velocity_ik/frequency", frequency, 200.0);

  VelocityIKTrackNode node(nh, frequency);
  if (!node.init()) return -1;
  node.run();
  return 0;
}

