#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import sys

class Joint_optimal:
    def __init__(self):
        # Node initialization
        rospy.init_node('Joint_optimal', anonymous=True)

        # Subscriber: listens to /passive_control/pos_quat
        self.sub_control = rospy.Subscriber(
            "/passive_control/pos_quat",  # Input topic
            Pose,
            self.callback_pose,
            queue_size=100,
            transport_hints=rospy.transport_hints.TransportHints().reliable().tcp_nodelay()
        )

        # Publisher: publishes to null_space_state
        self.pub_null_space_state = rospy.Publisher(
            "/optimal/null_space_state",  # Output topic
            JointState,
            queue_size=100
        )

    def calculate_optimal(self):
        # ------------- optimal here ------------



        # Create a new JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()  # Add a timestamp

        # Define joint names for the 7 joints
        joint_state_msg.name = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6", "joint_7"
        ]

        # Process Pose data into JointState position values
        # For this example, map position and orientation to 7 joints
        joint_state_msg.position = [
            1,1,1,1,1,1,1
        ]

        # Example: Set velocities and efforts to 0 for all joints
        joint_state_msg.velocity = [0.0] * 7
        joint_state_msg.effort = [0.0] * 7

        # Publish the JointState message
        self.pub_null_space_state.publish(joint_state_msg)
        rospy.loginfo(f"Published JointState: {joint_state_msg}")
        sys.stderr.write("--------------//////////////--------------------\n")

    def callback_pose(self, msg):
        rospy.loginfo(f"Received Pose: Position: {msg.position}, Orientation: {msg.orientation}")

        # Store the position and orientation into a new parameter (instance variable)
        self.current_position = {
            "x": msg.position.x,
            "y": msg.position.y,
            "z": msg.position.z
        }
        self.current_orientation = {
            "x": msg.orientation.x,
            "y": msg.orientation.y,
            "z": msg.orientation.z,
            "w": msg.orientation.w
        }
        sys.stderr.write("-------------------------------------------------\n")
        

    def run(self):
        """
        Keeps the node running.
        """
        rospy.loginfo("Joint_optimal node started. Listening to /passive_control/pos_quat...")
        rospy.spin()

if __name__ == '__main__':
    try:
        sys.stderr.write("----------qqqqq----------------------------------\n")
        node = Joint_optimal()
        node.run()
        sys.stderr.write("------------wwwww---------------------------------\n")
    except rospy.ROSInterruptException:
        pass
