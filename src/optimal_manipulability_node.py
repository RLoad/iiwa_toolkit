#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray  # Import Float64MultiArray
import sys

class JointOptimal:
    def __init__(self):
        # Node initialization
        rospy.init_node('Joint_optimal', anonymous=True)

        # Subscriber: listens to /desired_pose
        self.sub_control = rospy.Subscriber(
            "/desired_pose",  # Input topic
            Pose,
            self.callback_pose,
            queue_size=100
        )

        # Publisher: publishes to /optimal/null_space_state as Float64MultiArray
        self.pub_null_space_state = rospy.Publisher(
            "/optimal/null_space_state",  # Output topic
            Float64MultiArray,           # Change message type
            queue_size=100
        )

        # Store current position and orientation
        self.current_position = None
        self.current_orientation = None

    def calculate_optimal(self):
        """
        Process the Pose data and calculate an optimal 7D vector.
        """
        # Check if current position and orientation exist
        if self.current_position and self.current_orientation:
            # Create a Float64MultiArray message
            float_array_msg = Float64MultiArray()
            
            # Example: Pack 7 values into the Float64MultiArray data
            float_array_msg.data = [
                self.current_position["x"],
                self.current_position["y"],
                self.current_position["z"],
                self.current_orientation["x"],
                self.current_orientation["y"],
                self.current_orientation["z"],
                self.current_orientation["w"]
            ]

            # Publish the Float64MultiArray message
            self.pub_null_space_state.publish(float_array_msg)
            rospy.loginfo(f"Published Float64MultiArray: {float_array_msg.data}")
        else:
            rospy.logwarn("No Pose data received yet!")

    def callback_pose(self, msg):
        """
        Callback function for the Pose subscriber.
        Stores Pose data and triggers optimal calculation.
        """
        rospy.loginfo(f"Received Pose: Position: {msg.position}, Orientation: {msg.orientation}")

        # Store the position and orientation into instance variables
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

        # Trigger calculation of the optimal null-space state
        self.calculate_optimal()

    def run(self):
        """
        Keeps the node running.
        """
        rospy.loginfo("Joint_optimal node started. Listening to /desired_pose...")
        rospy.spin()

if __name__ == '__main__':
    try:
        sys.stderr.write("---------- Node Starting ----------------------------------\n")
        node = JointOptimal()
        node.run()
        sys.stderr.write("------------ Node Shutting Down ---------------------------\n")
    except rospy.ROSInterruptException:
        pass
