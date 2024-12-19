#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray  # Import Float64MultiArray
from jtpos_msgs.msg import JtPose
import sys
from scipy.optimize import minimize
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

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
        # Subscriber: listens to /desired_pose
        # self.sub_control = rospy.Subscriber(
        #     "/iiwa/join_states",  # Input topic
        #     JtPose,
        #     self.callback_pose,
        #     queue_size=100
        # )

        # Publisher: publishes to /optimal/null_space_state as Float64MultiArray
        self.pub_null_space_state = rospy.Publisher(
            "/optimal/null_space_state",  # Output topic
            Float64MultiArray,           # Change message type
            queue_size=100
        )

        # Store current position and orientation
        self.current_position = None
        self.current_orientation = None
        self.current_jt_position = None

        # Define DH parameters for each link
        L1 = RevoluteDH(a=0, alpha=-np.pi/2, d=0.360, offset=0)
        L2 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
        L3 = RevoluteDH(a=0, alpha=-np.pi/2, d=0.420, offset=0)
        L4 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
        L5 = RevoluteDH(a=0, alpha=-np.pi/2, d=0.400, offset=0)
        L6 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
        L7 = RevoluteDH(a=0, alpha=0, d=0.126, offset=0)
        # Define the robot
        self.robot = DHRobot([L1, L2, L3, L4, L5, L6, L7], name='7DOF_Robot')

        self.wman = 1.0
        self.lambda_ = 1.0  # 'lambda' is a reserved keyword in Python, so I use lambda_
        self.n = np.array([1, 0, 0])  # Descallback_poseired task-space direction (3x1)


        # Joint velocity limits in degrees per second
        velocity_limits_deg = np.array([98, 98, 100, 130, 140, 180, 180])
        # Convert to radians per second
        velocity_limits_rad = velocity_limits_deg * (np.pi / 180)
        # Compute the diagonal weights (inverse of velocity limits)
        self.W = np.diag(velocity_limits_rad / np.max(velocity_limits_rad))

        # Define bounds for joint angles
        lb = np.array([-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054])
        ub = np.array([2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054])
        # Define bounds for the joint angles (using the lb and ub arrays)
        self.bounds = list(zip(lb, ub))

    # Define the Jacobian function (example)
    def computeJacobian(self, qT):
        J = self.robot.jacob0(qT)
        return J[:3, :]

    # Define the cost function
    def manipulabilityCost(self, qT, Jfun):
        # Compute the forward kinematics to get current position xt
        Htmp = self.robot.fkine(qT)
        xt = Htmp.t
        
        # Compute equality constraint deviation
        eq_deviation = np.linalg.norm(self.current_position[0] - xt)
        
        # Penalize the equality constraint
        Ceq_penalty = self.lambda_ * eq_deviation**2

        # Compute Jacobian at qT
        J = self.Jfun(qT)  # J should return a 6x7 Jacobian
        
        # Compute the task-space vector
        #taskVector = W.T @ J.T @ n  # Dimension: 7x1
        taskVector = np.linalg.inv(np.dot(J,J.T))@self.n
        
        # Compute the cost
        CmanT = self.wman * (np.linalg.norm(taskVector)**-2) + Ceq_penalty
        return CmanT

    # Define the nonlinear constraints
    def manipulabilityConstraints(self, qT):
        Htmp = self.robot.fkine(qT)
        xt = Htmp.t
        xn = Htmp.n
        xo = Htmp.o
        xa = Htmp.a

        orientation = np.array([xn[0], xo[1], xa[2] ])
        
        c = [abs(self.current_orientation - orientation) - 0.1]
        ceq = []  # No equality constraints in this case
        return c, ceq

    # Define the objective function
    def objective(self,qT):
        return self.manipulabilityCost(qT, computeJacobian)

    def calculate_optimal(self):
        """
        Process the Pose data and calculate an optimal 7D vector.
        """
        # Check if current position and orientation exist
        if self.current_position and self.current_orientation:
            # Create a Float64MultiArray message
            Joint_optimal_configure = Float64MultiArray()

            # Define constraints as a dictionary
            constraints = {'type': 'ineq', 'fun': lambda qT: self.manipulabilityConstraints(qT)[0]}

            # Solve the optimization problem
            result = minimize(self.objective, self.current_jt_position, args=(self.robot,), method='SLSQP', bounds=bounds, constraints=constraints, options={'disp': True})

            # Display results
            qT_opt = result.x
            CmanT_min = result.fun
            
            # Example: Pack 7 values into the Float64MultiArray data
            # Joint_optimal_configure.data = [
            #     self.current_position["x"],
            #     self.current_position["y"],
            #     self.current_position["z"],
            #     self.current_orientation["x"],
            #     self.current_orientation["y"],
            #     self.current_orientation["z"],
            #     self.current_orientation["w"]
            # ]
            Joint_optimal_configure.data = qT_opt

            # Publish the Float64MultiArray message
            self.pub_null_space_state.publish(Joint_optimal_configure)
            rospy.loginfo(f"Published Float64MultiArray: {Joint_optimal_configure.data}")
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
