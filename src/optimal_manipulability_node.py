#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray  # Import Float64MultiArray
import sys
from scipy.optimize import minimize
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import tr2rpy
import numpy as np
import scipy as sp

class JointOptimal:
    def __init__(self):
        # Node initialization
        rospy.init_node('Joint_optimal', anonymous=True)

        #Subscriber: listens to /desired_pose
        self.sub_control = rospy.Subscriber(
            "/desired_pose",  # Input topic
            Pose,
            self.callback_pose,
            queue_size=100
        )
        
        # Subscriber: listens to /iiwa/joint_states
        self.joint_control = rospy.Subscriber(
            "/iiwa/joint_states",  # Input topic
            JointState,
            self.callback_jtstate,
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
        self.current_jt_position = None

        self.current_iteration = 0

        # Define DH parameters for each link
        L1 = RevoluteDH(a=0, alpha=-np.pi/2, d=0.360, offset=0)
        L2 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
        L3 = RevoluteDH(a=0, alpha=np.pi/2, d=0.420, offset=0)
        L4 = RevoluteDH(a=0, alpha=-np.pi/2, d=0, offset=0)
        L5 = RevoluteDH(a=0, alpha=-np.pi/2, d=0.400, offset=0)
        L6 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
        L7 = RevoluteDH(a=0, alpha=0, d=0.126, offset=0)
        # Define the robot
        self.robot = DHRobot([L1, L2, L3, L4, L5, L6, L7], name='7DOF_Robot')
        self.Htmp_init = None

        self.n = np.array([1, 0, 0])  # Descallback_poseired task-space direction (3x1)

        # Define bounds for joint angles
        lb = np.array([-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054])
        ub = np.array([2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054])
        # Define bounds for the joint angles (using the lb and ub arrays)
        self.bounds = list(zip(lb, ub))
        self.first_iter = True


    # Define the Jacobian function (example)
    def computeJacobian(self, qT):
        J = self.robot.jacob0(qT)
        return J[:3, :]

    # Define the cost function
    def manipulabilityCost(self, qT, Jfun):

        # Compute Jacobian at qT
        J = Jfun(qT)  # J should return a 6x7 Jacobian
        
        
        # Compute the cost
        CmanT = np.linalg.norm(J.T @ self.n)
        print(CmanT)
        return CmanT

    # Define the nonlinear constraints
    def manipulabilityConstraints(self, qT):
        Htmp = self.robot.fkine(qT)
        
        c = 0.5 - np.linalg.norm(sp.linalg.logm(Htmp.R @ self.Htmp_init.R.T))
        ceq = self.Htmp_init.t-Htmp.t
        return c, ceq

    # Define the objective function
    def objective(self,qT, computeJacobian):
        return self.manipulabilityCost(qT, self.computeJacobian)

    def calculate_optimal(self):
        """
        Process the Pose data and calculate an optimal 7D vector.
        """
        print("optimal loop")
        # Check if current position and orientation exist
        if self.current_jt_position is not None:
            # Create a Float64MultiArray message
            Joint_optimal_configure = Float64MultiArray()

            # Define constraints as a dictionary
            constraints = ({'type': 'ineq', 'fun': lambda qT: self.manipulabilityConstraints(qT)[0]},
                            {'type': 'eq', 'fun': lambda qT: self.manipulabilityConstraints(qT)[1]})
            
            print(self.current_jt_position)
            print(self.bounds)
            # Solve the optimization problem
            result = minimize(self.objective, self.current_jt_position, args=(self.robot,), method='SLSQP', 
                            bounds=self.bounds, constraints=constraints, options={'disp': True, "maxiter": 10000})

            # Display results
            qT_opt = result.x
            CmanT_min = result.fun

            print("???????????????????????????????????????????????????????????")
            print("optimal done")
            print(CmanT_min)
            print(qT_opt)
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
        Stores Pose data.
        """
        # rospy.loginfo(f"Received Pose: Position: {msg.position}, Orientation: {msg.orientation}")

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

        

    def callback_jtstate(self, msg):
        """
        Callback function for the JointState subscriber.
        Stores JointState data and triggers optimal calculation.
        """
        # rospy.loginfo(f"Received JointState: Position: {msg.position}")
        self.current_iteration += 1

        #print(self.current_iteration)

        # Store the joint position into instance variables
        self.current_jt_position = np.array([msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]])

        
        self.Htmp_init = self.robot.fkine(self.current_jt_position)

        # print current iteration each 1000 iterations
        if self.current_iteration % 1000 == 0:
            print(self.current_iteration)
        if self.current_iteration == 8000:
            print("Calculating optimal")
            self.calculate_optimal()

        #if self.first_iter is True:
        #    self.calculate_optimal()
        #    self.first_iter = False


        

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
