import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import tr2rpy
from mpl_toolkits.mplot3d import Axes3D



# Define the Jacobian function (example)
def computeJacobian(qT, robot):
    J = robot.jacob0(qT)
    return J[:3, :]

# Define the cost function
def manipulabilityCost(qT, W, Jfun, n, wman, robot, xd_init, lambda_):
    # Compute the forward kinematics to get current position xt
    global costHistory
    if costHistory is None:
        costHistory = []

    
    Htmp = robot.fkine(qT)
    xt = Htmp.t[:3] if hasattr(Htmp, 't') else Htmp[:3, -1]
    
    # Compute equality constraint deviation
    eq_deviation = np.linalg.norm(xd_init - xt)
    
    # Penalize the equality constraint
    Ceq_penalty = lambda_ * eq_deviation**2

    # Compute Jacobian at qT
    J = Jfun(qT)  # J should return a 6x7 Jacobian
    
    # Compute the task-space vector
    #taskVector = W.T @ J.T @ n  # Dimension: 7x1
    taskVector = np.linalg.inv(np.dot(J,J.T))*n
    
    # Compute the cost
    CmanT = wman * (np.linalg.norm(taskVector)**-2) + Ceq_penalty
    costHistory.append(CmanT)
    return CmanT

# Define the nonlinear constraints
def manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, robot):
    Htmp = robot.fkine(qT)
    xt = Htmp.t[:3] if hasattr(Htmp, 't') else Htmp[:3, -1]
    xn = Htmp.n[:3] if hasattr(Htmp, 'n') else np.zeros(3)
    xo = Htmp.o[:3] if hasattr(Htmp, 'o') else np.zeros(3)
    xa = Htmp.a[:3] if hasattr(Htmp, 'a') else np.zeros(3)
    
    c = np.concatenate([
        xn_init - xn - 0.01,
        xo_init - xo - 0.01,
        xa_init - xa - 0.01
    ])
    ceq = []  # No equality constraints in this case
    return c, ceq

# Define the objective function
def objective(qT, robot):
    return manipulabilityCost(qT, W, computeJacobian, n, wman, robot, xd_init, lambda_)



# Define the function to calculate the size of the ellipsoid in a specific direction
def ellipsoid_size_in_direction(M, u):
    u = u / np.linalg.norm(u)  # Normalize the direction vector to unit vector
    size_in_direction = np.sqrt(u.T @ M @ u)
    return size_in_direction

# Function to plot ellipsoid
def plot_ellipsoid(M, color, ax, label):
    # Generate the meshgrid for spherical coordinates
    theta, phi = np.meshgrid(np.linspace(0, 2 * np.pi, 50), np.linspace(0, np.pi, 25))
    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)
    
    # Compute the ellipsoid points
    eigenVectors, eigenValues = np.linalg.eig(M)
    ellipsoid_points = eigenVectors @ np.sqrt(np.diag(eigenValues)) @ np.vstack([x.flatten(), y.flatten(), z.flatten()])
    
    # Reshape the points back to a grid
    x_v = ellipsoid_points[0, :].reshape(x.shape)
    y_v = ellipsoid_points[1, :].reshape(y.shape)
    z_v = ellipsoid_points[2, :].reshape(z.shape)
    
    # Plot the ellipsoid
    ax.plot_surface(x_v, y_v, z_v, color=color, alpha=0.3, edgecolor='none')
    ax.set_xlabel('$x_1$', fontsize=14)
    ax.set_ylabel('$x_2$', fontsize=14)
    ax.text(0.2, 1, 0, label, fontsize=14)


# Define initial robot configuration
# qt = np.array([0.0437988, 0.773654, -0.00407287, 1.59702, -0.0121146, 0.821307, 0.0557394])  # Task 1 alternative
# qt = np.array([0.0, 0.0, -0.0, -0.0, -0.0, 0.0, 0.0])  # Task 1 alternative
q0 = np.array([-0.001639030, 0.67400, 0.00475, 1.3777, 0.0101549, -0.46403, -0.00577])  # Task 1 alternative
# q0 = np.array([0.6589869959101646, 1.4907288952412516, -1.8297133397366458, 
#                1.371943332339404, -0.31948018663916766, -0.7667572555517488, 1.9643502983179815])  # Optional task 1

# Define DH parameters for each link
L1 = RevoluteDH(a=0, alpha=-np.pi/2, d=3.60, offset=0)
L2 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
L3 = RevoluteDH(a=0, alpha=-np.pi/2, d=4.20, offset=0)
L4 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
L5 = RevoluteDH(a=0, alpha=-np.pi/2, d=4.00, offset=0)
L6 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
L7 = RevoluteDH(a=0, alpha=0, d=1.26, offset=0)

# Define the robot
robot = DHRobot([L1, L2, L3, L4, L5, L6, L7], name='7DOF_Robot')

# Number of Degrees of Freedom (DOF)
nbDOFs = 7

# Plot the robot in the specified configuration
robot.plot(q0, block=True)  # 'block=True' keeps the window open until closed
qt_init = q0
# Forward Kinematics for the initial configuration
Htmp = robot.fkine(q0)

# Extract end-effector position and orientation
if hasattr(Htmp, 'A'):  # Check if Htmp is a homogeneous transformation matrix
    xt = Htmp.A[0:3, 3]  # Position (translation vector)
    rpy = tr2rpy(Htmp.A, unit='deg')  # Orientation (RPY angles)
else:
    xt = Htmp.t  # Position
    rpy = Htmp.rpy()  # Orientation as Roll-Pitch-Yaw

# Setting desired task position as the first end-effector position
xd_init = xt
xn_init = Htmp.n
xo_init = Htmp.o
xa_init = Htmp.a

# Print initial task position and orientation
print("Initial End-Effector Position (xt):", xt)
print("Initial End-Effector Orientation (RPY angles in degrees):", rpy)







# Optimization parameters
wman = 1.0
lambda_ = 1.0  # 'lambda' is a reserved keyword in Python, so I use lambda_
n = np.array([1, 0, 0])  # Desired task-space direction (3x1)

# Joint velocity limits in degrees per second
velocity_limits_deg = np.array([98, 98, 100, 130, 140, 180, 180])
# Convert to radians per second
velocity_limits_rad = velocity_limits_deg * (np.pi / 180)
# Compute the diagonal weights (inverse of velocity limits)
W = np.diag(velocity_limits_rad / np.max(velocity_limits_rad))

# Define bounds for joint angles
lb = np.array([-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054])
ub = np.array([2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054])

# Define bounds for the joint angles (using the lb and ub arrays)
bounds = list(zip(lb, ub))

# Define constraints as a dictionary
constraints = {'type': 'ineq', 'fun': lambda qT: manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, robot)[0]}

# Solve the optimization problem
result = minimize(objective, qt_init, args=(robot,), method='SLSQP', bounds=bounds, constraints=constraints, options={'disp': True})

# Display results
qT_opt = result.x
CmanT_min = result.fun
print("Optimal joint configuration:", qT_opt)
print("Minimum cost:", CmanT_min)

robot.plot(qT_opt, block=True)

# Jacobian at the optimal solution
J_opt = robot.jacob0(qT_opt)
print("Jacobian at optimal configuration:", J_opt)

# Compute manipulability (Me_d) and Fe_d (as done in the original code)
J_opt_full = J_opt
J_opt = J_opt[:3, :]
Me_d = np.dot(J_opt, J_opt.T)  # Optimal manipulability
Fe_d = np.linalg.inv(np.dot(J_opt, J_opt.T))  # Inverse manipulability

# Forward kinematics for plotting
Htmp = robot.fkine(qT_opt)
xt_1 = Htmp.t[:3] if hasattr(Htmp, 't') else Htmp[:3, -1]

# Plot cost history (if needed)
# Assuming 'costHistory' is being logged somewhere
# matplotlib plotting can be used here if you want to graph the cost history

plt.plot(costHistory, '-x', linewidth=1.5)
plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('Cost vs Iterations')
plt.grid(True)
plt.show()







# Auxiliary variables
dt = 1E-1  # Time step
nbIter = 50  # Number of iterations
Kp = 8  # Gain for position control in task space
Km = 5  # Gain for manipulability control in nullspace

# Initial conditions
dxh = np.array([0, 0, 0])  # Desired Cartesian velocity

qt = q0  # Starting joint position (should be defined before)
xd = np.zeros(3)
it = 1  # Iterations counter
h1 = None

# Main control loop
fig, axs = plt.subplots(1, 2, figsize=(10, 5))

# Preallocate arrays (replace values with actual sizes)
rows, cols = 3, 3
Me_track = np.zeros((rows, cols, nbIter))
Fe_track = np.zeros((rows, cols, nbIter))
qt_track = np.zeros((rows, nbIter))

while it < nbIter:
    if h1 is not None:
        h1.remove()  # delete previous plot if needed

    Jt = robot.jacob0(qt)  # Current Jacobian
    print(Jt)
    
    Jt_full = Jt
    Jt = Jt[:3, :]  # Extract the top 3 rows (task space)
    
    Htmp = robot.fkine(qt)  # Forward kinematics (needed for plots)
    Me_ct = Jt @ Jt.T  # Current manipulability
    Fe_ct = np.linalg.inv(Jt @ Jt.T)
    
    eigenVectors_v, eigenValues_v = np.linalg.eig(Me_ct)
    force_eigenVectors_v, force_eigenValues_v = np.linalg.eig(Fe_ct)

    # Update tracks for plotting
    Me_track[:, :, it-1] = Me_ct
    Fe_track[:, :, it-1] = Fe_ct
    qt_track[:, it-1] = qt

    # Current end-effector position
    if isinstance(Htmp, np.ndarray):
        xt = Htmp[:3, -1]  # Extract the position
    else:
        xt = Htmp.t[:3]  # Extract the position if SE3 object

    # Setting desired task position as the first end-effector position
    if it == 1:
        xd = xt
    

    # Compute joint velocities
    dxr = dxh + Kp * (xd - xt)  # Reference task space velocity
    dq_T1 = np.linalg.pinv(Jt) @ dxr  # Main task joint velocities

    # Compute nullspace joint velocities
    q_diff = qT_opt - qt
    dq_ns_2 = -(np.eye(nbDOFs) - np.linalg.pinv(Jt) @ Jt) @ Km * q_diff  # Redundancy resolution

    # Plot the velocity ellipsoid at the 7th joint
    theta, phi = np.meshgrid(np.linspace(0, 2 * np.pi, 50), np.linspace(0, np.pi, 25))
    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)

    scale_velocity = 0.5  # Scale for velocity ellipsoid
    velocity_ellipsoid = eigenVectors_v @ np.sqrt(np.diag(eigenValues_v)) @ np.vstack([x.flatten(), y.flatten(), z.flatten()])
    x_v = velocity_ellipsoid[0, :].reshape(x.shape) * scale_velocity
    y_v = velocity_ellipsoid[1, :].reshape(y.shape) * scale_velocity
    z_v = velocity_ellipsoid[2, :].reshape(z.shape) * scale_velocity

    # Plot the force ellipsoid at the 7th joint
    scale_force = 7.1
    force_ellipsoid = force_eigenVectors_v @ np.sqrt(np.diag(force_eigenValues_v)) @ np.vstack([x.flatten(), y.flatten(), z.flatten()])
    x_f = force_ellipsoid[0, :].reshape(x.shape) * scale_force
    y_f = force_ellipsoid[1, :].reshape(y.shape) * scale_force
    z_f = force_ellipsoid[2, :].reshape(z.shape) * scale_force

    # Plotting robot and manipulability ellipsoids
    axs[0].cla()  # Clear previous plot
    axs[0].plot_surface(xt[0] + x_v, xt[1] + y_v, xt[2] + z_v, alpha=0.3, facecolors='r')
    axs[0].plot_surface(xt[0] + x_f, xt[1] + y_f, xt[2] + z_f, alpha=0.3, facecolors='b')

    # Plot robot (assuming 'robot.plot' is a method that works similarly in Python)
    robot.plot(qt)

    axs[0].set_box_aspect([1, 1, 1])
    axs[0].set_xlabel('x1')
    axs[0].set_ylabel('x2')

    # Plotting costs
    Ct = np.linalg.norm(np.logm(np.linalg.inv(Me_d) @ Me_ct @ np.linalg.inv(Me_d)), 'fro')
    axs[1].plot(it, Ct, 'xr')
    axs[1].set_xlabel('t')
    axs[1].set_ylabel('Ct')

    plt.draw()

    # Updating joint position
    qt = qt + (q_diff) * dt
    it += 1  # Iterations++

plt.show()






# Assume the following variables are defined elsewhere:
# Me_d, Me_track, Me_ct, Fe_d, Fe_track, Fe_ct, and direction vector

# Direction vector (e.g., x-axis)
direction = np.array([1, 0, 0])

# Compute sizes in this direction for force ellipsoids
size_ellipsoid1 = ellipsoid_size_in_direction(Fe_d, direction)
size_ellipsoid2 = ellipsoid_size_in_direction(Fe_track[:, :, 0], direction)

# Compare sizes and print the result
if size_ellipsoid1 > size_ellipsoid2:
    print(f'Ellipsoid 1 (Fe_d) is larger in the {direction} direction.')
    print(f'Size1: {size_ellipsoid1}')
    print(f'Size2: {size_ellipsoid2}')
else:
    print(f'Ellipsoid 2 (Fe_track(:,:,1)) is larger in the {direction} direction.')
    print(f'Size1: {size_ellipsoid1}')
    print(f'Size2: {size_ellipsoid2}')

# Create plots for manipulability ellipsoids
fig1 = plt.figure(figsize=(9, 9))
ax1 = fig1.add_subplot(121, projection='3d')
plot_ellipsoid(Me_d, 'g', ax1, 'Initial')

ax2 = fig1.add_subplot(122, projection='3d')
plot_ellipsoid(Me_d, 'g', ax2, 'Initial')
plot_ellipsoid(Me_ct, 'r', ax2, 'Final')

# Create plots for force ellipsoids
fig2 = plt.figure(figsize=(9, 9))
ax3 = fig2.add_subplot(121, projection='3d')
plot_ellipsoid(Fe_d, 'g', ax3, 'Initial')

ax4 = fig2.add_subplot(122, projection='3d')
plot_ellipsoid(Fe_d, 'g', ax4, 'Initial')
plot_ellipsoid(Fe_ct, 'b', ax4, 'Final')

plt.show()
















