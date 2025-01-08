import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import scipy as sp
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath.base import tr2rpy
from mpl_toolkits.mplot3d import Axes3D

costHistory = []

# Define the Jacobian function (example)
def computeJacobian(qT, robot):
    J = robot.jacob0(qT)
    return J[:3, :]

# Define the cost function
def manipulabilityCost(qT, W, Jfun, n, wman, robot, xd_init, xn_init, xo_init, xa_init, lambda_, costHistory=None):

    if costHistory is None:
        costHistory = []
    
    Htmp = robot.fkine(qT)
    xt = Htmp.t

    # Compute constraints
    #c, ceq = manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, robot)
    
    # Compute the penalty terms
    # penalty_eq = np.sum(ceq**2)  # Quadratic penalty for equality constraints
    # penalty_ineq = np.sum(np.maximum(0, -c)**2)  # Penalty for violated inequality constraints

    J = Jfun(qT, robot)  # J should return a 3x7 Jacobian
    Fe_d = np.linalg.inv(J @ J.T)
    eigenValues, eigenVectors = np.linalg.eig(Fe_d)

    taskVector = eigenValues @ n
    taskVector2 = np.linalg.inv(J @ J.T) @ n

    cost_tmp = np.linalg.norm(J.T @ n)
    
    #CmanT = wman * (np.linalg.norm(taskVector2)**-2)
    CmanT = cost_tmp
    costHistory.append(CmanT)
    return CmanT

# Define the nonlinear constraints
def manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, R_init, robot):
    Htmp = robot.fkine(qT)
    xt = Htmp.t
    # xn = Htmp.n
    # xo = Htmp.o
    # xa = Htmp.a
    
    # c = np.concatenate([
    #     xn_init - xn - 0.1,
    #     xo_init - xo - 0.1,
    #     xa_init - xa - 0.1
    # ])
    
    c = 0.5 - np.linalg.norm(sp.linalg.logm(Htmp.R @ R_init.T))



    print(c)
    print(Htmp.R)
    #robot.plot(qT, block=True) 
    ceq = xd_init - xt  
    return c, ceq

# Define the objective function
def objective(qT, robot):
    return manipulabilityCost(qT, W, computeJacobian, n, wman, robot, xd_init, xn_init, xo_init, xa_init, lambda_, costHistory)

x0 = np.array([-3.0, 3.0, 2.0])

#q0 = np.array([-0.001639030, 0.67400, 0.00475, -1.3777, 0.0101549, -0.46403, -0.00577])
q0 = np.array([-0.001639030, 0.87400, 0.00475, -1.3777, 0.0101549, +0.56403, -0.00577])
#q0 = np.array([-0.001639030, -0.67400, -0.00475, -0, -0.0101549, -0.46403, -0.00577])
#q0 = np.array([-0.41639030, 0.67400, -0.50475, 1.3777, 0.0101549, -0.46403, -0.00577])


L1 = RevoluteDH(a=0, alpha=-np.pi/2, d=3.60, offset=0)
L2 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
L3 = RevoluteDH(a=0, alpha=np.pi/2, d=4.20, offset=0)
L4 = RevoluteDH(a=0, alpha=-np.pi/2, d=0, offset=0)
L5 = RevoluteDH(a=0, alpha=-np.pi/2, d=4.00, offset=0)
L6 = RevoluteDH(a=0, alpha=np.pi/2, d=0, offset=0)
L7 = RevoluteDH(a=0, alpha=0, d=1.26, offset=0)
robot = DHRobot([L1, L2, L3, L4, L5, L6, L7], name='7DOF_Robot')

Htmp = robot.fkine(q0)
xd_init = Htmp.t
xn_init = Htmp.n
xo_init = Htmp.o
xa_init = Htmp.a
R_init = Htmp.R
print("Initial joint configuration:", q0)
print("Initial End-Effector Position (xt):", xd_init)
print(xn_init)
print(xo_init)
print(xa_init)
robot.plot(q0, block=True)  # 'block=True' keeps the window open until closed


wman = 1.0
lambda_ = 1.0  
n = np.array([0, 0, 1])  
velocity_limits_deg = np.array([98, 98, 100, 130, 140, 180, 180])
velocity_limits_rad = velocity_limits_deg * (np.pi / 180)
W = np.diag(velocity_limits_rad / np.max(velocity_limits_rad))

lb = np.array([-2.967, -2.094, -2.967, -2.094, -2.967, -2.094, -3.054])
ub = np.array([2.967, 2.094, 2.967, 2.094, 2.967, 2.094, 3.054])
bounds = list(zip(lb, ub))

# Define constraints as a dictionary
constraints = (
    {
         'type': 'eq',  # Equality constraint (end-effector position fixed)
         'fun': lambda qT: manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, R_init, robot)[1]
     }
     ,
        {
       'type': 'ineq',  # Equality constraint (end-effector position fixed)
        'fun': lambda qT: manipulabilityConstraints(qT, xd_init, xn_init, xo_init, xa_init, R_init, robot)[0]
        }

 )

result = minimize(objective, q0, args=(robot,), method='SLSQP', bounds=bounds, constraints = constraints, options={'disp': True, "maxiter": 10000})

if result.success:
    qT_opt = result.x
    CmanT_min = result.fun
    print("Optimal joint configuration:", qT_opt)
    print("Minimum cost:", CmanT_min)
else:
    print("ERROR!!!!!!!!!!!!")

Htmp = robot.fkine(qT_opt)
xt_1 = Htmp.t 
xn_1 = Htmp.n
xo_1 = Htmp.o
xa_1 = Htmp.a

print("Nouvelle end effector position:", xt_1)
print(xn_1)
print(xo_1)
print(xa_1)
robot.plot(qT_opt, block=True)

plt.plot(costHistory, '-x', linewidth=1.5)
plt.xlabel('Iteration')
plt.ylabel('Cost')
plt.title('Cost vs Iterations')
plt.grid(True)
plt.show()



# Function to plot ellipsoid
def plot_ellipsoid(M, color, ax, label, position):
    # Generate the meshgrid for spherical coordinates
    theta, phi = np.meshgrid(np.linspace(0, 2 * np.pi, 50), np.linspace(0, np.pi, 25))
    x = np.cos(theta) * np.sin(phi)
    y = np.sin(theta) * np.sin(phi)
    z = np.cos(phi)
    
    # Compute the ellipsoid points
    eigenValues, eigenVectors = np.linalg.eig(M)

    # print("x", (eigenValues[0])**0.5)
    # print("y", (eigenValues[1])**0.5)
    # print("z", (eigenValues[2])**0.5)

    x_ax = np.array([1, 0, 0])
    y_ax = np.array([0, 1, 0])
    z_ax = np.array([0, 0, 1])
    print("x", x_ax.T @ M @ x_ax)
    print("y", y_ax.T @ M @ y_ax)
    print("z", z_ax.T @ M @ z_ax)

    ellipsoid_points = eigenVectors @ np.sqrt(np.diag(eigenValues)) @ np.vstack([x.flatten(), y.flatten(), z.flatten()])
    # Add the translation (end-effector position)
    ellipsoid_points += np.expand_dims(position, axis=1)

    # Reshape the points back to a grid
    x_v = ellipsoid_points[0, :].reshape(x.shape)
    y_v = ellipsoid_points[1, :].reshape(y.shape)
    z_v = ellipsoid_points[2, :].reshape(z.shape)
    
    # Plot the ellipsoid
    ax.plot_surface(x_v, y_v, z_v, color=color, alpha=0.3, edgecolor='none')
    ax.set_xlabel('$x_1$', fontsize=14)
    ax.set_ylabel('$x_2$', fontsize=14)
    ax.text(0.2, 1, 0, label, fontsize=14)

J = computeJacobian(q0, robot)
Fe_d1 = np.linalg.inv(J @ J.T)

J_opt = computeJacobian(qT_opt, robot)
Fe_opt = np.linalg.inv(J_opt @ J_opt.T)

# Create a 3D plot
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Call the function to plot the ellipsoid
plot_ellipsoid(Fe_d1, color='blue', ax=ax, label='Initial Ellipsoid', position=xd_init)
# Call the function to plot the ellipsoid
plot_ellipsoid(Fe_opt, color='red', ax=ax, label='Optimized Ellipsoid', position=xd_init)
# Set the axis limits to be between 0 and 10
ax.set_xlim([6, 7])
ax.set_ylim([0, 3])
ax.set_zlim([0.5, 3.5])
#ax.set_xlim([0, 10])
#ax.set_ylim([-2, 2])
#ax.set_zlim([0, 10])
# Modification des noms des axes
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()



