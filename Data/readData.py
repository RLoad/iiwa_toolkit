import numpy as np
import matplotlib.pyplot as plt

# Reading and processing data from Force.txt
force_file = 'semester-project-Maksymiliann/data_plot_saved/realRobotSurfOptimal/Force.txt'

force_time = []
real_force_filtered = []
desired_force = []

with open(force_file, 'r') as f:
    lines = f.readlines()
    
for line in lines:
    if line.startswith('time:'):
        force_time.append(float(line.split('time:')[1].strip()))
    elif line.startswith('real_force_filtered_:'):
        real_force_filtered.append(list(map(float, line.split('real_force_filtered_:')[1].strip().split())))
    elif line.startswith('desired_force_:'):
        desired_force.append(list(map(float, line.split('desired_force_:')[1].strip().split())))

force_time = np.array(force_time)
real_force_filtered = np.array(real_force_filtered)
desired_force = np.array(desired_force)

# Removing Outliers from Force Data
force_mean = np.mean(real_force_filtered, axis=0)
force_std = np.std(real_force_filtered, axis=0)
valid_force_indices = np.all(np.abs(real_force_filtered - force_mean) <= 3 * force_std, axis=1)

force_time = force_time[valid_force_indices]
real_force_filtered = real_force_filtered[valid_force_indices]
desired_force = desired_force[valid_force_indices]

# Calculating Error in Z-direction
force_error_z = desired_force[:, 2] - real_force_filtered[:, 2]

# Plotting Real vs Desired Forces
plt.figure()
plt.plot(force_time, real_force_filtered[:, 0], 'r', label='Real Force X')
plt.plot(force_time, desired_force[:, 0], 'b', label='Desired Force X')
plt.plot(force_time, real_force_filtered[:, 1], 'g', label='Real Force Y')
plt.plot(force_time, desired_force[:, 1], 'm', label='Desired Force Y')
plt.plot(force_time, real_force_filtered[:, 2], 'k', label='Real Force Z')
plt.plot(force_time, desired_force[:, 2], 'c', label='Desired Force Z')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.title('Real vs Desired Forces over Time')
plt.ylim([-25, 5])
plt.grid(True)
plt.show()

# Plotting Z-direction Force Error
plt.figure()
plt.plot(force_time, force_error_z, 'b', label='Z-direction Force Error')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Error (N)')
plt.title('Error between Desired and Real Force in Z-direction')
plt.ylim([-25, 5])
plt.grid(True)
plt.show()

# Reading and processing data from robot_data.txt
robot_file = 'semester-project-Maksymiliann/data_plot_saved/realRobotSurfOptimal/robot_data.txt'

robot_time = []
joint_positions = []
end_effector_pos = []
end_effector_quat = []
manipulability = []
joint_effort = []

with open(robot_file, 'r') as f:
    lines = f.readlines()
    
for line in lines:
    if line.startswith('time:'):
        robot_time.append(float(line.split('time:')[1].strip()))
    elif line.startswith('Joint Positions:'):
        joint_positions.append(list(map(float, line.split('Joint Positions:')[1].strip().split())))
    elif line.startswith('End-Effector Position:'):
        end_effector_pos.append(list(map(float, line.split('End-Effector Position:')[1].strip().split())))
    elif line.startswith('End-Effector Quaternion:'):
        end_effector_quat.append(list(map(float, line.split('End-Effector Quaternion:')[1].strip().split())))
    elif line.startswith('Manipulability:'):
        manipulability.append(float(line.split('Manipulability:')[1].strip()))
    elif line.startswith('joint effort:'):
        joint_effort.append(list(map(float, line.split('joint effort:')[1].strip().split())))

robot_time = np.array(robot_time)
joint_positions = np.array(joint_positions)
end_effector_pos = np.array(end_effector_pos)
end_effector_quat = np.array(end_effector_quat)
manipulability = np.array(manipulability)
joint_effort = np.array(joint_effort)

# Removing Outliers from Joint Effort Data
joint_effort_mean = np.mean(joint_effort, axis=0)
joint_effort_std = np.std(joint_effort, axis=0)
valid_effort_indices = np.all(np.abs(joint_effort - joint_effort_mean) <= 3 * joint_effort_std, axis=1)

robot_time = robot_time[valid_effort_indices]
joint_effort = joint_effort[valid_effort_indices]
end_effector_pos = end_effector_pos[valid_effort_indices]

# Calculating Mean Effort at Each Time Step
mean_effort_per_time = np.mean(joint_effort, axis=1)

# Finding Maximum Effort Across All Joints and Time Steps
max_joint_effort = np.max(joint_effort)

# Displaying Results
print('Mean Effort at Each Time Step:')
for time, effort in zip(robot_time, mean_effort_per_time):
    print(f'Time {time:.4f} - Mean Effort: {effort:.4f}')

print(f'Maximum Effort Across All Joints and Time Steps: {max_joint_effort:.4f}')

# Plotting Joint Efforts
plt.figure()
for i in range(joint_effort.shape[1]):
    plt.plot(robot_time, joint_effort[:, i], label=f'Joint {i+1}')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Effort')
plt.title('Joint Efforts over Time')
plt.ylim([-25, 5])
plt.grid(True)
plt.show()

# Plotting Mean Effort Over Time
plt.figure()
plt.plot(robot_time, mean_effort_per_time, 'k', label='Mean Joint Effort')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Mean Effort')
plt.title('Mean Joint Effort Over Time')
plt.ylim([-25, 5])
plt.grid(True)
plt.show()

# Plotting End-Effector Position
plt.figure()
plt.plot(robot_time, end_effector_pos[:, 0], 'r', label='X Position')
plt.plot(robot_time, end_effector_pos[:, 1], 'g', label='Y Position')
plt.plot(robot_time, end_effector_pos[:, 2], 'b', label='Z Position')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('End-Effector Position over Time')
plt.ylim([-25, 5])
plt.grid(True)
plt.show()
