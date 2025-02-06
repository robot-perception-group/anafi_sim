import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



# data = pd.read_csv("data/wrong_files/new_mpc_xy_circle_3_1_z_rpm_1_1.csv")
# data = pd.read_csv("data/new_mpc_xy_circle_3_1_z_rpm_1_1.csv")
# data = pd.read_csv("data/pid_static_x_3_y_2_z_5_yaw_180.csv")
# data = pd.read_csv("real_world_data/static_waypoint_yaw_180_pid.csv")
# data = pd.read_csv("real_world_data/circle_3_2_pid.csv")
data = pd.read_csv("sim_data/episode_0.csv")
time = (data["time"] - data["time"].iloc[0])/10**9
x_sphinx = data["x_sphinx"]    
y_sphinx = data["y_sphinx"]    
z_sphinx = data["z_sphinx"]
x_anafi = data["x_anafi"]    
y_anafi = data["y_anafi"]    
z_anafi = data["z_anafi"]

wp_x = data["wp_x"]
wp_y = data["wp_y"]
wp_z = data["wp_z"]

# Create the figure and subplots
fig, axs = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

#Data
x_diff_sphinx = x_sphinx-wp_x
y_diff_sphinx = y_sphinx-wp_y
z_diff_sphinx = z_sphinx-wp_z

x_diff_mean = np.mean(x_diff_sphinx)
y_diff_mean = np.mean(y_diff_sphinx)
z_diff_mean = np.mean(z_diff_sphinx)


print("x_diff_mean =",x_diff_mean)
print("y_diff_mean =",y_diff_mean)
print("z_diff_mean =",z_diff_mean)






# Plot x-coordinate and wp_x
axs[0].plot(time, x_sphinx, color='b', label='x_sphinx')
axs[0].plot(time, x_anafi, color='darkorchid', label='x_anafi')
axs[0].plot(time, data["vx_anafi"], color='red', label='vx_anafi')
axs[0].plot(time, wp_x, color='c', linestyle='--', label='wp_x')
axs[0].set_ylabel('X Coordinate')
axs[0].set_title('X')
axs[0].legend()
axs[0].grid()


# Plot y-coordinate and wp_y
axs[1].plot(time, y_sphinx, color='g', label='y_sphinx')
axs[1].plot(time, y_anafi, color='yellowgreen', label='y_anafi')
axs[1].plot(time, wp_y, color='lime', linestyle='--', label='wp_y')
axs[1].plot(time, data["vy_anafi"], color='red', label='vy_anafi')

axs[1].set_ylabel('Y Coordinate')
axs[1].set_title('Y')
axs[1].legend()
axs[1].grid()


# Plot z-coordinate and wp_z
axs[2].plot(time, z_sphinx, color='r', label='z_sphinx')
axs[2].plot(time, z_anafi, color='orangered', label='z_anafi')
axs[2].plot(time, data["vz_anafi"], color='red', label='vz_anafi')

axs[2].plot(time, wp_z, color='m', linestyle='--', label='wp_z')
axs[2].set_ylabel('Z Coordinate')
axs[2].set_title('Z')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid()



# fig, axserr = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

# # Plot x-coordinate and wp_x
# axserr[0].plot(time, x_sphinx-wp_x, color='b', label='sphinx x_error')
# axserr[0].plot(time, x_anafi-wp_x, color='darkorchid', label='anafi x_error')
# axserr[0].set_ylabel('X [m]')
# axserr[0].set_title('X')
# axserr[0].legend()
# axserr[0].grid()

# # Plot y-coordinate and wp_y
# axserr[1].plot(time, y_sphinx-wp_y, color='g', label='sphinx y_error')
# axserr[1].plot(time, y_anafi-wp_y, color='yellowgreen', label='anafi y_error')
# axserr[1].set_ylabel('Y [m]')
# axserr[1].set_title('Y')
# axserr[1].legend()
# axserr[1].grid()

# # Plot z-coordinate and wp_z
# axserr[2].plot(time, z_sphinx-wp_z, color='r', label='sphinx z_error')
# axserr[2].plot(time, z_anafi-wp_z, color='orangered', label='anafi z_error')
# axserr[2].set_ylabel('Z [m]')
# axserr[2].set_title('Z')
# axserr[2].set_xlabel('Time (s)')
# axserr[2].legend()
# axserr[2].grid()


# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()
