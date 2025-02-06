import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



data_pid_yaw_0 =   pd.read_csv("real_world_data/static_waypoint_yaw_0_pid.csv")
data_mpc_yaw_0 =   pd.read_csv("real_world_data/static_waypoint_yaw_0_mpc.csv")
data_pid_yaw_180 = pd.read_csv("real_world_data/static_waypoint_yaw_180_pid.csv")
data_mpc_yaw_180 = pd.read_csv("real_world_data/static_waypoint_yaw_180_mpc.csv")

time_pid_yaw_0 = (data_pid_yaw_0["time"]-data_pid_yaw_0["time"].iloc[0])/10**9
time_mpc_yaw_0 = (data_mpc_yaw_0["time"]-data_mpc_yaw_0["time"].iloc[0])/10**9
time_pid_yaw_180 = (data_pid_yaw_180["time"]-data_pid_yaw_180["time"].iloc[0])/10**9
time_mpc_yaw_180 = (data_mpc_yaw_180["time"]-data_mpc_yaw_180["time"].iloc[0])/10**9



# Create the figure and subplots

major_ticks = np.arange(0, 4, 1)
minor_ticks = np.arange(0, 4, 0.5)
major_ticks_z = np.arange(5, 12, 1)
minor_ticks_z = np.arange(5, 12, 0.5)
major_ticks_yaw = np.arange(0, 190, 30)
minor_ticks_yaw = np.arange(0, 190, 15)

cm = 1/2.54
fig, axs = plt.subplots(4, 1, figsize=(20*cm, 15*cm), sharex=True)



# Plot x-coordinate and wp_x
axs[0].plot(time_pid_yaw_0, data_pid_yaw_0["x_anafi"], color='b', label='$PID_0$')
axs[0].plot(time_mpc_yaw_0, data_mpc_yaw_0["x_anafi"], color='darkorchid', label='$MPC_0$')
axs[0].plot(time_mpc_yaw_0, data_mpc_yaw_0["wp_x"], color='c', linestyle='--', label='$WP_{0/180}$')
axs[0].plot(time_pid_yaw_180, data_pid_yaw_180["x_anafi"], color='orange', label='$PID_{180}$')
axs[0].plot(time_mpc_yaw_180, data_mpc_yaw_180["x_anafi"], color='olive', label='$MPC_{180}$')
# axs[0].plot(time_mpc_yaw_180, data_mpc_yaw_180["wp_x"], color='c', linestyle='--', label='WP')
axs[0].set_ylabel('x [m]')
axs[0].set_title('X')
# axs[0].legend()
axs[0].set_yticks(major_ticks)
axs[0].set_yticks(minor_ticks, minor=True)
axs[0].grid(which='minor', alpha=0.6)
axs[0].grid(which='major', alpha=1)


# # Plot y-coordinate and wp_y
axs[1].plot(time_pid_yaw_0, data_pid_yaw_0["y_anafi"], color='b', label='$PID_0$')
axs[1].plot(time_mpc_yaw_0, data_mpc_yaw_0["y_anafi"], color='darkorchid', label='$MPC_0$')
axs[1].plot(time_mpc_yaw_0, data_mpc_yaw_0["wp_y"], color='c', linestyle='--', label='$WP_{0/180}$')
axs[1].plot(time_pid_yaw_180, data_pid_yaw_180["y_anafi"], color='orange', label='$PID_{180}$')
axs[1].plot(time_mpc_yaw_180, data_mpc_yaw_180["y_anafi"], color='olive', label='$MPC_{180}$')
# axs[1].plot(time_mpc_yaw_180, data_mpc_yaw_180["wp_x"], color='c', linestyle='--', label='WP')
axs[1].set_ylabel('y [m]')
axs[1].set_title('Y')
axs[1].set_yticks(major_ticks)
axs[1].set_yticks(minor_ticks, minor=True)
axs[1].grid(which='minor', alpha=0.6)
axs[1].grid(which='major', alpha=1)





# # Plot z-coordinate and wp_z
axs[2].plot(time_pid_yaw_0, data_pid_yaw_0["z_anafi"], color='b', label='$PID_0$')
axs[2].plot(time_mpc_yaw_0, data_mpc_yaw_0["z_anafi"], color='darkorchid', label='$MPC_0$')
axs[2].plot(time_mpc_yaw_0, data_mpc_yaw_0["wp_z"], color='c', linestyle='--', label='$WP_{0/180}$')
axs[2].plot(time_pid_yaw_180, data_pid_yaw_180["z_anafi"], color='orange', label='$PID_{180}$')
axs[2].plot(time_mpc_yaw_180, data_mpc_yaw_180["z_anafi"], color='olive', label='$MPC_{180}$')
# axs[0].plot(time_mpc_yaw_180, data_mpc_yaw_180["wp_x"], color='c', linestyle='--', label='WP')
axs[2].set_ylabel('z [m]')
axs[2].set_title('Z')
# axs[2].legend()
axs[2].set_yticks(major_ticks_z)
axs[2].set_yticks(minor_ticks_z, minor=True)
axs[2].grid(which='minor', alpha=0.6)
axs[2].grid(which='major', alpha=1)

# # Plot yaw-coordinate and wp_z
axs[3].plot(time_pid_yaw_0,   np.rad2deg(data_pid_yaw_0["yaw_anafi"]), color='b', label='$PID_0$')
axs[3].plot(time_mpc_yaw_0,   np.rad2deg(data_mpc_yaw_0["yaw_anafi"]), color='darkorchid', label='$MPC_0$')
axs[3].plot(time_pid_yaw_180, np.rad2deg(data_pid_yaw_180["yaw_anafi"]), color='orange', label='$PID_{180}$')
axs[3].plot(time_mpc_yaw_180, np.rad2deg(data_mpc_yaw_180["yaw_anafi"]), color='olive', label='$MPC_{180}$')
axs[3].plot(time_mpc_yaw_0,   data_mpc_yaw_0["wp_yaw"], color='c', linestyle='--', label='$WP_0$')
axs[3].plot(time_mpc_yaw_180, data_mpc_yaw_180["wp_yaw"], color='c', linestyle='--', label='$WP_{180}$')
axs[3].set_ylabel('yaw [deg]')
axs[3].set_title('Yaw')
# axs[3].legend()
axs[3].set_yticks(major_ticks_yaw)
axs[3].set_yticks(minor_ticks_yaw, minor=True)
axs[3].grid(which='minor', alpha=0.6)
axs[3].grid(which='major', alpha=1)





# Adjust layout
plt.tight_layout()
plt.savefig("pics/static_comparison_real_world.pdf", format="pdf", bbox_inches="tight")

# Show the plot
plt.show()
