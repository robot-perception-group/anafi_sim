import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


#Data handling of scenario with yaw=0
data_pid_yaw_0 =   pd.read_csv("sim_data/static/static_3_2_5_0_pid.csv")
data_blmpc_yaw_0 =   pd.read_csv("sim_data/static/static_3_2_5_0_blmpc.csv")
data_cempc_yaw_0 =   pd.read_csv("sim_data/static/static_3_2_5_0_cempc.csv")

#Data handling of scenario with yaw=180
data_pid_yaw_180 = pd.read_csv("sim_data/static/static_3_2_5_180_pid.csv")
data_blmpc_yaw_180 = pd.read_csv("sim_data/static/static_3_2_5_180_blmpc.csv")
data_cempc_yaw_180 = pd.read_csv("sim_data/static/static_3_2_5_180_cempc.csv")

#Time extraction of scenario with yaw=0
time_pid_yaw_0 = (data_pid_yaw_0["time"]-data_pid_yaw_0["time"].iloc[0])/10**9
time_cempc_yaw_0 = (data_cempc_yaw_0["time"]-data_cempc_yaw_0["time"].iloc[0])/10**9
time_blmpc_yaw_0 = (data_blmpc_yaw_0["time"]-data_blmpc_yaw_0["time"].iloc[0])/10**9

#Time extraction of scenario with yaw=180
time_pid_yaw_180 = (data_pid_yaw_180["time"]-data_pid_yaw_180["time"].iloc[0])/10**9
time_cempc_yaw_180 = (data_cempc_yaw_180["time"]-data_cempc_yaw_180["time"].iloc[0])/10**9
time_blmpc_yaw_180 = (data_blmpc_yaw_180["time"]-data_blmpc_yaw_180["time"].iloc[0])/10**9



# Create the figure and subplots
major_ticks = np.arange(0, 4, 1)
minor_ticks = np.arange(0, 4, 0.5)
major_ticks_z = np.arange(0, 6, 1)
minor_ticks_z = np.arange(0, 6, 0.5)
major_ticks_yaw = np.arange(0, 190, 30)
minor_ticks_yaw = np.arange(0, 190, 15)
cm = 1/2.54
fig, axs = plt.subplots(4, 1, figsize=(21*cm, 21*cm), sharex=True)



# Plot x-coordinate and wp_x

# #Response yaw = 0
# axs[0].plot(time_pid_yaw_0, data_pid_yaw_0["x_anafi"], color='b', label='$PID_0$')
# axs[0].plot(time_blmpc_yaw_0, data_blmpc_yaw_0["x_anafi"], color='darkorchid', label='$MPC-BL_0$')
# axs[0].plot(time_cempc_yaw_0, data_cempc_yaw_0["x_anafi"], color='darkslategrey', label='$MPC-CE_0$')
#Response yaw = 180
axs[0].plot(time_pid_yaw_180, data_pid_yaw_180["x_anafi"], color='orange', label='$PID$')
axs[0].plot(time_blmpc_yaw_180, data_blmpc_yaw_180["x_anafi"], color='blue', label='$MPC-BL$')
axs[0].plot(time_cempc_yaw_180, data_cempc_yaw_180["x_anafi"], color='darkslategrey', label='$MPC-CE$')
#Waypoint yaw = 0
axs[0].plot(time_blmpc_yaw_0, data_blmpc_yaw_0["wp_x"], color='c', linestyle='--', label='$WP$')
axs[0].set_ylabel('x [m]')
axs[0].set_title('X')
axs[0].set_yticks(major_ticks)
axs[0].set_yticks(minor_ticks, minor=True)
axs[0].grid(which='minor', alpha=0.6)
axs[0].grid(which='major', alpha=1)

handles, labels = axs[0].get_legend_handles_labels()
# sort both labels and handles by labels
# labels, handles = zip(*sorted(zip(labels, handles), key=lambda t: t[0]))
# axs[0].legend(handles, labels,ncol=3,loc = "lower right")
# axs[0].legend(handles, labels, ncol=3,loc = "lower right")

fig.legend(handles, labels, loc= "lower center",ncol = 7)
# # Plot y-coordinate and wp_y
# Waypoint yaw = 180
axs[1].plot(time_cempc_yaw_0, data_cempc_yaw_0["wp_y"], color='c', linestyle='--', label='$WP$')
# #Response yaw = 0
# axs[1].plot(time_pid_yaw_0, data_pid_yaw_0["y_anafi"], color='b', label='$PID_0$')
# axs[1].plot(time_blmpc_yaw_0, data_blmpc_yaw_0["y_anafi"], color='darkorchid', label='$MPC-BL_0$')
# axs[1].plot(time_cempc_yaw_0, data_cempc_yaw_0["y_anafi"], color='darkslategrey', label='$MPC-CE_0$')
#Response yaw = 180
axs[1].plot(time_pid_yaw_180, data_pid_yaw_180["y_anafi"], color='orange', label='$PID$')
axs[1].plot(time_blmpc_yaw_180, data_blmpc_yaw_180["y_anafi"], color='blue', label='$MPC-BL$')
axs[1].plot(time_cempc_yaw_180, data_cempc_yaw_180["y_anafi"], color='darkslategrey', label='$MPC-CE$')
axs[1].set_ylabel('y [m]')
axs[1].set_title('Y')
axs[1].set_yticks(major_ticks)
axs[1].set_yticks(minor_ticks, minor=True)
axs[1].grid(which='minor', alpha=0.6)
axs[1].grid(which='major', alpha=1)





# # Plot z-coordinate and wp_z
#Waypoint yaw = 180
axs[2].plot(time_cempc_yaw_0, data_cempc_yaw_0["wp_z"], color='c', linestyle='--', label='$WP$')
#Response yaw = 180
axs[2].plot(time_pid_yaw_180, data_pid_yaw_180["z_anafi"], color='orange', label='$PID$')
axs[2].plot(time_blmpc_yaw_180, data_blmpc_yaw_180["z_anafi"], color='blue', label='$MPC-BL$')
axs[2].plot(time_cempc_yaw_180, data_cempc_yaw_180["z_anafi"], color='darkslategrey', label='$MPC-CE$')
axs[2].set_ylabel('z [m]')
axs[2].set_title('Z')
# axs[2].legend()
axs[2].set_yticks(major_ticks_z)
axs[2].set_yticks(minor_ticks_z, minor=True)
axs[2].grid(which='minor', alpha=0.6)
axs[2].grid(which='major', alpha=1)

# # Plot yaw-coordinate and wp_z
# axs[3].plot(time_pid_yaw_0,   np.rad2deg(data_pid_yaw_0["yaw_anafi"]), color='b', label='$PID_0$')
# axs[3].plot(time_blmpc_yaw_0,   np.rad2deg(data_blmpc_yaw_0["yaw_anafi"]), color='darkorchid', label='$MPC-BL_0$')
# axs[3].plot(time_cempc_yaw_0,   np.rad2deg(data_cempc_yaw_0["yaw_anafi"]), color='darkorchid', label='$MPC-CE_0$')
# Response yaw = 0
axs[3].plot(time_pid_yaw_180, np.rad2deg(data_pid_yaw_180["yaw_anafi"]), color='orange', label='$PID$')
axs[3].plot(time_blmpc_yaw_180, np.rad2deg(data_blmpc_yaw_180["yaw_anafi"]), color='blue', label='$MPC-BL$')
axs[3].plot(time_cempc_yaw_180, np.rad2deg(data_cempc_yaw_180["yaw_anafi"]), color='darkslategrey', label='$MPC-CE$')
# Response yaw = 180
# axs[3].plot(time_blmpc_yaw_0,   data_blmpc_yaw_0["wp_yaw"], color='c', linestyle='--', label='$WP$')
axs[3].plot(time_cempc_yaw_180, data_cempc_yaw_180["wp_yaw"], color='c', linestyle='--', label='$WP$')
axs[3].set_ylabel('yaw [deg]')
axs[3].set_title('Yaw')
axs[3].set_yticks(major_ticks_yaw)
axs[3].set_yticks(minor_ticks_yaw, minor=True)
axs[3].grid(which='minor', alpha=0.6)
axs[3].grid(which='major', alpha=1)

# axs[0].set_xlabel("t [s]")
# axs[1].set_xlabel("t [s]")
axs[3].set_xlabel("t [s]")



# Adjust layout
# plt.tight_layout()
plt.savefig("pics/static_yaw_180_sim.pdf", format="pdf", bbox_inches="tight")

# Show the plot
plt.show()
