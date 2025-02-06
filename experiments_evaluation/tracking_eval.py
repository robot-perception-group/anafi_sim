import matplotlib.pyplot as plt
import numpy as np
import pandas as pd



data = pd.read_csv("sim_data/z_rpm_2_2_pid.csv")
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
x_diff_anafi = wp_x-x_anafi
y_diff_anafi = wp_y-y_anafi
z_diff_anafi = wp_z-z_anafi
xyz_diff = np.vstack((x_diff_anafi,y_diff_anafi,z_diff_anafi))
dist = np.linalg.norm(xyz_diff,axis=0)


x_diff_abs_max = np.max(np.abs(x_diff_anafi))
y_diff_abs_max = np.max(np.abs(y_diff_anafi))
z_diff_abs_max = np.max(np.abs(z_diff_anafi))
dist_abs_max = np.max(np.abs(dist))

# print("x_diff_abs_max  =",x_diff_abs_max)
# print("y_diff_abs_max  =",y_diff_abs_max)
# print("z_diff_abs_max =",z_diff_abs_max)
print("max dist        =",dist_abs_max)
print("mean dist       =",dist.mean())
print("std  dist       =",dist.std())




# Plot x-coordinate and wp_x
axs[0].plot(time, x_sphinx, color='b', label='x_sphinx')
axs[0].plot(time, x_anafi, color='darkorchid', label='x_anafi')
axs[0].plot(time, wp_x, color='c', linestyle='--', label='wp_x')
axs[0].set_ylabel('X Coordinate')
axs[0].set_title('X')
axs[0].legend()
axs[0].grid()


# Plot y-coordinate and wp_y
axs[1].plot(time, y_sphinx, color='g', label='y_sphinx')
axs[1].plot(time, y_anafi, color='yellowgreen', label='y_anafi')
axs[1].plot(time, wp_y, color='lime', linestyle='--', label='wp_y')
axs[1].set_ylabel('Y Coordinate')
axs[1].set_title('Y')
axs[1].legend()
axs[1].grid()


# Plot z-coordinate and wp_z
axs[2].plot(time, z_sphinx, color='r', label='z_sphinx')
axs[2].plot(time, z_anafi, color='orangered', label='z_anafi')
axs[2].plot(time, wp_z, color='m', linestyle='--', label='wp_z')
axs[2].set_ylabel('Z Coordinate')
axs[2].set_title('Z')
axs[2].set_xlabel('Time (s)')
axs[2].legend()
axs[2].grid()



fig, axserr = plt.subplots(3, 1, figsize=(8, 10), sharex=True)

# Plot x-coordinate and wp_x
axserr[0].plot(time, x_sphinx-wp_x, color='b', label='sphinx x_error')
axserr[0].plot(time, x_anafi-wp_x, color='darkorchid', label='anafi x_error')
axserr[0].set_ylabel('X [m]')
axserr[0].set_title('X')
axserr[0].legend()
axserr[0].grid()

# Plot y-coordinate and wp_y
axserr[1].plot(time, y_sphinx-wp_y, color='g', label='sphinx y_error')
axserr[1].plot(time, y_anafi-wp_y, color='yellowgreen', label='anafi y_error')
axserr[1].set_ylabel('Y [m]')
axserr[1].set_title('Y')
axserr[1].legend()
axserr[1].grid()

# Plot z-coordinate and wp_z
axserr[2].plot(time, z_sphinx-wp_z, color='r', label='sphinx z_error')
axserr[2].plot(time, z_anafi-wp_z, color='orangered', label='anafi z_error')
axserr[2].set_ylabel('Z [m]')
axserr[2].set_title('Z')
axserr[2].set_xlabel('Time (s)')
axserr[2].legend()
axserr[2].grid()


# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()

