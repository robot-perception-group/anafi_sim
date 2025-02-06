import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


data = pd.read_csv("sim_data/moveto_demo_extended_move.csv")
time = (data["time"] - data["time"].iloc[0])/10**9
x_anafi = data["x_anafi"]    
wp_x = data["wp_x"]


# Create the figure and subplots
fig, ax1 = plt.subplots(figsize=(10, 2))









# Plot x-coordinate and wp_x
line0, = ax1.plot(time, data["x_anafi"], color='darkorchid', label='Anafi Drone ')
line1, = ax1.plot(time, data["wp_x"], color='black',  label='Waypoint ')
ax1.set_title('Waypoint Tracking (x-Direction)')
ax1.set_ylabel('x [m]')
# ax1.tick_params(axis="y",labelcolor="b")
ax1.grid()

ax2=ax1.twinx()
line2, = ax2.plot(time,np.rad2deg(data["pitch_anafi"]),color="r",label="Pitch angle",alpha=0.5)
ax2.set_ylabel("Pitch [deg]",color="r")
ax2.tick_params(axis="y",labelcolor="r")

#Create legend 
lines = [line0,line1, line2]
labels = [line.get_label() for line in lines]
ax1.legend(lines,labels)

plt.savefig("pics/moveto.pdf", format="pdf", bbox_inches="tight")

# Adjust layout
plt.tight_layout()

# Show the plot
plt.show()
