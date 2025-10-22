import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import MultipleLocator



#Inpit
data_0 = pd.read_csv("sim_data/circular/circular_3_3_cempc.csv")
data_1 = pd.read_csv("sim_data/circular/circular_3_5_cempc.csv")


#Calculation
time_0 = (data_0["time"]-data_0["time"].iloc[0])/10**9
time_1 = (data_1["time"]-data_1["time"].iloc[0])/10**9
#Calculate period duration
T0 = time_0.iloc[-1]
T1 = time_1.iloc[-1]
#Calculate normalized time
nt0 = time_0/T0
nt1 = time_1/T1

print("T0: ",T0,2*np.pi)
print("T1: ",T1,2*np.pi/(1/3))


#Visualization

# Create a figure with two vertically stacked subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=False)

# First subplot
ax1.plot(nt0, data_0["x_anafi"], label='$x$', linestyle = "dashed",color='blue')
ax1.plot(nt0, data_0["y_anafi"], label='$y$', linestyle = "dashed",color='red')
ax1.set_title('Circular Trajectory $r=3m$, $v=3m/s$')
ax1.set_ylabel('Position [m]')
ax1_right = ax1.twinx()
ax1_right.plot(nt0, np.rad2deg(data_0["pitch_anafi"]), label='pitch', linestyle = "solid",color='blue')
ax1_right.plot(nt0, np.rad2deg(data_0["roll_anafi"]), label='roll', linestyle = "solid",color='red')
ax1.legend()
ax1_right.legend()
ax1_right.set_ylabel("Roll / Pitch [°]")
ax1_right.set_ylim([-35,35])
ax1_right.grid(True)

ax1_right.yaxis.set_major_locator(MultipleLocator(10))   # every 2 units on x-axis
ax1.set_xlabel("$t/T$ $[-]$")




# Second subplot
ax2.plot(nt1, data_1["x_anafi"], label='$x$', linestyle = "dashed",color='blue')
ax2.plot(nt1, data_1["y_anafi"], label='$y$', linestyle = "dashed",color='red')
ax2.set_title('Circular Trajectory $r=3m$, $v=5m/s$')
ax2.set_ylabel('Position [m]')
ax2_right = ax2.twinx()
ax2_right.plot(nt1, np.rad2deg(data_1["pitch_anafi"]), label='pitch', linestyle = "solid",color='blue')
ax2_right.plot(nt1, np.rad2deg(data_1["roll_anafi"]), label='roll', linestyle = "solid",color='red')
ax2.legend()
ax2_right.legend()
ax2_right.grid(True)
ax2_right.set_ylabel("Roll / Pitch [°]")
ax2_right.set_ylim([-35,35])
ax2_right.yaxis.set_major_locator(MultipleLocator(10))   # every 2 units on x-axis
ax2.set_xlabel("$t/T$ $[-]$")







# Adjust layout for better spacing
plt.tight_layout()
plt.savefig("attitude.pdf")
plt.show()

