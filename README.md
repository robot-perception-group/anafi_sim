# Integrated Multi-Simulation Environments for Aerial Robotics Research

Welcome to the repository providing the code used for the paper 

**Integrated Multi-Simulation Environments for Aerial Robotics Research**

by Pascal Goldschmid and Aamir Ahmad

The paper can be found [here](https://arxiv.org/abs/2502.10218).

Experimental data (log files, videos of real world flights, etc.) can be found [here](https://keeper.mpdl.mpg.de/d/d9aace8256ae41e8878f/).

If you have any questions, comments or suggestions please contact pascal.goldschmid@ifr.uni-stuttgart.de

![Sphinx and Gazebo running in parallel](/sphinx_gazebo.gif)

---


## General

### Scope of the code
The code has been developed with Python 3.8.10 on Ubuntu 20, using ROS Noetic and Gazebo 11 in combination with Sphinx 2.15.1. It provides all necessary scripts to set up the Sphinx simulator jointly with Gazebo to include a mirrored instance of the Anafi drone in a Gazebo-based (multi-robot) environment. Furthermore, it contains all necessary files to deploy the presented controller frameworks in simulation and on real hardware in real flight experiments.

### Structure of the repository
The repository contains two branches, *simulation* and *real_flights*. The branch *simulation* contains data for setting up the simulators Sphinx and Gazebo in parallel to create multi-robot simulations featuring the Anafi drone simulated by Sphinx as described in the paper. The branch *real_flights* only contains files which are required to use this framework for real world experiments.

Each branch  contains two main folders, [sphinx_gazebo_ws](/sphinx_gazebo_ws/) and [experiments_evaluation](/experiments_evaluation/). [sphinx_gazebo_ws](/sphinx_gazebo_ws/) is a catkin workspace which, once built, allows the joint usage of Sphinx with Gazebo (branch *simulation*) or our control frameworks on real hardware (branch *real_flights*). [experiments_evaluation](/experiments_evaluation/) contains the scripts necessary to evaluate the data recorded in either simulation or real world experiments using the logging node (branch *simulation*) or to record the data processed by ROS using rosbag files (branch *real_flights*).

#### Structure of the ROS catkin workspace
The catkin workspace provided in [sphinx_gazebo_ws](/sphinx_gazebo_ws/) contains six different ROS packages.
1. [airship_simulation](/sphinx_gazebo_ws/src/airship_simulation/)
2. [anafi_control](/sphinx_gazebo_ws/src/anafi_control/)
3. [anafi_description](/sphinx_gazebo_ws/src/anafi_description/)
4. [anafi_ros](/sphinx_gazebo_ws/src/anafi_ros)
5. [pid](/sphinx_gazebo_ws/src/pid)
6. [sphinx_with_gazebo](/sphinx_gazebo_ws/src/sphinx_with_gazebo/)
7. [uav_msgs](/sphinx_gazebo_ws/src/uav_msgs/)

A short description of each ROS package is given below.

##### airship_simulation
This ROS package contains a modified version of the code used for [this](https://github.com/robot-perception-group/airship_simulation) project. It allows the simulation of an airship within Gazebo considering numerous real-world effects such as deflation or a non-rigid structure. We use this ROS package to demonstrate how a multi-robot simulation can be set up in Gazebo that includes the Anafi drone by Parrot Drone.

##### anafi_control
In this ROS package, the developed control frameworks are provided. There are two types of control frameworks. The first one is a PID-controller-based framework that leverages four independent (cascaded) control loops to control the longitudinal, lateral, vertical and yaw motion of the Anafi drone independently. The second leverages a model predictive controller (MPC) to mitigate the tracking error which can be observed in the PID-controller-based framework.

##### anafi_description
This ROS package contains the required files to set up in Gazebo a mirrored instance of the Anafi drone based on information retrieved from the Sphinx simulator in which the flight physics of the Anafi drone are simulated. 

##### anafi_ros
This is a slightly modified version of [this](https://github.com/andriyukr/anafi_ros/tree/ros1) ROS package. The modifications comprise mainly a position estimation which is purely based on the visual odometry capabilities of the Anafi drone. This allows testing our code in GPS-denied areas as the visual odometry of the Anafi drone is sufficiently accurate. 


##### pid
A ROS package provided [here](http://wiki.ros.org/pid) to set up (multiple) PID controllers. This package is used to realize the (cascaded) PID controller framework provided by the anafi_control package.

##### sphinx_with_gazebo
This ROS package contains the core functionalities to set up the Gazebo simulator jointly with Sphinx.

##### uav_msgs
This ROS packages contains the definition of ROS messages which are required by the airship simulation.


### Structure of the code
The code is designed in a way that each ROS node is run in a separate virtual screen. To get a list of all virtual screens that are currently active, run
```
screen -list
```
To attach to a screen, use 
```
screen -r SCREEN_NAME
```
To detach from a screen, press ctrl+a and then ctrl+d.

## Requirements
### Linux packages
Install some basic packages first by running the commands
```
sudo apt update
sudo apt install screen
sudo apt install python3-pip
sudo apt install libgoogle-glog-dev 
```
### Downloading the code
The repository includes several submodules that need to be considered during download. This can be achieved with the following command
```
git clone --recurse-submodules https://github.com/robot-perception-group/anafi_sim.git
```

## Installation
### Installing ROS and Gazebo
You can find the installation instructions for ROS Noetic and Gazebo  [here](http://wiki.ros.org/noetic/Installation/Ubuntu).
A list of required ROS packages is given [here](ros_packages_list.txt). You can install a package by using apt as follows
```
sudo apt install ros-noetic-PACKAGE-NAME
```
### Installation of Parrot Sphinx and Olympe
To install the Sphinx simulator, follow the instructions given [here](https://developer.parrot.com/docs/sphinx/installation.html).
To install Olympe, the Python-based API to access the firmware of Anafi drones, follow the instructions provided [here](https://developer.parrot.com/docs/olympe/installation.html).

### Installing Python modules 
A list of required additional python modules is given [here](requirements.txt). You can install it by running
```
pip3 install -r requirements.txt
```


### Initializing the catkin workspaces
Make sure ROS is installed, then run
```
cd ~/anafi_sim/sphinx_gazebo_ws
catkin_make
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
```

## Using the framework
There are several ways to test different components of the framework. 

1. *Only Sphinx*: You can run the Sphinx simulator without Gazebo, e.g. to test the tracking controllers provided in this repository. 
2. *Sphinx with an empty Gazebo environment*: You can run the full method, i.e. Sphinx and Gazebo to test the capabilities of our framework
3. *Sphinx with a populated Gazebo environment*: This allows to include the Anafi drone simulated by Sphinx in any custom Gazebo environment

These approaches are explained in more detail in the following.


#### Only Sphinx
The following comands will launch the Sphinx simulator, the simulated Anafi firmware, the anafi_ros bridge and the interface retrieving the drone state from the Sphinx simulator (ground truth data, i.e. no noise is present).
In a terminal window, run
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
./src/sphinx_with_gazebo/launch/launch_sphinx_mpc_test_environment_in_virtual_screens.sh
```
#### Sphinx with empty Gazebo environment
The following comands will launch the Sphinx simulator jointly with an empty world in Gazebo, the simulated Anafi firmware, the anafi_ros bridge and the interface retrieving the drone state from the Sphinx simulator (ground truth data, i.e. no noise is present).
In a terminal window, run
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
./src/sphinx_with_gazebo/launch/launch_sphinx_with_gazebo_environment_in_virtual_screens.sh
```
#### Sphinx with custom Gazebo environment
To include the anafi drone in your own Gazebo environment, you can modify the file [launch_anafi_in_world.sh](sphinx_gazebo_ws/src/sphinx_with_gazebo/launch/launch_anafi_in_world.sh). Just comment out the  line ```roslaunch anafi_description anafi_in_world.launch``` and replace it with your own launch command. In the .launch file include the following line to make sure that the anafi drone is loaded in your world.
```
	<include file="$(find anafi_description)/launch/spawn_anafi.launch"></include>

```
If you wish to load our airship simulation together with the anafi drone you can use the launch command ```roslaunch blimp_description sphinx_blimp_gcs_wind.launch```. Then, you can launch the simulation as follows.

```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
./src/sphinx_with_gazebo/launch/sphinx_with_gazebo_environment_in_virtual_screens.sh
```

### Launching a tracking controller
#### Preliminaries
Before you can launch one of the proivided tracking controllers, two preliminary steps have to be executed.

To do so, once a simulation environment has been launched, in another terminal window run 
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
rostopic pub /anafi/drone/location_spawn_point_enu geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
point:
  x: -3.3 
  y: 0.0
  z: 0.0" 

```
This is required to shift the spawn point of the drone in Sphinx into the origin of the simulated world, i.e. all waypoints commands sent to the drone will be relative to the true origin of the simulated world instead of the spawn point of the drone.
Furthermore, it is advisable to start publishing waypoints before launching a controller. In order to send a waypoint making the drone hover at a stationary position right after takeoff, run the following commands
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
rostopic pub /anafi/position_control/waypoint anafi_control/Waypoint "{x: 0.0, y: 0.0, z: 1.0, v_x: 0.0, v_y: 0.0, v_z: 0.0, yaw: 0.0}" -r10
```
This will publish a waypoint with a frequency of 10hz that makes the drone hover right above origin of the world in an altitude of 1m.

#### Tracking controller frameworks
Two control frameworks are available
1. A PID-based (cascaded) controller framework
2. A model predictive controller designed to reduce the tracking error between the drone and a moving waypoint.

#### Launching the PID-based controller framework
To launch the PID-based controller framework, run
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
roslaunch anafi_control anafi_controll_all_cascaded_pid_controllers.launch
```
In this framework, four control loops are created that independently control the drone's motion in longitudinal, lateral, and vertical direction as well as its heading, i.e. the yaw angle.
All control loops are in a single-input-single-output configuration where the control loops associated with the longitudinal and lateral direction have a cascaded structure with two cascades. 
#### Launching the model predictive controller framework

To launch the MPC framework, run
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
roslaunch anafi_control anafi_control_waypoint_mpc.launch
```
This launches a MPC to control calculating optimal solutions for the motion in lontitudinal, lateral and vertical direction. The motion around the vertical axis, i.e. the heading and therefore the yaw angle is controlled by the same PID-based control loop than in the PID-based controller framework.

### Log data
In order to log the waypoints sent to the drone as well as the drone's ground truth and estimated position, orientation and velocity data, you can run a logging node using the following commands.
In a terminal window, execute
```
cd ~/anafi_sim/sphinx_gazebo_ws
source ~/anafi_sim/sphinx_gazebo_ws/other_files/setup.bash
roslaunch sphinx_with_gazebo anafi_control_logger.launch
```
You can specifiy a path to where the log files should be saved in the file [anafi_control_logger.py](sphinx_gazebo_ws/src/sphinx_with_gazebo/scripts/anafi_control_logger.py)
The logger begins a new log file as soon as a message is received on the topic "/trajectory_generator/begin". Messages are either sent manually using the command ```rostopic pub /trajectory_generator/begin std_msgs/Bool "data: true"``` or automatically by the [trajectory_generator_node](sphinx_gazebo_ws/src/anafi_control/scripts/trajectory_generator.py).

### Generating trajectories
The [trajectory_generator_node](sphinx_gazebo_ws/src/anafi_control/scripts/trajectory_generator.py) can be used to send waypoints following predefined trajectories. The following trajectories are available:
1. Static waypoints
2. Circular trajectories
3. Rectilinear periodic movements
The trajectories can be set for each direction of motion independently. To specifiy the parameters of a trajectory type for one specific direction of motion, change the following values accordingly given in the init function of the TrajectoryGenerator class defined in the [trajectory_generator_node](sphinx_gazebo_ws/src/anafi_control/scripts/trajectory_generator.py).
```
self.r_x = 0
self.r_y = 0
self.r_z = 1
self.v_x = 0
self.v_y = 0
self.v_z = 0
```
These values define the radii and velocities of the circular and rectilinear periodic trajectories.
When started, the trajectory generator publishes a static waypoint. To activate a moving waypoint, run e.g. the following command.
```
rostopic pub /trajectory_generator/trajectory_type_modifer std_msgs/String "data: 'circle_xy_z:rectilinear_z'"
```
This activates a circular trajectory in the xy-plane with a simultaeous rectilinear motion in the vertical direction. For other trajectory types, please check the code of the [trajectory_generator_node](sphinx_gazebo_ws/src/anafi_control/scripts/trajectory_generator.py).

##### Disclaimer
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
