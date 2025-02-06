#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh
export GAZEBO_MASTER_URI=http://localhost:11346

roslaunch anafi_description anafi_in_world.launch

#Add here the launch command of your own world
#roslaunch your_ros_package your_launch_file.launch

#Use this launch command to launch the airship simulation
# roslaunch blimp_description sphinx_blimp_gcs_wind.launch




