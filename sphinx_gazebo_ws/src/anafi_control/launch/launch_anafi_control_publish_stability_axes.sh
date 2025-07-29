#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch anafi_control anafi_control_publish_stability_axes.launch drone_name:=$DRONE_NAME