#Source workspace
source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch sphinx_with_gazebo sphinx_publish_body_fixed_frame.launch drone_name:=$DRONE_NAME