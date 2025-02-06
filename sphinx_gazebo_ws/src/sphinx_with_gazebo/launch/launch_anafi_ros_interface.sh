source $ROS_PROJECT_ROOT/other_files/setup.bash
source $ROS_PROJECT_ROOT/other_files/activate_training_venv.sh

roslaunch olympe_bridge anafi.launch ip:='10.202.0.1' model:='4k' skycontroller:=false