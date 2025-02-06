echo -e "\e[1;33m WARNING: This script will delete the folder of temporary gazebo files located here ~/.gazebo. Proceeding in 8s. Press ctrl+c to abort.\e[0m"
sleep 8

# export GAZEBO_MASTER_URI=http://localhost:11346


if [ -z "$ROS_PROJECT_ROOT" ]; then
  echo "ROS_PROJECT_ROOT is not set. "
  echo "Please run your_workspace_path/other_files/setup.bash"
  exit
else
  echo "ROS_PROJECT_ROOT is set to '$ROS_PROJECT_ROOT'"
fi


SIM_ID="sphinx"

rm -rf ~/.gazebo



screen -d -m -S "$(echo $SIM_ID)_roscore" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_roscore.sh"&
echo "Launched roscore. Waiting 3 seconds to ensure proper start up..."
sleep 3



screen -d -m -S "$(echo $SIM_ID)_sphinx_firmware_interface" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_anafi_firmware_interface.sh 60" &
echo "Launched firmware interface. Waiting 3 seconds to ensure proper start up..."
sleep 3

screen -d -m -S "$(echo $SIM_ID)_sphinx_simulation" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_empty_sphinx_simulation.sh"&
echo "Launched parrot sphinx simulator. Waiting 12 seconds to ensure proper start up..."
sleep 12



screen -d -m -S "$(echo $SIM_ID)_interface" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_sphinx_interface.sh"&
echo "Launched parrot sphinx interface. Waiting 3 seconds to ensure proper start up..."
sleep 3

screen -d -m -S "$(echo $SIM_ID)_publish_stability_axes" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_publish_stability_axes.sh" &
echo "Launched stability axes publisher. Waiting 4 seconds to ensure proper start up..."
sleep 4

screen -d -m -S "$(echo $SIM_ID)_publish_body_fixed_frame" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_publish_body_fixed_frame.sh" &
echo "Launched publisher of body fixed frames. Waiting 4 seconds to ensure proper start up..."
sleep 4

screen -d -m -S "$(echo $SIM_ID)_anafi_in_gazebo" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_anafi_in_world.sh"&
echo "Launched gazebo simulator. Waiting 15 seconds to ensure proper start up..."
sleep 15



screen -d -m -S "$(echo $SIM_ID)_anafi_ros_interface" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_anafi_ros_interface.sh" &
echo "Launched anafi ros interface. Waiting 7 seconds to ensure proper start up..."
sleep 7

#Send take off command to have drone armed and flying
rosservice call /anafi/drone/takeoff "{}"
echo "Sent take off command. Waiting 10s to ensure proper execution..."
sleep 10



screen -d -m -S "$(echo $SIM_ID)_sync_with_blimp_sim" bash -i -c "$(echo $ROS_PROJECT_ROOT)/src/sphinx_with_gazebo/launch/launch_sphinx_update_copter_gazebo_states_from_sphinx.sh"&
echo "Launched sync node."


