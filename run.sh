#!/bin/bash
gnome-terminal --title="Zenoh Router" -- bash -c "
source /opt/ros/humble/setup.bash;
echo 'Starting Zenoh Bridge...';
ros2 run rmw_zenoh_cpp rmw_zenohd;
exec bash"

gnome-terminal --title="StoneFish Simulation" -- bash -c "
source /opt/ros/humble/setup.bash;
source install/setup.sh;

echo 'Starting StoneFish Simulation Node...';
ros2 run stonefish_ros2 stonefish_simulator src/stonefish_ros2/data/ src/stonefish_ros2/scenario/waterlinked.scn 24 1000 1000 low;
exec bash"


gnome-terminal --title="StoneFish LaunchTools" -- bash -c "
source /opt/ros/humble/setup.bash;
source install/setup.sh;

echo 'Starting LaunchTools Node...';
ros2 launch launch_tools stonefish_cloud_launch.py;
exec bash"

gnome-terminal --title="StoneFish Keyboard" -- bash -c "
source /opt/ros/humble/setup.bash;
source install/setup.sh;

echo 'Starting LaunchTools Node...';
ros2 run launch_tools my_keyboard;
exec bash" 

