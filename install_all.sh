# Install dependencies 
sudo apt update && sudo apt upgrade -y
sudo apt install libeigen3-dev -y
sudo apt install ros-humble-smach ros-humble-smach-ros -y

# Colcon update
sudo apt install python3-colcon-common-extensions -y

# Pip update
python3 -m pip install --upgrade pip

# SeupTools current version
python3 -m pip install setuptools==58.1.0

# Source Ros2
source /opt/ros/humble/setup.bash

# Rosdep update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build dependencies
colcon build --symlink-install --packages-select robot_arm_lib_ros2 task_msg
source install/setup.bash

# Build all
colcon build --symlink-install --packages-ignore sim_ros2_interface
source install/setup.bash