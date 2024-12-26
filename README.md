# frcobot_ros2
This is the ROS2 API project of Fairino robot(sofeware version must greater than V3.7.1), a serial of functions which based on Fair SDK API but were simplified are created, user can call them through service message.
API_description.md list all API functions.
Tutorial of installing and uasage of ROS2 API, please refer to the Fair document platform:https://fair-documentation.readthedocs.io/en/latest/ROSGuide/index.html#frcobot-ros2.

Version histroy:
2023.7.18 V1.0


https://github.com/fzi-forschungszentrum-informatik/ros2_ros_bt_py.git

https://github.com/moveit/moveit_task_constructor.git


git submodule add https://github.com/ros-drivers/velodyne colcon_ws/src/OrcaRL2/localization/velodyne


$ git config --global user.name "John Doe" $ git config --global user.email hejhe@gmail.com

git config --global user.name "FIRST_NAME LAST_NAME"
git config --global user.email "MY_NAME@example.com"

killall -9 gzserver
killall -9 gzclient


git submodule update --init --recursive
git submodule update --init --recursive --force

cd ~/orca_robot/colcon_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
export MAKEFLAGS="-j2"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release