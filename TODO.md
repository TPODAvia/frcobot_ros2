- Check the tesk executor node
- Add Callibration pipeline
- Add octomap
- Move to object still working even if clear scene triggered (done)
- Dettach also need to be registered the current position (done)
- Add tool for absolute end tool position and orientation (done)
- Executor code does not do anything
- absolute_move world tip_link box are still now are not moving to the object after detaching it.

https://github.com/AndrejOrsula/moveit2_calibration/tree/ros2_port


vboxuser@Ubuntu22:~/colcon_ws$ ros2 launch fairino_mtc_demo mtc_executor_json.launch.py 
[INFO] [launch]: All log files can be found below /home/vboxuser/.ros/log/2025-03-19-13-13-31-233203-Ubuntu22-65390
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [task_executor-1]: process started with pid [65391]
[task_executor-1] [ERROR] [1742379221.379264082] [task_executor_node]: Could not find parameter robot_description_semantic and did not receive robot_description_semantic via std_msgs::msg::String subscription within 10.000000 seconds.
[task_executor-1] Error:   Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT ErrorID=13 (0xd) Line number=0
[task_executor-1]          at line 732 in ./src/model.cpp
[task_executor-1] [ERROR] [1742379221.383220657] [moveit_rdf_loader.rdf_loader]: Unable to parse SRDF
[task_executor-1] [FATAL] [1742379221.383456026] [move_group_interface]: Unable to construct robot model. Please make sure all needed information is on the parameter server.
[task_executor-1] terminate called after throwing an instance of 'std::runtime_error'
[task_executor-1]   what():  Unable to construct robot model. Please make sure all needed information is on the parameter server.
[ERROR] [task_executor-1]: process has died [pid 65391, exit code -6, cmd '/home/vboxuser/colcon_ws/install/fairino_mtc_demo/lib/fairino_mtc_demo/task_executor manipulator tool0 false /home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json 1.0 1.0 0.01 my_gripper --ros-args'].