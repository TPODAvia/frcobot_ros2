- Check the tesk executor node (done)
- Add octomap (done)
- Move to object still working even if clear scene triggered (done)
- Dettach also need to be registered the current position (done)
- Add tool for absolute end tool position and orientation (done)
- Executor code does not do anything (done)
- The sensor_3d.yaml does not configure properly (Also added gazebo depth camera in xacro file) (done)
- robot_state_publisher are terminating during launch (done)
- camera_pointcloud not working (done)
- move camera config to a better place and also refining launch pipeline (done)
- absolute_move world tip_link box are still now are not moving to the object after detaching it.
- add callibration pipeline
- add gripper pipeline. add use_gripper in the launch

https://github.com/AndrejOrsula/moveit2_calibration/tree/ros2_port


ERRORS:

[robot_state_publisher-4] Error:   Failed to build tree: parent link [base_link] of joint [camera_joint] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [camera_joint] from your urdf file, or add "<link name="base_link" />" to your urdf file.
[robot_state_publisher-4]          at line 252 in ./urdf_parser/src/model.cpp
[robot_state_publisher-4] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-4] terminate called after throwing an instance of 'std::runtime_error'
[robot_state_publisher-4]   what():  Unable to initialize urdf::model from robot description


[rviz2-1] [WARN] [1743533149.332044533] [moveit_ros.planning_scene_monitor.planning_scene_monitor]: Maybe failed to update robot state, time diff: 1743533127.662s
[move_group-3] [WARN] [1743533172.935550154] [move_group]: [image_transport] Topics '/sync/camera_depth/depth/image_raw' and '/sync/camera_depth/depth/camera_info' do not appear to be synchronized. In the last 10s:
[move_group-3] 	Image messages received:      1
[move_group-3] 	CameraInfo messages received: 4
[move_group-3] 	Synchronized pairs:           1


[robot_state_publisher-4] Error:   No link elements found in urdf file
[robot_state_publisher-4]          at line 206 in ./urdf_parser/src/model.cpp
[robot_state_publisher-4] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-4] terminate called after throwing an instance of 'std::runtime_error'
[robot_state_publisher-4]   what():  Unable to initialize urdf::model from robot description
[robot_state_publisher-2] Error:   No link elements found in urdf file
[robot_state_publisher-2]          at line 206 in ./urdf_parser/src/model.cpp
[robot_state_publisher-2] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-2] [WARN] [1743533925.448793700] [robot_state_publisher]: Failed to parse new URDF: Unable to initialize urdf::model from robot description
