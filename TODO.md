- Check the tesk executor node (done)
- Add Callibration pipeline
- Add octomap (done)
- Move to object still working even if clear scene triggered (done)
- Dettach also need to be registered the current position (done)
- Add tool for absolute end tool position and orientation (done)
- Executor code does not do anything (done)
- absolute_move world tip_link box are still now are not moving to the object after detaching it.
- The sensor_3d.yaml does not configure properly (Also added gazebo depth camera in xacro file) (done)
- robot_state_publisher are terminating during launch
- camera_pointcloud not working
- move camera config to a better place and also refining launch pipeline

https://github.com/AndrejOrsula/moveit2_calibration/tree/ros2_port


ERRORS:

[robot_state_publisher-4] Error:   Failed to build tree: parent link [base_link] of joint [camera_joint] not found.  This is not valid according to the URDF spec. Every link you refer to from a joint needs to be explicitly defined in the robot description. To fix this problem you can either remove this joint [camera_joint] from your urdf file, or add "<link name="base_link" />" to your urdf file.
[robot_state_publisher-4]          at line 252 in ./urdf_parser/src/model.cpp
[robot_state_publisher-4] Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
[robot_state_publisher-4] terminate called after throwing an instance of 'std::runtime_error'
[robot_state_publisher-4]   what():  Unable to initialize urdf::model from robot description
