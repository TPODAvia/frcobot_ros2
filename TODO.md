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


The log indicates that MoveIt2 detected a collision during execution, so it stopped the trajectory. Here’s what is happening:

- **Invalid Waypoint:**  
  The message  
  > "Trajectory component 'plan' is invalid for waypoint 0 out of 144"  
  means that the very first waypoint in the planned trajectory is now considered invalid.

- **Collision Detection:**  
  MoveIt2’s collision detection found a contact between an `<octomap>` object and the robot’s hand link (`hand_hand`). This indicates that the environment (or the octomap representing it) is reporting a collision with your robot's end-effector.

- **Scene Update:**  
  After the collision check, MoveIt2 determined that the execution path is no longer valid. The planning scene appears to have changed (or the update revealed a collision), which led to cancelling the motion for safety.

### What You Can Do

1. **Review the Planning Scene in RViz:**  
   - Check the collision objects (especially the octomap) to see if they are accurately representing the environment.
   - Verify that the robot's collision geometry is correct.

2. **Adjust Collision Margins:**  
   - Sometimes the default collision margins are too conservative. You might want to tweak these settings in your MoveIt2 configuration if the collision is a false positive.

3. **Validate the Octomap Data:**  
   - Ensure that the octomap is updated correctly. Sensor noise or misconfigured octomap parameters can cause the planning scene to erroneously detect collisions.

4. **Synchronize Models:**  
   - Make sure the robot model used for planning (collision geometry) matches the physical dimensions in simulation. Discrepancies here can lead to unexpected collisions.

In summary, your robot stops because the collision detection system identified an unsafe condition (collision between `hand_hand` and an object in the octomap), making the planned trajectory invalid. Addressing the collision objects and adjusting your planning scene parameters should help resolve the issue.