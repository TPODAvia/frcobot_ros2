# depth_camera_sync

A ROS 2 package that subscribes to `/camera_depth/depth/image_raw` and `/camera_depth/depth/camera_info`, synchronizes them if enabled, and republishes them under a new topic prefix `/sync/`.

## Overview

The node in this package, `depth_sync_node`, supports two operational modes controlled by the parameter `enable_sync`:

- **Synchronized Mode (default):**  
  Uses an approximate time synchronizer to pair the depth image with its corresponding camera info. Both messages are then republished preserving their original timestamps under:
  - `/sync/camera_depth/depth/image_raw`
  - `/sync/camera_depth/depth/camera_info`

- **Pass-Through Mode:**  
  If synchronization is disabled, the node simply forwards each message as received under the `/sync/` namespace without any processing.

## Parameters

- **enable_sync (bool, default: true):**  
  Enables or disables approximate time synchronization.  
  - `true`: Synchronize the image and camera info topics.  
  - `false`: Forward each topic independently.

## Subscribed Topics

- `/camera_depth/depth/image_raw` (`sensor_msgs/msg/Image`)
- `/camera_depth/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)

## Published Topics

- `/sync/camera_depth/depth/image_raw` (`sensor_msgs/msg/Image`)
- `/sync/camera_depth/depth/camera_info` (`sensor_msgs/msg/CameraInfo`)

## Building the Package

Make sure you have sourced your ROS 2 Humble workspace. Then, build the package with:

```bash
colcon build --packages-select depth_camera_sync
source install/setup.bash
```

## Running the Node

### Synchronized Mode (default)
```bash
ros2 run depth_camera_sync depth_sync_node --ros-args -p enable_sync:=true
```

### Pass-Through Mode
```bash
ros2 run depth_camera_sync depth_sync_node --ros-args -p enable_sync:=false
```

## License

Specify your license here (e.g., Apache-2.0).