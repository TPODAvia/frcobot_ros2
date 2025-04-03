## Usage

1. **Launch** with defaults:

   ```bash
   ros2 launch gripper_hardware gripper_launch.py
   ```

   - This will read parameters from `gripper_params.yaml`.
   - Subscribes on `/tool_input`, publishes on `/tool_output`.
   - Uses QoS = 0 (system default).
   - Uses default gripper configuration (`precision`, `power=0.01`, etc.).

2. **Override** from the command line:

   ```bash
   ros2 launch gripper_hardware gripper_launch.py \
       qos:=1 \
       input_topic:=/my_custom_input \
       output_topic:=/my_custom_output \
       param_file:=gripper_params.yaml
   ```

   - This sets **QoS** to **RELIABLE** (1).
   - Changes subscription to `/my_custom_input` and publication to `/my_custom_output`.
   - Still uses the same parameter file for other gripper parameters.
