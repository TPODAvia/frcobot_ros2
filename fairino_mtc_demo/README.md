ros2 run fairino_mtc_demo task_generator joints_position 0 -1.57 1.57 0 0 0
ros2 launch fairino_mtc_demo mtc_builder.launch.py
ros2 launch fairino_mtc_demo moveit_sim.launch.py