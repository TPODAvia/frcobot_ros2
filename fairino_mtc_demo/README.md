ros2 run fairino_mtc_demo task_generator joints_move 0 -1.57 1.57 0 0 0
ros2 launch fairino_mtc_demo mtc_builder.launch.py
ros2 launch fairino_mtc_demo moveit_sim.launch.py

rosrun moveit_python task_generator.py help
rosrun moveit_python task_generator.py robot get_robot_param
rosrun moveit_python task_generator.py fr10 joints_move
rosrun moveit_python task_generator.py fr10 joints_move 0 0 0 0 0 0
rosrun moveit_python task_generator.py fr10 absolute_move tf_end
rosrun moveit_python task_generator.py fr10 absolute_move hello_box
rosrun moveit_python task_generator.py fr10 spawn_object hello_box 0 0.5 0.2
rosrun moveit_python task_generator.py fr10 attach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 detach_object hello_box tf_end
rosrun moveit_python task_generator.py fr10 remove_object hello_box
rosrun moveit_python task_generator.py fr10 clear_scene
rosrun moveit_python task_generator.py fr10 gripper_open
rosrun moveit_python task_generator.py fr10 gripper_close
rosrun moveit_python task_generator.py fr10 choose_pipeline OMPL RRTConnect
rosrun moveit_python task_generator.py fr10 choose_pipeline PILZ LIN
rosrun moveit_python task_generator.py fr10 choose_follow_mode
rosrun moveit_python task_generator.py fr10 check_json_files
rosrun moveit_python task_generator.py fr10 delete_json_sim_content test.json
rosrun moveit_python task_generator.py fr10 delete_json_temp