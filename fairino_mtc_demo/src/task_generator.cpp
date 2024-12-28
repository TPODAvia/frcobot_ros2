#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "task_builder.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("task_generator_node", options);

  //    argv[1] => arm_group_name
  //    argv[2] => tip_frame
  //    argv[3] => command
  if (argc < 4) {
    RCLCPP_ERROR(node->get_logger(), 
                 "Usage: task_generator <arm_group_name> <tip_frame> <command> [args...]");
    rclcpp::shutdown();
    return 1;
  }

  // Extract arguments
  std::string arm_group_name = argv[1];
  std::string tip_frame      = argv[2];
  std::string command        = argv[3];

  if (!node->has_parameter("robot_description")) {
    RCLCPP_WARN(node->get_logger(), "Parameter 'robot_description' not found! Did you load the MoveIt config?");
  } else {
    std::string urdf_string = node->get_parameter("robot_description").as_string();
    RCLCPP_INFO(node->get_logger(), "Loaded robot_description: %zu characters", urdf_string.size());
  }

  // Similarly for "robot_description_semantic", "joint_limits", etc.
  // - depends on your usage in MTC

  // Create your MTC builder with the node + chosen group/tip
  TaskBuilder builder(node, arm_group_name, tip_frame);

  // Start a new MTC Task: "demo_task" is just an arbitrary label
  builder.newTask("demo_task");

  // Dispatch the command
  if (command == "clear_scene") {
    builder.clearScene();

  } else if (command == "remove_object") {
    if (argc < 5) {
      RCLCPP_ERROR(node->get_logger(), 
                   "remove_object requires an object_name (4th argument)");
      rclcpp::shutdown();
      return 1;
    }
    builder.removeObject(argv[4]);

  } else if (command == "spawn_object") {
    // spawn_object hello_box 0 0.3 0.2 0.0 0.383 0.0 0.924
    if (argc < 11) {
      RCLCPP_ERROR(node->get_logger(), 
        "Usage: spawn_object <obj_name> <x> <y> <z> <rx> <ry> <rz> <rw>");
      rclcpp::shutdown();
      return 1;
    }
    std::string obj_name = argv[4];
    double x  = std::stod(argv[5]);
    double y  = std::stod(argv[6]);
    double z  = std::stod(argv[7]);
    double rx = std::stod(argv[8]);
    double ry = std::stod(argv[9]);
    double rz = std::stod(argv[10]);
    double rw = std::stod(argv[11]);
    builder.spawnObject(obj_name, x, y, z, rx, ry, rz, rw);

  } else if (command == "choose_pipeline") {
    // choose_pipeline OMPL RRTConnect
    if (argc < 6) {
      RCLCPP_ERROR(node->get_logger(), "Usage: choose_pipeline <pipeline_name> <planner_id>");
      rclcpp::shutdown();
      return 1;
    }
    builder.choosePipeline(argv[4], argv[5]);

  } else if (command == "end_coordinate") {
    // end_coordinate tf_end -0.33 0.63 0.11 -0.01 0.709 -0.01 0.705
    if (argc < 5) {
      RCLCPP_ERROR(node->get_logger(), 
                   "end_coordinate requires at least a frame_id");
      rclcpp::shutdown();
      return 1;
    }
    std::string frame_id = argv[4];
    // if fewer than 5 => just pass default
    if (argc == 5) {
      builder.endCoordinate(frame_id);
    } else if (argc >= 12) {
      double x  = std::stod(argv[5]);
      double y  = std::stod(argv[6]);
      double z  = std::stod(argv[7]);
      double rx = std::stod(argv[8]);
      double ry = std::stod(argv[9]);
      double rz = std::stod(argv[10]);
      double rw = std::stod(argv[11]);
      builder.endCoordinate(frame_id, x, y, z, rx, ry, rz, rw);
    } else {
      RCLCPP_ERROR(node->get_logger(), 
                   "Usage: end_coordinate <frame_id> [x y z rx ry rz rw]");
      rclcpp::shutdown();
      return 1;
    }

  } else if (command == "attach_object") {
    if (argc < 6) {
      RCLCPP_ERROR(node->get_logger(), 
                   "Usage: attach_object <object_name> <link_name>");
      rclcpp::shutdown();
      return 1;
    }
    builder.attachObject(argv[4], argv[5]);

  } else if (command == "gripper_close") {
    builder.gripperClose();

  } else if (command == "gripper_open") {
    builder.gripperOpen();

  } else if (command == "detach_object") {
    if (argc < 6) {
      RCLCPP_ERROR(node->get_logger(), 
                   "Usage: detach_object <object_name> <link_name>");
      rclcpp::shutdown();
      return 1;
    }
    builder.detachObject(argv[4], argv[5]);

  } else if (command == "delete_json_sim_content") {
    if (argc < 5) {
      RCLCPP_ERROR(node->get_logger(), 
                   "Usage: delete_json_sim_content <filename>");
      rclcpp::shutdown();
      return 1;
    }
    builder.deleteJsonSimContent(argv[4]);

  } else {
    RCLCPP_ERROR(node->get_logger(), "Unknown command: %s", command.c_str());
    rclcpp::shutdown();
    return 1;
  }

  // 7) Once we've added MTC stages for the chosen command, plan & execute the Task
  if (!builder.initTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to init MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (!builder.planTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (!builder.executeTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to execute MTC task");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Task complete!");
  rclcpp::shutdown();
  return 0;
}
