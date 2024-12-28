#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include "task_builder.hpp"

enum class CommandKind {
  CLEAR_SCENE,
  REMOVE_OBJECT,
  SPAWN_OBJECT,
  CHOOSE_PIPELINE,
  JOINTS_POSITION,
  END_COORDINATE,
  VECTOR_MOVE,
  TRAJECTORY_MOVE,
  FEEDBACK_MOVE,
  GRIPPER_CLOSE,
  GRIPPER_OPEN,
  ATTACH_OBJECT,
  DETACH_OBJECT,
  DELETE_JSON_SIM_CONTENT,
  UNKNOWN
};

static CommandKind parseCommand(const std::string& cmd)
{
  if (cmd == "clear_scene")               return CommandKind::CLEAR_SCENE;
  if (cmd == "remove_object")             return CommandKind::REMOVE_OBJECT;
  if (cmd == "spawn_object")              return CommandKind::SPAWN_OBJECT;
  if (cmd == "choose_pipeline")           return CommandKind::CHOOSE_PIPELINE;
  if (cmd == "joints_position")           return CommandKind::JOINTS_POSITION;
  if (cmd == "end_coordinate")            return CommandKind::END_COORDINATE;
  if (cmd == "vector_move")               return CommandKind::VECTOR_MOVE;
  if (cmd == "trajectory_move")           return CommandKind::TRAJECTORY_MOVE;
  if (cmd == "feedback_move")             return CommandKind::FEEDBACK_MOVE;
  if (cmd == "gripper_close")             return CommandKind::GRIPPER_CLOSE;
  if (cmd == "gripper_open")              return CommandKind::GRIPPER_OPEN;
  if (cmd == "attach_object")             return CommandKind::ATTACH_OBJECT;
  if (cmd == "detach_object")             return CommandKind::DETACH_OBJECT;
  if (cmd == "delete_json_sim_content")   return CommandKind::DELETE_JSON_SIM_CONTENT;
  return CommandKind::UNKNOWN;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("task_generator_node", options);

  // argv[1] => arm_group_name
  // argv[2] => tip_frame
  // argv[3] => command
  if (argc < 4) {
    RCLCPP_ERROR(node->get_logger(), 
                 "Usage: task_generator <arm_group_name> <tip_frame> <command> [args...]");
    rclcpp::shutdown();
    return 1;
  }

  // Extract arguments
  std::string arm_group_name = argv[1];
  std::string tip_frame      = argv[2];
  std::string command_str    = argv[3];

  if (!node->has_parameter("robot_description")) {
    RCLCPP_WARN(node->get_logger(), 
                "Parameter 'robot_description' not found! Did you load the MoveIt config?");
  } else {
    std::string urdf_string = node->get_parameter("robot_description").as_string();
    RCLCPP_INFO(node->get_logger(), "Loaded robot_description: %zu characters", urdf_string.size());
  }

  // Create your MTC builder
  TaskBuilder builder(node, arm_group_name, tip_frame);
  builder.newTask("demo_task"); // just an arbitrary label

  // Switch on the parsed command
  switch (parseCommand(command_str))
  {
    case CommandKind::CLEAR_SCENE:
    {
      builder.clearScene();
      break;
    }

    case CommandKind::REMOVE_OBJECT:
    {
      // "remove_object <object_name>"
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "remove_object requires an object_name (4th argument)");
        rclcpp::shutdown();
        return 1;
      }
      builder.removeObject(argv[4]);
      break;
    }

    case CommandKind::SPAWN_OBJECT:
    {
      // spawn_object object 1 1 1 0 0 0 1
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
      break;
    }

    case CommandKind::CHOOSE_PIPELINE:
    {
      // choose_pipeline <pipeline_name> <planner_id>
      if (argc < 6) {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: choose_pipeline <pipeline_name> <planner_id>");
        rclcpp::shutdown();
        return 1;
      }
      builder.choosePipeline(argv[4], argv[5]);
      break;
    }

    case CommandKind::JOINTS_POSITION:
    {
      // joints_position <j1> <j2> <j3> <j4> <j5> <j6>
      if (argc < 10) {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: joints_position <j1> <j2> <j3> <j4> <j5> <j6>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<double> joint_values;
      for (int i = 4; i < 10; ++i) {
        joint_values.push_back(std::stod(argv[i]));
      }
      builder.jointsPosition(joint_values);
      break;
    }


    case CommandKind::END_COORDINATE:
    {
      // end_coordinate <frame_id> [x y z rx ry rz rw]
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(), 
                     "end_coordinate requires at least a frame_id");
        rclcpp::shutdown();
        return 1;
      }
      std::string frame_id = argv[4];
      // If exactly 5 args => pass default
      if (argc == 5) {
        builder.endCoordinate(frame_id);
      } 
      // If >= 12 => parse the pose
      else if (argc >= 12) {
        double x  = std::stod(argv[5]);
        double y  = std::stod(argv[6]);
        double z  = std::stod(argv[7]);
        double rx = std::stod(argv[8]);
        double ry = std::stod(argv[9]);
        double rz = std::stod(argv[10]);
        double rw = std::stod(argv[11]);
        builder.endCoordinate(frame_id, x, y, z, rx, ry, rz, rw);
      } 
      else {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: end_coordinate <frame_id> [x y z rx ry rz rw]");
        rclcpp::shutdown();
        return 1;
      }
      break;
    }


    case CommandKind::VECTOR_MOVE:
    {
      // vector_move <x> <y> <z>
      if (argc < 7) {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: vector_move <x> <y> <z>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<double> move_vector;
      for (int i = 4; i < 7; ++i) {
        move_vector.push_back(std::stod(argv[i]));
      }
      builder.vectorMove(move_vector);
      break;
    }

    case CommandKind::TRAJECTORY_MOVE:
    {
      // trajectory_move <pose1_x> <pose1_y> <pose1_z> <pose1_rx> <pose1_ry> <pose1_rz> <pose1_rw> ... <poseN_x> <poseN_y> <poseN_z> <poseN_rx> <poseN_ry> <poseN_rz> <poseN_rw>
      // Assuming 7 arguments per pose
      if ((argc - 4) % 7 != 0 || argc < 11) {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: trajectory_move <pose1_x> <pose1_y> <pose1_z> <pose1_rx> <pose1_ry> <pose1_rz> <pose1_rw> ... <poseN_x> <poseN_y> <poseN_z> <poseN_rx> <poseN_ry> <poseN_rz> <poseN_rw>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<geometry_msgs::msg::Pose> trajectory;
      int num_poses = (argc - 4) / 7;
      for (int p = 0; p < num_poses; ++p) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = std::stod(argv[4 + p * 7]);
        pose.position.y = std::stod(argv[5 + p * 7]);
        pose.position.z = std::stod(argv[6 + p * 7]);
        pose.orientation.x = std::stod(argv[7 + p * 7]);
        pose.orientation.y = std::stod(argv[8 + p * 7]);
        pose.orientation.z = std::stod(argv[9 + p * 7]);
        pose.orientation.w = std::stod(argv[10 + p * 7]);
        trajectory.push_back(pose);
      }
      builder.trajectoryMove(trajectory);
      break;
    }

    case CommandKind::FEEDBACK_MOVE:
    {
      // feedback_move
      // No additional arguments
      builder.feedbackMove();
      break;
    }

    case CommandKind::GRIPPER_CLOSE:
    {
      builder.gripperClose();
      break;
    }

    case CommandKind::GRIPPER_OPEN:
    {
      builder.gripperOpen();
      break;
    }

    case CommandKind::ATTACH_OBJECT:
    {
      // attach_object object tip_frame
      if (argc < 6) {
        RCLCPP_ERROR(node->get_logger(), 
                     "Usage: attach_object <object_name> <link_name>");
        rclcpp::shutdown();
        return 1;
      }
      builder.attachObject(argv[4], argv[5]);
      break;
    }

    case CommandKind::DETACH_OBJECT:
    {
      // detach_object object tip_frame
      if (argc < 6) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: detach_object <object_name> <link_name>");
        rclcpp::shutdown();
        return 1;
      }
      builder.detachObject(argv[4], argv[5]);
      break;
    }

    case CommandKind::DELETE_JSON_SIM_CONTENT:
    {
      // delete_json_sim_content <filename>
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: delete_json_sim_content <filename>");
        rclcpp::shutdown();
        return 1;
      }
      builder.deleteJsonSimContent(argv[4]);
      break;
    }

    default:
    {
      RCLCPP_ERROR(node->get_logger(), "Unknown command: %s", command_str.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }

  // Once we've added MTC stages for the chosen command, plan & execute the Task
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
