#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

// For geometry_msgs::msg::Pose (if not already included in task_builder.hpp)
#include <geometry_msgs/msg/pose.hpp>

#include "task_builder.hpp"

enum class CommandKind {
  CLEAR_SCENE,
  REMOVE_OBJECT,
  SPAWN_OBJECT,
  CHOOSE_PIPELINE,
  JOINTS_MOVE,
  ABSOLUTE_MOVE,
  DISPLACEMENT_MOVE,
  TRAJECTORY_MOVE,
  FEEDBACK_MOVE,
  COLLABORATIVE_MOVE,
  GRIPPER_CLOSE,
  GRIPPER_OPEN,
  ATTACH_OBJECT,
  DETACH_OBJECT,
  DELETE_JSON_SIM_CONTENT,
  SCAN_LINE,           // 1) Scan an object with a line trajectory
  CALIBRATE_CAMERA,    // 2) Calibrate camera with 20 positions
  GCODE_TO_TRAJECTORY, // 3) Convert G-code to trajectory

  UNKNOWN
};

// Extend parseCommand to handle the 3 new commands
static CommandKind parseCommand(const std::string& cmd)
{
  if (cmd == "clear_scene")               return CommandKind::CLEAR_SCENE;
  if (cmd == "remove_object")             return CommandKind::REMOVE_OBJECT;
  if (cmd == "spawn_object")              return CommandKind::SPAWN_OBJECT;
  if (cmd == "choose_pipeline")           return CommandKind::CHOOSE_PIPELINE;
  if (cmd == "joints_move")               return CommandKind::JOINTS_MOVE;
  if (cmd == "absolute_move")             return CommandKind::ABSOLUTE_MOVE;
  if (cmd == "displacement_move")         return CommandKind::DISPLACEMENT_MOVE;
  if (cmd == "trajectory_move")           return CommandKind::TRAJECTORY_MOVE;
  if (cmd == "feedback_move")             return CommandKind::FEEDBACK_MOVE;
  if (cmd == "collaborative_move")        return CommandKind::COLLABORATIVE_MOVE;
  if (cmd == "gripper_close")             return CommandKind::GRIPPER_CLOSE;
  if (cmd == "gripper_open")              return CommandKind::GRIPPER_OPEN;
  if (cmd == "attach_object")             return CommandKind::ATTACH_OBJECT;
  if (cmd == "detach_object")             return CommandKind::DETACH_OBJECT;
  if (cmd == "delete_json_sim_content")   return CommandKind::DELETE_JSON_SIM_CONTENT;
  if (cmd == "scan_line")                 return CommandKind::SCAN_LINE;
  if (cmd == "calibrate_camera")          return CommandKind::CALIBRATE_CAMERA;
  if (cmd == "gcode_to_trajectory")       return CommandKind::GCODE_TO_TRAJECTORY;

  return CommandKind::UNKNOWN;
}

/**
 * Example helper function to linearly interpolate between two poses.
 * You might want a more sophisticated interpolation (e.g., SLERP for orientation).
 */
static geometry_msgs::msg::Pose interpolatePose(
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::Pose& end,
    double t)
{
  geometry_msgs::msg::Pose result;
  // Linear interpolation for position
  result.position.x = start.position.x + t * (end.position.x - start.position.x);
  result.position.y = start.position.y + t * (end.position.y - start.position.y);
  result.position.z = start.position.z + t * (end.position.z - start.position.z);

  // Linear interpolation for orientation (not SLERP, just naive approach)
  result.orientation.x = start.orientation.x + t * (end.orientation.x - start.orientation.x);
  result.orientation.y = start.orientation.y + t * (end.orientation.y - start.orientation.y);
  result.orientation.z = start.orientation.z + t * (end.orientation.z - start.orientation.z);
  result.orientation.w = start.orientation.w + t * (end.orientation.w - start.orientation.w);

  return result;
}

/**
 * Example helper function to parse G-code lines and generate a set of poses.
 * This is just a stub to show how you might implement it; modify as needed.
 */
static std::vector<geometry_msgs::msg::Pose> parseGCodeFile(const std::string& filename)
{
  std::vector<geometry_msgs::msg::Pose> path;
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open G-code file: " << filename << std::endl;
    return path;
  }

  // For example, read lines of G-code like: "G1 X10.0 Y20.0 Z5.0"
  // Then convert them to geometry_msgs::msg::Pose.
  // This is a simplistic example:
  std::string line;
  double x = 0, y = 0, z = 0;
  // Let's keep orientation fixed for demonstration
  while (std::getline(file, line)) {
    // A naive parse: find X, Y, Z values if present
    // e.g. line = "G1 X10.0 Y20.0 Z5.0 F800"
    std::stringstream ss(line);
    std::string token;
    while (ss >> token) {
      if (token.rfind("X", 0) == 0) {
        x = std::stod(token.substr(1));
      } else if (token.rfind("Y", 0) == 0) {
        y = std::stod(token.substr(1));
      } else if (token.rfind("Z", 0) == 0) {
        z = std::stod(token.substr(1));
      }
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    // Keep some fixed orientation, e.g. pointing down
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    path.push_back(pose);
  }

  file.close();
  return path;
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

    case CommandKind::JOINTS_MOVE:
    {
      // joints_move <j1> <j2> <j3> <j4> <j5> <j6>
      if (argc < 10) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: joints_move <j1> <j2> <j3> <j4> <j5> <j6>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<double> joint_values;
      for (int i = 4; i < 10; ++i) {
        joint_values.push_back(std::stod(argv[i]));
      }
      builder.jointsMove(joint_values);
      break;
    }

    case CommandKind::ABSOLUTE_MOVE:
    {
      // absolute_move <frame_id> [x y z rx ry rz rw]
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "absolute_move requires at least a frame_id");
        rclcpp::shutdown();
        return 1;
      }
      std::string frame_id = argv[4];
      // If exactly 5 args => pass default
      if (argc == 5) {
        builder.absoluteMove(frame_id);
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
        builder.absoluteMove(frame_id, x, y, z, rx, ry, rz, rw);
      }
      else {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: absolute_move <frame_id> [x y z rx ry rz rw]");
        rclcpp::shutdown();
        return 1;
      }
      break;
    }

    case CommandKind::DISPLACEMENT_MOVE:
    {
      // displacement_move <x> <y> <z>
      if (argc < 7) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: displacement_move <x> <y> <z>");
        rclcpp::shutdown();
        return 1;
      }
      std::vector<double> move_vector;
      for (int i = 4; i < 7; ++i) {
        move_vector.push_back(std::stod(argv[i]));
      }
      builder.displacementMove(move_vector);
      break;
    }

    case CommandKind::TRAJECTORY_MOVE:
    {
      // trajectory_move <pose1_x> <pose1_y> <pose1_z> <pose1_rx> <pose1_ry> <pose1_rz> <pose1_rw>
      //                ... (repeat for more poses) ...
      // Assuming 7 arguments per pose
      if ((argc - 4) % 7 != 0 || argc < 11) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: trajectory_move <p1_x> <p1_y> <p1_z> <p1_rx> <p1_ry> <p1_rz> <p1_rw> ...");
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
      builder.feedbackMove();
      break;
    }

    case CommandKind::COLLABORATIVE_MOVE:
    {
      // 
      // builder.feedbackMove();
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
      // attach_object <object_name> <link_name>
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
      // detach_object <object_name> <link_name>
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

    case CommandKind::SCAN_LINE:
    {
      // Example usage:
      //   scan_line <start_x> <start_y> <start_z> <start_rx> <start_ry> <start_rz> <start_rw>
      //             <end_x>   <end_y>   <end_z>   <end_rx>   <end_ry>   <end_rz>   <end_rw>
      //             <num_steps>
      // That is 1 command + 14 pose values + 1 integer => 16 arguments after <arm_group_name> <tip_frame> <command>
      if (argc < 19) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: scan_line <sx> <sy> <sz> <srx> <sry> <srz> <srw> <ex> <ey> <ez> <erx> <ery> <erz> <erw> <num_steps>");
        rclcpp::shutdown();
        return 1;
      }
      geometry_msgs::msg::Pose start_pose, end_pose;
      start_pose.position.x = std::stod(argv[4]);
      start_pose.position.y = std::stod(argv[5]);
      start_pose.position.z = std::stod(argv[6]);
      start_pose.orientation.x = std::stod(argv[7]);
      start_pose.orientation.y = std::stod(argv[8]);
      start_pose.orientation.z = std::stod(argv[9]);
      start_pose.orientation.w = std::stod(argv[10]);

      end_pose.position.x = std::stod(argv[11]);
      end_pose.position.y = std::stod(argv[12]);
      end_pose.position.z = std::stod(argv[13]);
      end_pose.orientation.x = std::stod(argv[14]);
      end_pose.orientation.y = std::stod(argv[15]);
      end_pose.orientation.z = std::stod(argv[16]);
      end_pose.orientation.w = std::stod(argv[17]);

      int num_steps = std::stoi(argv[18]);
      if (num_steps < 2) num_steps = 2; // at least 2 points

      // Build a trajectory from start to end
      std::vector<geometry_msgs::msg::Pose> scan_trajectory;
      scan_trajectory.reserve(num_steps);

      for (int i = 0; i < num_steps; ++i) {
        double t = static_cast<double>(i) / (num_steps - 1);
        scan_trajectory.push_back(interpolatePose(start_pose, end_pose, t));
      }

      // Perform the trajectory
      builder.trajectoryMove(scan_trajectory);
      break;
    }

    case CommandKind::CALIBRATE_CAMERA:
    {
      // Example usage:
      //   calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>
      // Then we generate 20 different positions (poses) that revolve or offset
      // around this base pose but keep the same "look-at" point.
      if (argc < 11) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>");
        rclcpp::shutdown();
        return 1;
      }

      geometry_msgs::msg::Pose center_pose;
      center_pose.position.x    = std::stod(argv[4]);
      center_pose.position.y    = std::stod(argv[5]);
      center_pose.position.z    = std::stod(argv[6]);
      center_pose.orientation.x = std::stod(argv[7]);
      center_pose.orientation.y = std::stod(argv[8]);
      center_pose.orientation.z = std::stod(argv[9]);
      center_pose.orientation.w = std::stod(argv[10]);

      // We'll define 20 poses around this center_pose.
      // For simplicity, let's move in a small circle in XY-plane
      // but keep orientation pointing to the same "look-at" point.
      std::vector<geometry_msgs::msg::Pose> calibration_poses;
      calibration_poses.reserve(20);

      double radius = 0.05; // 5 cm offset in a circle
      for (int i = 0; i < 20; ++i) {
        double angle = (2.0 * M_PI) * (static_cast<double>(i) / 20.0);

        geometry_msgs::msg::Pose p = center_pose;
        // Slightly offset the X/Y around the center
        p.position.x += radius * std::cos(angle);
        p.position.y += radius * std::sin(angle);

        // Orientation logic: we can keep the same orientation
        // or you can recalculate so that the end-effector "points" to center.
        // We'll keep the same orientation for this example.

        calibration_poses.push_back(p);
      }

      builder.trajectoryMove(calibration_poses);
      break;
    }

    case CommandKind::GCODE_TO_TRAJECTORY:
    {
      // Example usage:
      //   gcode_to_trajectory <gcode_filename>
      // We'll parse the G-code lines, convert them to a list of poses,
      // then run trajectoryMove.
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: gcode_to_trajectory <gcode_filename>");
        rclcpp::shutdown();
        return 1;
      }

      std::string gcode_file = argv[4];
      auto gcode_poses = parseGCodeFile(gcode_file);

      if (gcode_poses.empty()) {
        RCLCPP_ERROR(node->get_logger(),
                     "No valid trajectory from G-code file: %s", gcode_file.c_str());
        rclcpp::shutdown();
        return 1;
      }

      builder.trajectoryMove(gcode_poses);
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
