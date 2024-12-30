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
  SCAN_LINE,
  CALIBRATE_CAMERA,
  GCODE_MOVE,
  STEP_MOVE,

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
  if (cmd == "gcode_move")                return CommandKind::GCODE_MOVE;
  if (cmd == "step_move")                 return CommandKind::STEP_MOVE;

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

static std::vector<geometry_msgs::msg::Pose> parseStepFile(const std::string& filename)
{
  std::vector<geometry_msgs::msg::Pose> result;
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "parseStepFile: could not open file: " << filename << std::endl;
    return result;
  }

  // We'll store lines in memory to do multiple passes
  std::vector<std::string> lines;
  {
    std::string line;
    while (std::getline(file, line))
      lines.push_back(line);
  }
  file.close();

  // 1) Identify a B-spline definition. For example:
  //   #47=B_SPLINE_CURVE_WITH_KNOTS('',5,(#59,#60,#61, ... ), ...
  //   We'll do a naive search for the first occurrence
  std::string spline_ref; // e.g. "#47"
  std::vector<std::string> cartesian_refs; // e.g. "#59", "#60", ...
  for (auto& l : lines) {
    if (l.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos) {
      // Save the entire line
      spline_ref = l;
      break;
    }
  }
  if (spline_ref.empty()) {
    std::cerr << "No B_SPLINE_CURVE_WITH_KNOTS found in " << filename << std::endl;
    return result;
  }

  // 2) Extract the references to the CARTESIAN_POINTs. 
  //    For example, from:
  //    #47=B_SPLINE_CURVE_WITH_KNOTS('',5,(#59,#60,#61,#62,#63,#64,#65,#66,#67,#68),....
  //    We want "59", "60", "61", etc.
  {
    // naive parse: find "(#"
    auto startPos = spline_ref.find("(#");
    if (startPos == std::string::npos) {
      std::cerr << "Could not parse control points list." << std::endl;
      return result;
    }
    // find the matching closing parenthesis
    auto endPos = spline_ref.find(")", startPos);
    if (endPos == std::string::npos) {
      std::cerr << "Could not parse control points list (no closing parenthesis)." << std::endl;
      return result;
    }
    // substring of "#59,#60,#61,..."
    std::string refs = spline_ref.substr(startPos, endPos - startPos);

    // Now split on commas
    // e.g. "(#59,#60,#61,#62,#63,#64,#65,#66,#67,#68"
    // remove parentheses first
    refs.erase(std::remove(refs.begin(), refs.end(), '('), refs.end());
    refs.erase(std::remove(refs.begin(), refs.end(), ')'), refs.end());
    // now " #59,#60,#61,#62,#63,#64,#65,#66,#67,#68"
    std::stringstream ss(refs);
    std::string token;
    while (std::getline(ss, token, ',')) {
      // token: e.g. "#59"
      // trim spaces
      if (!token.empty() && token.find("#") != std::string::npos) {
        cartesian_refs.push_back(token);
      }
    }
  }
  if (cartesian_refs.empty()) {
    std::cerr << "No cartesian point references found." << std::endl;
    return result;
  }

  // 3) For each reference #xx, find the corresponding line with CARTESIAN_POINT, 
  //    parse out the X, Y, Z. We'll store them in a vector
  //    e.g.: #59=CARTESIAN_POINT('',(-61.5859780503913,-48.5929862720562,42.201469967362));
  struct Point3 {
    double x, y, z;
  };
  std::vector<Point3> control_points;
  for (auto& ref : cartesian_refs) {
    // Find line that starts with ref + "=" and "CARTESIAN_POINT"
    for (auto& l : lines) {
      if (l.rfind(ref + "=", 0) == 0 && 
          l.find("CARTESIAN_POINT") != std::string::npos) {
        // parse out "(-61.5859, -48.5929, 42.2014)"
        auto startP = l.find("(");
        auto endP   = l.find(")", startP+1);
        if (startP == std::string::npos || endP == std::string::npos) {
          continue;
        }
        std::string coords = l.substr(startP+1, endP - (startP+1));
        // coords e.g. "-61.5859,-48.5929,42.2014"
        std::stringstream ssc(coords);
        std::string val;
        std::vector<double> vals;
        while (std::getline(ssc, val, ',')) {
          vals.push_back(std::stod(val));
        }
        if (vals.size() == 3) {
          control_points.push_back({vals[0], vals[1], vals[2]});
        }
        break;
      }
    }
  }

  // 4) Minimal check if it is "correct to handle" 
  if (control_points.size() < 2) {
    std::cerr << "Not enough control points to form a curve." << std::endl;
    return result;
  }

  // 5) Create a (potentially) B-spline. 
  //    For demonstration, we can use a simple Eigen::Spline or just 
  //    linearly sample control points. 
  //    Let's do a naive uniform sampling:
  
  //    If you want a real B-spline interpolation, you’d do something like:
  //      Eigen::MatrixXd data(3, control_points.size());
  //      for (size_t i=0; i<control_points.size(); i++){
  //        data(0,i) = control_points[i].x;
  //        data(1,i) = control_points[i].y;
  //        data(2,i) = control_points[i].z;
  //      }
  //      auto spline = Eigen::SplineFitter<double>::Interpolate(data, 3);
  //      // Then sample the spline
  //
  //    For brevity, let's do a simpler approach: 
  int num_samples = 20; 
  for (int i = 0; i < num_samples; ++i) {
    double t = (double)i / (num_samples - 1); // range [0..1]
    // scale that to index in [0..control_points.size()-1]
    double float_index = t * (control_points.size() - 1);

    // integer portion:
    int idx0 = (int)std::floor(float_index);
    int idx1 = std::min(idx0 + 1, (int)control_points.size()-1);
    double ratio = float_index - (double)idx0;

    // linear interpolation between idx0 and idx1
    double x = control_points[idx0].x + ratio * (control_points[idx1].x - control_points[idx0].x);
    double y = control_points[idx0].y + ratio * (control_points[idx1].y - control_points[idx0].y);
    double z = control_points[idx0].z + ratio * (control_points[idx1].z - control_points[idx0].z);

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    // Just use an identity orientation (adjust as needed)
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    result.push_back(pose);
  }

  return result;
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
      // Option 1) user calls:
      //   trajectory_move p1_x p1_y p1_z p1_rx p1_ry p1_rz p1_rw ...
      //
      // Option 2) user calls (with --use-vel):
      //   trajectory_move --use-vel p1_x p1_y p1_z p1_rx p1_ry p1_rz p1_rw p1_vx p1_vy p1_vz ...
      //
      // Decide which parsing approach to use
      bool use_velocities = false;
      int start_index = 4; 
      if (std::string(argv[4]) == "--use-vel") {
        use_velocities = true;
        start_index = 5;
      }

      if (!use_velocities)
      {
        // ------- Original positions-only parsing -------
        // Must have multiples of 7 from `start_index`
        if (((argc - start_index) % 7 != 0) || (argc - start_index) < 7) {
          RCLCPP_ERROR(node->get_logger(),
                      "Usage (positions only): trajectory_move [--use-vel] <p1_x> <p1_y> <p1_z> <p1_rx> <p1_ry> <p1_rz> <p1_rw> ...");
          rclcpp::shutdown();
          return 1;
        }
        std::vector<geometry_msgs::msg::Pose> trajectory;
        int num_poses = (argc - start_index) / 7;
        for (int p = 0; p < num_poses; ++p) {
          geometry_msgs::msg::Pose pose;
          pose.position.x    = std::stod(argv[start_index + p * 7 + 0]);
          pose.position.y    = std::stod(argv[start_index + p * 7 + 1]);
          pose.position.z    = std::stod(argv[start_index + p * 7 + 2]);
          pose.orientation.x = std::stod(argv[start_index + p * 7 + 3]);
          pose.orientation.y = std::stod(argv[start_index + p * 7 + 4]);
          pose.orientation.z = std::stod(argv[start_index + p * 7 + 5]);
          pose.orientation.w = std::stod(argv[start_index + p * 7 + 6]);
          trajectory.push_back(pose);
        }
        // Original function call
        builder.trajectoryMove(trajectory);
      }
      else
      {
        // ------- Positions + velocities parsing -------
        // We expect 13 values per waypoint: 
        // 7 for pose + 6 for velocity (vx, vy, vz, wx, wy, wz)
        // Adjust to your needs (some prefer linear + angular speeds or something else).
        if (((argc - start_index) % 13 != 0) || (argc - start_index) < 13) {
          RCLCPP_ERROR(node->get_logger(),
                      "Usage (with velocities): trajectory_move --use-vel <p1_x> ... <p1_w> <p1_vx> <p1_vy> <p1_vz> <p1_wx> <p1_wy> <p1_wz> ...");
          rclcpp::shutdown();
          return 1;
        }
        int num_waypoints = (argc - start_index) / 13;
        std::vector<geometry_msgs::msg::Pose>   poses(num_waypoints);
        std::vector<geometry_msgs::msg::Twist>  velocities(num_waypoints);

        for (int i = 0; i < num_waypoints; ++i) {
          int base = start_index + i * 13;
          poses[i].position.x    = std::stod(argv[base + 0]);
          poses[i].position.y    = std::stod(argv[base + 1]);
          poses[i].position.z    = std::stod(argv[base + 2]);
          poses[i].orientation.x = std::stod(argv[base + 3]);
          poses[i].orientation.y = std::stod(argv[base + 4]);
          poses[i].orientation.z = std::stod(argv[base + 5]);
          poses[i].orientation.w = std::stod(argv[base + 6]);

          velocities[i].linear.x  = std::stod(argv[base + 7]);
          velocities[i].linear.y  = std::stod(argv[base + 8]);
          velocities[i].linear.z  = std::stod(argv[base + 9]);
          velocities[i].angular.x = std::stod(argv[base + 10]);
          velocities[i].angular.y = std::stod(argv[base + 11]);
          velocities[i].angular.z = std::stod(argv[base + 12]);
        }

        // Call the new variant that accepts velocities
        builder.trajectoryMoveV(poses, velocities);
      }
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


    case CommandKind::GCODE_MOVE:
    {
      // Example usage:
      //   GCODE_MOVE <gcode_filename>
      // We'll parse the G-code lines, convert them to a list of poses,
      // then run trajectoryMove.
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(),
                     "Usage: GCODE_MOVE <gcode_filename>");
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

    case CommandKind::STEP_MOVE:
    {
      // Usage example: step_move <filename>
      if (argc < 5) {
        RCLCPP_ERROR(node->get_logger(), "Usage: step_move <step_filename>");
        rclcpp::shutdown();
        return 1;
      }
      std::string step_filename = argv[4];
      // We'll create a new function to parse the B-spline(s) from the file:
      auto curve_poses = parseStepFile(step_filename);

      // Basic check
      if (curve_poses.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No valid poses generated from STEP file: %s", step_filename.c_str());
        rclcpp::shutdown();
        return 1;
      }

      // Pass to the normal trajectory move
      builder.trajectoryMove(curve_poses);
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
