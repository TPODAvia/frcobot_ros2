#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>

#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

#include "task_builder.hpp"  // Your MTC TaskBuilder interface

using json = nlohmann::json;

// Helper to check if a JSON object has exactly one key at the top level
static std::string getSingleCommandKey(const json& j)
{
  if (!j.is_object() || j.size() != 1)
    return std::string();
  // Return the (only) key in this object
  for (auto it = j.begin(); it != j.end(); ++it)
    return it.key();
  return std::string();
}

// Helper to parse a joint positions object: e.g.
//   "positions": { "j1": 0.47, "j2": -1.25, ... }
static std::vector<double> parseJointPositions(const json& positions, size_t expected_count = 6)
{
  // If you know your robot has 6 joints, parse j1..j6 in that order:
  std::vector<double> result;
  result.reserve(expected_count);
  // For example, parse in order j1, j2, j3, j4, j5, j6:
  double j1 = positions.value("j1", 0.0);
  double j2 = positions.value("j2", 0.0);
  double j3 = positions.value("j3", 0.0);
  double j4 = positions.value("j4", 0.0);
  double j5 = positions.value("j5", 0.0);
  double j6 = positions.value("j6", 0.0);
  result.push_back(j1);
  result.push_back(j2);
  result.push_back(j3);
  result.push_back(j4);
  result.push_back(j5);
  result.push_back(j6);
  return result;
}

// Main executor
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // ------------------------------------------------------------------
  // Basic usage:
  // task_executor <arm_group_name> <tip_frame> <exec_task> <json_file> <velocity> <accel> <tolerance> <gripper>
  //
  //   - arm_group_name: e.g. "manipulator"
  //   - tip_frame: e.g. "tool0"
  //   - exec_task: "true" or "false" (whether to physically execute)
  //   - json_file: path to the JSON array file
  //   - velocity: e.g. "1.0"
  //   - accel: e.g. "1.0"
  //   - tolerance: e.g. "0.01"
  //   - gripper: (optional) name or usage string
  // ------------------------------------------------------------------
  if (argc < 5) {
    std::cout << "Usage: task_executor <arm_group_name> <tip_frame> <exec_task> <json_file> "
                 "<velocity> <accel> <tolerance> <gripper>\n";
    rclcpp::shutdown();
    return 1;
  }

  // Parse arguments
  std::string arm_group_name   = argv[1];
  std::string tip_frame        = argv[2];
  std::string exec_task        = argv[3];  // "true" or "false"
  std::string json_file        = argv[4];
  double velocity_scale        = (argc > 5) ? std::stod(argv[5]) : 1.0;
  double accel_scale           = (argc > 6) ? std::stod(argv[6]) : 1.0;
  double pose_tolerance        = (argc > 7) ? std::stod(argv[7]) : 0.01;
  std::string gripper_name     = (argc > 8) ? argv[8] : "";

  // Load the JSON array
  if (!std::filesystem::exists(json_file)) {
    std::cerr << "Error: JSON file not found: " << json_file << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  json tasks_array;
  try {
    std::ifstream f(json_file);
    f >> tasks_array;
  } catch (std::exception& e) {
    std::cerr << "Error reading/parsing JSON file: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // Sanity check: must be a JSON array
  if (!tasks_array.is_array()) {
    std::cerr << "Error: top-level JSON is not an array.\n";
    rclcpp::shutdown();
    return 1;
  }

  // Create an MTC builder
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("task_executor_node", options);

  // Create a single MTC TaskBuilder
  TaskBuilder builder(node, arm_group_name, tip_frame);
  // Start a new MTC Task
  builder.newTask("demo_task");

  // Optionally, you could choose a pipeline here if you want a default:
  //   builder.choosePipeline("OMPL", "RRTConnect", velocity_scale, accel_scale);

  // ------------------------------------------------------------------
  // Loop through each entry in the array
  // Each entry is an object with a single numeric timestamp key:
  // {
  //   "<timestamp>": {
  //      "spawn_object": { ... }   // or "remove_object" or "joints_move", ...
  //   }
  // }
  // ------------------------------------------------------------------

  for (auto& entry : tasks_array) 
  {
    if (!entry.is_object()) {
      std::cerr << "Warning: skipping a non-object entry\n";
      continue;
    }

    // Each entry has a single key: the timestamp string
    // e.g. "1739453995.189439"
    // The corresponding value is the actual command object
    // e.g. { "spawn_object": { "cylinder": { "x":0.25, "y":0.25, "z":0.45 } } }
    for (auto& [timestamp_str, command_obj] : entry.items()) {

      if (!command_obj.is_object()) {
        std::cerr << "Warning: skipping timestamp " << timestamp_str
                  << " because command_obj is not an object\n";
        continue;
      }

      // We expect exactly 1 of these possible keys: 
      //   "spawn_object", "remove_object", "clear_scene", "joints_move",
      //   "gripper_open", "gripper_close", "attach_object", "detach_object", "choose_pipeline", ...
      // Let's check them one by one.

      if (command_obj.contains("clear_scene")) {
        // "clear_scene": { "clear_scene": 1 }
        std::cout << "[" << timestamp_str << "] Clear scene\n";
        builder.clearScene();
      }
      else if (command_obj.contains("remove_object")) {
        // "remove_object": { "object_name": 1 }
        auto& remove_data = command_obj["remove_object"];
        // Typically there's a single key: the object name
        for (auto& [obj_name, val] : remove_data.items()) {
          std::cout << "[" << timestamp_str << "] Remove object: " << obj_name << "\n";
          builder.removeObject(obj_name);
        }
      }
      else if (command_obj.contains("spawn_object")) {
        // "spawn_object": {
        //   "<object_name>": { "x":..., "y":..., "z":..., ... }
        // }
        auto& spawn_data = command_obj["spawn_object"];
        for (auto& [obj_name, pose_data] : spawn_data.items()) {
          // For simplicity, parse x,y,z only (like your JSON example).
          double x = pose_data.value("x", 0.0);
          double y = pose_data.value("y", 0.0);
          double z = pose_data.value("z", 0.0);
          // If you want orientation/size, parse them here as well
          std::cout << "[" << timestamp_str << "] Spawn object: " 
                    << obj_name << " at (" << x << ", " << y << ", " << z << ")\n";
          builder.spawnObject(obj_name, obj_name, x, y, z, 0, 0, 0, 1, 0.05, 0.05, 0.05);
        }
      }
      else if (command_obj.contains("joints_move")) {
        // "joints_move": {
        //   "positions": { "j1": 0.47, "j2": -1.25, ... }
        // }
        auto& jm = command_obj["joints_move"];
        if (jm.contains("positions")) {
          auto joint_values = parseJointPositions(jm["positions"], 6);
          std::cout << "[" << timestamp_str << "] Joints move -> [";
          for (size_t i = 0; i < joint_values.size(); ++i)
            std::cout << joint_values[i] << (i + 1 < joint_values.size() ? ", " : "");
          std::cout << "]\n";
          builder.jointsMove(joint_values);
        }
      }
      else if (command_obj.contains("choose_pipeline")) {
        // "choose_pipeline": {
        //   "OMPL": "RRTConnect"
        // }
        auto& pipe_data = command_obj["choose_pipeline"];
        for (auto& [pipeline_name, planner_id] : pipe_data.items()) {
          std::cout << "[" << timestamp_str << "] choose_pipeline: " 
                    << pipeline_name << " / " << planner_id << "\n";
          builder.choosePipeline(pipeline_name, planner_id, velocity_scale, accel_scale);
        }
      }
      else if (command_obj.contains("gripper_open")) {
        // "gripper_open": { "gripper_open": 0.0 }
        std::cout << "[" << timestamp_str << "] Gripper open\n";
        builder.gripperOpen();
      }
      else if (command_obj.contains("gripper_close")) {
        // "gripper_close": { "gripper_close": 1.0 }
        std::cout << "[" << timestamp_str << "] Gripper close\n";
        builder.gripperClose();
      }
      else if (command_obj.contains("attach_object")) {
        // "attach_object": { "<object_name>": "<link_name>" }
        auto& attach_data = command_obj["attach_object"];
        for (auto& [obj_name, link_name] : attach_data.items()) {
          std::cout << "[" << timestamp_str << "] Attach object: "
                    << obj_name << " to link: " << link_name << "\n";
          builder.attachObject(obj_name, link_name);
        }
      }
      else if (command_obj.contains("detach_object")) {
        // "detach_object": { "<object_name>": "<link_name>" }
        auto& detach_data = command_obj["detach_object"];
        for (auto& [obj_name, link_name] : detach_data.items()) {
          std::cout << "[" << timestamp_str << "] Detach object: "
                    << obj_name << " from link: " << link_name << "\n";
          builder.detachObject(obj_name, link_name);
        }
      }
      else {
        std::cerr << "[" << timestamp_str << "] Unknown command in JSON. Skipping.\n";
      }
    } // end of for each [timestamp, command_obj]
  } // end of for each entry in tasks_array

  // Once all commands are added to the pipeline, initialize + plan + optionally execute
  if (!builder.initTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize MTC task");
    rclcpp::shutdown();
    return 1;
  }
  if (!builder.planTask()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan MTC task");
    rclcpp::shutdown();
    return 1;
  }

  if (exec_task == "true") {
    if (!builder.executeTask()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to execute MTC task");
      rclcpp::shutdown();
      return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Task Executor complete!");
  rclcpp::shutdown();
  return 0;
}
