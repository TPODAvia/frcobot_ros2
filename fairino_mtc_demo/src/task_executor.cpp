/****************************************************
 * File: task_executor.cpp
 * Description: Reads a JSON array of commands, builds
 *              an MTC Task, and optionally executes it.
 ****************************************************/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <map>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <limits>

#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "task_builder.hpp"  // Your MTC TaskBuilder interface

using json = nlohmann::json;

// ---------------------------------------------------------------
// Optional helper functions for handling JSON files, checking syntax,
// removing objects, etc. as shown in your original snippet.
// ---------------------------------------------------------------

static void removeUnwantedKeys(nlohmann::json& j, const std::vector<std::string>& keys_to_remove)
{
    if (j.is_object()) {
        for (auto it = j.begin(); it != j.end(); ) {
        if (std::find(keys_to_remove.begin(), keys_to_remove.end(), it.key()) != keys_to_remove.end())
            it = j.erase(it);
        else {
            removeUnwantedKeys(it.value(), keys_to_remove);
            ++it;
        }
        }
    } else if (j.is_array()) {
        for (auto& elem : j) {
        removeUnwantedKeys(elem, keys_to_remove);
        }
    }
}

static int handleDeleteJsonSimContent(const std::string& filename, rclcpp::Node::SharedPtr node)
{
    if (!std::filesystem::exists(filename)) {
        RCLCPP_ERROR(node->get_logger(), "No file found: %s", filename.c_str());
        return 1;
    }
    std::filesystem::path p(filename);
    std::filesystem::path mod_file = p.parent_path() / ("mod_" + p.filename().string());
    if (std::filesystem::exists(mod_file)) {
        std::filesystem::remove(mod_file);
        RCLCPP_INFO(node->get_logger(), "Removed existing mod-file: %s", mod_file.string().c_str());
    }
    nlohmann::json data;
    {
        std::ifstream fin(filename);
        if (!fin.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file: %s", filename.c_str());
        return 1;
        }
        try {
        fin >> data;
        } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error parsing JSON: %s", e.what());
        return 1;
        }
    }
    // Just remove certain command keys
    std::vector<std::string> unwanted_keys {
        "remove_object","detach_object","clear_scene","attach_object","spawn_object"
    };
    removeUnwantedKeys(data, unwanted_keys);

    if (data.is_array()) {
        nlohmann::json filtered = nlohmann::json::array();
        for (auto& item : data) {
        if (!item.is_object() || !item.empty()) {
            filtered.push_back(item);
        }
        }
        data = filtered;
    }
    {
        std::ofstream fout(mod_file);
        if (!fout.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open: %s", mod_file.string().c_str());
        return 1;
        }
        fout << data.dump(2) << std::endl;
    }
    RCLCPP_INFO(node->get_logger(), "delete_json_sim_content finished.");
    return 0;
}

static int handleDeleteJsonTemp(const std::string& directory, rclcpp::Node::SharedPtr node)
{
    std::filesystem::path test_file     = std::filesystem::path(directory) / "test.json";
    std::filesystem::path mod_test_file = std::filesystem::path(directory) / "mod_test.json";

    if (std::filesystem::exists(test_file)) {
        std::filesystem::remove(test_file);
        RCLCPP_INFO(node->get_logger(), "Removed: %s", test_file.string().c_str());
    }
    if (std::filesystem::exists(mod_test_file)) {
        std::filesystem::remove(mod_test_file);
        RCLCPP_INFO(node->get_logger(), "Removed: %s", mod_test_file.string().c_str());
    }
    RCLCPP_INFO(node->get_logger(), "delete_json_temp finished.");
    return 0;
    }

    static int handleCheckJsonFiles(const std::string& directory, rclcpp::Node::SharedPtr node)
    {
    if (!std::filesystem::exists(directory) || !std::filesystem::is_directory(directory)) {
        RCLCPP_ERROR(node->get_logger(), "Invalid directory: %s", directory.c_str());
        return 1;
    }
    std::vector<std::string> valid_json_files;
    std::vector<std::string> invalid_json_files;
    for (auto& entry : std::filesystem::directory_iterator(directory)) {
        if (entry.is_regular_file()) {
        std::filesystem::path fpath = entry.path();
        if (fpath.extension() == ".json") {
            std::ifstream fin(fpath.string());
            if (!fin.is_open()) {
            invalid_json_files.push_back(fpath.filename().string());
            continue;
            }
            try {
            nlohmann::json data;
            fin >> data;
            valid_json_files.push_back(fpath.filename().string());
            } catch (const std::exception& e) {
            invalid_json_files.push_back(fpath.filename().string());
            RCLCPP_WARN(node->get_logger(),
                        "Invalid JSON in file: %s - Error: %s",
                        fpath.filename().string().c_str(),
                        e.what());
            }
        }
        }
    }
    std::cout << "Directory: " << directory << "\n\n";
    std::cout << "Valid JSON files:\n";
    if (valid_json_files.empty()) {
        std::cout << "  [WARNING] No valid JSON files found!\n";
    } else {
        for (auto& f : valid_json_files) {
        std::cout << "  " << f << "\n";
        }
    }
    std::cout << "\nInvalid JSON files:\n";
    if (invalid_json_files.empty()) {
        std::cout << "  [WARNING] No invalid JSON files found!\n";
    } else {
        for (auto& f : invalid_json_files) {
        std::cout << "  " << f << "\n";
        }
    }
    std::cout << "check_json_files finished!\n";
    return 0;
    }

    // For demonstration only; if you want to parse real numeric args from JSON
    // we can define a check for numeric strings or just rely on standard conversions.
    static bool isNumeric(const std::string& s)
    {
    try {
        std::size_t pos;
        std::stod(s, &pos);  
        if (pos != s.size())
        return false;
        return true;
    } catch (...) {
        return false;
    }
}

// ---------------------------------------------------------------
// Helper: parse a JSON object for 6-joint positions in order j1..j6.
// Example input JSON snippet for "joints_move":
//  "joints_move": {
//    "positions": {
//      "j1": 1.57,
//      "j2": 0.1,
//      "j3": -1.2,
//      "j4": 0.0,
//      "j5": 0.0,
//      "j6": 0.0
//    }
//  }
// ---------------------------------------------------------------
static std::vector<double> parseJointPositions(const json& positions, size_t expected_count = 6)
{
    std::vector<double> result;
    result.reserve(expected_count);

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

// ---------------------------------------------------------------
// Main Executor
//   task_executor <arm_group_name> <tip_frame> <exec_task> <json_file>
//                 <velocity_scale> <accel_scale> <pose_tolerance> <virtual_base>
//
//  - <exec_task> is "true" or "false"
//  - <json_file> is the path to your JSON array
//  - The final 3 are optional numeric parameters for pipeline usage, etc.
//  - <virtual_base> "true" or "false"
// ---------------------------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    if (argc < 5) {
        std::cerr << "Usage:\n"
                << "  task_executor <arm_group> <tip_frame> <exec_task> <json_file> "
                << "[velocity_scale] [accel_scale] [pose_tolerance] [virtual_base]\n";
        rclcpp::shutdown();
        return 1;
    }

    // Parse arguments
    std::string arm_group_name = argv[1];
    std::string tip_frame      = argv[2];
    std::string exec_task      = argv[3];  // "true" or "false"
    std::string json_file      = argv[4];

    double velocity_scale   = (argc > 5) ? std::stod(argv[5]) : 1.0;
    double accel_scale      = (argc > 6) ? std::stod(argv[6]) : 1.0;
    double pose_tolerance   = (argc > 7) ? std::stod(argv[7]) : 0.01;
    bool   virtual_base     = (argc > 8) && (std::string(argv[8]) == "true");

    // Check the JSON file
    if (!std::filesystem::exists(json_file)) {
        std::cerr << "Error: JSON file not found: " << json_file << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Load the JSON array
    json tasks_array;
    try {
        std::ifstream f(json_file);
        f >> tasks_array;
    } catch (const std::exception& e) {
        std::cerr << "Error reading/parsing JSON: " << e.what() << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Must be an array
    if (!tasks_array.is_array()) {
        std::cerr << "Error: top-level JSON is not an array.\n";
        rclcpp::shutdown();
        return 1;
    }

    // Create node & builder
    rclcpp::NodeOptions node_opts;
    node_opts.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("task_executor_node", node_opts);

    TaskBuilder builder(node, arm_group_name, tip_frame);
    builder.newTask("json_task");

    // If you want a plane as "virtual base"
    builder.spawn_virtual_base(virtual_base);

    // Optionally: pick a pipeline by default, or parse from JSON. E.g.:
    // builder.choosePipeline("ompl","RRTConnect",velocity_scale,accel_scale,pose_tolerance);

    // ------------------------------------------------------------------
    // Loop over each element in the JSON array. Each element is expected
    // to have a single timestamp key -> an object with 1 command inside.
    // Example element:
    // {
    //   "1713337825.8489513": {
    //       "joints_move": {
    //         "positions": { "j1": 0.47, "j2": -1.25, ... }
    //       }
    //   }
    // }
    // ------------------------------------------------------------------
    for (auto& entry : tasks_array)
    {
        if (!entry.is_object()) {
        std::cerr << "[Warning] Skipping a non-object entry in the array.\n";
        continue;
        }
        // Each entry has a single top-level key: the timestamp
        for (auto& [timestamp_str, command_obj] : entry.items()) {

        if (!command_obj.is_object()) {
            std::cerr << "[Warning] Command object for timestamp " << timestamp_str
                    << " is not an object. Skipping.\n";
            continue;
        }

        // 1) get_robot_param
        if (command_obj.contains("get_robot_param")) {
            std::cout << "[" << timestamp_str << "] get_robot_param\n";
            // Just prints robot info
            builder.printRobotParams();
        }
        // 2) clear_scene
        else if (command_obj.contains("clear_scene")) {
            std::cout << "[" << timestamp_str << "] clear_scene\n";
            builder.clearScene();
        }
        // 3) remove_object
        else if (command_obj.contains("remove_object")) {
            auto& rm_data = command_obj["remove_object"];
            for (auto& [obj_name, val] : rm_data.items()) {
            std::cout << "[" << timestamp_str << "] remove_object: " << obj_name << "\n";
            builder.removeObject(obj_name);
            }
        }
        // 4) spawn_object
        else if (command_obj.contains("spawn_object")) {
            auto& spawn_data = command_obj["spawn_object"];
            // Expect something like: "spawn_object": { "box1": { "x":0.2, "y":0.3, "z":0.1 } }
            for (auto& [obj_name, pose_data] : spawn_data.items()) {
            double x  = pose_data.value("x", 0.0);
            double y  = pose_data.value("y", 0.0);
            double z  = pose_data.value("z", 0.0);
            // If you want orientation or shape dims from JSON, parse them here, e.g.:
            double rx = pose_data.value("rx", 0.0);
            double ry = pose_data.value("ry", 0.0);
            double rz = pose_data.value("rz", 0.0);
            double rw = pose_data.value("rw", 1.0);
            double da = pose_data.value("da", 0.05);
            double db = pose_data.value("db", 0.05);
            double dc = pose_data.value("dc", 0.05);

            builder.spawnObject(obj_name, /*shape=*/obj_name, x, y, z, rx, ry, rz, rw, da, db, dc);
            std::cout << "[" << timestamp_str << "] spawn_object: " << obj_name
                        << " at (" << x << "," << y << "," << z << ")\n";
            }
        }
        // 5) choose_pipeline
        else if (command_obj.contains("choose_pipeline")) {
            auto& pipe_data = command_obj["choose_pipeline"];
            // E.g.: { "OMPL": "RRTConnect", "max_vel_factor":0.2, "max_acc_factor":0.2, "tolerance":0.01 }
            for (auto& [pipeline_name, planner_id_val] : pipe_data.items()) {
            // skip special fields
            if (pipeline_name == "max_vel_factor" ||
                pipeline_name == "max_acc_factor" ||
                pipeline_name == "tolerance")
                continue;

            std::string planner_id = planner_id_val.get<std::string>();
            double max_vel = pipe_data.value("max_vel_factor", velocity_scale);
            double max_acc = pipe_data.value("max_acc_factor", accel_scale);
            double tol     = pipe_data.value("tolerance", pose_tolerance);

            builder.savePipelineConfig(pipeline_name, planner_id, max_vel, max_acc, tol);
            builder.choosePipeline(pipeline_name, planner_id, max_vel, max_acc, tol);

            std::cout << "[" << timestamp_str << "] choose_pipeline: "
                        << pipeline_name << " / " << planner_id << "\n";
            }
        }
        // 6) joints_move
        else if (command_obj.contains("joints_move")) {
            auto& jm = command_obj["joints_move"];
            if (jm.contains("positions")) {
            auto joint_values = parseJointPositions(jm["positions"], 6);
            builder.jointsMove(joint_values);
            std::cout << "[" << timestamp_str << "] joints_move\n";
            }
        }
        // 7) absolute_move
        else if (command_obj.contains("absolute_move")) {
            // Example shape:
            // "absolute_move": {
            //   "some_frame": {
            //     "position": [x,y,z],
            //     "quaternion": [rx, ry, rz, rw]
            //   }
            // }
            auto& abs_data = command_obj["absolute_move"];
            for (auto& [frame_id, pose_data] : abs_data.items()) {
            if (!pose_data.is_object()) continue;
            auto pos_arr = pose_data.value("position", std::vector<double>{0.0,0.0,0.0});
            auto quat_arr= pose_data.value("quaternion", std::vector<double>{0.0,0.0,0.0,1.0});
            if (pos_arr.size() < 3 || quat_arr.size() < 4) continue;

            double x=pos_arr[0], y=pos_arr[1], z=pos_arr[2];
            double rx=quat_arr[0], ry=quat_arr[1], rz=quat_arr[2], rw=quat_arr[3];

            // Hard-code tip_frame or parse if you prefer
            builder.absoluteMove(frame_id, /*tip_frame=*/tip_frame, /*target_frame=*/"",
                                x, y, z, rx, ry, rz, rw);
            std::cout << "[" << timestamp_str << "] absolute_move -> " << frame_id << "\n";
            }
        }
        // 8) displacement_move
        else if (command_obj.contains("displacement_move")) {
            // Example shape:
            // "displacement_move": {
            //   "some_key": {
            //     "world_frame": "world",
            //     "tip_frame": "tool0",
            //     "translation": [0.1, 0.2, 0.0],
            //     "rotation": [0.0, 0.0, 1.57]
            //   }
            // }
            auto& disp_data = command_obj["displacement_move"];
            for (auto& [any_key, data_obj] : disp_data.items()) {
            std::string world = data_obj.value("world_frame", "world");
            std::string tip   = data_obj.value("tip_frame", tip_frame);
            auto t_arr = data_obj.value("translation", std::vector<double>{0,0,0});
            auto r_arr = data_obj.value("rotation",    std::vector<double>{0,0,0});
            if (t_arr.size() < 3 || r_arr.size() < 3) continue;

            builder.displacementMove(world, tip, t_arr, r_arr);
            std::cout << "[" << timestamp_str << "] displacement_move\n";
            }
        }
        // 9) trajectory_move
        else if (command_obj.contains("trajectory_move")) {
            // e.g. "trajectory_move": { "csv_file": "path.csv", "vel":0.3, "acc":0.3, "tol":0.01 }
            auto& traj_data = command_obj["trajectory_move"];
            std::string csv_file = traj_data.value("csv_file", "");
            double vel = traj_data.value("vel", velocity_scale);
            double acc = traj_data.value("acc", accel_scale);
            double tol= traj_data.value("tol", pose_tolerance);

            builder.trajectoryMove(csv_file, vel, acc, tol);
            std::cout << "[" << timestamp_str << "] trajectory_move, CSV=" << csv_file << "\n";
        }
        // 10) feedback_move
        else if (command_obj.contains("feedback_move")) {
            // e.g. "feedback_move": { "topic": "/pose_feedback" }
            auto& fb_data = command_obj["feedback_move"];
            std::string topic = fb_data.value("topic","/pose_feedback");
            builder.feedbackMove(topic);
            std::cout << "[" << timestamp_str << "] feedback_move\n";
        }
        // 11) collaborative_move
        else if (command_obj.contains("collaborative_move")) {
            // e.g. "collaborative_move": { "torque_topic":"...", "record_file":"..." }
            auto& coll_data = command_obj["collaborative_move"];
            std::string torque_topic = coll_data.value("torque_topic","/torque_feedback");
            std::string record_file  = coll_data.value("record_file","collab_record.csv");
            builder.collaborativeMove(torque_topic, record_file);
            std::cout << "[" << timestamp_str << "] collaborative_move\n";
        }
        // 12) tool_control
        else if (command_obj.contains("tool_control")) {
            // e.g. "tool_control": {
            //   "mode": "precision",
            //   "feedback_enabled": 1.0,
            //   "position": 0.5,
            //   "tolerance": 0.01,
            //   "power": 0.1,
            //   "velocity": 0.1,
            //   "acceleration": 0.1,
            //   "force_limit": 10.0
            // }
            auto& tc = command_obj["tool_control"];
            TaskBuilder::ToolControlConfig cfg;
            cfg.mode             = tc.value("mode", "precision");
            cfg.feedback_enabled = tc.value("feedback_enabled", 0.0);
            cfg.position         = tc.value("position", 0.0);
            cfg.tolerance        = tc.value("tolerance", 0.01);
            cfg.power           = tc.value("power", 0.01);
            cfg.velocity        = tc.value("velocity", 0.01);
            cfg.acceleration    = tc.value("acceleration", 0.01);
            cfg.force_limit     = tc.value("force_limit", 10.0);

            builder.toolControl(cfg);
            std::cout << "[" << timestamp_str << "] tool_control\n";
        }
        // 13) attach_object
        else if (command_obj.contains("attach_object")) {
            auto& attach_data = command_obj["attach_object"];
            for (auto& [obj_name, link_name_val] : attach_data.items()) {
            std::string link_name = link_name_val.get<std::string>();
            builder.attachObject(obj_name, link_name);
            std::cout << "[" << timestamp_str << "] attach_object: " 
                        << obj_name << " -> " << link_name << "\n";
            }
        }
        // 14) detach_object
        else if (command_obj.contains("detach_object")) {
            auto& detach_data = command_obj["detach_object"];
            for (auto& [obj_name, link_name_val] : detach_data.items()) {
            std::string link_name = link_name_val.get<std::string>();
            builder.detachObject(obj_name, link_name);
            std::cout << "[" << timestamp_str << "] detach_object: " 
                        << obj_name << " from " << link_name << "\n";
            }
        }
        // 15) delete_json_sim_content
        else if (command_obj.contains("delete_json_sim_content")) {
            // e.g. "delete_json_sim_content":{"filename":"path/to.json"}
            auto& del_data = command_obj["delete_json_sim_content"];
            std::string filename = del_data.value("filename","");
            int rc = handleDeleteJsonSimContent(filename, node);
            rclcpp::shutdown();
            return rc;
        }
        // 16) delete_json_temp
        else if (command_obj.contains("delete_json_temp")) {
            // e.g. "delete_json_temp":{"directory":"/home/user"}
            auto& del_data = command_obj["delete_json_temp"];
            std::string directory = del_data.value("directory","");
            int rc = handleDeleteJsonTemp(directory, node);
            rclcpp::shutdown();
            return rc;
        }
        // 17) check_json_files
        else if (command_obj.contains("check_json_files")) {
            // e.g. "check_json_files":{"directory":"/home/user"}
            auto& cjf_data = command_obj["check_json_files"];
            std::string directory = cjf_data.value("directory","");
            int rc = handleCheckJsonFiles(directory, node);
            rclcpp::shutdown();
            return rc;
        }
        // If you want "scan_line", "calibrate_camera", "gcode_load", "step_load"
        // (from your snippet) do them similarly. E.g.:
        else if (command_obj.contains("scan_line")) {
            auto& sl_data = command_obj["scan_line"];
            // e.g. "scan_line":{"world_frame":"world","start":[0,0,0],"end":[0.2,0.2,0.0]}
            // Then call builder.scanLine(...)
            // ...
            std::cout << "[" << timestamp_str << "] scan_line (not fully implemented in example)\n";
        }
        else if (command_obj.contains("calibrate_camera")) {
            // ...
            std::cout << "[" << timestamp_str << "] calibrate_camera (not fully implemented)\n";
        }
        else if (command_obj.contains("gcode_load")) {
            // ...
            std::cout << "[" << timestamp_str << "] gcode_load (not fully implemented)\n";
        }
        else if (command_obj.contains("step_load")) {
            // ...
            std::cout << "[" << timestamp_str << "] step_load (not fully implemented)\n";
        }
        // ------------------------------------------------------------------
        else {
            // Unknown command
            std::cerr << "[" << timestamp_str << "] Unknown command. Skipping.\n";
        }

        } // end for [timestamp_str, command_obj]
    } // end for tasks_array

    // Now we have built up the MTC pipeline. Plan and execute if requested.
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
