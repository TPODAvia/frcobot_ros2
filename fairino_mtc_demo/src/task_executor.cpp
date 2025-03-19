/****************************************************
 * File: task_executor.cpp
 * Description: Loads a JSON array of commands from a file,
 *              builds an MTC Task using TaskBuilder, and
 *              optionally executes it.
 ****************************************************/
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "task_builder.hpp"

using json = nlohmann::json;

// Helper to parse joint positions from a JSON object.
static std::vector<double> parseJointPositions(const json& positions) {
    std::vector<double> joints(6, 0.0);
    joints[0] = positions.value("j1", 0.0);
    joints[1] = positions.value("j2", 0.0);
    joints[2] = positions.value("j3", 0.0);
    joints[3] = positions.value("j4", 0.0);
    joints[4] = positions.value("j5", 0.0);
    joints[5] = positions.value("j6", 0.0);
    return joints;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Expected command-line arguments:
    // <arm_group> <tip_frame> <exec_task> <json_file>
    // [velocity_scale] [accel_scale] [pose_tolerance] [virtual_base]
    if (argc < 5) {
        std::cerr << "Usage:\n  task_executor <arm_group> <tip_frame> <exec_task> <json_file> "
                     "[velocity_scale] [accel_scale] [pose_tolerance] [virtual_base]\n";
        rclcpp::shutdown();
        return 1;
    }

    std::string arm_group   = argv[1];
    std::string tip_frame   = argv[2];
    std::string exec_task   = argv[3];  // "true" or "false"
    std::string json_file   = argv[4];
    double velocity_scale   = (argc > 5) ? std::stod(argv[5]) : 1.0;
    double accel_scale      = (argc > 6) ? std::stod(argv[6]) : 1.0;
    double pose_tolerance   = (argc > 7) ? std::stod(argv[7]) : 0.01;
    bool virtual_base       = (argc > 8) && (std::string(argv[8]) == "true");

    if (!std::filesystem::exists(json_file)) {
        std::cerr << "Error: JSON file not found: " << json_file << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    // Create node. The moveit configuration parameters are expected to be loaded
    // via the launch file (passed as parameters).
    auto node = rclcpp::Node::make_shared("task_executor_node");

    // Create TaskBuilder and start a new task.
    TaskBuilder builder(node, arm_group, tip_frame);
    builder.newTask("json_task");

    if (virtual_base) {
        builder.spawn_virtual_base(true);
    }

    // Load JSON file
    json tasks_array;
    try {
        std::ifstream fin(json_file);
        fin >> tasks_array;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error reading/parsing JSON: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    if (!tasks_array.is_array()) {
        RCLCPP_ERROR(node->get_logger(), "JSON file must contain an array of command objects.");
        rclcpp::shutdown();
        return 1;
    }

    // Iterate over each JSON entry.
    for (auto& entry : tasks_array) {
        if (!entry.is_object()) {
            RCLCPP_WARN(node->get_logger(), "Skipping non-object entry in JSON array.");
            continue;
        }
        // Each entry should have a single top-level key (e.g., a timestamp)
        for (auto& [timestamp, cmd_obj] : entry.items()) {
            if (!cmd_obj.is_object()) {
                RCLCPP_WARN(node->get_logger(), "Skipping entry with non-object command.");
                continue;
            }

            // Process supported commands (mirroring task_generator.cpp style):

            if (cmd_obj.contains("get_robot_param")) {
                builder.printRobotParams();
            }
            else if (cmd_obj.contains("clear_scene")) {
                builder.clearScene();
            }
            else if (cmd_obj.contains("remove_object")) {
                auto& rm = cmd_obj["remove_object"];
                for (auto& [obj_name, value] : rm.items()) {
                    builder.removeObject(obj_name);
                }
            }
            else if (cmd_obj.contains("spawn_object")) {
                auto& sp = cmd_obj["spawn_object"];
                for (auto& [obj_name, pose_obj] : sp.items()) {
                    double x  = pose_obj.value("x", 0.0);
                    double y  = pose_obj.value("y", 0.0);
                    double z  = pose_obj.value("z", 0.0);
                    double rx = pose_obj.value("rx", 0.0);
                    double ry = pose_obj.value("ry", 0.0);
                    double rz = pose_obj.value("rz", 0.0);
                    double rw = pose_obj.value("rw", 1.0);
                    double da = pose_obj.value("da", 0.05);
                    double db = pose_obj.value("db", 0.05);
                    double dc = pose_obj.value("dc", 0.05);
                    builder.spawnObject(obj_name, obj_name, x, y, z, rx, ry, rz, rw, da, db, dc);
                }
            }
            else if (cmd_obj.contains("choose_pipeline")) {
                auto& cp = cmd_obj["choose_pipeline"];
                for (auto& [pipeline, planner] : cp.items()) {
                    // Skip special numeric fields.
                    if (pipeline == "max_vel_factor" || pipeline == "max_acc_factor" || pipeline == "tolerance")
                        continue;
                    std::string planner_id = planner.get<std::string>();
                    double max_vel = cp.value("max_vel_factor", velocity_scale);
                    double max_acc = cp.value("max_acc_factor", accel_scale);
                    double tol     = cp.value("tolerance", pose_tolerance);
                    builder.savePipelineConfig(pipeline, planner_id, max_vel, max_acc, tol);
                    builder.choosePipeline(pipeline, planner_id, max_vel, max_acc, tol);
                }
            }
            else if (cmd_obj.contains("joints_move")) {
                auto& jm = cmd_obj["joints_move"];
                if (jm.contains("positions")) {
                    auto joints = parseJointPositions(jm["positions"]);
                    builder.jointsMove(joints);
                }
            }
            else if (cmd_obj.contains("absolute_move")) {
                auto& am = cmd_obj["absolute_move"];
                for (auto& [frame, pose] : am.items()) {
                    auto pos  = pose.value("position", std::vector<double>{0.0, 0.0, 0.0});
                    auto quat = pose.value("quaternion", std::vector<double>{0.0, 0.0, 0.0, 1.0});
                    if (pos.size() >= 3 && quat.size() >= 4) {
                        builder.absoluteMove(frame, tip_frame, "", pos[0], pos[1], pos[2],
                                             quat[0], quat[1], quat[2], quat[3]);
                    }
                }
            }
            else if (cmd_obj.contains("displacement_move")) {
                auto& dm = cmd_obj["displacement_move"];
                for (auto& [key, data] : dm.items()) {
                    std::string world = data.value("world_frame", "world");
                    std::string tip   = data.value("tip_frame", tip_frame);
                    auto translation  = data.value("translation", std::vector<double>{0.0, 0.0, 0.0});
                    auto rotation     = data.value("rotation", std::vector<double>{0.0, 0.0, 0.0});
                    if (translation.size() >= 3 && rotation.size() >= 3) {
                        builder.displacementMove(world, tip, translation, rotation);
                    }
                }
            }
            else if (cmd_obj.contains("trajectory_move")) {
                auto& tm = cmd_obj["trajectory_move"];
                std::string csv_file = tm.value("csv_file", "");
                double vel = tm.value("vel", velocity_scale);
                double acc = tm.value("acc", accel_scale);
                double tol = tm.value("tol", pose_tolerance);
                builder.trajectoryMove(csv_file, vel, acc, tol);
            }
            else if (cmd_obj.contains("feedback_move")) {
                auto& fm = cmd_obj["feedback_move"];
                std::string topic = fm.value("topic", "/pose_feedback");
                builder.feedbackMove(topic);
            }
            else if (cmd_obj.contains("collaborative_move")) {
                auto& cm = cmd_obj["collaborative_move"];
                std::string torque_topic = cm.value("torque_topic", "/torque_feedback");
                std::string record_file  = cm.value("record_file", "collab_record.csv");
                builder.collaborativeMove(torque_topic, record_file);
            }
            else if (cmd_obj.contains("tool_control")) {
                auto& tc = cmd_obj["tool_control"];
                TaskBuilder::ToolControlConfig cfg;
                cfg.mode             = tc.value("mode", "precision");
                cfg.feedback_enabled = tc.value("feedback_enabled", 0.0);
                cfg.position         = tc.value("position", 0.0);
                cfg.tolerance        = tc.value("tolerance", 0.01);
                cfg.power            = tc.value("power", 0.01);
                cfg.velocity         = tc.value("velocity", 0.01);
                cfg.acceleration     = tc.value("acceleration", 0.01);
                cfg.force_limit      = tc.value("force_limit", 10.0);
                builder.toolControl(cfg);
            }
            else if (cmd_obj.contains("attach_object")) {
                auto& ao = cmd_obj["attach_object"];
                for (auto& [obj, link] : ao.items()) {
                    builder.attachObject(obj, link.get<std::string>());
                }
            }
            else if (cmd_obj.contains("detach_object")) {
                auto& dao = cmd_obj["detach_object"];
                for (auto& [obj, link] : dao.items()) {
                    builder.detachObject(obj, link.get<std::string>());
                }
            }
            else {
                RCLCPP_WARN(node->get_logger(), "Unknown command in JSON. Skipping.");
            }
        }
    }

    // Initialize, plan, and (if requested) execute the MTC task.
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
