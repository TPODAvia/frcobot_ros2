#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <map>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <limits>
#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nlohmann/json.hpp>
#include "task_builder.hpp"

// ------------------- Helper Functions for JSON & File Operations -------------------

static void appendToJson(const nlohmann::json& entry_content, rclcpp::Node::SharedPtr node)
{
	// 1) Fixed file path (adjust as needed)
	static const std::string JSON_FILE_PATH =
		"/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/tasks/fr10/test.json";

	// 2) Load existing JSON (if any)
	nlohmann::json json_array = nlohmann::json::array();
	if (std::filesystem::exists(JSON_FILE_PATH))
	{
		try {
		std::ifstream fin(JSON_FILE_PATH);
		if (fin.is_open())
			fin >> json_array;
		} catch (const std::exception& e) {
		RCLCPP_WARN(node->get_logger(),
					"Error parsing existing test.json: %s. Will overwrite with new array.",
					e.what());
		json_array = nlohmann::json::array();
		}
	}

	// 3) Get current time as a string key
	double now_float = std::chrono::duration<double>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	std::string now_str = std::to_string(now_float);

	// 4) Build a top-level JSON object and add to array.
	nlohmann::json new_entry;
	new_entry[now_str] = entry_content;
	json_array.push_back(new_entry);

	// 5) Write back to file (pretty print)
	try {
		std::ofstream fout(JSON_FILE_PATH);
		fout << json_array.dump(2) << std::endl;
		fout.close();
		RCLCPP_INFO(node->get_logger(), "Appended new entry to %s", JSON_FILE_PATH.c_str());
	}
	catch (const std::exception& e) {
		RCLCPP_ERROR(node->get_logger(),
					"Error writing JSON file: %s. Exception: %s",
					JSON_FILE_PATH.c_str(), e.what());
	}
}

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
		for (auto& elem : j)
		removeUnwantedKeys(elem, keys_to_remove);
	}
}

static int handleDeleteJsonSimContent(const std::string& filename, rclcpp::Node::SharedPtr node)
{
	if (!std::filesystem::exists(filename)) {
		RCLCPP_ERROR(node->get_logger(), "No file found at path: %s", filename.c_str());
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
		RCLCPP_ERROR(node->get_logger(), "Failed to open JSON file: %s", filename.c_str());
		return 1;
		}
		try {
		fin >> data;
		} catch (const std::exception& e) {
		RCLCPP_ERROR(node->get_logger(), "Error parsing JSON: %s", e.what());
		return 1;
		}
	}

	std::vector<std::string> unwanted_keys = { "remove_object", "detach_object", "clear_scene", "attach_object", "spawn_object" };
	removeUnwantedKeys(data, unwanted_keys);

	if (data.is_array()) {
		nlohmann::json filtered = nlohmann::json::array();
		for (auto& item : data) {
		if (!item.is_object() || !item.empty())
			filtered.push_back(item);
		}
		data = filtered;
	}

	{
		std::ofstream fout(mod_file);
		if (!fout.is_open()) {
		RCLCPP_ERROR(node->get_logger(), "Failed to open output file: %s", mod_file.string().c_str());
		return 1;
		}
		fout << data.dump(2) << std::endl;
	}

	RCLCPP_INFO(node->get_logger(), "delete_json_sim_content finished.");
	return 0;
}

static int handleDeleteJsonTemp(const std::string& directory, rclcpp::Node::SharedPtr node)
{
	std::cout << "test.json and mod_test.json will be deleted in: " << directory << "\n";
	std::cout << "Proceed? (y/n): y" << std::endl;
	std::string answer = "y";
	if (answer == "y" || answer == "Y") {
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
	} else {
		std::cout << "Deletion cancelled. Files are not deleted.\n";
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
	std::cout << std::string(80, '#') << "\n";
	std::cout << "Directory: " << directory << "\n";
	std::cout << std::string(80, '#') << "\n\n";
	std::cout << "Valid JSON files:\n";
	if (valid_json_files.empty()){
		std::cout << "  [WARNING] No valid JSON files found!\n";
	}
	else {
		for (auto& f : valid_json_files) {
			std::cout << "  " << f << "\n";
		}
	}
	std::cout << "\nInvalid JSON files:\n";
	if (invalid_json_files.empty()) {
		std::cout << "  [WARNING] No invalid JSON files found!\n";
	}
	else {
		for (auto& f : invalid_json_files) {
			std::cout << "  " << f << "\n";
		}		
	}
	std::cout << std::string(80, '#') << "\n";
	std::cout << "WARNING: This only checks JSON syntax. Further manual checks may be necessary.\n";
	std::cout << "check_json_files finished!\n";
	return 0;
}

static void savePosesToFile(const std::string& filename, const std::vector<geometry_msgs::msg::Pose>& poses) {
	std::ofstream file(filename);
	if (!file.is_open()) {
		std::cerr << "Failed to open file for saving poses: " << filename << std::endl;
		return;
	}
	file << "Saved Poses (" << poses.size() << " poses):" << std::endl;
	for (size_t i = 0; i < poses.size(); ++i) {
		const auto& pose = poses[i];
		file << "Pose #" << i + 1 << ":" << std::endl;
		file << "  Position: (" 
			<< pose.position.x << ", " 
			<< pose.position.y << ", " 
			<< pose.position.z << ")" << std::endl;
		file << "  Orientation: (" 
			<< pose.orientation.x << ", " 
			<< pose.orientation.y << ", " 
			<< pose.orientation.z << ", " 
			<< pose.orientation.w << ")" << std::endl;
	}
	file.close();
	std::cout << "Poses successfully saved to " << filename << std::endl;
}

static bool isNumeric(const std::string& s)
{
  try {
    /* Attempt to parse as a double */
    std::size_t pos;
    std::stod(s, &pos);  // may throw
    // If pos != s.size(), there is extra non-numeric text
    if (pos != s.size()) 
      return false;
    return true;
  } catch (...) {
    return false; 
  }
}

// ------------------- Command Parsing -------------------

enum class CommandKind {
	ROBOT_PARAM,
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
	TOOL_CONTROL,
	ATTACH_OBJECT,
	DETACH_OBJECT,
	DELETE_JSON_SIM_CONTENT,
	DELETE_JSON_TEMP,
	CHECK_JSON_FILES,
	SCAN_LINE,
	CALIBRATE_CAMERA,
	GCODE_LOAD,
	STEP_LOAD,
	UNKNOWN
};

static CommandKind parseCommand(const std::string& cmd)
{
	std::string lower;
	for (auto c : cmd)
		lower.push_back(std::tolower(c));

	if (lower == "get_robot_param")              return CommandKind::ROBOT_PARAM;
	if (lower == "clear_scene")                  return CommandKind::CLEAR_SCENE;
	if (lower == "remove_object")                return CommandKind::REMOVE_OBJECT;
	if (lower == "spawn_object")                 return CommandKind::SPAWN_OBJECT;
	if (lower == "choose_pipeline")              return CommandKind::CHOOSE_PIPELINE;
	if (lower == "joints_move")                  return CommandKind::JOINTS_MOVE;
	if (lower == "absolute_move")                return CommandKind::ABSOLUTE_MOVE;
	if (lower == "displacement_move")            return CommandKind::DISPLACEMENT_MOVE;
	if (lower == "trajectory_move")              return CommandKind::TRAJECTORY_MOVE;
	if (lower == "feedback_move")                return CommandKind::FEEDBACK_MOVE;
	if (lower == "collaborative_move")           return CommandKind::COLLABORATIVE_MOVE;
	if (lower == "tool_control")                 return CommandKind::TOOL_CONTROL;
	if (lower == "attach_object")                return CommandKind::ATTACH_OBJECT;
	if (lower == "detach_object")                return CommandKind::DETACH_OBJECT;
	if (lower == "delete_json_sim_content")      return CommandKind::DELETE_JSON_SIM_CONTENT;
	if (lower == "delete_json_temp")             return CommandKind::DELETE_JSON_TEMP;
	if (lower == "check_json_files")             return CommandKind::CHECK_JSON_FILES;
	if (lower == "scan_line")                    return CommandKind::SCAN_LINE;
	if (lower == "calibrate_camera")             return CommandKind::CALIBRATE_CAMERA;
	if (lower == "gcode_load")                   return CommandKind::GCODE_LOAD;
	if (lower == "step_load")                    return CommandKind::STEP_LOAD;
	return CommandKind::UNKNOWN;
}

// ------------------- Main -------------------

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.automatically_declare_parameters_from_overrides(true);
	auto node = std::make_shared<rclcpp::Node>("task_generator_node", options);


	// Print all argv arguments
	std::cout << "Command-line arguments:\n";
	std::cout << "argc: " << argc << std::endl;
	for (int i = 0; i < argc; ++i) {
		std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
	}

	// Expected Syntax Error! Usage:
	// task_generator <arm_group_name> <tip_frame> <command> [arguments...]
	if (argc < 8) {
		RCLCPP_ERROR(node->get_logger(),
					"Syntax Error! Usage: task_generator <arm_group_name> <tip_frame> <command> [arguments...]");
		rclcpp::shutdown();
		return 1;
	}

	std::string arm_group_name  =     argv[1];
	std::string tip_frame       =     argv[2];
	std::string exec_task       =     argv[3];
	std::string save_json       =     argv[4];
	std::string velosity_limit  =     argv[5];
	std::string acceleration_limit =  argv[6];
	std::string tolerance       =     argv[7];
	std::string virtual_base    =     argv[8];
	std::string command_str     =     argv[9];

	int internal_variables = 6;
	int task_variables = 9;
	int default_variables = internal_variables + task_variables;
	// Create TaskBuilder and start a new task.
	TaskBuilder builder(node, arm_group_name, tip_frame);
	builder.newTask("demo_task");
	nlohmann::json entry;

	builder.spawn_virtual_base(virtual_base == "true");

	// Process commands.
	switch (parseCommand(command_str))
	{
		case CommandKind::ROBOT_PARAM:
			builder.printRobotParams();
			save_json = false;
		break;

		case CommandKind::CLEAR_SCENE:
		{
			builder.clearScene();
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713337691.1535506": {
			//   "clear_scene": {
			//    "clear_scene": 1
			//   }
			//  }
			entry["clear_scene"] = { { "clear_scene", 1 } };
		}
		break;

		case CommandKind::REMOVE_OBJECT:
		{
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: remove_object <object_name>");
				rclcpp::shutdown();
				return 1;
			}
			builder.removeObject(argv[task_variables + 1]);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			// {
			//  "1713337798.1825986": {
			//   "remove_object": {
			//    "hello_box": 1
			//   }
			//  }
			std::string object_name = argv[task_variables + 1];
			entry["remove_object"] = { { object_name, 1 } };
		}
		break;

		case CommandKind::SPAWN_OBJECT:
		{
			if (argc != (default_variables + 1) && argc != (default_variables + 11)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: spawn_object <obj_name> <x> <y> <z> <rx> <ry> <rz> <rw> <da> <db> <dc>");
				rclcpp::shutdown();
				return 1;
			}
			std::string obj_name = argv[task_variables + 1];
			
			// Read numeric parameters if provided; otherwise, assign NaN.
			double x  = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 2])
															: std::numeric_limits<double>::quiet_NaN();
			double y  = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 3])
															: std::numeric_limits<double>::quiet_NaN();
			double z  = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 4])
															: std::numeric_limits<double>::quiet_NaN();
			double rx = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 5])
															: std::numeric_limits<double>::quiet_NaN();
			double ry = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 6])
															: std::numeric_limits<double>::quiet_NaN();
			double rz = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 7])
															: std::numeric_limits<double>::quiet_NaN();
			double rw = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 8])
															: std::numeric_limits<double>::quiet_NaN();
			double da = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 9])
															: std::numeric_limits<double>::quiet_NaN();
			double db = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 10])
															: std::numeric_limits<double>::quiet_NaN();
			double dc = (argc == (default_variables + 11)) ? std::stod(argv[task_variables + 11])
															: std::numeric_limits<double>::quiet_NaN();
			
			if (exec_task == "true") {
				// Call spawnObject with the values from the command line (which may be NaN)
				builder.spawnObject(obj_name, obj_name, x, y, z, rx, ry, rz, rw, da, db, dc);
			
				// For each parameter, if the command line value is NaN, use the corresponding value from builder.mem_data.
				if (std::isnan(x))
					x = builder.mem_data.pose.position.x;
				if (std::isnan(y))
				y = builder.mem_data.pose.position.y;
				if (std::isnan(z))
					z = builder.mem_data.pose.position.z;
				if (std::isnan(rx))
					rx = builder.mem_data.pose.orientation.x;
				if (std::isnan(ry))
				ry = builder.mem_data.pose.orientation.y;
				if (std::isnan(rz))
					rz = builder.mem_data.pose.orientation.z;
				if (std::isnan(rw))
					rw = builder.mem_data.pose.orientation.w;
				if (std::isnan(da) && !builder.mem_data.dimensions.empty() && builder.mem_data.dimensions.size() > 0)
					da = builder.mem_data.dimensions[0];
				if (std::isnan(db) && !builder.mem_data.dimensions.empty() && builder.mem_data.dimensions.size() > 1)
					db = builder.mem_data.dimensions[1];
				if (std::isnan(dc) && !builder.mem_data.dimensions.empty() && builder.mem_data.dimensions.size() > 2)
					dc = builder.mem_data.dimensions[2];
			}
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			
			// Log to JSON: here we use argv[10] as the object key.
			std::string object_name = argv[10];
			entry["spawn_object"][object_name] = {
				{ "x",  x },
				{ "y",  y },
				{ "z",  z },
				{ "rx", rx },
				{ "ry", ry },
				{ "rz", rz },
				{ "rw", rw }
			};

			// Log dimensions if available.
			if (!builder.mem_data.dimensions.empty()) {
				if (builder.mem_data.dimensions.size() > 0)
				entry["spawn_object"][object_name]["da"] = da;
				if (builder.mem_data.dimensions.size() > 1)
				entry["spawn_object"][object_name]["db"] = db;
				if (builder.mem_data.dimensions.size() > 2)
				entry["spawn_object"][object_name]["dc"] = dc;
			}
		}
		break;

		case CommandKind::CHOOSE_PIPELINE:
		{
			if (argc != (default_variables + 5)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: choose_pipeline <pipeline_name> <planner_id>");
				rclcpp::shutdown();
				return 1;
			}
			std::string pipeline 	= argv[task_variables + 1];
			std::string planner  	= argv[task_variables + 2];
			// if value are zero then it use the default
			double max_vel_factor 	= std::stod(argv[task_variables + 3]);
			double max_acc_factor 	= std::stod(argv[task_variables + 4]);
			double tolerance 		= std::stod(argv[task_variables + 5]);
			builder.savePipelineConfig(pipeline, planner, max_vel_factor, max_acc_factor, tolerance);
			builder.choosePipeline(pipeline, planner, max_vel_factor, max_acc_factor, tolerance);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713337876.5688505": {
			//   "choose_pipeline": {
			//    "OMPL": "RRTConnect"
			//    "max_vel_factor": 0.0  if value is 0.0 then it use the default
			//    "max_acc_factor": 0.0
			//    "tolerance": 0.0
			//   }
			//  }
			// should receive the value
			entry["choose_pipeline"][pipeline] = planner;
			entry["choose_pipeline"]["max_vel_factor"] = max_vel_factor;
			entry["choose_pipeline"]["max_acc_factor"] = max_acc_factor;
			entry["choose_pipeline"]["tolerance"] = tolerance;
		}
		break;

		case CommandKind::JOINTS_MOVE:
		{
			std::vector<double> joint_values;
			nlohmann::json joint_map;
			if (argc == (default_variables)) {
				builder.jointsMove(joint_values);

				auto current_joints = builder.getCurrentJointPositions();

				for (const auto& kv: current_joints) {
					joint_map[kv.first] = kv.second;
				}
			}
			else {
				if (argc != (default_variables + 6)) {
					RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: joints_move <j1> <j2> <j3> <j4> <j5> <j6>");
					rclcpp::shutdown();
					return 1;
				}
				// Check if argv[10] and argv[11] are non-numeric strings
				if (!isNumeric(argv[task_variables + 1]) || !isNumeric(argv[task_variables + 2])) {
					RCLCPP_ERROR(node->get_logger(),
								"choose_pipeline: expected tip_frame and target_frame as non-numeric strings");
					rclcpp::shutdown();
					return 1;
				}
				// Assuming we have 6 joints
				for (int i = (task_variables + 1); i < (task_variables + 7); ++i) {
					joint_values.push_back(std::stod(argv[i]));
				}
				builder.jointsMove(joint_values);

				joint_map["j1"] = std::stod(argv[task_variables + 1]);
				joint_map["j2"] = std::stod(argv[task_variables + 2]);
				joint_map["j3"] = std::stod(argv[task_variables + 3]);
				joint_map["j4"] = std::stod(argv[task_variables + 4]);
				joint_map["j5"] = std::stod(argv[task_variables + 5]);
				joint_map["j6"] = std::stod(argv[task_variables + 6]);
			}

			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713337825.8489513": {
			//   "joints_move": {
			//    "positions": {
			//     "j1": -0.05924035042911946,
			//     "j2": -1.3807693204190459,
			//     "j3": 1.4017875208377042,
			//     "j4": -0.01751446328105022,
			//     "j5": -0.07052746890781059,
			//     "j6": 0.02017181009719593
			//    }
			//   }
			//  }
			entry["joints_move"]["positions"] = joint_map;
		}
		break;

		case CommandKind::ABSOLUTE_MOVE:
		{
			// Two modes: either 3 extra args (frame_id tip_frame target_frame) or 7 extra args (frame_id x y z rx ry rz rw)
			if (argc == (default_variables + 3)) {
				// Check if argv[10] to argv[12] are non-numeric strings
				if (isNumeric(argv[task_variables + 1]) || isNumeric(argv[task_variables + 2]) || isNumeric(argv[12])) {
					RCLCPP_ERROR(node->get_logger(),
								"absolute_move: expected tip_frame and target_frame as non-numeric strings");
					rclcpp::shutdown();
					return 1;
				}
				std::string frame_id = argv[task_variables + 1];
				std::string tip      = argv[task_variables + 2];
				std::string target   = argv[task_variables + 3];
				builder.absoluteMove(frame_id, tip, target);
			
			} else if (argc == (default_variables + 8)) {
				// Check if argv[11] to argv[17] are all numeric values
				bool numeric_args = true;
				for (int i = (task_variables + 2); i <= (task_variables + 6); ++i) {
					if (!isNumeric(argv[i])) {
						numeric_args = false;
						break;
					}
				}
				if (!numeric_args) {
					RCLCPP_ERROR(node->get_logger(),
								"absolute_move: expected 7 numeric arguments for x, y, z, rx, ry, rz, rw");
					rclcpp::shutdown();
					return 1;
				}
				std::string frame_id = argv[task_variables + 1];
				double x  = std::stod(argv[task_variables + 2]);
				double y  = std::stod(argv[task_variables + 3]);
				double z  = std::stod(argv[task_variables + 4]);
				double rx = std::stod(argv[task_variables + 5]);
				double ry = std::stod(argv[task_variables + 6]);
				double rz = std::stod(argv[task_variables + 7]);
				double rw = std::stod(argv[task_variables + 8]);
				builder.absoluteMove(frame_id, "wrist3_link", "", x, y, z, rx, ry, rz, rw);
			} else {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: absolute_move <frame_id> <tip_frame> <target_frame> OR "
							"absolute_move <frame_id> <x> <y> <z> <rx> <ry> <rz> <rw>");
				rclcpp::shutdown();
				return 1;
			}
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713338342.2387867": {
			//   "absolute_move": {
			//    "tf_end": {
			//     "position": [
			//      0.0,
			//      0.5,
			//      0.2
			//     ],
			//     "quaternion": [
			//      0,
			//      0,
			//      0,
			//      1
			//     ]
			//    }
			//   }
			//  }
			// We'll replicate old "end_coordinate" wrapping with "absolute_move":
			std::string frame_id = argv[task_variables + 1];
			double x  = std::stod(argv[task_variables + 2]);
			double y  = std::stod(argv[task_variables + 3]);
			double z  = std::stod(argv[task_variables + 4]);
			double rx = std::stod(argv[task_variables + 5]);
			double ry = std::stod(argv[task_variables + 6]);
			double rz = std::stod(argv[task_variables + 7]);
			double rw = std::stod(argv[task_variables + 8]);
			entry["absolute_move"][frame_id] = {
			{ "position",   { x, y, z } },
			{ "quaternion", { rx, ry, rz, rw } }
			};
		}
		break;

		case CommandKind::DISPLACEMENT_MOVE:
		{
			// Check if argv[10] and argv[11] are non-numeric strings
			if (isNumeric(argv[task_variables + 1]) || isNumeric(argv[task_variables + 2])) {
				RCLCPP_ERROR(node->get_logger(),
							"displacement_move: expected tip_frame and target_frame as non-numeric strings");
				rclcpp::shutdown();
				return 1;
			}
			if (argc != (default_variables + 8)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: displacement_move <world_frame> <tip_frame> <x> <y> <z> <rx> <ry> <rz>");
				rclcpp::shutdown();
				return 1;
			}
			std::string world_frame = argv[task_variables + 1];
			std::string tip = argv[task_variables + 2];
			std::vector<double> translation_vector;
			for (int i = (task_variables + 3); i < (task_variables + 6); ++i)
			translation_vector.push_back(std::stod(argv[i]));
			std::vector<double> rotation_vector;
			for (int i = (task_variables + 6); i < (task_variables + 9); ++i)
			rotation_vector.push_back(std::stod(argv[i]));
			builder.displacementMove(world_frame, tip, translation_vector, rotation_vector);

			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[collaborative_move]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::TRAJECTORY_MOVE:
		{
			if (argc != (default_variables + 4)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: trajectory_move <csv_file> <velocity_scale> <accel_scale> <pose_tolerance>");
				rclcpp::shutdown();
				return 1;
			}
			std::string csv_file = 				  argv[task_variables + 1];
			double vel_scale = 			std::stod(argv[task_variables + 2]);
			double accel_scale = 		std::stod(argv[task_variables + 3]);
			double pose_tol = 			std::stod(argv[task_variables + 4]);
			builder.trajectoryMove(csv_file, vel_scale, accel_scale, pose_tol);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[collaborative_move]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::FEEDBACK_MOVE:
		{
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: feedback_move <pose_topic>");
				rclcpp::shutdown();
				return 1;
			}
			builder.feedbackMove(argv[task_variables + 1]);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[feedback_move]: json saving is not suppported");
			save_json=false;
		}
		break;

		case CommandKind::COLLABORATIVE_MOVE:
		{
			if (argc != (default_variables + 2)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: collaborative_move <torque_topic> <record_filename>");
				rclcpp::shutdown();
				return 1;
			}
			builder.collaborativeMove(argv[task_variables + 1], argv[task_variables + 2]);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[collaborative_move]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::TOOL_CONTROL:
		{
			if (argc != (default_variables + 8)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: tool_control");
				rclcpp::shutdown();
				return 1;
			}

			std::string mode = 				      argv[task_variables + 1];
			double feedback_enabled = 	std::stod(argv[task_variables + 2]);
			double position = 			std::stod(argv[task_variables + 3]);
			double tolerance = 			std::stod(argv[task_variables + 4]);
			double power = 				std::stod(argv[task_variables + 5]);
			double velocity = 			std::stod(argv[task_variables + 6]);
			double acceleration = 		std::stod(argv[task_variables + 7]);
			double force_limit = 		std::stod(argv[task_variables + 8]);
			
			TaskBuilder::ToolControlConfig tool_config;
			tool_config.mode = "precision";
			tool_config.feedback_enabled = feedback_enabled;
			tool_config.position = position;
			tool_config.tolerance = tolerance;
			tool_config.power = power;
			tool_config.velocity = velocity;
			tool_config.acceleration = acceleration;
			tool_config.force_limit = force_limit;

			builder.toolControl(tool_config);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//   "1714837303.1926935": {
			//     "tool_control": {
			//       "mode": "precision",       // (string) Mode of gripping ("precision", "collaborative", "weld", "print", "")
			//       "feedback_enabled": 0.0    // (unit 0 1) Enables or disables feedback from sensors
			//       "position": 0.0,           // (unit: relative position) Target opening position (0.0 - fully closed, 1.0 - fully open)
			//       "tolerance": 0.01,         // (unit: meters) Acceptable deviation in the position
			//       "power": 0.01,             // (unit: relative scale, 0.0 - 1.0) Power applied to the motor
			//       "velocity": 0.01,          // (unit: m/s) Speed at which the moves
			//       "acceleration": 0.01,      // (unit: m/s^2) Acceleration of the movement
			//       "force_limit": 10.0,       // (unit: N) Maximum force can exert
			//     }
			//   }

			entry["tool_control"]["mode"] = "precision";
			entry["tool_control"]["feedback_enabled"] = feedback_enabled;
			entry["tool_control"]["position"] = position;
			entry["tool_control"]["tolerance"] = tolerance;
			entry["tool_control"]["power"] = power;
			entry["tool_control"]["velocity"] = velocity;
			entry["tool_control"]["acceleration"] = acceleration;
			entry["tool_control"]["force_limit"] = force_limit;
		}
		break;

		case CommandKind::ATTACH_OBJECT:
		{
			if (argc != default_variables + 2) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: attach_object <object_name> <link_name>");
				rclcpp::shutdown();
				return 1;
			}
			builder.attachObject(argv[task_variables + 1], argv[task_variables + 2]);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713338391.0690901": {
			//   "attach_object": {
			//    "hello_box": "tf_end"
			//   }
			//  }
			std::string object_name = argv[task_variables + 1];
			std::string link_name   = argv[task_variables + 2];
			entry["attach_object"][object_name] = link_name;
		}
		break;

		case CommandKind::DETACH_OBJECT:
		{
			if (argc != (default_variables + 2)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: detach_object <object_name> <link_name>");
				rclcpp::shutdown();
				return 1;
			}
			builder.detachObject(argv[task_variables + 1], argv[task_variables + 2]);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			// Format:
			//  "1713338391.0690901": {
			//   "detach_object": {
			//    "hello_box": "tf_end"
			//   }
			//  }
			std::string object_name = argv[task_variables + 1];
			std::string link_name   = argv[task_variables + 2];
			entry["detach_object"][object_name] = link_name;
		}
		break;

		case CommandKind::DELETE_JSON_SIM_CONTENT:
		{
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: delete_json_sim_content <filename>");
				rclcpp::shutdown();
				return 1;
			}
			int rc = handleDeleteJsonSimContent(argv[task_variables + 1], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::DELETE_JSON_TEMP:
		{
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: delete_json_temp <directory>");
				rclcpp::shutdown();
				return 1;
			}
			int rc = handleDeleteJsonTemp(argv[task_variables + 1], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::CHECK_JSON_FILES:
		{
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: check_json_files <directory>");
				rclcpp::shutdown();
				return 1;
			}
			int rc = handleCheckJsonFiles(argv[task_variables + 1], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::SCAN_LINE:
		{
			if (argc != (default_variables + 7)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: scan_line <x1> <y1> <z1> <x2> <y2> <z2> <num_steps>");
				rclcpp::shutdown();
				return 1;
			}

			geometry_msgs::msg::Point start, end;
			std::string world_frame = argv[task_variables + 1];
			start.x = 		std::stod(argv[task_variables + 2]);
			start.y = 		std::stod(argv[task_variables + 3]);
			start.z = 		std::stod(argv[task_variables + 4]);
			end.x = 		std::stod(argv[task_variables + 5]);
			end.y = 		std::stod(argv[task_variables + 6]);
			end.z = 		std::stod(argv[task_variables + 7]);
			// If TaskBuilder::scanLine is implemented, call it.
			builder.scanLine(world_frame, tip_frame, start, end);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[scan_line]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::CALIBRATE_CAMERA:
		{
			if (argc != (default_variables + 3)) {
				RCLCPP_ERROR(node->get_logger(),
							"Syntax Error! Usage: calibrate_camera <x> <y> <z>");
				rclcpp::shutdown();
				return 1;
			}
			double x = std::stod(argv[task_variables + 1]);
			double y = std::stod(argv[task_variables + 2]);
			double z = std::stod(argv[task_variables + 3]);
			// The orientation is not used in our simple example.
			builder.calibrateCamera(tip_frame, x, y, z);
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[scan_line]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::GCODE_LOAD:
		{
			// Expect: gcode_load <gcode_filename>
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: gcode_load <gcode_filename>");
				rclcpp::shutdown();
				return 1;
			}
			std::string gcode_file = argv[task_variables + 1];
			auto gcode_poses = builder.gcodeLoad(gcode_file, ""); // mode can be used if needed
			if (gcode_poses.empty()) {
				RCLCPP_ERROR(node->get_logger(), "No valid trajectory from G-code file: %s", gcode_file.c_str());
				rclcpp::shutdown();
				return 1;
			}
			std::string debug_file = "gcode_poses_debug.txt";
			savePosesToFile(debug_file, gcode_poses);
			RCLCPP_INFO(node->get_logger(), "G-code poses saved to %s", debug_file.c_str());
			// Optionally, call builder.trajectoryMove() if you have an overload.
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[scan_line]: json saving is not suppported");
			save_json = false;
		}
		break;

		case CommandKind::STEP_LOAD:
		{
			// Expect: step_load <step_filename>
			if (argc != (default_variables + 1)) {
				RCLCPP_ERROR(node->get_logger(), "Syntax Error! Usage: step_load <step_filename>");
				rclcpp::shutdown();
				return 1;
			}
			std::string step_filename = argv[task_variables + 1];
			auto curve_poses = builder.stepLoad(step_filename);
			if (curve_poses.empty()) {
				RCLCPP_ERROR(node->get_logger(), "No valid poses generated from STEP file: %s", step_filename.c_str());
				rclcpp::shutdown();
				return 1;
			}
			// Optionally, save debug file:
			// savePosesToFile("step_curve_poses_debug.txt", curve_poses);
			// Optionally, call builder.trajectoryMove() if you have an overload.
			if (!builder.ok()) {
				RCLCPP_ERROR(node->get_logger(), "Task Interrupted!");
				return 1;
			}
			RCLCPP_WARN(node->get_logger(),	"[scan_line]: json saving is not suppported");
			save_json = false;
		}
		break;

		default:
		RCLCPP_ERROR(node->get_logger(), "Unknown command: %s", command_str.c_str());
		rclcpp::shutdown();
		return 1;
	} // end switch

	// After adding the desired stages, initialize, plan, and execute the task.
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

	// Finally, if `entry` is non-empty, append it to test.json
	if (save_json == "true") {
		if (entry.empty()) {
			RCLCPP_ERROR(node->get_logger(), "Failed to save to JSON entry is empty");
			rclcpp::shutdown();
			return 1;
		}
		appendToJson(entry, node);
		RCLCPP_INFO(node->get_logger(), "Saving to JSON file!");
	}

	RCLCPP_INFO(node->get_logger(), "Task complete!");
	rclcpp::shutdown();
	return 0;
}
