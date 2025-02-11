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

static void appendToJsonLog(const nlohmann::json& entry_content, rclcpp::Node::SharedPtr node)
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

static int saveJsonFile(const std::string& filepath, const nlohmann::json& json_data, rclcpp::Node::SharedPtr node)
{
	try {
		std::filesystem::path file_path(filepath);
		std::filesystem::path parent_dir = file_path.parent_path();
		if (!std::filesystem::exists(parent_dir)) {
		std::filesystem::create_directories(parent_dir);
		RCLCPP_INFO(node->get_logger(), "Created directory: %s", parent_dir.string().c_str());
		}
		std::ofstream fout(filepath);
		if (!fout.is_open()) {
		RCLCPP_ERROR(node->get_logger(), "Failed to open file for writing: %s", filepath.c_str());
		return 1;
		}
		fout << json_data.dump(2) << std::endl;
		RCLCPP_INFO(node->get_logger(), "Successfully saved JSON file: %s", filepath.c_str());
		fout.close();
		return 0;
	} catch (const std::exception& e) {
		RCLCPP_ERROR(node->get_logger(), "Error saving JSON file: %s. Exception: %s", filepath.c_str(), e.what());
		return 1;
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
	for (auto& f : valid_json_files) {
		std::cout << "  " << f << "\n";
	}
	std::cout << "\nInvalid JSON files:\n";
	for (auto& f : invalid_json_files) {
		std::cout << "  " << f << "\n";
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
	GRIPPER_CLOSE,
	GRIPPER_OPEN,
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
	if (lower == "gripper_close")                return CommandKind::GRIPPER_CLOSE;
	if (lower == "gripper_open")                 return CommandKind::GRIPPER_OPEN;
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
	auto node = rclcpp::Node::make_shared("task_generator_node");

	// Print all argv arguments
	std::cout << "Command-line arguments:\n";
	std::cout << "argc: " << argc << std::endl;
	for (int i = 0; i < argc; ++i) {
		std::cout << "argv[" << i << "]: " << argv[i] << std::endl;
	}

	// Expected usage:
	// task_generator <arm_group_name> <tip_frame> <command> [arguments...]
	if (argc < 8) {
		RCLCPP_ERROR(node->get_logger(),
					"Usage: task_generator <arm_group_name> <tip_frame> <command> [arguments...]");
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
	std::string gripper         =     argv[8];
	std::string command_str     =     argv[9];

	int internal_variables = 6;
	int task_variables = 9;

	// Create TaskBuilder and start a new task.
	TaskBuilder builder(node, arm_group_name, tip_frame);
	builder.newTask("demo_task");

	// Process commands.
	switch (parseCommand(command_str))
	{
		case CommandKind::ROBOT_PARAM:
		builder.printRobotParams();
		break;

		case CommandKind::CLEAR_SCENE:
		builder.clearScene();
		break;

		case CommandKind::REMOVE_OBJECT:
		if (argc != (internal_variables + task_variables + 1)) {
			RCLCPP_ERROR(node->get_logger(), "Usage: remove_object <object_name>");
			rclcpp::shutdown();
			return 1;
		}
		builder.removeObject(argv[task_variables + 1]);
		break;

		case CommandKind::SPAWN_OBJECT:
		if (argc != 26) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: spawn_object <obj_name> <x> <y> <z> <rx> <ry> <rz> <rw> <da> <db> <dc>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string obj_name = argv[10];
			double x = (argc == 26) ? std::stod(argv[11]) : std::numeric_limits<double>::quiet_NaN();
			double y = (argc == 26) ? std::stod(argv[12]) : std::numeric_limits<double>::quiet_NaN();
			double z = (argc == 26) ? std::stod(argv[13]) : std::numeric_limits<double>::quiet_NaN();
			double rx = (argc == 26) ? std::stod(argv[14]) : std::numeric_limits<double>::quiet_NaN();
			double ry = (argc == 26) ? std::stod(argv[15]) : std::numeric_limits<double>::quiet_NaN();
			double rz = (argc == 26) ? std::stod(argv[16]) : std::numeric_limits<double>::quiet_NaN();
			double rw = (argc == 26) ? std::stod(argv[17]) : std::numeric_limits<double>::quiet_NaN();
			double da = (argc == 26) ? std::stod(argv[18]) : std::numeric_limits<double>::quiet_NaN();
			double db = (argc == 26) ? std::stod(argv[19]) : std::numeric_limits<double>::quiet_NaN();
			double dc = (argc == 26) ? std::stod(argv[20]) : std::numeric_limits<double>::quiet_NaN();
			builder.spawnObject(obj_name, obj_name, x, y, z, rx, ry, rz, rw, da, db, dc);
		}
		break;

		case CommandKind::CHOOSE_PIPELINE:
		if (argc != 6) {
			RCLCPP_ERROR(node->get_logger(), "Usage: choose_pipeline <pipeline_name> <planner_id>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string pipeline = argv[4];
			std::string planner  = argv[5];
			builder.savePipelineConfig(pipeline, planner, 0.0, 0.0);
			builder.choosePipeline(pipeline, planner, 0.0, 0.0);
		}
		break;

		case CommandKind::JOINTS_MOVE:
		if (argc != 10) {
			RCLCPP_ERROR(node->get_logger(), "Usage: joints_move <j1> <j2> <j3> <j4> <j5> <j6>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::vector<double> joint_values;
			for (int i = 4; i < 10; ++i)
			joint_values.push_back(std::stod(argv[i]));
			builder.jointsMove(joint_values);
		}
		break;

		case CommandKind::ABSOLUTE_MOVE:
		// Two modes: either 3 extra args (frame_id tip_frame target_frame) or 7 extra args (frame_id x y z rx ry rz rw)
		if (argc == 7) {
			std::string frame_id = argv[4];
			std::string tip      = argv[5];
			std::string target   = argv[6];
			builder.absoluteMove(frame_id, tip, target);
		} else if (argc == 12) {
			std::string frame_id = argv[4];
			double x  = std::stod(argv[5]);
			double y  = std::stod(argv[6]);
			double z  = std::stod(argv[7]);
			double rx = std::stod(argv[8]);
			double ry = std::stod(argv[9]);
			double rz = std::stod(argv[10]);
			double rw = std::stod(argv[11]);
			builder.absoluteMove(frame_id, "", "", x, y, z, rx, ry, rz, rw);
		} else {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: absolute_move <frame_id> <tip_frame> <target_frame> OR "
						"absolute_move <frame_id> <x> <y> <z> <rx> <ry> <rz> <rw>");
			rclcpp::shutdown();
			return 1;
		}
		break;

		case CommandKind::DISPLACEMENT_MOVE:
		if (argc != 12) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: displacement_move <world_frame> <tip_frame> <x> <y> <z> <rx> <ry> <rz>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string world_frame = argv[4];
			std::string tip = argv[5];
			std::vector<double> translation_vector;
			for (int i = 6; i < 9; ++i)
			translation_vector.push_back(std::stod(argv[i]));
			std::vector<double> rotation_vector;
			for (int i = 9; i < 12; ++i)
			rotation_vector.push_back(std::stod(argv[i]));
			builder.displacementMove(world_frame, tip, translation_vector, rotation_vector);
		}
		break;

		case CommandKind::TRAJECTORY_MOVE:
		if (argc != 8) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: trajectory_move <csv_file> <velocity_scale> <accel_scale> <pose_tolerance>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string csv_file = argv[4];
			double vel_scale = std::stod(argv[5]);
			double accel_scale = std::stod(argv[6]);
			double pose_tol = std::stod(argv[7]);
			builder.trajectoryMove(csv_file, vel_scale, accel_scale, pose_tol);
		}
		break;

		case CommandKind::FEEDBACK_MOVE:
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: feedback_move <pose_topic>");
			rclcpp::shutdown();
			return 1;
		}
		builder.feedbackMove(argv[4]);
		break;

		case CommandKind::COLLABORATIVE_MOVE:
		if (argc != 6) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: collaborative_move <torque_topic> <record_filename>");
			rclcpp::shutdown();
			return 1;
		}
		builder.collaborativeMove(argv[4], argv[5]);
		break;

		case CommandKind::GRIPPER_CLOSE:
		if (argc != 4) {
			RCLCPP_ERROR(node->get_logger(), "Usage: gripper_close");
			rclcpp::shutdown();
			return 1;
		}
		builder.gripperClose();
		break;

		case CommandKind::GRIPPER_OPEN:
		if (argc != 4) {
			RCLCPP_ERROR(node->get_logger(), "Usage: gripper_open");
			rclcpp::shutdown();
			return 1;
		}
		builder.gripperOpen();
		break;

		case CommandKind::ATTACH_OBJECT:
		if (argc != 6) {
			RCLCPP_ERROR(node->get_logger(), "Usage: attach_object <object_name> <link_name>");
			rclcpp::shutdown();
			return 1;
		}
		builder.attachObject(argv[4], argv[5]);
		break;

		case CommandKind::DETACH_OBJECT:
		if (argc != 6) {
			RCLCPP_ERROR(node->get_logger(), "Usage: detach_object <object_name> <link_name>");
			rclcpp::shutdown();
			return 1;
		}
		builder.detachObject(argv[4], argv[5]);
		break;

		case CommandKind::DELETE_JSON_SIM_CONTENT:
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: delete_json_sim_content <filename>");
			rclcpp::shutdown();
			return 1;
		}
		{
			int rc = handleDeleteJsonSimContent(argv[4], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::DELETE_JSON_TEMP:
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: delete_json_temp <directory>");
			rclcpp::shutdown();
			return 1;
		}
		{
			int rc = handleDeleteJsonTemp(argv[4], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::CHECK_JSON_FILES:
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: check_json_files <directory>");
			rclcpp::shutdown();
			return 1;
		}
		{
			int rc = handleCheckJsonFiles(argv[4], node);
			rclcpp::shutdown();
			return rc;
		}
		break;

		case CommandKind::SCAN_LINE:
		// Expect: scan_line <sx> <sy> <sz> <srx> <sry> <srz> <srw> <ex> <ey> <ez> <erx> <ery> <erz> <erw> <num_steps>
		if (argc != 19) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: scan_line <sx> <sy> <sz> <srx> <sry> <srz> <srw> <ex> <ey> <ez> <erx> <ery> <erz> <erw> <num_steps>");
			rclcpp::shutdown();
			return 1;
		}
		{
			geometry_msgs::msg::Point start, end;
			start.x = std::stod(argv[4]);
			start.y = std::stod(argv[5]);
			start.z = std::stod(argv[6]);
			end.x = std::stod(argv[11]);
			end.y = std::stod(argv[12]);
			end.z = std::stod(argv[13]);
			int num_steps = std::stoi(argv[18]);
			if (num_steps < 2) num_steps = 2;
			// If TaskBuilder::scanLine is implemented, call it.
			builder.scanLine(start, end);
		}
		break;

		case CommandKind::CALIBRATE_CAMERA:
		// Expect: calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>
		if (argc != 11) {
			RCLCPP_ERROR(node->get_logger(),
						"Usage: calibrate_camera <x> <y> <z> <rx> <ry> <rz> <rw>");
			rclcpp::shutdown();
			return 1;
		}
		{
			double x = std::stod(argv[4]);
			double y = std::stod(argv[5]);
			double z = std::stod(argv[6]);
			// The orientation is not used in our simple example.
			builder.calibrateCamera(x, y, z);
		}
		break;

		case CommandKind::GCODE_LOAD:
		// Expect: gcode_load <gcode_filename>
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: gcode_load <gcode_filename>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string gcode_file = argv[4];
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
		}
		break;

		case CommandKind::STEP_LOAD:
		// Expect: step_load <step_filename>
		if (argc != 5) {
			RCLCPP_ERROR(node->get_logger(), "Usage: step_load <step_filename>");
			rclcpp::shutdown();
			return 1;
		}
		{
			std::string step_filename = argv[4];
			auto curve_poses = builder.stepLoad(step_filename);
			if (curve_poses.empty()) {
			RCLCPP_ERROR(node->get_logger(), "No valid poses generated from STEP file: %s", step_filename.c_str());
			rclcpp::shutdown();
			return 1;
			}
			// Optionally, save debug file:
			// savePosesToFile("step_curve_poses_debug.txt", curve_poses);
			// Optionally, call builder.trajectoryMove() if you have an overload.
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
	if (!builder.executeTask()) {
		RCLCPP_ERROR(node->get_logger(), "Failed to execute MTC task");
		rclcpp::shutdown();
		return 1;
	}

	RCLCPP_INFO(node->get_logger(), "Task complete!");
	rclcpp::shutdown();
	return 0;
}
