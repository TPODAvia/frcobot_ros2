#include "task_builder.hpp"
#include <moveit/robot_state/robot_state.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <thread>
#include <sstream>
#include <cmath>
#include <cctype>
#include <algorithm>

namespace fs = std::filesystem;

namespace {
// --- Helper structures and functions for G-code parsing ---
struct GCodeCommand
{
  std::string original_line;
  std::string code;
  std::map<char, double> params;
  std::string comment;
};

std::string extractCommandToken(const std::string& token)
{
  if (token.empty())
    return "";
  char c = std::toupper(token[0]);
  if (c == 'G' || c == 'M' || c == 'T')
    return token;
  return "";
}

GCodeCommand parseGCodeLine(const std::string& raw_line)
{
  GCodeCommand cmd;
  cmd.original_line = raw_line;
  std::string line = raw_line;
  auto sc_pos = line.find(';');
  if (sc_pos != std::string::npos)
  {
    cmd.comment = line.substr(sc_pos + 1);
    line = line.substr(0, sc_pos);
  }
  auto openParen = line.find('(');
  auto closeParen = line.find(')', openParen + 1);
  if (openParen != std::string::npos && closeParen != std::string::npos)
  {
    std::string parenComment = line.substr(openParen + 1, closeParen - openParen - 1);
    if (!cmd.comment.empty())
      cmd.comment += " | ";
    cmd.comment += parenComment;
    line.erase(openParen, closeParen - openParen + 1);
  }
  auto trimSpace = [](std::string & s)
  {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch){ return !std::isspace(ch); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end());
  };
  trimSpace(line);
  std::stringstream ss(line);
  std::string token;
  bool firstTokenProcessed = false;
  while (ss >> token)
  {
    if (!firstTokenProcessed)
    {
      std::string possibleCmd = extractCommandToken(token);
      if (!possibleCmd.empty())
      {
        cmd.code = possibleCmd;
        firstTokenProcessed = true;
        continue;
      }
      firstTokenProcessed = true;
    }
    if (token.size() >= 2)
    {
      char paramLetter = std::toupper(token[0]);
      std::string valStr = token.substr(1);
      try
      {
        double value = std::stod(valStr);
        cmd.params[paramLetter] = value;
      }
      catch(...) { /* ignore non-numeric tokens */ }
    }
  }
  return cmd;
}
} // end anonymous namespace


TaskBuilder::TaskBuilder(rclcpp::Node::SharedPtr node,
                         const std::string& arm_group_name,
                         const std::string& tip_frame)
  : node_{node}
  , arm_group_name_{arm_group_name}
  , tip_frame_{tip_frame}
{
  // Initialize solvers
  sampling_planner_  = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();

  // Set default solver
  current_solver_ = sampling_planner_;

  // Initialize default solvers map if needed
  solvers_["sampling_planner"] = sampling_planner_;

  // Initialize move_group_ (using MoveIt2’s MoveGroupInterface)
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_name_);

  // ---------------------------------------------------------------------
  // Use ament_index_cpp to get the path to your ROS 2 package's share dir
  // Adjust the package name as needed:
  // ---------------------------------------------------------------------
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to get package share directory for 'fairino_mtc_demo': %s",
                 e.what());
    // Fallback or handle error
    package_share_directory = "."; 
  }

  // Memory directory
  std::string memory_dir = package_share_directory + "/memory/";
  // YAML config file
  std::string config_file = memory_dir + "current_solver_config.yaml";

  // Attempt to load solver config
  if (loadSolverConfig(config_file)) {
    RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration from %s", config_file.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to load solver configuration. Using default solver.");
  }

  // Use Reliable QoS profile for joint states subscription
  rclcpp::QoS qos_profile = rclcpp::QoS(2).reliable();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",  // or another topic if applicable
      qos_profile,
      std::bind(&TaskBuilder::jointStateCallback, this, std::placeholders::_1));
}

void TaskBuilder::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (joint_names_.empty() && !msg->name.empty()) {
    joint_names_ = msg->name;  // Store joint names from the first message
  }

  // Store joint positions in the map
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->position.size()) {
      current_joint_positions_[msg->name[i]] = msg->position[i];
    }
  }
}

void TaskBuilder::newTask(const std::string& task_name)
{
  // Clear any existing task
  task_.clear();
  // Create a brand new MTC Task
  task_.stages()->setName(task_name);
  task_.loadRobotModel(node_);

  // Set MTC properties from our member variables
  task_.setProperty("group", arm_group_name_);
  task_.setProperty("ik_frame", tip_frame_);

  // Always start with a CurrentState stage as the first stage
  auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current state");
  task_.add(std::move(current_state_stage));
}

void TaskBuilder::savePipelineConfig(const std::string& pipeline_name,
                                     const std::string& planner_id,
                                     double max_vel_factor,
                                     double max_acc_factor)
{

  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to get package share directory for 'fairino_mtc_demo': %s",
                 e.what());
    return;
  }

  const std::string memory_dir = package_share_directory + "/memory/";
  if (!fs::exists(memory_dir)) {
    try {
      fs::create_directories(memory_dir);
      RCLCPP_INFO(node_->get_logger(), "Created memory directory at %s", memory_dir.c_str());
    } catch (const fs::filesystem_error& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to create memory directory: %s",
                   e.what());
      return;
    }
  }

  YAML::Node config;
  config["pipeline_name"] = pipeline_name;
  config["planner_id"] = planner_id;
  // Use the factors provided by the function parameters as a baseline
  config["max_velocity_scaling_factor"] = max_vel_factor;
  config["max_acceleration_scaling_factor"] = max_acc_factor;

  if (pipeline_name == "pilz_industrial_motion_planner")
  {
    if (planner_id == "PTP")
    {
      config["max_velocity_scaling_factor"] = 0.5;
      config["max_acceleration_scaling_factor"] = 0.5;
    }
    else if (planner_id == "LIN")
    {
      config["max_velocity_scaling_factor"] = 0.2;
      config["max_acceleration_scaling_factor"] = 0.2;
    }
    else
    {
      config["planner_id"] = "unknown";
    }
  }
  else if (pipeline_name == "ompl")
  {
    if (planner_id == "RRTConnect")
    {
      config["max_velocity_scaling_factor"] = 0.7;
      config["max_acceleration_scaling_factor"] = 0.7;
    }
    else if (planner_id == "PRM")
    {
      config["max_velocity_scaling_factor"] = 0.8;
      config["max_acceleration_scaling_factor"] = 0.8;
    }
    else
    {
      config["planner_id"] = "unknown";
    }
  }
  else
  {
    config["pipeline_name"] = "unknown";
  }

  if (config["pipeline_name"].as<std::string>() == "unknown" ||
      config["planner_id"].as<std::string>() == "unknown")
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to save solver configuration: pipeline_name or planner_id is unknown");
    return;
  }

  const std::string file_path = memory_dir + "current_solver_config.yaml";
  try {
    std::ofstream fout(file_path);
    fout << config;
    fout.close();
    RCLCPP_INFO(node_->get_logger(),
                "Saved solver configuration to %s",
                file_path.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to save solver configuration: %s",
                 e.what());
  }
}

bool TaskBuilder::loadSolverConfig(const std::string& file_path)
{
  if (!fs::exists(file_path)) {
    RCLCPP_WARN(node_->get_logger(), "Solver configuration file %s does not exist.", file_path.c_str());
    return false;
  }

  try {
    YAML::Node config = YAML::LoadFile(file_path);
    if (!config["pipeline_name"]) {
      RCLCPP_WARN(node_->get_logger(), "Missing 'pipeline_name' in config YAML.");
      return false;
    }

    std::string pipeline_name = config["pipeline_name"].as<std::string>();
    RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration: pipeline=%s", pipeline_name.c_str());

    if (!config["planner_id"]) {
      RCLCPP_WARN(node_->get_logger(), "Missing 'pipeline_name' in config YAML.");
      return false;
    }

    std::string planner_id = config["planner_id"].as<std::string>();
    RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration: planner_id=%s", planner_id.c_str());

    double vel_scale = 0.0, acc_scale = 0.0;

    if (pipeline_name == "pilz_industrial_motion_planner" || pipeline_name == "ompl") {
      vel_scale = config["max_velocity_scaling_factor"].as<double>();
      acc_scale = config["max_acceleration_scaling_factor"].as<double>();

      choosePipeline(pipeline_name, planner_id, vel_scale, acc_scale);
    }
    else {
      RCLCPP_ERROR(node_->get_logger(), "Unknown pipeline name in config: %s", pipeline_name.c_str());
      return false;
    }

    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load solver configuration: %s", e.what());
    return false;
  }
}

void TaskBuilder::choosePipeline(const std::string& pipeline_name,
                                 const std::string& planner_id,
                                 double max_vel_factor,
                                 double max_acc_factor)
{
  // Clear existing solvers
  solvers_.clear();

  RCLCPP_INFO(node_->get_logger(),
              "[choose_pipeline] pipeline=%s, planner_id=%s",
              pipeline_name.c_str(), planner_id.c_str());

  if (pipeline_name == "pilz_industrial_motion_planner") {
    // Initialize Pilz PTP Planner
    auto pilz_ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_ptp_planner->setPlannerId(planner_id.empty() ? "PTP" : planner_id);
    pilz_ptp_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    pilz_ptp_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
    solvers_["pilz_PTP"] = pilz_ptp_planner;

    // Initialize Pilz LIN Planner
    auto pilz_lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_lin_planner->setPlannerId(planner_id.empty() ? "LIN" : planner_id);
    pilz_lin_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    pilz_lin_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
    solvers_["pilz_LIN"] = pilz_lin_planner;

    // Initialize Pilz CIRC Planner
    auto pilz_circ_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_circ_planner->setPlannerId(planner_id.empty() ? "CIRC" : planner_id);
    pilz_circ_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    pilz_circ_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
    solvers_["pilz_CIRC"] = pilz_circ_planner;

    RCLCPP_INFO(node_->get_logger(), "Initialized Pilz solvers: PTP, LIN, CIRC");

    // Set a default active solver
    if (!solvers_.empty()) {
      current_solver_ = solvers_.begin()->second;
      RCLCPP_INFO(node_->get_logger(), "Set active solver to %s", solvers_.begin()->first.c_str());
    }
  }
  else if (pipeline_name == "ompl") {
    // Initialize OMPL Planner
    auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    ompl_planner->setPlannerId(planner_id.empty() ? "RRTConnectkConfigDefault" : planner_id);
    ompl_planner->setProperty("max_velocity_scaling_factor", max_vel_factor);
    ompl_planner->setProperty("max_acceleration_scaling_factor", max_acc_factor);
    solvers_["ompl_RRTConnect"] = ompl_planner;

    RCLCPP_INFO(node_->get_logger(), "Initialized OMPL solver: RRTConnect");

    // Set a default active solver
    current_solver_ = ompl_planner;
    RCLCPP_INFO(node_->get_logger(), "Set active solver to ompl_RRTConnect");
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown pipeline name: %s", pipeline_name.c_str());
    return;
  }
}

void TaskBuilder::printRobotParams() const
{
  auto robot_model = task_.getRobotModel();
  if (!robot_model) {
    RCLCPP_WARN(node_->get_logger(),
                "No robot model is currently loaded. "
                "Please call initTask() before querying robot parameters.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Planning frame: %s", robot_model->getModelFrame().c_str());

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Root link: %s", robot_model->getRootLinkName().c_str());

  const moveit::core::JointModelGroup* jmg =
      robot_model->getJointModelGroup(arm_group_name_);
  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Active joints in group '%s':", arm_group_name_.c_str());
  if (jmg) {
    for (const auto& aj : jmg->getActiveJointModelNames())
      RCLCPP_INFO(node_->get_logger(), "  %s", aj.c_str());
  } else {
    RCLCPP_WARN(node_->get_logger(),
                "JointModelGroup '%s' not found. Check MoveIt config.",
                arm_group_name_.c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "All joint names in the robot:");
  for (const auto& joint_name : robot_model->getJointModelNames())
    RCLCPP_INFO(node_->get_logger(), "  %s", joint_name.c_str());

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "All link names in the robot:");
  for (const auto& link_name : robot_model->getLinkModelNames())
    RCLCPP_INFO(node_->get_logger(), "  %s", link_name.c_str());

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Defined groups in SRDF:");
  for (const auto& group_name : robot_model->getJointModelGroupNames())
    RCLCPP_INFO(node_->get_logger(), "  %s", group_name.c_str());

  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Robot markers: (no direct single call in MoveIt C++).");

  // ------------------------------------------------------------------
  // Wait for joint states if they haven't been received yet
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Live Joint Positions (from /joint_states subscriber):");

  constexpr int max_retries = 5;
  constexpr int sleep_ms = 500;
  for (int attempt = 0; attempt < max_retries && current_joint_positions_.empty(); ++attempt) {
    rclcpp::spin_some(node_);  // Let ROS process any incoming /joint_states messages
    if (current_joint_positions_.empty()) {
      RCLCPP_WARN(node_->get_logger(), "No joint states received yet. Retrying...");
      rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
  }
  if (current_joint_positions_.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Failed to receive joint states after %d retries. "
                 "Check if the robot driver is publishing on /joint_states.",
                 max_retries);
    return;
  }

  for (const auto& kv : current_joint_positions_) {
    RCLCPP_INFO(node_->get_logger(), "  %s: %f", kv.first.c_str(), kv.second);
  }
}

void TaskBuilder::clearScene()
{
  RCLCPP_INFO(node_->get_logger(), "[clear_scene] Called");

  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<std::string> object_ids = psi.getKnownObjectNames();

  if (object_ids.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No collision objects found in the scene to remove.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Removing %zu collision object(s) from the scene.", object_ids.size());
  psi.removeCollisionObjects(object_ids);

  rclcpp::sleep_for(std::chrono::milliseconds(500));
  std::vector<std::string> remaining_objects = psi.getKnownObjectNames();
  if (remaining_objects.empty()) {
    RCLCPP_INFO(node_->get_logger(), "All collision objects have been successfully removed from the scene.");
  } else {
    RCLCPP_WARN(node_->get_logger(), "Some collision objects could not be removed:");
    for (const auto& obj : remaining_objects) {
      RCLCPP_WARN(node_->get_logger(), " - %s", obj.c_str());
    }
  }
}

std::map<std::string, geometry_msgs::msg::Pose> TaskBuilder::loadObjectLocations(const std::string& file_path)
{
  std::map<std::string, geometry_msgs::msg::Pose> object_map;

  if (!fs::exists(file_path))
    return object_map;  // return empty if file doesn't exist yet

  try {
    YAML::Node yaml_root = YAML::LoadFile(file_path);
    if (!yaml_root["objects"]) 
      return object_map;

    YAML::Node objects_node = yaml_root["objects"];
    for (auto it = objects_node.begin(); it != objects_node.end(); ++it) {
      std::string obj_name = it->first.as<std::string>();
      YAML::Node pose_node = it->second;

      geometry_msgs::msg::Pose pose;
      pose.position.x = pose_node["x"].as<double>();
      pose.position.y = pose_node["y"].as<double>();
      pose.position.z = pose_node["z"].as<double>();
      pose.orientation.x = pose_node["rx"].as<double>();
      pose.orientation.y = pose_node["ry"].as<double>();
      pose.orientation.z = pose_node["rz"].as<double>();
      pose.orientation.w = pose_node["rw"].as<double>();

      object_map[obj_name] = pose;
    }
  }
  catch (const std::exception& e) {
    std::cerr << "[loadObjectLocations] Error reading file: " << e.what() << std::endl;
  }
  return object_map;
}

void TaskBuilder::saveObjectLocations(const std::string& file_path,
                                const std::map<std::string, geometry_msgs::msg::Pose>& object_map)
{
  YAML::Node root;
  YAML::Node objects_node;

  if (fs::exists(file_path)) {
    try {
      root = YAML::LoadFile(file_path);
      if (!root["objects"]) {
        root["objects"] = YAML::Node(YAML::NodeType::Map);
      }
      objects_node = root["objects"];
    }
    catch (const std::exception& e) {
      std::cerr << "[saveObjectLocations] Warning: Could not parse existing YAML. Overwriting. " << e.what() << std::endl;
      root = YAML::Node();
      objects_node = YAML::Node(YAML::NodeType::Map);
    }
  } else {
    objects_node = YAML::Node(YAML::NodeType::Map);
  }

  for (const auto& kv : object_map) {
    const std::string& obj_name = kv.first;
    const auto& pose = kv.second;
    objects_node[obj_name]["x"]  = pose.position.x;
    objects_node[obj_name]["y"]  = pose.position.y;
    objects_node[obj_name]["z"]  = pose.position.z;
    objects_node[obj_name]["rx"] = pose.orientation.x;
    objects_node[obj_name]["ry"] = pose.orientation.y;
    objects_node[obj_name]["rz"] = pose.orientation.z;
    objects_node[obj_name]["rw"] = pose.orientation.w;
  }

  root["objects"] = objects_node;

  try {
    std::ofstream fout(file_path);
    fout << root;
    fout.close();
  }
  catch (const std::exception& e) {
    std::cerr << "[saveObjectLocations] Error writing file: " << e.what() << std::endl;
  }
}

void TaskBuilder::removeObject(const std::string& object_name)
{
  RCLCPP_INFO(node_->get_logger(), "[remove_object] Removing %s", object_name.c_str());
  moveit::planning_interface::PlanningSceneInterface psi;

  std::vector<std::string> known_objects = psi.getKnownObjectNames();
  if (std::find(known_objects.begin(), known_objects.end(), object_name) == known_objects.end()) {
    RCLCPP_INFO(node_->get_logger(), "Object '%s' not found in the scene.", object_name.c_str());
    return;
  }

  auto object_map = psi.getObjects({object_name});
  if (object_map.find(object_name) != object_map.end()) {
    const moveit_msgs::msg::CollisionObject& obj_msg = object_map[object_name];
    if (!obj_msg.primitive_poses.empty()) {
      geometry_msgs::msg::Pose pose = obj_msg.primitive_poses[0];

      std::string package_share_directory;
      try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
      } catch(const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s",
                     e.what());
        return;
      }
      std::string memory_dir = package_share_directory + "/memory/";
      if (!fs::exists(memory_dir)) {
        try {
          fs::create_directories(memory_dir);
        } catch (const fs::filesystem_error& e) {
          RCLCPP_ERROR(node_->get_logger(), "Failed to create memory directory: %s", e.what());
          return;
        }
      }
      std::string object_locations_file = memory_dir + "object_locations.yaml";

      auto existing_locations = loadObjectLocations(object_locations_file);
      existing_locations[object_name] = pose;
      saveObjectLocations(object_locations_file, existing_locations);
      RCLCPP_INFO(node_->get_logger(), "Saved location of '%s' to %s", 
                  object_name.c_str(), object_locations_file.c_str());
    }
  }

  psi.removeCollisionObjects({object_name});
  RCLCPP_INFO(node_->get_logger(), "Object '%s' removed from scene.", object_name.c_str());
}

void TaskBuilder::spawnObject(const std::string& object_name, const std::string& object_shape,
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw, 
                              double da, double db, double dc)
{
  bool coordinates_provided = !(std::isnan(x) || std::isnan(y) || std::isnan(z) ||
                                std::isnan(rx) || std::isnan(ry) || std::isnan(rz) || std::isnan(rw));

  if (!coordinates_provided) {
    RCLCPP_INFO(node_->get_logger(), 
                "[spawn_object] Coordinates not provided; reading from memory if available.");

    std::string package_share_directory;
    try {
      package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
    } catch(const std::exception& e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Failed to get package share directory for 'fairino_mtc_demo': %s",
                   e.what());
      return;
    }
    std::string memory_dir = package_share_directory + "/memory/";
    std::string object_locations_file = memory_dir + "object_locations.yaml";

    auto existing_locations = loadObjectLocations(object_locations_file);
    if (existing_locations.find(object_name) != existing_locations.end()) {
      geometry_msgs::msg::Pose remembered_pose = existing_locations[object_name];
      x  = remembered_pose.position.x;
      y  = remembered_pose.position.y;
      z  = remembered_pose.position.z;
      rx = remembered_pose.orientation.x;
      ry = remembered_pose.orientation.y;
      rz = remembered_pose.orientation.z;
      rw = remembered_pose.orientation.w;

      RCLCPP_INFO(node_->get_logger(),
        "[spawn_object] Found previous location for '%s': (%.2f, %.2f, %.2f) / (%.2f, %.2f, %.2f, %.2f)",
        object_name.c_str(), x, y, z, rx, ry, rz, rw);
    } else {
      RCLCPP_WARN(node_->get_logger(), 
        "No recorded location for '%s' found in memory. Using default (0,0,0, identity orientation).",
        object_name.c_str());
      x = 0; y = 0; z = 0;
      rx = 0; ry = 0; rz = 0; rw = 1;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, shape=%s, pose=(%.2f, %.2f, %.2f), orient=(%.2f, %.2f, %.2f, %.2f)",
     object_name.c_str(), object_shape.c_str(), x, y, z, rx, ry, rz, rw);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = "world"; // or your planning frame

  if (object_shape == "cylinder") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {da, db}; 
  }
  else if (object_shape == "box") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {da, db, dc};
  }
  else if (object_shape == "sphere") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    object.primitives[0].dimensions = {da};
  }
  else if (object_shape == "cone") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CONE;
    object.primitives[0].dimensions = {da, db};
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported shape type: %s", object_shape.c_str());
    return;
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = rx;
  pose.orientation.y = ry;
  pose.orientation.z = rz;
  pose.orientation.w = rw;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  RCLCPP_INFO(node_->get_logger(),
              "Spawned object '%s' with shape '%s' in the scene.",
              object_name.c_str(), object_shape.c_str());
}

void TaskBuilder::jointsMove(const std::vector<double>& joint_values)
{
  if (joint_values.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Waiting for joint states...");
    constexpr int max_retries = 5;
    constexpr int sleep_ms = 500;
    for (int attempt = 0; attempt < max_retries; ++attempt) {
      rclcpp::spin_some(node_);
      if (!current_joint_positions_.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Live Joint Positions (from /joint_states subscriber):");
        for (const auto& joint_name : joint_names_) {
          RCLCPP_INFO(node_->get_logger(), "  Joint %s: %f",
                      joint_name.c_str(), current_joint_positions_[joint_name]);
        }
        return;
      }
      RCLCPP_WARN(node_->get_logger(), "No joint positions received yet. Retrying...");
      rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
    }
    RCLCPP_ERROR(node_->get_logger(), "Failed to receive joint states after %d retries. Aborting.", max_retries);
    return;
  }

  rclcpp::spin_some(node_);
  if (joint_values.size() != joint_names_.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "jointsMove() called with %zu values, but expects %zu joints",
                 joint_values.size(), joint_names_.size());
    return;
  }

  std::map<std::string, double> joints_map;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joints_map[joint_names_[i]] = joint_values[i];
  }

  if (!current_solver_) {
    RCLCPP_ERROR(node_->get_logger(), "No active solver selected. Use choosePipeline() first.");
    return;
  }

  auto stage = std::make_unique<mtc::stages::MoveTo>("move to joints", current_solver_);
  stage->setGroup(arm_group_name_);
  stage->setGoal(joints_map);
  stage->setTimeout(10.0);

  task_.add(std::move(stage));
}

void TaskBuilder::absoluteMove(const std::string& frame_id, 
                               const std::string& tip_frame,
                               const std::string& target_frame,
                               double x, double y, double z,
                               double rx, double ry, double rz, double rw)
{
  if (tip_frame != "" && target_frame != "") {
    RCLCPP_INFO(node_->get_logger(),
                "[absolute_move] Using tip_frame=%s and target_frame=%s (no pose provided).",
                tip_frame.c_str(), target_frame.c_str());
    auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_frames", current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setGoal(target_frame);
    stage->setIKFrame(frame_id);
    task_.add(std::move(stage));
    return;
  }

  if (std::isnan(x) || std::isnan(y) || std::isnan(z) ||
      std::isnan(rx) || std::isnan(ry) || std::isnan(rz) || std::isnan(rw)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[absolute_move] Invalid arguments: specify (tip_frame, target_frame) or (x, y, z, rx, ry, rz, rw).");
    return;
  }
  
  RCLCPP_INFO(node_->get_logger(),
              "[absolute_move] Moving to pose (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, %.2f, %.2f) in frame_id=%s.",
              x, y, z, rx, ry, rz, rw, frame_id.c_str());

  geometry_msgs::msg::PoseStamped goal_pose_stamped;
  goal_pose_stamped.header.frame_id = frame_id;
  goal_pose_stamped.header.stamp = node_->now();
  goal_pose_stamped.pose.position.x = x;
  goal_pose_stamped.pose.position.y = y;
  goal_pose_stamped.pose.position.z = z;
  goal_pose_stamped.pose.orientation.x = rx;
  goal_pose_stamped.pose.orientation.y = ry;
  goal_pose_stamped.pose.orientation.z = rz;
  goal_pose_stamped.pose.orientation.w = rw;

  auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_absolute_pose", current_solver_);
  stage->setGroup(arm_group_name_);
  stage->setGoal(goal_pose_stamped);
  stage->setIKFrame(tip_frame != "None" ? tip_frame : frame_id);
  stage->setTimeout(10.0);

  task_.add(std::move(stage));
}

void TaskBuilder::displacementMove(const std::string& world_frame, 
                                   const std::string& tip_frame,
                                   const std::vector<double>& translation_vector,
                                   const std::vector<double>& rotation_vector)
{
  if (translation_vector.size() != 3 || rotation_vector.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(),
                 "displacementMove() requires exactly 3 elements for translation and 3 for rotation");
    return;
  }

  bool is_translation_zero = (translation_vector[0] == 0.0 && 
                              translation_vector[1] == 0.0 && 
                              translation_vector[2] == 0.0);
  bool is_rotation_zero = (rotation_vector[0] == 0.0 && 
                           rotation_vector[1] == 0.0 && 
                           rotation_vector[2] == 0.0);

  if (is_translation_zero && is_rotation_zero) {
    RCLCPP_WARN(node_->get_logger(), 
                "Both translation and rotation vectors are zero. Nothing to do.");
    return;
  }

  if (!is_translation_zero) {
    RCLCPP_INFO(node_->get_logger(), 
                "[displacement_move] Moving by translation (%.2f, %.2f, %.2f)", 
                translation_vector[0], translation_vector[1], translation_vector[2]);

    auto stage_translate = std::make_unique<mtc::stages::MoveRelative>("translate", cartesian_planner_);
    stage_translate->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_translate->setMinMaxDistance(0.1, 0.2);
    stage_translate->setIKFrame(tip_frame);
    stage_translate->properties().set("marker_ns", "translate");

    geometry_msgs::msg::Vector3Stamped translation;
    translation.header.frame_id = world_frame;
    translation.vector.x = translation_vector[0];
    translation.vector.y = translation_vector[1];
    translation.vector.z = translation_vector[2];
    stage_translate->setDirection(translation);

    task_.add(std::move(stage_translate));
  }

  if (!is_rotation_zero) {
    RCLCPP_INFO(node_->get_logger(), 
                "[displacement_move] Rotating by (%.2f, %.2f, %.2f)", 
                rotation_vector[0], rotation_vector[1], rotation_vector[2]);

    auto stage_rotate = std::make_unique<mtc::stages::MoveRelative>("rotate", cartesian_planner_);
    stage_rotate->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage_rotate->setMinMaxDistance(0.01, 0.2);
    stage_rotate->setIKFrame(tip_frame);
    stage_rotate->properties().set("marker_ns", "rotate");

    geometry_msgs::msg::TwistStamped rotation;
    rotation.header.frame_id = world_frame;
    rotation.twist.angular.x = rotation_vector[0];
    rotation.twist.angular.y = rotation_vector[1];
    rotation.twist.angular.z = rotation_vector[2];
    stage_rotate->setDirection(rotation);

    task_.add(std::move(stage_rotate));
  }

  RCLCPP_INFO(node_->get_logger(), "Performed displacement move with translation/rotation.");
}

void TaskBuilder::trajectoryMove(const std::string& csv_file,
                                 double velocity_scale,
                                 double accel_scale,
                                 double pose_tolerance)
{
  RCLCPP_INFO(node_->get_logger(), "[trajectoryMove] CSV: %s, vel=%.2f, accel=%.2f, tol=%.2f",
              csv_file.c_str(), velocity_scale, accel_scale, pose_tolerance);

  // 1) Parse CSV
  std::vector<geometry_msgs::msg::PoseStamped> waypoints = parseCsv(csv_file);
  if (waypoints.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No valid waypoints from CSV. Aborting trajectory_move.");
    return;
  }
  // 2) Optionally set the move_group tolerances
  move_group_->setGoalPositionTolerance(pose_tolerance);
  move_group_->setGoalOrientationTolerance(pose_tolerance);

  // 3) Plan & Execute
  planAndExecute(waypoints, velocity_scale, accel_scale, pose_tolerance);
}

void TaskBuilder::feedbackPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(),
              "[feedbackPoseCallback] Received new target Pose: (%.2f, %.2f, %.2f)",
              msg->pose.position.x, 
              msg->pose.position.y, 
              msg->pose.position.z);

  std::vector<geometry_msgs::msg::PoseStamped> single_target;
  single_target.push_back(*msg);

  planAndExecute(single_target, 0.5, 0.5, 0.01);
}

void TaskBuilder::feedbackMove(const std::string& pose_topic)
{
  RCLCPP_INFO(node_->get_logger(), "[feedbackMove] Subscribing to: %s", pose_topic.c_str());

  feedback_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 
      10, 
      std::bind(&TaskBuilder::feedbackPoseCallback, this, std::placeholders::_1));

  rclcpp::Time start_time = node_->now();
  while ((node_->now() - start_time).seconds() < 10.0) {
    rclcpp::spin_some(node_);
  }

  RCLCPP_INFO(node_->get_logger(), 
              "feedback_move: done waiting for new poses. Unsubscribing from %s.",
              pose_topic.c_str());

  feedback_sub_.reset();
}

void TaskBuilder::attachObject(const std::string& object_name, const std::string& link_name)
{
  RCLCPP_INFO(node_->get_logger(), "[attach_object] object=%s, link=%s", 
              object_name.c_str(), link_name.c_str());

  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage->attachObject(object_name, link_name);
  task_.add(std::move(stage));
}

void TaskBuilder::detachObject(const std::string& object_name, const std::string& link_name)
{
  RCLCPP_INFO(node_->get_logger(), "[detach_object] object=%s, link=%s", 
              object_name.c_str(), link_name.c_str());

  auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_name, link_name);
  task_.add(std::move(stage));
}

void TaskBuilder::gripperClose()
{
  RCLCPP_INFO(node_->get_logger(), "[gripper_close] Called");
  // Implement gripper-close action here.
}

void TaskBuilder::gripperOpen()
{
  RCLCPP_INFO(node_->get_logger(), "[gripper_open] Called");
  // Implement gripper-open action here.
}

void TaskBuilder::collaborativeMove(const std::string& torque_topic, const std::string& record_filename)
{
  RCLCPP_INFO(node_->get_logger(), "[collaborativeMove] torque_topic=%s, record=%s",
              torque_topic.c_str(), record_filename.c_str());

  torque_sub_ = node_->create_subscription<std_msgs::msg::String>(
      torque_topic, 10,
      std::bind(&TaskBuilder::torqueFeedbackCallback, this, std::placeholders::_1));

  rclcpp::Time start_time = node_->now();
  while ((node_->now() - start_time).seconds() < 5.0) {
    rclcpp::spin_some(node_);
    geometry_msgs::msg::Pose dummy_pose;
    dummy_pose.position.x = 0.0;
    dummy_pose.position.y = 0.0;
    dummy_pose.position.z = 0.0;
    dummy_pose.orientation.x = 0.0;
    dummy_pose.orientation.y = 0.0;
    dummy_pose.orientation.z = 0.0;
    dummy_pose.orientation.w = 1.0;
    recordPose(dummy_pose, record_filename);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(node_->get_logger(), "collaborative_move: finished. Motion was recorded to %s.", record_filename.c_str());
}

void TaskBuilder::torqueFeedbackCallback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "[torqueFeedbackCallback] Received torque feedback: %s", msg->data.c_str());
}

void TaskBuilder::recordPose(const geometry_msgs::msg::Pose& pose, const std::string& filename)
{
  std::ofstream outfile(filename, std::ios::app);
  if (!outfile.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not open file %s for recording pose", filename.c_str());
    return;
  }
  outfile << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
          << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << "," << pose.orientation.w << "\n";
  outfile.close();
  RCLCPP_INFO(node_->get_logger(), "Recorded pose to %s", filename.c_str());
}

bool TaskBuilder::initTask()
{
  RCLCPP_INFO(node_->get_logger(), "[initTask] Initializing MTC Task");
  try {
    task_.init();
    return true;
  } catch(const mtc::InitStageException& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Task init failed: " << ex);
    return false;
  }
}

bool TaskBuilder::planTask(unsigned max_solutions)
{
  RCLCPP_INFO(node_->get_logger(), "[planTask] Planning MTC Task");
  try {
    if (!task_.plan(max_solutions)) {
      RCLCPP_ERROR(node_->get_logger(), "Task planning failed");
      return false;
    }
    return true;
  } catch(const mtc::InitStageException& ex) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Task planning threw: " << ex);
    return false;
  }
}

bool TaskBuilder::executeTask()
{
  RCLCPP_INFO(node_->get_logger(), "[executeTask] Executing MTC Task");
  for (auto& solution : task_.solutions()) {
    task_.introspection().publishSolution(*solution);
    auto result = task_.execute(*solution);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Task execution failed!");
      return false;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "Task execution SUCCESS!");
  return true;
}

bool TaskBuilder::planAndExecute(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                                 double velocity_scale,
                                 double accel_scale,
                                 double pose_tolerance)
{
  if (!move_group_) {
    RCLCPP_ERROR(node_->get_logger(), "MoveGroupInterface not initialized!");
    return false;
  }
  std::vector<geometry_msgs::msg::Pose> poses;
  for (const auto& wp : waypoints) {
    poses.push_back(wp.pose);
  }
  moveit_msgs::msg::RobotTrajectory trajectory;
  double fraction = move_group_->computeCartesianPath(poses, 0.01, 0.0, trajectory);
  if (fraction < 0.9) {
    RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning failed (only achieved %.2f%% of path)", fraction * 100.0);
    return false;
  }
  move_group_->setMaxVelocityScalingFactor(velocity_scale);
  move_group_->setMaxAccelerationScalingFactor(accel_scale);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  bool success = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success) {
    RCLCPP_ERROR(node_->get_logger(), "Execution of Cartesian path failed");
  }
  return success;
}

std::vector<geometry_msgs::msg::PoseStamped> TaskBuilder::parseCsv(const std::string& csv_file)
{
  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::ifstream infile(csv_file);
  if (!infile.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not open CSV file: %s", csv_file.c_str());
    return waypoints;
  }
  std::string line;
  while (std::getline(infile, line)) {
    if (line.empty())
      continue;
    std::istringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      try {
        values.push_back(std::stod(token));
      } catch (...) {
        RCLCPP_WARN(node_->get_logger(), "Invalid number in CSV file: %s", token.c_str());
      }
    }
    if (values.size() < 3)
      continue;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";
    pose_stamped.header.stamp = node_->now();
    pose_stamped.pose.position.x = values[0];
    pose_stamped.pose.position.y = values[1];
    pose_stamped.pose.position.z = values[2];
    pose_stamped.pose.orientation.x = 0;
    pose_stamped.pose.orientation.y = 0;
    pose_stamped.pose.orientation.z = 0;
    pose_stamped.pose.orientation.w = 1;
    waypoints.push_back(pose_stamped);
  }
  infile.close();
  return waypoints;
}

void TaskBuilder::parseGcode(const std::string& gcode_file,
                  std::vector<geometry_msgs::msg::PoseStamped>& out_poses)
{
  RCLCPP_WARN(node_->get_logger(), "parseGcode is not implemented");
}

void TaskBuilder::parseSplineFile(const std::string& step_file,
                       std::vector<geometry_msgs::msg::PoseStamped>& out_poses)
{
  RCLCPP_WARN(node_->get_logger(), "parseSplineFile is not implemented");
}

bool TaskBuilder::generateSplineTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                const std::vector<std::vector<double>>& joint_waypoints,
                                moveit_msgs::msg::RobotTrajectory& trajectory_out,
                                double total_time,
                                double time_step)
{
  RCLCPP_WARN(node_->get_logger(), "generateSplineTrajectory is not implemented");
  return false;
}

std::vector<geometry_msgs::msg::Pose> TaskBuilder::gcodeLoad(const std::string& gcode_file, const std::string& mode)
{
  std::vector<GCodeCommand> all_commands;
  std::ifstream file(gcode_file);
  if (!file.is_open())
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open G-code file: %s", gcode_file.c_str());
    return {};
  }
  std::string raw_line;
  while (std::getline(file, raw_line))
  {
    if (raw_line.empty())
      continue;
    GCodeCommand cmd = parseGCodeLine(raw_line);
    if (!cmd.code.empty() || !cmd.params.empty())
      all_commands.push_back(cmd);
  }
  file.close();

  double x = 0.0, y = 0.0, z = 0.0;
  std::vector<geometry_msgs::msg::Pose> path;
  for (auto& c : all_commands)
  {
    if (c.code == "G0" || c.code == "G1")
    {
      if (c.params.find('X') != c.params.end())
        x = c.params.at('X');
      if (c.params.find('Y') != c.params.end())
        y = c.params.at('Y');
      if (c.params.find('Z') != c.params.end())
        z = c.params.at('Z');

      geometry_msgs::msg::Pose pose;
      pose.position.x = x;
      pose.position.y = y;
      pose.position.z = z;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      path.push_back(pose);
    }
  }
  return path;
}

std::vector<geometry_msgs::msg::Pose> TaskBuilder::stepLoad(const std::string& step_file)
{
  std::vector<geometry_msgs::msg::Pose> result;
  std::ifstream file(step_file);
  if (!file.is_open())
  {
    RCLCPP_ERROR(node_->get_logger(), "stepLoad: could not open file: %s", step_file.c_str());
    return result;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line))
    lines.push_back(line);
  file.close();

  std::string spline_ref;
  for (const auto &l : lines)
  {
    if (l.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos)
    {
      spline_ref = l;
      break;
    }
  }
  if (spline_ref.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No B_SPLINE_CURVE_WITH_KNOTS found in %s", step_file.c_str());
    return result;
  }
  std::vector<std::string> cartesian_refs;
  {
    auto startPos = spline_ref.find("(#");
    if (startPos == std::string::npos)
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not parse control points list.");
      return result;
    }
    auto endPos = spline_ref.find(")", startPos);
    if (endPos == std::string::npos)
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not parse control points list (no closing parenthesis).");
      return result;
    }
    std::string refs = spline_ref.substr(startPos, endPos - startPos);
    refs.erase(std::remove(refs.begin(), refs.end(), '('), refs.end());
    refs.erase(std::remove(refs.begin(), refs.end(), ')'), refs.end());
    std::stringstream ss(refs);
    std::string token;
    while (std::getline(ss, token, ','))
    {
      if (!token.empty() && token.find("#") != std::string::npos)
      {
        while (!token.empty() && std::isspace(token.front())) token.erase(token.begin());
        while (!token.empty() && std::isspace(token.back())) token.pop_back();
        cartesian_refs.push_back(token);
      }
    }
  }
  if (cartesian_refs.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No cartesian point references found.");
    return result;
  }
  struct Point3 { double x, y, z; };
  std::vector<Point3> control_points;
  for (const auto &ref : cartesian_refs)
  {
    for (const auto &l : lines)
    {
      if (l.rfind(ref + "=", 0) == 0 && l.find("CARTESIAN_POINT") != std::string::npos)
      {
        auto cpointPos = l.find("CARTESIAN_POINT(");
        if (cpointPos == std::string::npos)
          continue;
        cpointPos += std::strlen("CARTESIAN_POINT(");
        auto secondParen = l.find("(", cpointPos);
        if (secondParen == std::string::npos)
          continue;
        auto endParen = l.find(")", secondParen + 1);
        if (endParen == std::string::npos)
          continue;
        std::string coords = l.substr(secondParen + 1, endParen - (secondParen + 1));
        std::stringstream ssc(coords);
        std::string val;
        std::vector<double> vals;
        while (std::getline(ssc, val, ','))
        {
          while (!val.empty() && std::isspace(val.front())) val.erase(val.begin());
          while (!val.empty() && std::isspace(val.back())) val.pop_back();
          try {
            vals.push_back(std::stod(val));
          } catch (...) { }
        }
        if (vals.size() == 3)
          control_points.push_back({vals[0], vals[1], vals[2]});
        break;
      }
    }
  }
  if (control_points.size() < 2)
  {
    RCLCPP_ERROR(node_->get_logger(), "Not enough control points to form a curve.");
    return result;
  }
  int num_samples = 20;
  for (int i = 0; i < num_samples; ++i)
  {
    double t = static_cast<double>(i) / (num_samples - 1);
    double float_index = t * (control_points.size() - 1);
    int idx0 = static_cast<int>(std::floor(float_index));
    int idx1 = std::min(idx0 + 1, static_cast<int>(control_points.size() - 1));
    double ratio = float_index - idx0;
    double x = control_points[idx0].x + ratio * (control_points[idx1].x - control_points[idx0].x);
    double y = control_points[idx0].y + ratio * (control_points[idx1].y - control_points[idx0].y);
    double z = control_points[idx0].z + ratio * (control_points[idx1].z - control_points[idx0].z);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    result.push_back(pose);
  }
  return result;
}

void TaskBuilder::scanLine(const geometry_msgs::msg::Point& start,
                           const geometry_msgs::msg::Point& end)
{
  RCLCPP_INFO(node_->get_logger(), "[scanLine] from (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",
              start.x, start.y, start.z, end.x, end.y, end.z);

  geometry_msgs::msg::PoseStamped poseA;
  poseA.header.frame_id = "base_link";
  poseA.pose.position = start;
  poseA.pose.orientation.x = 0.0;
  poseA.pose.orientation.y = 0.0;
  poseA.pose.orientation.z = 0.0;
  poseA.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped poseB = poseA;
  poseB.pose.position = end;

  std::vector<geometry_msgs::msg::PoseStamped> path;
  path.push_back(poseA);
  path.push_back(poseB);

  planAndExecute(path, 0.5, 0.5, 0.01);

  RCLCPP_INFO(node_->get_logger(), "Scan Finish");
}

void TaskBuilder::calibrateCamera(double x, double y, double z)
{
  RCLCPP_INFO(node_->get_logger(), "[calibrateCamera] target = (%.2f,%.2f,%.2f) in base_link", x, y, z);

  std::vector<geometry_msgs::msg::PoseStamped> path;
  double radius = 0.05;
  for (int i = 0; i < 8; ++i) {
    double angle = 2.0 * M_PI * (static_cast<double>(i) / 8.0);
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "base_link";
    p.pose.position.x = x + radius * std::cos(angle);
    p.pose.position.y = y + radius * std::sin(angle);
    p.pose.position.z = z;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = 0.0;
    p.pose.orientation.w = 1.0;
    path.push_back(p);
  }

  planAndExecute(path, 0.2, 0.2, 0.01);
}
