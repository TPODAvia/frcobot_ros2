#include "task_builder.hpp"

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>  // C++17
#include <fstream>
#include <chrono>
#include <thread>

namespace fs = std::filesystem;

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
    // Debug
    // RCLCPP_DEBUG(node_->get_logger(), "Received joint names:");
    // for (const auto& name : joint_names_) {
    //   RCLCPP_DEBUG(node_->get_logger(), "  %s", name.c_str());
    // }
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

// ---------------------------------------------------------------------
// The rest of your methods remain basically the same, except we have
// removed references to any hard-coded /home/... path in them. 
// ---------------------------------------------------------------------
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
  RCLCPP_INFO(node_->get_logger(),
              "Robot markers: (no direct single call in MoveIt C++).");

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
    // Log error as needed
    std::cerr << "[loadObjectLocations] Error reading file: " << e.what() << std::endl;
  }
  return object_map;
}

void TaskBuilder::saveObjectLocations(const std::string& file_path,
                                const std::map<std::string, geometry_msgs::msg::Pose>& object_map)
{
  YAML::Node root;
  YAML::Node objects_node;

  // If file already exists, load it first to preserve any unknown fields
  if (fs::exists(file_path)) {
    try {
      root = YAML::LoadFile(file_path);
      if (!root["objects"]) {
        root["objects"] = YAML::Node(YAML::NodeType::Map);
      }
      objects_node = root["objects"];
    }
    catch (const std::exception& e) {
      // If parsing fails, we start fresh
      std::cerr << "[saveObjectLocations] Warning: Could not parse existing YAML. Overwriting. " << e.what() << std::endl;
      root = YAML::Node();
      objects_node = YAML::Node(YAML::NodeType::Map);
    }
  } else {
    // No file -> start fresh
    objects_node = YAML::Node(YAML::NodeType::Map);
  }

  // Update objects_node with the current map
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

  // Place back in root
  root["objects"] = objects_node;

  // Write out to disk
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

  // Get known objects
  std::vector<std::string> known_objects = psi.getKnownObjectNames();
  if (std::find(known_objects.begin(), known_objects.end(), object_name) == known_objects.end()) {
    RCLCPP_INFO(node_->get_logger(), "Object '%s' not found in the scene.", object_name.c_str());
    return;
  }

  // -----------------------------------------------------
  // Retrieve the object's pose from the scene 
  // (assuming a single pose if the object has only one primitive).
  // -----------------------------------------------------
  auto object_map = psi.getObjects({object_name});
  if (object_map.find(object_name) != object_map.end()) {
    const moveit_msgs::msg::CollisionObject& obj_msg = object_map[object_name];
    if (!obj_msg.primitive_poses.empty()) {
      geometry_msgs::msg::Pose pose = obj_msg.primitive_poses[0];

      // Load existing locations from file
      std::string package_share_directory;
      try {
        package_share_directory = ament_index_cpp::get_package_share_directory("fairino_mtc_demo");
      } catch(const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Failed to get package share directory for 'fairino_mtc_demo': %s",
                     e.what());
        // fallback
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

      // Load map from file
      auto existing_locations = loadObjectLocations(object_locations_file);

      // Update with the new location for this object
      existing_locations[object_name] = pose;

      // Save back to file
      saveObjectLocations(object_locations_file, existing_locations);
      RCLCPP_INFO(node_->get_logger(), "Saved location of '%s' to %s", 
                  object_name.c_str(), object_locations_file.c_str());
    }
  }

  // Finally remove the object from the scene
  psi.removeCollisionObjects({object_name});
  RCLCPP_INFO(node_->get_logger(), "Object '%s' removed from scene.", object_name.c_str());
}

void TaskBuilder::spawnObject(const std::string& object_name, const std::string& object_shape,
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw, 
                              double da, double db, double dc)
{
  // -----------------------------------------------------
  // Check if user-provided coordinates are valid or "missing"
  // We'll treat them as missing if they are std::isnan().
  // Another approach could be (x==0 && y==0 && z==0) but that
  // might conflict if a valid location is genuinely at 0,0,0.
  // -----------------------------------------------------
  bool coordinates_provided = !(std::isnan(x) || std::isnan(y) || std::isnan(z) ||
                                std::isnan(rx) || std::isnan(ry) || std::isnan(rz) || std::isnan(rw));

  if (!coordinates_provided) {
    RCLCPP_INFO(node_->get_logger(), 
                "[spawn_object] Coordinates not provided; reading from memory if available.");

    // Attempt to load from memory
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
        "[spawn_object] Found previous location for '%s': (%.2f, %.2f, %.2f) / (%.2f,%.2f,%.2f,%.2f)",
        object_name.c_str(), x, y, z, rx, ry, rz, rw);
    } else {
      RCLCPP_WARN(node_->get_logger(), 
        "No recorded location for '%s' found in memory. Using default (0,0,0, identity orientation).",
        object_name.c_str());
      x=0; y=0; z=0;
      rx=0; ry=0; rz=0; rw=1;
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, shape=%s, pose=(%.2f, %.2f, %.2f), orient=(%.2f, %.2f, %.2f, %.2f)",
     object_name.c_str(), object_shape.c_str(), x, y, z, rx, ry, rz, rw);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = "world"; // or your planning frame

  // Set the object shape
  if (object_shape == "cylinder") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // da = height, db = radius
    object.primitives[0].dimensions = {da, db}; 
  }
  else if (object_shape == "box") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // da, db, dc = x, y, z
    object.primitives[0].dimensions = {da, db, dc};
  }
  else if (object_shape == "sphere") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    // da = radius
    object.primitives[0].dimensions = {da};
  }
  else if (object_shape == "cone") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CONE;
    // da = height, db = radius
    object.primitives[0].dimensions = {da, db};
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported shape type: %s", object_shape.c_str());
    return;
  }

  // Set pose
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = rx;
  pose.orientation.y = ry;
  pose.orientation.z = rz;
  pose.orientation.w = rw;
  object.pose = pose;

  // Finally spawn in the scene
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

  // Check for valid numeric poses
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
    task_.add(std::move(stage));

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

void TaskBuilder::trajectoryMove(const std::vector<geometry_msgs::msg::Pose>& trajectory_poses)
{
  RCLCPP_INFO(node_->get_logger(),
              "[trajectory_move] Moving along a trajectory with %zu poses",
              trajectory_poses.size());

  if (trajectory_poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "trajectoryMove() requires at least one pose");
    return;
  }

  auto robot_model = task_.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(node_->get_logger(), "No robot model found in the MTC task. Did you call initTask()?");
    return;
  }

  moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();

  std::vector<std::vector<double>> joint_waypoints;
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name_);
  if (!jmg) {
    RCLCPP_ERROR(node_->get_logger(), "Joint Model Group '%s' not found.", arm_group_name_.c_str());
    return;
  }

  for (size_t i = 0; i < trajectory_poses.size(); ++i) {
    bool found_ik = false;
    constexpr int    max_attempts        = 10;
    constexpr double per_attempt_timeout = 0.01;

    for (int attempt = 0; attempt < max_attempts && !found_ik; ++attempt) {
      found_ik = robot_state->setFromIK(jmg, trajectory_poses[i], tip_frame_, per_attempt_timeout);
    }

    if (!found_ik) {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for pose #%zu after %d attempts.", i, max_attempts);
      return;
    }

    std::vector<double> current_joints;
    robot_state->copyJointGroupPositions(jmg, current_joints);
    joint_waypoints.push_back(std::move(current_joints));
  }

  robot_trajectory::RobotTrajectory trajectory(robot_model, arm_group_name_);
  double total_time = 10.0;
  double dt = (joint_waypoints.size() > 1)
               ? total_time / (joint_waypoints.size() - 1)
               : total_time;

  for (size_t i = 0; i < joint_waypoints.size(); ++i) {
    robot_state->setJointGroupPositions(jmg, joint_waypoints[i]);
    trajectory.addSuffixWayPoint(*robot_state, i * dt);
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success_time = iptp.computeTimeStamps(trajectory, 0.7, 0.7);
  if (!success_time) {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed.");
    return;
  }

  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  trajectory.getRobotTrajectoryMsg(robot_traj_msg);
  RCLCPP_INFO(node_->get_logger(),
              "[trajectory_move] Successfully generated a time-parameterized "
              "trajectory with %zu waypoints.",
              joint_waypoints.size());
}

void TaskBuilder::trajectoryMoveV(const std::vector<geometry_msgs::msg::Pose>& trajectory,
                                  const std::vector<geometry_msgs::msg::Twist>& velocities)
{
  RCLCPP_INFO(node_->get_logger(),
    "[trajectory_move w/ vel] #poses=%zu, #vel=%zu",
    trajectory.size(), velocities.size());

  if (trajectory.size() != velocities.size() || trajectory.empty()) {
    RCLCPP_ERROR(node_->get_logger(), 
      "Mismatch in #poses and #velocities, or empty input. Aborting!");
    return;
  }

  auto robot_model = task_.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(node_->get_logger(), "No robot model found. Did you call initTask()?");
    return;
  }

  moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name_);
  if (!jmg) {
    RCLCPP_ERROR(node_->get_logger(), "Joint Model Group '%s' not found.", arm_group_name_.c_str());
    return;
  }

  std::vector<std::vector<double>> joint_waypoints;
  joint_waypoints.reserve(trajectory.size());

  for (size_t i = 0; i < trajectory.size(); ++i) {
    bool found_ik = false;
    constexpr int max_attempts = 10;
    constexpr double per_attempt_timeout = 0.01;
    for (int attempt = 0; attempt < max_attempts && !found_ik; ++attempt) {
      found_ik = robot_state->setFromIK(jmg, trajectory[i], tip_frame_, per_attempt_timeout);
    }
    if (!found_ik) {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for waypoint %zu", i);
      return;
    }
    std::vector<double> joints_now;
    robot_state->copyJointGroupPositions(jmg, joints_now);
    joint_waypoints.push_back(joints_now);
  }

  robot_trajectory::RobotTrajectory rt(robot_model, arm_group_name_);
  double time_so_far = 0.0;

  for (size_t i = 0; i < joint_waypoints.size(); ++i) {
    robot_state->setJointGroupPositions(jmg, joint_waypoints[i]);
    rt.addSuffixWayPoint(*robot_state, time_so_far);

    if (i + 1 < joint_waypoints.size()) {
      // naive distance between consecutive points
      double dx = trajectory[i+1].position.x - trajectory[i].position.x;
      double dy = trajectory[i+1].position.y - trajectory[i].position.y;
      double dz = trajectory[i+1].position.z - trajectory[i].position.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      double speed = std::sqrt(velocities[i].linear.x*velocities[i].linear.x +
                               velocities[i].linear.y*velocities[i].linear.y +
                               velocities[i].linear.z*velocities[i].linear.z);
      if (speed < 1e-6) speed = 0.01; // avoid zero
      double dt = dist / speed;
      time_so_far += dt;
    }
  }

  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, 0.7, 0.7)) {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed");
    return;
  }

  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  rt.getRobotTrajectoryMsg(robot_traj_msg);

  RCLCPP_INFO(node_->get_logger(),
    "Trajectory with velocities was created with %zu waypoints", joint_waypoints.size());
}

void TaskBuilder::feedbackMove()
{
  RCLCPP_INFO(node_->get_logger(), "[feedback_move] Executing feedback-based move");
  // This is highly application-specific. Not implemented here.
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
  // e.g., MoveTo with a "gripper" group and "closed" joint positions
}

void TaskBuilder::gripperOpen()
{
  RCLCPP_INFO(node_->get_logger(), "[gripper_open] Called");
  // e.g., MoveTo with a "gripper" group and "open" joint positions
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
