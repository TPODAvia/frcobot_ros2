#include "task_builder.hpp"
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream> // For file I/O
#include <filesystem> // For directory handling (C++17)
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
  // For example, add sampling_planner_ to solvers_ map
  solvers_["sampling_planner"] = sampling_planner_;
  
  // Optionally, load solver configuration from memory
  std::string memory_dir = "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/memory/";
  std::string config_file = memory_dir + "current_solver_config.yaml";
  
  if (loadSolverConfig(config_file)) {
    RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration from %s", config_file.c_str());
  }
  else {
    RCLCPP_WARN(node_->get_logger(), "Failed to load solver configuration. Using default solver.");
  }
  
  // Use Reliable QoS profile
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
        // RCLCPP_INFO(node_->get_logger(), "Received joint names:");
        // for (const auto& name : joint_names_) {
        //     RCLCPP_INFO(node_->get_logger(), "  %s", name.c_str());
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

void TaskBuilder::saveSolverConfig(const std::string& file_path)
{
  YAML::Node config;
  
  // Find the current solver's key in the solvers_ map
  std::string current_solver_name = "unknown";
  std::string planner_id = "unknown";
  
  for (const auto& [name, solver_ptr] : solvers_) {
    if (solver_ptr == current_solver_) {
      current_solver_name = name;
      // Assuming PlannerInterface has a method getPlannerId(), which might not be the case.
      // If not, you'll need to store planner_id separately.
      // For this example, we'll assume we have stored planner_id when initializing the solver.
      // This requires additional member variables or a different approach.
      // Here, we'll skip setting planner_id in the config.
      break;
    }
  }

  config["pipeline_name"] = current_solver_name;
  // config["planner_id"] = planner_id; // Uncomment if planner_id is retrievable

  try {
    std::ofstream fout(file_path);
    fout << config;
    fout.close();
    RCLCPP_INFO(node_->get_logger(), "Saved solver configuration to %s", file_path.c_str());
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to save solver configuration: %s", e.what());
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
    std::string pipeline_name = config["pipeline_name"].as<std::string>();
    // std::string planner_id = config["planner_id"].as<std::string>(); // Uncomment if planner_id is saved

    RCLCPP_INFO(node_->get_logger(), "Loaded solver configuration: pipeline=%s", pipeline_name.c_str());

    // Re-initialize solvers based on loaded pipeline_name
    // Note: This assumes that `choosePipeline` logic can be reused here
    // Alternatively, factor out the solver initialization logic into a separate method
    choosePipeline(pipeline_name, ""); // Pass planner_id if available

    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to load solver configuration: %s", e.what());
    return false;
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
  RCLCPP_INFO(node_->get_logger(),
              "Robot markers: (no direct single call in MoveIt C++).");

  // ------------------------------------------------------------------
  // Wait for joint states if they haven’t been received yet
  // ------------------------------------------------------------------
  RCLCPP_INFO(node_->get_logger(), "****************************************************");
  RCLCPP_INFO(node_->get_logger(), "Live Joint Positions (from /joint_states subscriber):");

  constexpr int max_retries = 5;
  constexpr int sleep_ms = 500;
  for (int attempt = 0; attempt < max_retries && current_joint_positions_.empty(); ++attempt) {
    rclcpp::spin_some(node_);  // <-- Let ROS process any incoming /joint_states messages
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

  // Initialize the PlanningSceneInterface
  moveit::planning_interface::PlanningSceneInterface psi;

  // Retrieve all known collision object names in the planning scene
  std::vector<std::string> object_ids = psi.getKnownObjectNames();

  // Check if there are any objects to remove
  if (object_ids.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No collision objects found in the scene to remove.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Removing %zu collision object(s) from the scene.", object_ids.size());

  // Remove all retrieved collision objects
  psi.removeCollisionObjects(object_ids);

  // Optional: Wait briefly to ensure objects are removed before proceeding
  // This can be adjusted based on your system's performance
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // Verify removal by checking the known object names again
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

void TaskBuilder::removeObject(const std::string& object_name)
{
  RCLCPP_INFO(node_->get_logger(), "[remove_object] Removing %s", object_name.c_str());
  moveit::planning_interface::PlanningSceneInterface psi;

  // Retrieve all known collision object names in the planning scene
  std::vector<std::string> object_ids = psi.getKnownObjectNames();

  // Check if the specified object exists in the scene
  if (std::find(object_ids.begin(), object_ids.end(), object_name) == object_ids.end()) {
    RCLCPP_INFO(node_->get_logger(), "Object '%s' not found in the scene.", object_name.c_str());
    return;
  }

  psi.removeCollisionObjects({object_name});
}

void TaskBuilder::spawnObject(const std::string& object_name, const std::string& object_shape,
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw, 
                              double da, double db, double dc)
{
  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, pos=(%.2f, %.2f, %.2f), orient=(%.2f, %.2f, %.2f, %.2f)",
            object_name.c_str(), x, y, z, rx, ry, rz, rw);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_name; // Use the provided object_name
  object.header.frame_id = "world"; // Adjust as needed

  // Set the object shape based on the object_shape argument
  if (object_shape == "cylinder") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = {da, db}; // da: height, db: radius
  }
  else if (object_shape == "box") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    object.primitives[0].dimensions = {da, db, dc}; // da, db, dc: x, y, z dimensions
  }
  else if (object_shape == "sphere") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    object.primitives[0].dimensions = {da}; // da: radius
  }
  else if (object_shape == "cone") {
    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CONE;
    object.primitives[0].dimensions = {da, db}; // da: height, db: radius
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "Unsupported shape type: %s. Defaulting to cylinder.", object_shape.c_str());
    return;
  }

  // Set the object's pose
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = rx;
  pose.orientation.y = ry;
  pose.orientation.z = rz;
  pose.orientation.w = rw;
  object.pose = pose;

  // Apply the collision object to the planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

  RCLCPP_INFO(node_->get_logger(), "Spawned object '%s' with shape '%s' in the scene.", object_name.c_str(), object_shape.c_str());
}

void TaskBuilder::choosePipeline(const std::string& pipeline_name, const std::string& planner_id)
{
  RCLCPP_INFO(node_->get_logger(),
    "[choose_pipeline] pipeline=%s, planner_id=%s",
    pipeline_name.c_str(), planner_id.c_str());

  // Clear existing solvers
  solvers_.clear();

  // Example: Initialize different solvers based on pipeline_name
  if (pipeline_name == "pilz_industrial_motion_planner") {
    // Initialize Pilz PTP Planner
    auto pilz_ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_ptp_planner->setPlannerId(planner_id); // e.g., "PTP"
    pilz_ptp_planner->setProperty("max_velocity_scaling_factor", 0.5);
    pilz_ptp_planner->setProperty("max_acceleration_scaling_factor", 0.5);
    solvers_["pilz_PTP"] = pilz_ptp_planner;

    // Initialize Pilz LIN Planner
    auto pilz_lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_lin_planner->setPlannerId(planner_id); // e.g., "LIN"
    pilz_lin_planner->setProperty("max_velocity_scaling_factor", 0.2);
    pilz_lin_planner->setProperty("max_acceleration_scaling_factor", 0.2);
    solvers_["pilz_LIN"] = pilz_lin_planner;

    // Initialize Pilz CIRC Planner
    auto pilz_circ_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
    pilz_circ_planner->setPlannerId(planner_id); // e.g., "CIRC"
    pilz_circ_planner->setProperty("max_velocity_scaling_factor", 0.3);
    pilz_circ_planner->setProperty("max_acceleration_scaling_factor", 0.3);
    solvers_["pilz_CIRC"] = pilz_circ_planner;

    RCLCPP_INFO(node_->get_logger(), "Initialized Pilz solvers: PTP, LIN, CIRC");

    // Optionally, set a default active solver
    if (!solvers_.empty()) {
      current_solver_ = solvers_.begin()->second;
      RCLCPP_INFO(node_->get_logger(), "Set active solver to %s", solvers_.begin()->first.c_str());
    }
  }
  else if (pipeline_name == "ompl") {
    // Initialize OMPL Planner
    auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    ompl_planner->setPlannerId(planner_id); // e.g., "RRTConnectkConfigDefault"
    ompl_planner->setProperty("max_velocity_scaling_factor", 0.8);
    ompl_planner->setProperty("max_acceleration_scaling_factor", 0.8);
    solvers_["ompl_RRTConnect"] = ompl_planner;

    RCLCPP_INFO(node_->get_logger(), "Initialized OMPL solver: RRTConnect");

    // Set active solver
    current_solver_ = solvers_["ompl_RRTConnect"];
    RCLCPP_INFO(node_->get_logger(), "Set active solver to ompl_RRTConnect");
  }
  else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown pipeline name: %s", pipeline_name.c_str());
    // Optionally, set to default solver or handle error
    return;
  }

  // Save the selected solver configuration to the memory folder
  std::string memory_dir = "/home/vboxuser/colcon_ws/src/frcobot_ros2/fairino_mtc_demo/memory/";
  
  // Ensure the memory directory exists
  if (!fs::exists(memory_dir)) {
    try {
      fs::create_directories(memory_dir);
      RCLCPP_INFO(node_->get_logger(), "Created memory directory at %s", memory_dir.c_str());
    }
    catch (const fs::filesystem_error& e) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create memory directory: %s", e.what());
      return;
    }
  }

  std::string config_file = memory_dir + "current_solver_config.yaml";
  saveSolverConfig(config_file);
}

void TaskBuilder::jointsMove(const std::vector<double>& joint_values)
{
    if (joint_values.empty()) {
        RCLCPP_INFO(node_->get_logger(), "Waiting for joint states...");

        constexpr int max_retries = 5;  // Maximum number of retries
        constexpr int sleep_ms = 500;   // Sleep duration in milliseconds between retries

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
            RCLCPP_WARN(node_->get_logger(), "No joint positions received yet from /joint_states. Retrying...");
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

    // Build a map<joint_name, double>
    std::map<std::string, double> joints_map;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        joints_map[joint_names_[i]] = joint_values[i];
    }

    if (!current_solver_) {
        RCLCPP_ERROR(node_->get_logger(), "No active solver selected. Use choosePipeline() first.");
        return;
    }

    // Create a MoveTo stage to go to the new joint values
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
  // Check if tip_frame and target_frame are specified
  if (tip_frame != "None" && target_frame != "None") {
    RCLCPP_INFO(node_->get_logger(),
                "[absolute_move] Using tip_frame=%s and target_frame=%s (no pose provided).",
                tip_frame.c_str(), target_frame.c_str());

    // Call MoveIt logic to move based on frames only (no position/orientation)
    auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_frames", current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setGoal(target_frame); // Assume this method exists
    stage->setIKFrame(frame_id);
    task_.add(std::move(stage));

    return;
  }

  // Check if position and orientation values are valid (not "None")
  if (x == std::numeric_limits<double>::quiet_NaN() ||
      y == std::numeric_limits<double>::quiet_NaN() ||
      z == std::numeric_limits<double>::quiet_NaN() ||
      rx == std::numeric_limits<double>::quiet_NaN() ||
      ry == std::numeric_limits<double>::quiet_NaN() ||
      rz == std::numeric_limits<double>::quiet_NaN() ||
      rw == std::numeric_limits<double>::quiet_NaN())
  {
      // If none of the above cases matched, log an error
  RCLCPP_ERROR(node_->get_logger(),
               "[absolute_move] Invalid arguments: either specify (tip_frame, target_frame) or (x, y, z, rx, ry, rz, rw).");
    return;
  }
  
    RCLCPP_INFO(node_->get_logger(),
                "[absolute_move] Moving to pose (%.2f, %.2f, %.2f) with orientation (%.2f, %.2f, %.2f, %.2f) in frame_id=%s.",
                x, y, z, rx, ry, rz, rw, frame_id.c_str());

    // Define the absolute pose in the specified frame
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

    // Create a MoveTo stage with the specified pose
    auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_absolute_pose", current_solver_);
    stage->setGroup(arm_group_name_);
    stage->setGoal(goal_pose_stamped);
    stage->setIKFrame(tip_frame != "None" ? tip_frame : frame_id); // Use tip_frame if specified, else frame_id
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
                 "displacementMove() requires exactly 3 elements for translation (x, y, z) "
                 "and 3 elements for rotation (rx, ry, rz)");
    return;
  }

  // Check if all elements in translation_vector are zero
  bool is_translation_zero = (translation_vector[0] == 0.0 && 
                              translation_vector[1] == 0.0 && 
                              translation_vector[2] == 0.0);

  // Check if all elements in rotation_vector are zero
  bool is_rotation_zero = (rotation_vector[0] == 0.0 && 
                           rotation_vector[1] == 0.0 && 
                           rotation_vector[2] == 0.0);

  if (is_translation_zero && is_rotation_zero) {
    RCLCPP_WARN(node_->get_logger(), 
                "Both translation and rotation vectors are zero. No displacement move will be performed.");
    return;
  }

  if (!is_translation_zero) {
    RCLCPP_INFO(node_->get_logger(), 
                "[displacement_move] Moving by translation (%.2f, %.2f, %.2f)", 
                translation_vector[0], translation_vector[1], translation_vector[2]);

    // Create a MoveRelative stage for translation
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

    // Create a MoveRelative stage for rotation
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

  RCLCPP_INFO(node_->get_logger(), "Performed displacement move with translation and/or rotation.");
}



void TaskBuilder::trajectoryMove(const std::vector<geometry_msgs::msg::Pose>& trajectory_poses)
{
  RCLCPP_INFO(node_->get_logger(),
              "[trajectory_move] Moving along a trajectory with %zu poses",
              trajectory_poses.size());

  if (trajectory_poses.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "trajectoryMove() requires at least one pose");
    return;
  }

  // 1) Get RobotModel from the Task
  auto robot_model = task_.getRobotModel();
  if (!robot_model)
  {
    RCLCPP_ERROR(node_->get_logger(), "No robot model found in the MTC task. Did you call initTask()?");
    return;
  }

  // 2) Create a RobotState for IK
  moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();

  // 3) Prepare to store joint waypoints
  std::vector<std::vector<double>> joint_waypoints;
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name_);
  if (!jmg)
  {
    RCLCPP_ERROR(node_->get_logger(), "Joint Model Group '%s' not found.", arm_group_name_.c_str());
    return;
  }

  // 4) For each pose, do multiple IK attempts in a loop
  for (size_t i = 0; i < trajectory_poses.size(); ++i)
  {
    const auto& pose = trajectory_poses[i];
    
    bool found_ik = false;
    constexpr int    max_attempts        = 10;
    constexpr double per_attempt_timeout = 0.01;

    for (int attempt = 0; attempt < max_attempts && !found_ik; ++attempt)
    {
      found_ik = robot_state->setFromIK(
        jmg,         // JointModelGroup
        pose,        // Pose
        tip_frame_,  // End-effector link name
        per_attempt_timeout
        // Optional: GroupStateValidityCallbackFn, KinematicsQueryOptions, ...
      );
    }

    if (!found_ik)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "IK failed for pose #%zu after %d attempts.",
                   i, max_attempts);
      return;
    }

    // Store the resulting joint positions
    std::vector<double> current_joints;
    robot_state->copyJointGroupPositions(jmg, current_joints);
    joint_waypoints.push_back(std::move(current_joints));
  }

  // 5) Build a RobotTrajectory from these joint waypoints
  robot_trajectory::RobotTrajectory trajectory(robot_model, arm_group_name_);

  // Example total time of 10 seconds for the entire path
  double total_time = 10.0;
  double dt = (joint_waypoints.size() > 1)
               ? total_time / (joint_waypoints.size() - 1)
               : total_time;

  // Reuse 'robot_state' to add each waypoint into the trajectory
  for (size_t i = 0; i < joint_waypoints.size(); ++i)
  {
    robot_state->setJointGroupPositions(jmg, joint_waypoints[i]);
    // "time_from_start" for each waypoint
    trajectory.addSuffixWayPoint(*robot_state, i * dt);
  }

  // 6) Time-parameterize using Iterative Parabolic Time Parameterization (IPTP)
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  bool success_time = iptp.computeTimeStamps(
    trajectory,
    0.7,  // velocity_scaling_factor
    0.7   // acceleration_scaling_factor
  );
  if (!success_time)
  {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed.");
    return;
  }

  // 7) Optionally convert to a ROS message for logging, debugging, or manual execution
  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  trajectory.getRobotTrajectoryMsg(robot_traj_msg);

  // Example: If you have a standard MoveIt controller interface or action client:
  // some_trajectory_execution_client.execute(robot_traj_msg);

  RCLCPP_INFO(node_->get_logger(),
              "[trajectory_move] Successfully generated a time-parameterized "
              "trajectory with %zu waypoints.",
              joint_waypoints.size());

  // Done! The trajectory is now fully time-parameterized and can be published or executed.
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

  // 1) Robot model
  auto robot_model = task_.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(node_->get_logger(), "No robot model found. Did you call initTask()?");
    return;
  }

  // 2) IK
  moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
  robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name_);
  if (!jmg) {
    RCLCPP_ERROR(node_->get_logger(), "Joint Model Group '%s' not found.", arm_group_name_.c_str());
    return;
  }

  std::vector<std::vector<double>> joint_waypoints;
  joint_waypoints.reserve(trajectory.size());

  for (size_t i = 0; i < trajectory.size(); ++i)
  {
    const auto& pose = trajectory[i];
    bool found_ik = false;
    constexpr int max_attempts = 10;
    constexpr double per_attempt_timeout = 0.01;

    for (int attempt = 0; attempt < max_attempts && !found_ik; ++attempt) {
      found_ik = robot_state->setFromIK(jmg, pose, tip_frame_, per_attempt_timeout);
    }

    if (!found_ik) {
      RCLCPP_ERROR(node_->get_logger(), "IK failed for waypoint %zu", i);
      return;
    }
    // Store joint positions
    std::vector<double> joints_now;
    robot_state->copyJointGroupPositions(jmg, joints_now);
    joint_waypoints.push_back(joints_now);
  }

  // 3) Build RobotTrajectory
  robot_trajectory::RobotTrajectory rt(robot_model, arm_group_name_);

  // We'll do a naive time estimate by using the linear velocity magnitude 
  // between successive waypoints. This is very approximate:
  double time_so_far = 0.0;
  for (size_t i = 0; i < joint_waypoints.size(); ++i)
  {
    // Put the joints in `robot_state`:
    robot_state->setJointGroupPositions(jmg, joint_waypoints[i]);

    // Add to the trajectory
    rt.addSuffixWayPoint(*robot_state, time_so_far);

    if (i + 1 < joint_waypoints.size()) {
      // distance in cartesian space from pose[i] to pose[i+1]
      auto dx = trajectory[i+1].position.x - trajectory[i].position.x;
      auto dy = trajectory[i+1].position.y - trajectory[i].position.y;
      auto dz = trajectory[i+1].position.z - trajectory[i].position.z;
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

      // We'll treat `velocities[i].linear` as the speed in m/s
      double speed = std::sqrt(
        velocities[i].linear.x * velocities[i].linear.x +
        velocities[i].linear.y * velocities[i].linear.y +
        velocities[i].linear.z * velocities[i].linear.z
      );
      // Avoid divide by zero
      if (speed < 1e-6) speed = 0.01;

      double dt = dist / speed;
      time_so_far += dt;
    }
  }

  // 4) Time parameterization
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  if (!iptp.computeTimeStamps(rt, 0.7, 0.7)) {
    RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed");
    return;
  }

  // 5) Convert to msg or do further usage
  moveit_msgs::msg::RobotTrajectory robot_traj_msg;
  rt.getRobotTrajectoryMsg(robot_traj_msg);

  RCLCPP_INFO(node_->get_logger(),
    "Trajectory with velocities was created with %zu waypoints", joint_waypoints.size());
}


void TaskBuilder::feedbackMove()
{
  RCLCPP_INFO(node_->get_logger(), "[feedback_move] Executing feedback-based move");

  // Implement feedback-based movement, e.g., using hand movement or monitoring
  // This is highly application-specific and may involve ROS actions or services

  // Example placeholder:
  RCLCPP_WARN(node_->get_logger(), "feedbackMove() is not yet implemented.");
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
  // For example: if you have a "gripper" group, do a MoveTo to the "closed" pose.
}

void TaskBuilder::gripperOpen()
{
  RCLCPP_INFO(node_->get_logger(), "[gripper_open] Called");
  // Similarly for open
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