#include "task_builder.hpp"
#include <moveit/task_constructor/stages/move_to.h>
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
  psi.removeCollisionObjects({object_name});
}

void TaskBuilder::spawnObject(const std::string& object_name, 
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw)
{
  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, pos=(%.2f, %.2f, %.2f), orient=(%.2f, %.2f, %.2f, %.2f)",
            object_name.c_str(),    x, y, z,                  rx, ry, rz, rw);

  moveit_msgs::msg::CollisionObject object;
  object.id = object_name; // Use the provided object_name
  object.header.frame_id = "world"; // Adjust as needed
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.02}; // Example dimensions

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

  RCLCPP_INFO(node_->get_logger(), "Spawned object '%s' in the scene.", object_name.c_str());
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
  // Example for a 6-joint robot; adapt to your robot’s actual joint names/count:
  static const std::vector<std::string> JOINT_NAMES = {
    "j1", "j2", "j3", "j4", "j5", "j6"
  };

  // Build a map<joint_name, value>
  if (joint_values.size() != JOINT_NAMES.size()) {
    RCLCPP_ERROR(node_->get_logger(), 
      "jointsMove() called with %zu values, but robot expects %zu joints",
      joint_values.size(), JOINT_NAMES.size());
    return;
  }

  std::map<std::string, double> joints_map;
  for (size_t i = 0; i < JOINT_NAMES.size(); ++i) {
    joints_map[JOINT_NAMES[i]] = joint_values[i];
  }

  if (!current_solver_) {
    RCLCPP_ERROR(node_->get_logger(), "No active solver selected. Use choose_pipeline first.");
    return;
  }

  // Create a MoveTo stage
  auto stage = std::make_unique<mtc::stages::MoveTo>("move to joints", current_solver_);
  stage->setGroup(arm_group_name_);
  stage->setGoal(joints_map);  // <-- setGoal with a map
  stage->setTimeout(10.0);

  // Add to the task
  task_.add(std::move(stage));
}

void TaskBuilder::absoluteMove(const std::string& frame_id, 
                                double x, double y, double z,
                                double rx, double ry, double rz, double rw)
{
  RCLCPP_INFO(node_->get_logger(),
    "[absolute_move] frame_id=%s, xyz=(%.2f,%.2f,%.2f), quat=(%.2f,%.2f,%.2f,%.2f)",
    frame_id.c_str(), x, y, z, rx, ry, rz, rw);

  // Define the absolute pose in "world" frame
  geometry_msgs::msg::PoseStamped goal_pose_stamped;
  goal_pose_stamped.header.frame_id = frame_id; // Set to "world"
  goal_pose_stamped.header.stamp = node_->now();
  goal_pose_stamped.pose.position.x = x;
  goal_pose_stamped.pose.position.y = y;
  goal_pose_stamped.pose.position.z = z;
  goal_pose_stamped.pose.orientation.x = rx;
  goal_pose_stamped.pose.orientation.y = ry;
  goal_pose_stamped.pose.orientation.z = rz;
  goal_pose_stamped.pose.orientation.w = rw;

  if (!current_solver_) {
    RCLCPP_ERROR(node_->get_logger(), "No active solver selected. Use choose_pipeline first.");
    return;
  }

  // Create a MoveTo stage
  auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_absolute_pose", current_solver_);
  stage->setGroup(arm_group_name_);
  stage->setGoal(goal_pose_stamped); // Correct: Pass PoseStamped
  stage->setIKFrame(tip_frame_); // Set IK frame to "world"
  stage->setTimeout(10.0);

  // Optionally, set additional properties if needed
  // stage->properties().set("description", "Moving to absolute coordinate in world frame");

  // Add to the task
  task_.add(std::move(stage));

  RCLCPP_INFO(node_->get_logger(), "Added MoveTo stage to move to coordinate in frame '%s'.", frame_id.c_str());
}

void TaskBuilder::displacementMove(const std::vector<double>& move_vector)
{
  RCLCPP_INFO(node_->get_logger(), "[vector_move] Moving by vector (%.2f, %.2f, %.2f)", 
              move_vector[0], move_vector[1], move_vector[2]);

  if (move_vector.size() != 3) {
    RCLCPP_ERROR(node_->get_logger(), "displacementMove() requires exactly 3 elements (x, y, z)");
    return;
  }

  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("liftobj", cartesian_planner_);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.1, 0.2);
  stage_lift->setIKFrame(tip_frame_);
  stage_lift->properties().set("marker_ns", "liftobj");

  geometry_msgs::msg::Vector3Stamped vec;
  vec.vector.x = move_vector[0];
  vec.vector.y = move_vector[1];
  vec.vector.z = move_vector[2];
  stage_lift->setDirection(vec);
  task_.add(std::move(stage_lift));


  RCLCPP_INFO(node_->get_logger(), "Performed vector move.");
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

void TaskBuilder::deleteJsonSimContent(const std::string& filename)
{
  RCLCPP_INFO(node_->get_logger(), "[delete_json_sim_content] file=%s", filename.c_str());
  // Do whatever is needed to remove or reset JSON-based simulation content
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