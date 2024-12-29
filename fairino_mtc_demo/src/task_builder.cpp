#include "task_builder.hpp"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

TaskBuilder::TaskBuilder(rclcpp::Node::SharedPtr node,
                         const std::string& arm_group_name,
                         const std::string& tip_frame)
  : node_{node}
  , arm_group_name_{arm_group_name}
  , tip_frame_{tip_frame}
{
  sampling_planner_  = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
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

void TaskBuilder::clearScene()
{
  // For demonstration, "clearScene" might remove all collision objects.
  RCLCPP_INFO(node_->get_logger(), "[clear_scene] Called");
  // ... your logic to clear the scene ...
}

void TaskBuilder::removeObject(const std::string& object_name)
{
  RCLCPP_INFO(node_->get_logger(), "[remove_object] Removing %s", object_name.c_str());
  // ... your logic to remove object from scene ...
}

void TaskBuilder::spawnObject(const std::string& object_name, 
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw)
{
  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, pos=(%.2f,%.2f,%.2f), orient=(%.2f,%.2f,%.2f,%.2f)",
            object_name.c_str(),    x, y, z,                  rx, ry, rz, rw);

  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1,0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -0.55;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

void TaskBuilder::choosePipeline(const std::string& pipeline_name, const std::string& planner_id)
{
  RCLCPP_INFO(node_->get_logger(),
    "[choose_pipeline] pipeline=%s, planner_id=%s",
    pipeline_name.c_str(), planner_id.c_str());

  // sampling_planner_->setPlannerId(planner_id);
  // sampling_planner_->setPlannerInterfaceName(pipeline_name);

  // // PTP Planner
  // auto pilz_ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  // pilz_ptp_planner->setPlannerId("pilz_industrial_motion_planner", "PTP");
  // pilz_ptp_planner->setProperty("max_velocity_scaling_factor", 0.5);
  // pilz_ptp_planner->setProperty("max_acceleration_scaling_factor", 0.5);
  // solvers_["pilz_PTP"] = pilz_ptp_planner;

  // // LIN Planner
  // auto pilz_lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  // pilz_lin_planner->setPlannerId("pilz_industrial_motion_planner", "LIN");
  // pilz_lin_planner->setProperty("max_velocity_scaling_factor", 0.2);
  // pilz_lin_planner->setProperty("max_acceleration_scaling_factor", 0.2);
  // solvers_["pilz_LIN"] = pilz_lin_planner;

  // // CIRC Planner
  // auto pilz_circ_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz_industrial_motion_planner");
  // pilz_circ_planner->setPlannerId("pilz_industrial_motion_planner", "CIRC");
  // pilz_circ_planner->setProperty("max_velocity_scaling_factor", 0.3);
  // pilz_circ_planner->setProperty("max_acceleration_scaling_factor", 0.3);
  // solvers_["pilz_CIRC"] = pilz_circ_planner;

  // RCLCPP_INFO(node_->get_logger(), "Initialized Pilz solvers: PTP, LIN, CIRC");

  // // Example OMPL planner
  // auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
  // ompl_planner->setPlannerId("RRTConnectkConfigDefault"); // Replace with your desired OMPL planner
  // // Set any additional properties for OMPL
  // ompl_planner->setProperty("max_velocity_scaling_factor", 0.8);
  // ompl_planner->setProperty("max_acceleration_scaling_factor", 0.8);
  // solvers_["ompl_RRTConnect"] = ompl_planner;

  // RCLCPP_INFO(node_->get_logger(), "Initialized OMPL solver: RRTConnect");
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

  // Create a MoveTo stage
  auto stage = std::make_unique<mtc::stages::MoveTo>("move to joints", sampling_planner_);
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

  // Create a MoveTo stage
  auto stage = std::make_unique<mtc::stages::MoveTo>("move_to_absolute_pose", sampling_planner_);
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
    RCLCPP_INFO(node_->get_logger(), "[trajectory_move] Moving along a trajectory with %zu poses", trajectory_poses.size());

    if (trajectory_poses.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "trajectoryMove() requires at least one pose");
        return;
    }

    // // Initialize RobotState
    // moveit::core::RobotStatePtr robot_state = std::make_shared<moveit::core::RobotState>(task_.robotModel());
    // robot_state->setToDefaultValues();

    // // Convert Poses to Joint Waypoints using IK
    // std::vector<std::vector<double>> joint_waypoints;
    // for (const auto& pose : trajectory_poses) {
    //     bool found_ik = robot_state->setFromIK(task_.robotModel()->getJointModelGroup(arm_group_name_), pose, tip_frame_, 10, 0.1);
    //     if (!found_ik) {
    //         RCLCPP_ERROR(node_->get_logger(), "IK failed for a pose in trajectory.");
    //         return;
    //     }
    //     joint_waypoints.emplace_back(robot_state->getJointGroupPositions(arm_group_name_));
    // }

    // // Generate Spline Trajectory
    // moveit_msgs::msg::RobotTrajectory robot_traj;
    // bool success = generateSplineTrajectory(task_.robotModel(), joint_waypoints, robot_traj, 10.0, 0.05); // total_time=10s, time_step=50ms
    // if (!success) {
    //     RCLCPP_ERROR(node_->get_logger(), "Failed to generate spline trajectory.");
    //     return;
    // }

    // // Optionally, time parameterization to compute accurate timing and velocities
    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // bool success_time = iptp.computeTimeStamps(robot_traj);
    // if (!success_time) {
    //     RCLCPP_ERROR(node_->get_logger(), "Time parameterization failed.");
    //     return;
    // }

    // // Create a GenerateTrajectory stage
    // auto generate_traj_stage = std::make_unique<mtc::stages::GenerateTrajectory>("Generate Spline Trajectory");
    // generate_traj_stage->setRobotState(robot_state);
    // generate_traj_stage->setTrajectory(robot_traj);
    // task_.add(std::move(generate_traj_stage));

    // // Create a ExecuteTrajectory stage
    // auto execute_traj_stage = std::make_unique<mtc::stages::ExecuteTrajectory>("Execute Spline Trajectory");
    // execute_traj_stage->properties().configureInitFrom(Stage::PARENT, { "group" });
    // execute_traj_stage->setTrajectoryGenerator([&](const mtc::SolutionBase& solution, moveit_msgs::msg::RobotTrajectory& trajectory) -> bool {
    //     trajectory = robot_traj;
    //     return true;
    // });
    // task_.add(std::move(execute_traj_stage));

    // RCLCPP_INFO(node_->get_logger(), "Added spline trajectory to the task.");
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