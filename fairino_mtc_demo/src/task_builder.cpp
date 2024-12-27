#include "task_builder.hpp"
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

TaskBuilder::TaskBuilder(rclcpp::Node::SharedPtr node)
  : node_{node}
{
  // Initialize planners once
  sampling_planner_      = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
//   interpolation_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  cartesian_planner_     = std::make_shared<mtc::solvers::CartesianPath>();
}

void TaskBuilder::newTask(const std::string& task_name)
{
  // Clear any existing task
  task_.clear();
  // Create a brand new MTC Task
  task_.stages()->setName(task_name);
  task_.loadRobotModel(node_);
  task_.setProperty("group", arm_group_name_);
  task_.setProperty("ik_frame", tip_frame_);

  // Always start with a CurrentState stage as the first stage
  auto current_state_stage = std::make_unique<mtc::stages::CurrentState>("current state");
  task_.add(std::move(current_state_stage));
}

void TaskBuilder::clearScene()
{
  // For demonstration, "clearScene" might remove all collision objects.
  // You could also do direct calls to PlanningSceneInterface, etc.
  // We'll just log here for now.
  RCLCPP_INFO(node_->get_logger(), "[clear_scene] Called");
  // ... your logic to clear the scene ...
}

void TaskBuilder::removeObject(const std::string& object_name)
{
  // Typically you might remove the object from the planning scene.
  RCLCPP_INFO(node_->get_logger(), "[remove_object] Removing %s", object_name.c_str());
  // ... your logic to remove object from scene ...
}

void TaskBuilder::jointsPosition(const std::vector<double>& joint_values)
{
  // Example: Add a "MoveTo" stage with specific joint values
  auto stage = std::make_unique<mtc::stages::MoveTo>("move to joints", sampling_planner_);
  stage->setGroup(arm_group_name_);
  stage->setGoal(joint_values);
  stage->setTimeout(10.0);
  task_.add(std::move(stage));
}

void TaskBuilder::spawnObject(const std::string& object_name, 
                              double x, double y, double z,
                              double rx, double ry, double rz, double rw)
{
  RCLCPP_INFO(node_->get_logger(),
    "[spawn_object] name=%s, pos=(%.2f,%.2f,%.2f), orient=(%.2f,%.2f,%.2f,%.2f)",
    object_name.c_str(), x, y, z, rx, ry, rz, rw);

  // Typically you'd call PlanningSceneInterface::applyCollisionObject() or similar.
  // This is just a placeholder.
}

void TaskBuilder::choosePipeline(const std::string& pipeline_name, const std::string& planner_id)
{
  RCLCPP_INFO(node_->get_logger(),
    "[choose_pipeline] pipeline=%s, planner_id=%s",
    pipeline_name.c_str(), planner_id.c_str());

  // Switch pipeline or set properties. For example, in MTC:
  // sampling_planner_->setPlannerId(planner_id);
  // sampling_planner_->setPlannerInterfaceName(pipeline_name);
}

void TaskBuilder::endCoordinate(const std::string& frame_id, 
                                double x, double y, double z,
                                double rx, double ry, double rz, double rw)
{
  RCLCPP_INFO(node_->get_logger(),
    "[end_coordinate] frame_id=%s, xyz=(%.2f,%.2f,%.2f), quat=(%.2f,%.2f,%.2f,%.2f)",
    frame_id.c_str(), x, y, z, rx, ry, rz, rw);

  // For a simple example, create a MoveRelative or MoveTo stage to move to that pose
  // using your cartesian_planner_ or sampling_planner_.
  // The exact approach depends on your usage. 
  // E.g., we can do a MoveRelative in the 'frame_id' coordinate frame:
  //
  // auto stage = createMoveRelativeStage("end_coordinate", x, y, z);
  // task_.add(std::move(stage));

  // If you actually want an absolute pose, you'd do MoveTo with a geometry_msgs::PoseStamped.
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
  // Here you might add a stage that closes the gripper joints,
  // or if your gripper is a separate group, a MoveTo that sets
  // those joint values. E.g.:
  // auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", sampling_planner_);
  // stage->setGroup("gripper_group");
  // stage->setGoal("close");  // or a set of joint values
  // task_.add(std::move(stage));
}

void TaskBuilder::gripperOpen()
{
  RCLCPP_INFO(node_->get_logger(), "[gripper_open] Called");
  // Similar to gripperClose(), but for open joints
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

std::unique_ptr<mtc::stages::MoveRelative>
TaskBuilder::createMoveRelativeStage(const std::string& name, double x, double y, double z)
{
  auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian_planner_);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.2);
  stage->setIKFrame(tip_frame_);
  stage->properties().set("marker_ns", name);

  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = x;
  vec.vector.y = y;
  vec.vector.z = z;
  stage->setDirection(vec);

  return stage;
}
