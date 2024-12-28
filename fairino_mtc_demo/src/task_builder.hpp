#pragma once

#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

namespace mtc = moveit::task_constructor;

/** 
 * The TaskBuilder class is responsible for building an MTC Task out of various 
 * "command" calls, each representing a single manipulation or scene operation.
 */
class TaskBuilder
{
public:
  /**
   * Constructor that sets the node and also allows you to specify which group
   * and tip frame you’ll use in the MTC pipeline.
   */
  TaskBuilder(rclcpp::Node::SharedPtr node,
              const std::string& arm_group_name,
              const std::string& tip_frame);

  // Clears the internal reference to the current MTC task and starts a new one.
  void newTask(const std::string& task_name);

  // ------------- Example "Command" Functions ---------------
  
  // Clear the scene
  void clearScene();

  // Remove an object by name
  void removeObject(const std::string& object_name);

  // Move the manipulator to a specified joint position
  void jointsPosition(const std::vector<double>& joint_values);

  // Spawn an object in the scene with the given pose
  void spawnObject(const std::string& object_name, double x, double y, double z,
                   double rx, double ry, double rz, double rw);

  // Switch pipeline / planner
  void choosePipeline(const std::string& pipeline_name, const std::string& planner_id);

  // Move end effector to coordinate
  void endCoordinate(const std::string& frame_id, 
                     double x=0, double y=0, double z=0,
                     double rx=0, double ry=0, double rz=0, double rw=1.0);

  // Attach object
  void attachObject(const std::string& object_name, const std::string& link_name);

  // Detach object
  void detachObject(const std::string& object_name, const std::string& link_name);

  // Gripper close
  void gripperClose();

  // Gripper open
  void gripperOpen();

  // Delete or clear some JSON simulation content
  void deleteJsonSimContent(const std::string& filename);

  // ------------- Build & Execute the MTC Task --------------
  
  // Initializes the underlying MTC Task
  bool initTask();

  // Plans the newly built MTC Task
  bool planTask(unsigned max_solutions = 10);

  // Executes the planned MTC Task
  bool executeTask();

private:
  // Helper: create a MoveRelative stage
  std::unique_ptr<mtc::stages::MoveRelative>
  createMoveRelativeStage(const std::string& name, double x, double y, double z);

  // ------------- Private Members ---------------
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

  // Robot group and tip frame
  std::string arm_group_name_;
  std::string tip_frame_;
};

