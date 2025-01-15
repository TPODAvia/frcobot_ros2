#pragma once

#include <string>
#include <vector>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>

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
  /** 
   * @brief Prints various robot parameters, akin to Python's "get_planning_frame", "get_root_link",
   *        "get_joint_names", etc.
   */
  void printRobotParams() const;

  // Clear the scene
  void clearScene();

  // Remove an object by name
  void removeObject(const std::string& object_name);

  // Spawn an object in the scene with the given pose
  void spawnObject(const std::string& object_name, const std::string& object_shape, 
                  double x, double y, double z,
                  double rx, double ry, double rz, double rw, 
                  double da, double db, double dc);

  // Switch pipeline / planner
  void choosePipeline(const std::string& pipeline_name, const std::string& planner_id);

  // Move the manipulator to a specified joint position
  void jointsMove(const std::vector<double>& joint_values);

  // Move end effector to coordinate
  void absoluteMove(const std::string& frame_id, const std::string& tip_frame, const std::string& target_frame,
                     double x=0, double y=0, double z=0,
                     double rx=0, double ry=0, double rz=0, double rw=1.0);

  // Move manipulator with a vector (e.g., relative move)
  void displacementMove(const std::string& world_frame, const std::string& tip_frame, const std::vector<double>& move_vector, const std::vector<double>& rotation_vector);
  
  // Move manipulator along a trajectory
  void trajectoryMove(const std::vector<geometry_msgs::msg::Pose>& trajectory);
  void trajectoryMoveV(const std::vector<geometry_msgs::msg::Pose>& trajectory,
                      const std::vector<geometry_msgs::msg::Twist>& velocities);
  
  // Move manipulator with feedback (e.g., using feedback control)
  void feedbackMove();

  // Attach object
  void attachObject(const std::string& object_name, const std::string& link_name);

  // Detach object
  void detachObject(const std::string& object_name, const std::string& link_name);

  // Gripper close
  void gripperClose();

  // Gripper open
  void gripperOpen();

  // ------------- Build & Execute the MTC Task --------------
  
  // Initializes the underlying MTC Task
  bool initTask();

  // Plans the newly built MTC Task
  bool planTask(unsigned max_solutions = 10);

  // Executes the planned MTC Task
  bool executeTask();

private:

  // ------------- Private Members ---------------
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  // Solvers map: solver name -> solver instance
  std::map<std::string, mtc::solvers::PlannerInterfacePtr> solvers_;

  // Pointer to the currently active solver
  mtc::solvers::PlannerInterfacePtr current_solver_;
  
  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

  // Robot group and tip frame
  std::string arm_group_name_;
  std::string tip_frame_;

  // We'll store the latest joint positions from /joint_states in here:
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, double> current_joint_positions_;
  
  // The subscriber to /joint_states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // The callback that updates current_joint_positions_
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  
  /**
   * @brief Generates a spline-interpolated trajectory from joint waypoints.
   * 
   * @param robot_model The robot's kinematic model.
   * @param joint_waypoints Vector of joint positions (each element is a vector of joint values).
   * @param trajectory_out Output RobotTrajectory with positions and velocities.
   * @param total_time Total time to execute the trajectory.
   * @param time_step Time step between trajectory points.
   * @return true if successful, false otherwise.
   */
  bool generateSplineTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                const std::vector<std::vector<double>>& joint_waypoints,
                                moveit_msgs::msg::RobotTrajectory& trajectory_out,
                                double total_time = 5.0,
                                double time_step = 0.05);
  
  /**
   * @brief Saves the current solver configuration to a YAML file.
   * 
   * @param file_path The path to the YAML configuration file.
   */
  void saveSolverConfig(const std::string& file_path);
  
  /**
   * @brief Loads the solver configuration from a YAML file and sets the current_solver_.
   * 
   * @param file_path The path to the YAML configuration file.
   * @return true if successful, false otherwise.
   */
  bool loadSolverConfig(const std::string& file_path);
};

