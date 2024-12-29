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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
// #include <moveit/task_constructor/stages/execute_trajectory.h>
// #include <moveit/trajectory_processing/spline_smoothing.h>
// #include <moveit/task_constructor/stages/generate_trajectory.h>
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

  // Spawn an object in the scene with the given pose
  void spawnObject(const std::string& object_name, double x, double y, double z,
                   double rx, double ry, double rz, double rw);

  // Switch pipeline / planner
  void choosePipeline(const std::string& pipeline_name, const std::string& planner_id);

  // Move the manipulator to a specified joint position
  void jointsMove(const std::vector<double>& joint_values);

  // Move end effector to coordinate
  void absoluteMove(const std::string& frame_id, 
                     double x=0, double y=0, double z=0,
                     double rx=0, double ry=0, double rz=0, double rw=1.0);

  // Move manipulator with a vector (e.g., relative move)
  void displacementMove(const std::vector<double>& move_vector);
  
  // Move manipulator along a trajectory
  void trajectoryMove(const std::vector<geometry_msgs::msg::Pose>& trajectory);
  
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

  // ------------- Private Members ---------------
  rclcpp::Node::SharedPtr node_;
  mtc::Task task_;

  // Planners
  std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

  // Robot group and tip frame
  std::string arm_group_name_;
  std::string tip_frame_;

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
                                double time_step = 0.05)
  {
      if (joint_waypoints.empty()) {
          RCLCPP_ERROR(rclcpp::get_logger("TaskBuilder"), "No joint waypoints provided for spline generation.");
          return false;
      }

      size_t num_joints = joint_waypoints[0].size();
      size_t num_waypoints = joint_waypoints.size();

      // Convert joint_waypoints to Eigen matrix
      Eigen::MatrixXd waypoints_matrix(num_waypoints, num_joints);
      for (size_t i = 0; i < num_waypoints; ++i) {
          if (joint_waypoints[i].size() != num_joints) {
              RCLCPP_ERROR(rclcpp::get_logger("TaskBuilder"), "Inconsistent number of joints in waypoints.");
              return false;
          }
          for (size_t j = 0; j < num_joints; ++j) {
              waypoints_matrix(i, j) = joint_waypoints[i][j];
          }
      }

      // // Perform spline interpolation for each joint
      // std::vector<std::vector<double>> interpolated_waypoints;
      // size_t interpolated_points = static_cast<size_t>(total_time / time_step) + 1;

      // for (size_t j = 0; j < num_joints; ++j) {
      //     Eigen::RowVectorXd joint_waypoints_eigen = waypoints_matrix.col(j).transpose();

      //     // Create a spline with the number of waypoints
      //     Eigen::Spline<double, 1> spline = Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
      //         joint_waypoints_eigen, num_waypoints - 1, Eigen::MatrixXd::Zero(1,1));

      //     // Generate interpolated points
      //     for (size_t i = 0; i < interpolated_points; ++i) {
      //         double t = static_cast<double>(i) / static_cast<double>(interpolated_points - 1);
      //         double joint_pos = spline(t)(0);
      //         if (j == 0) {
      //             interpolated_waypoints.emplace_back();
      //             interpolated_waypoints.back().resize(num_joints, 0.0);
      //         }
      //         interpolated_waypoints[i][j] = joint_pos;
      //     }
      // }

      // // Assign positions and compute velocities
      // trajectory_out.joint_trajectory.joint_names = robot_model->getJointModelGroup("arm")->getJointModelNames();

      // for (size_t i = 0; i < interpolated_waypoints.size(); ++i) {
      //     trajectory_msgs::msg::JointTrajectoryPoint point;
      //     point.positions = interpolated_waypoints[i];

      //     if (i == 0 || i == interpolated_waypoints.size() - 1) {
      //         point.velocities.assign(num_joints, 0.0);
      //     } else {
      //         // Simple finite difference for velocity estimation
      //         for (size_t j = 0; j < num_joints; ++j) {
      //             double velocity = (interpolated_waypoints[i + 1][j] - interpolated_waypoints[i - 1][j]) / (2 * time_step);
      //             point.velocities.push_back(velocity);
      //         }
      //     }

      //     // Assign time_from_start
      //     point.time_from_start = rclcpp::Duration::from_seconds(i * time_step).to_chrono<std::chrono::nanoseconds>();

      //     trajectory_out.joint_trajectory.points.push_back(point);
      // }

      return true;
  }

};

