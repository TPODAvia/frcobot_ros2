#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/Splines>
#include <cmath>  // for std::isnan

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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

    // Prints various robot parameters, akin to Python's "get_planning_frame", "get_root_link" etc.
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
    void choosePipeline(const std::string& pipeline_name,
                        const std::string& planner_id,
                        double max_vel_factor,
                        double max_acc_factor);

    // Saves the current solver configuration to a YAML file.
    void savePipelineConfig(const std::string& pipeline_name,
                            const std::string& planner_id,
                            double max_vel_factor,
                            double max_acc_factor);

    // Move the manipulator to a specified joint position
    void jointsMove(const std::vector<double>& joint_values);

    // Move end effector to coordinate
    void absoluteMove(const std::string& frame_id, const std::string& tip_frame, const std::string& target_frame,
                        double x = 0, double y = 0, double z = 0,
                        double rx = 0, double ry = 0, double rz = 0, double rw = 1.0);

    // Move manipulator with a vector (e.g., relative move)
    void displacementMove(const std::string& world_frame, const std::string& tip_frame, 
                            const std::vector<double>& move_vector, const std::vector<double>& rotation_vector);
    
    // Move manipulator along a trajectory
    // Load a trajectory from a CSV file that has columns [x, y, z].
    // Orientation is free (we set it to [0,0,0,1] by default).
    // Then plan and execute with optional velocity/accel/tolerance arguments.
    void trajectoryMove(const std::string& csv_file,
                        double velocity_scale,
                        double accel_scale,
                        double pose_tolerance);

    // Subscribes to a PoseStamped topic (e.g. "/pose_feedback") and moves in real-time
    // from A to B. Simplified example that stops after receiving a certain message.
    void feedbackMove(const std::string& pose_topic);

    // Subscribes to a torque topic (simplified) to sense an external force,
    // then moves collaboratively. The motion is recorded to a local file
    // so we can "play it back" later if desired.
    void collaborativeMove(const std::string& torque_topic, const std::string& record_filename);

    // Attach object
    void attachObject(const std::string& object_name, const std::string& link_name);

    // Detach object
    void detachObject(const std::string& object_name, const std::string& link_name);

    // Gripper close
    void gripperClose();

    // Gripper open
    void gripperOpen();

    // Initializes the underlying MTC Task
    bool initTask();

    // Plans the newly built MTC Task
    bool planTask(unsigned max_solutions = 10);

    // Executes the planned MTC Task
    bool executeTask();

    // Moves straight line from point A to B, orientation forced to [0,0,0,1].
    // After finishing, print "Scan Finish".
    void scanLine(const geometry_msgs::msg::Point& start,
                    const geometry_msgs::msg::Point& end);

    // Moves the end-effector "arbitrarily" but keeps it pointed at the 3D point [x,y,z]
    // relative to 'base_link'. (Simplified example with a fixed path.)
    void calibrateCamera(double x, double y, double z);

    // Loads G-code from file. We handle 3 user options: "CNC", "Weld", "3D Print".
    // Then convert the G-code lines to a path and make the robot follow them.
    std::vector<geometry_msgs::msg::Pose> gcodeLoad(const std::string& gcode_file, const std::string& mode);

    // Reads a file that contains a Spline (STEP or custom format),
    // and follows it exactly like `trajectoryMove`.
    std::vector<geometry_msgs::msg::Pose> stepLoad(const std::string& step_file);

private:

    // ------------- Private Members ---------------
    rclcpp::Node::SharedPtr node_;
    mtc::Task task_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    // Planners
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;

    // Declare the subscribers for feedback and torque
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr feedback_sub_;
    // The subscriber to /joint_states
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr torque_sub_;
    
    // Solvers map: solver name -> solver instance
    std::map<std::string, mtc::solvers::PlannerInterfacePtr> solvers_;

    // Pointer to the currently active solver
    mtc::solvers::PlannerInterfacePtr current_solver_;

    // Robot group and tip frame
    std::string arm_group_name_;
    std::string tip_frame_;

    // We'll store the latest joint positions from /joint_states in here:
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, double> current_joint_positions_;

    // The callback that updates current_joint_positions_
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Generates a spline-interpolated trajectory from joint waypoints.
    bool generateSplineTrajectory(const moveit::core::RobotModelConstPtr& robot_model,
                                    const std::vector<std::vector<double>>& joint_waypoints,
                                    moveit_msgs::msg::RobotTrajectory& trajectory_out,
                                    double total_time = 5.0,
                                    double time_step = 0.05);
    

    // Loads the solver configuration from a YAML file and sets the current_solver_.
    bool loadSolverConfig(const std::string& file_path);

    bool planAndExecute(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                        double velocity_scale,
                        double accel_scale,
                        double pose_tolerance);

    // Load object poses from memory file into a map<string, geometry_msgs::msg::Pose>.
    static std::map<std::string, geometry_msgs::msg::Pose> loadObjectLocations(const std::string& file_path);

    static void saveObjectLocations(const std::string& file_path,
                                    const std::map<std::string, geometry_msgs::msg::Pose>& object_map);

    std::vector<geometry_msgs::msg::PoseStamped> parseCsv(const std::string& csv_file);

    void parseGcode(const std::string& gcode_file,
                    std::vector<geometry_msgs::msg::PoseStamped>& out_poses);

    void parseSplineFile(const std::string& step_file,
                        std::vector<geometry_msgs::msg::PoseStamped>& out_poses);

    void feedbackPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void torqueFeedbackCallback(const std_msgs::msg::String::SharedPtr msg);

    void recordPose(const geometry_msgs::msg::Pose& pose, const std::string& filename);
};
