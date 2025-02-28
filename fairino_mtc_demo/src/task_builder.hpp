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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <random>


namespace mtc = moveit::task_constructor;

/** 
 * The TaskBuilder class is responsible for building an MTC Task out of various 
 * "command" calls, each representing a single manipulation or scene operation.
 */
class TaskBuilder
{
public:

    struct StoredObjectData
    {
    geometry_msgs::msg::Pose pose;       // position + orientation
    std::string shape;                   // e.g. "cylinder", "box", "sphere", ...
    std::vector<double> dimensions;      // shape-dependent, e.g. [height, radius] or [x,y,z]
    std::string color;                   // e.g. "#FF0000" or "red" (future usage)
    double alpha;                        // transparency: 0.0 (invisible) -> 1.0 (opaque)
    };

    /**
     * Constructor that sets the node and also allows you to specify which group
     * and tip frame you’ll use in the MTC pipeline.
     */
    TaskBuilder(rclcpp::Node::SharedPtr node,
                const std::string& arm_group_name,
                const std::string& tip_frame);

    std::unordered_map<std::string, double> getCurrentJointPositions() const {
        return current_joint_positions_;
    }

    bool ok() const {
        return executed_;
    }
    
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
    void scanLine(std::string world_frame, std::string tip_frame, const geometry_msgs::msg::Point& start,
                    const geometry_msgs::msg::Point& end);

    // Moves the end-effector "arbitrarily" but keeps it pointed at the 3D point [x,y,z]
    // relative to 'base_link'. (Simplified example with a fixed path.)
    void calibrateCamera(std::string tip_frame, double x, double y, double z);

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

    geometry_msgs::msg::Quaternion initial_orientation_;

    bool executed_{false};

    // The callback that updates current_joint_positions_
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    // Loads the solver configuration from a YAML file and sets the current_solver_.
    bool loadSolverConfig(const std::string& file_path);

    bool planWaypoints(const std::vector<geometry_msgs::msg::PoseStamped>& waypoints,
                        double velocity_scale,
                        double accel_scale,
                        double pose_tolerance);

    // Load object poses from memory file into a map<string, geometry_msgs::msg::Pose>.
    static std::map<std::string, geometry_msgs::msg::Pose> loadObjectLocations(const std::string& file_path);

    static void saveObjectLocations(const std::string& file_path,
                                    const std::map<std::string, geometry_msgs::msg::Pose>& object_map);

    void feedbackPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    void torqueFeedbackCallback(const std_msgs::msg::String::SharedPtr msg);

    void recordPose(const geometry_msgs::msg::Pose& pose, const std::string& filename);


    // Helper: compute orientation that points +Z to (targetX, targetY, targetZ) from (baseX, baseY, baseZ)
    static geometry_msgs::msg::Quaternion lookAtOrientation(double baseX, double baseY, double baseZ,
                                                            double targetX, double targetY, double targetZ)
    {
        // We want the +Z axis to point from base -> target
        tf2::Vector3 zAxis(0.0, 0.0, 1.0);
        tf2::Vector3 dir(targetX - baseX, targetY - baseY, targetZ - baseZ);

        // Avoid zero-length
        if (dir.length2() < 1e-10) {
            // Fallback to identity orientation if the target is the same as base
            geometry_msgs::msg::Quaternion q;
            q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
            return q;
        }
        dir.normalize();

        // Axis = cross( zAxis, dir ), angle = acos( dot(zAxis, dir) )
        double dot = zAxis.dot(dir);
        // clamp dot to [-1,1]
        if (dot > 1.0) dot = 1.0;
        if (dot < -1.0) dot = -1.0;

        double angle = std::acos(dot);
        tf2::Vector3 axis = zAxis.cross(dir);
        if (axis.length2() < 1e-16) {
            // zAxis and dir are nearly parallel or antiparallel
            // If dot < 0 => 180 deg rotation about x or y
            // If dot > 0 => angle=0 => identity
            if (dot < 0.0) {
                // 180 deg around X (or Y). Let's pick X
                axis = tf2::Vector3(1.0, 0.0, 0.0);
                angle = M_PI;
            } else {
                // 0 deg => identity
                geometry_msgs::msg::Quaternion q;
                q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
                return q;
            }
        }
        axis.normalize();

        tf2::Quaternion q(axis, angle);
        q.normalize();

        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();
        return q_msg;
    }

    // Helper to parse CSV waypoints
    std::vector<geometry_msgs::msg::PoseStamped> parseCsv(const std::string& csv_file)
    {
        std::vector<geometry_msgs::msg::PoseStamped> waypoints;
        std::ifstream infile(csv_file);
        if (!infile.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Could not open CSV file: %s", csv_file.c_str());
            return waypoints;
        }
        std::string line;
        while (std::getline(infile, line))
        {
            if (line.empty()) continue;
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
            if (values.size() < 3) continue;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            pose_stamped.header.stamp = node_->now();
            pose_stamped.pose.position.x = values[0];
            pose_stamped.pose.position.y = values[1];
            pose_stamped.pose.position.z = values[2];
            // Default orientation
            pose_stamped.pose.orientation.x = 0;
            pose_stamped.pose.orientation.y = 0;
            pose_stamped.pose.orientation.z = 0;
            pose_stamped.pose.orientation.w = 1;
            waypoints.push_back(pose_stamped);
        }
        infile.close();
        return waypoints;
    }

    geometry_msgs::msg::Quaternion inverseQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        // For a unit quaternion, the inverse is its conjugate.
        geometry_msgs::msg::Quaternion q_inv;
        q_inv.x = -q.x;
        q_inv.y = -q.y;
        q_inv.z = -q.z;
        q_inv.w = q.w;
        return q_inv;
    }

};
