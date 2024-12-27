
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class AddCollisionObjectsNode : public rclcpp::Node
{
public:
    AddCollisionObjectsNode()
        : Node("add_collision_objects_node")
    {
        // Create a timer to delay the execution of the add_ground_plane function
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AddCollisionObjectsNode::add_ground_plane, this));
    }

private:
    void add_ground_plane()
    {
        RCLCPP_INFO(this->get_logger(), "Adding ground plane to planning scene...");

        // Create a PlanningSceneInterface
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        // Define the collision object for the ground plane
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.id = "ground_plane";
        collision_object.header.frame_id = "world"; // Replace with "base_link" if needed

        // Define a box primitive for the ground plane
        shape_msgs::msg::SolidPrimitive plane_primitive;
        plane_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        plane_primitive.dimensions = {5.0, 5.0, 0.01}; // Size: x, y, z

        // Define the pose for the ground plane
        geometry_msgs::msg::Pose plane_pose;
        plane_pose.position.x = 0.0;
        plane_pose.position.y = 0.0;
        plane_pose.position.z = -0.005; // Slightly below the origin
        plane_pose.orientation.w = 1.0;

        // Add the primitive and pose to the collision object
        collision_object.primitives.push_back(plane_primitive);
        collision_object.primitive_poses.push_back(plane_pose);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Apply the collision object to the planning scene
        planning_scene_interface.applyCollisionObject(collision_object);

        RCLCPP_INFO(this->get_logger(), "Ground plane added to planning scene!");

        // Cancel the timer after execution
        timer_->cancel();
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AddCollisionObjectsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}