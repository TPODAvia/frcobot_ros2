#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class DepthCameraSyncNode : public rclcpp::Node
{
public:
  explicit DepthCameraSyncNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("depth_camera_sync_node", options)
  {
    // Declare a parameter to enable or disable synchronization
    enable_sync_ = this->declare_parameter<bool>("enable_sync", true);

    // Create custom QoS profile:
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(5))
      .best_effort()
      .durability_volatile()
      .liveliness_lease_duration(rclcpp::Duration(0, 0));
    // Set Lifespan and Deadline as infinite (Duration(0,0) means infinite)
    custom_qos.lifespan(rclcpp::Duration(0, 0));
    custom_qos.deadline(rclcpp::Duration(0, 0));

    // Publishers on /sync/... topics using the custom QoS profile
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/sync/camera_depth/depth/image_raw", custom_qos);
    pub_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sync/camera_depth/depth/camera_info", custom_qos);
    pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/camera_depth/points", custom_qos);

    if (enable_sync_) {
      RCLCPP_INFO(this->get_logger(), "Synchronization enabled. Publishing synced messages.");

      using SyncPolicy = message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

      // Create message_filters subscribers using the custom QoS profile
      sub_image_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, "/camera_depth/depth/image_raw");
      sub_info_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
          this, "/camera_depth/depth/camera_info");

      // Set up approximate time synchronizer with a queue size of 10
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), *sub_image_filter_, *sub_info_filter_);
      sync_->registerCallback(std::bind(&DepthCameraSyncNode::syncCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));
    } else {
      RCLCPP_INFO(this->get_logger(), "Synchronization disabled. Forwarding messages directly.");

      // If sync is disabled, create regular subscriptions using custom QoS
      sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/camera_depth/depth/image_raw", 10,
          std::bind(&DepthCameraSyncNode::imageCallback, this, std::placeholders::_1));
      sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/camera_depth/depth/camera_info", 10,
          std::bind(&DepthCameraSyncNode::infoCallback, this, std::placeholders::_1));
    }

    // Create a subscription for /camera_depth/points using the custom QoS profile
    sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/camera_depth/points", 10,
          std::bind(&DepthCameraSyncNode::pointsCallback, this, std::placeholders::_1));
  }

private:
  // Callback used when sync is ENABLED:
  // Here we force the camera info timestamp to match the image timestamp.
  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // Create a copy of the camera info and update its timestamp to match the image timestamp
    sensor_msgs::msg::CameraInfo synced_info = *info_msg;
    synced_info.header.stamp = image_msg->header.stamp;

    // Publish the synchronized messages under /sync/ topics
    pub_image_->publish(*image_msg);
    pub_info_->publish(synced_info);
  }

  // Callbacks used when sync is DISABLED (pass-through)
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    pub_image_->publish(*msg);
  }
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
  {
    pub_info_->publish(*msg);
  }

  // Callback for /camera_depth/points, simply forward the message
  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    pub_points_->publish(*msg);
  }

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;

  // For approximate sync
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_image_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_info_filter_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>> sync_;

  // For direct pass-through
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;

  bool enable_sync_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthCameraSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
