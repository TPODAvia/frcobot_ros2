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
  : Node("depth_camera_sync_node", options),
    // Initialize the min_publish_interval_ to avoid calling the private default constructor.
    min_publish_interval_(rclcpp::Duration::from_seconds(0.0)),
    // Initialize last_sync_pub_time_ to 0 time.
    last_sync_pub_time_(0, 0)
  {
    // Declare parameters.
    enable_sync_ = this->declare_parameter<bool>("enable_sync", true);
    max_publish_rate_ = this->declare_parameter<double>("max_publish_rate", 30.0);
    // Compute the minimum interval (in seconds) between publishes in the sync callback.
    min_publish_interval_ = rclcpp::Duration::from_seconds(1.0 / max_publish_rate_);

    // Create a custom QoS profile.
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(5))
      .best_effort()
      .durability_volatile()
      .liveliness_lease_duration(rclcpp::Duration(0, 0));
    custom_qos.lifespan(rclcpp::Duration(0, 0));
    custom_qos.deadline(rclcpp::Duration(0, 0));

    // Create publishers on /sync/... topics using the custom QoS profile.
    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/sync/camera_depth/depth/image_raw", custom_qos);
    pub_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>("/sync/camera_depth/depth/camera_info", custom_qos);
    pub_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/camera_depth/points", custom_qos);

    if (enable_sync_) {
      RCLCPP_INFO(this->get_logger(), "Synchronization enabled. Publishing synced messages with rate limiting.");
      using SyncPolicy = message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;

      // Create message_filters subscribers.
      sub_image_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
          this, "/camera_depth/depth/image_raw");
      sub_info_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
          this, "/camera_depth/depth/camera_info");

      // Set up approximate time synchronizer with a queue size of 10.
      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), *sub_image_filter_, *sub_info_filter_);
      sync_->registerCallback(std::bind(&DepthCameraSyncNode::syncCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));
    } else {
      RCLCPP_INFO(this->get_logger(), "Synchronization disabled. Forwarding messages directly.");
      // Create direct subscriptions (without rate limiting).
      sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
          "/camera_depth/depth/image_raw", 10,
          std::bind(&DepthCameraSyncNode::imageCallback, this, std::placeholders::_1));
      sub_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/camera_depth/depth/camera_info", 10,
          std::bind(&DepthCameraSyncNode::infoCallback, this, std::placeholders::_1));
    }

    // Subscription for points is always pass-through.
    sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/camera_depth/points", 10,
          std::bind(&DepthCameraSyncNode::pointsCallback, this, std::placeholders::_1));
  }

private:
  // The synchronized callback applies rate trimming.
  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    // Only publish if the required interval has elapsed.
    auto now = this->now();
    if ((now - last_sync_pub_time_) < min_publish_interval_) {
      return;
    }
    last_sync_pub_time_ = now;

    // Force the camera info timestamp to match the image timestamp.
    sensor_msgs::msg::CameraInfo synced_info = *info_msg;
    synced_info.header.stamp = image_msg->header.stamp;

    pub_image_->publish(*image_msg);
    pub_info_->publish(synced_info);
  }

  // Direct pass-through callbacks (no rate trimming).
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    pub_image_->publish(*msg);
  }
  void infoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
  {
    pub_info_->publish(*msg);
  }
  void pointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    pub_points_->publish(*msg);
  }

  // Publishers.
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_info_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;

  // For approximate sync.
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_image_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> sub_info_filter_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>>> sync_;

  // For direct pass-through.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;

  // Parameters.
  bool enable_sync_;
  double max_publish_rate_;
  rclcpp::Duration min_publish_interval_;

  // For rate limiting in syncCallback.
  rclcpp::Time last_sync_pub_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthCameraSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
