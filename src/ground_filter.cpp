#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

using std::placeholders::_1;

class GroundFilterNode : public rclcpp::Node
{
public:
  GroundFilterNode()
  : Node("ground_filter_node")
  {
    this->declare_parameter("height_threshold", 0.2);
    this->get_parameter("height_threshold", heightThreshold_);

    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_custom", 10, std::bind(&GroundFilterNode::topic_callback, this, _1));
    pub_nonground_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar" /* /nonground is published as /livox/lidar subscribed by fast_lio */, 10);
    pub_ground_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/ground", 10);
  }

private:
  double heightThreshold_;

  livox_ros_driver2::msg::CustomMsg nonground;
  livox_ros_driver2::msg::CustomMsg ground;
  
  void filterGround(const livox_ros_driver2::msg::CustomMsg & points_in)
  {
    nonground = points_in;
    ground = points_in;
    nonground.point_num = 0;
    ground.point_num = 0;
    nonground.points.clear();
    ground.points.clear();
    for (const auto & point : points_in.points) {
      // Assign to nonground or ground based on z value
      if (point.z > heightThreshold_) {
        nonground.points.push_back(point);
      } else {
        ground.points.push_back(point);
      }
    }
    // Update the number of points in nonground and ground
    nonground.point_num = nonground.points.size();
    ground.point_num = ground.points.size();
    ground.header.stamp = points_in.header.stamp /* fast_lio requires LiDAR to be synced with IMU -> reuse the original timestamp */;
    nonground.header.stamp = points_in.header.stamp;
  }
  
  void topic_callback(const livox_ros_driver2::msg::CustomMsg & livox_lidar)
  {
    filterGround(livox_lidar);
    pub_nonground_->publish(nonground);
    pub_ground_->publish(ground);
  }

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_nonground_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr pub_ground_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
