#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

using std::placeholders::_1;

class GroundFilterNode : public rclcpp::Node
{
public:
  GroundFilterNode()
  : Node("ground_filter_node")
  {
    this->declare_parameter("distance_threshold", 0.1);
    this->get_parameter("distance_threshold", distanceThreshold_);

    subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_custom", 10, std::bind(&GroundFilterNode::filterGround, this, _1));
    pub_nonground_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar" /* /nonground is published as /livox/lidar subscribed by fast_lio */, 10);
    pub_ground_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/ground", 10);
  }

private:
  double distanceThreshold_;

  void filterGround(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_lidar_msg)
  {
    // Convert Livox_ROS_Driver2 CustomMsg to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr livox_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto & point : livox_lidar_msg->points) {
      pcl::PointXYZI pcl_point;
      pcl_point.x = point.x;
      pcl_point.y = point.y;
      pcl_point.z = point.z;
      pcl_point.intensity = point.reflectivity;
      livox_lidar_cloud->push_back(pcl_point);
    }

    // RANSAC plane segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold_);
    seg.setInputCloud(livox_lidar_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty() /*if there is no inliers aka ground points*/) {
      RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
      return;
    }
    
    // Extract ground and nonground points
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(livox_lidar_cloud);
    extract.setIndices(inliers);
    extract.setNegative(false /*inliers positive -> ground*/);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    extract.filter(*ground_cloud);
    extract.setNegative(true /*inliers negative -> nonground*/);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    extract.filter(*nonground_cloud);

    // Convert filtered PCL clouds back to Livox_ROS_Driver2 CustomMsg
    livox_ros_driver2::msg::CustomMsg ground_msg;
    ground_msg.point_num = ground_cloud->size();
    for (const auto & pcl_point : *ground_cloud) {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = pcl_point.x;
      point.y = pcl_point.y;
      point.z = pcl_point.z;
      point.reflectivity = pcl_point.intensity;
      ground_msg.points.push_back(point);
    }
    ground_msg.header = livox_lidar_msg->header;
    livox_ros_driver2::msg::CustomMsg nonground_msg;
    nonground_msg.point_num = nonground_cloud->size();
    for (const auto & pcl_point : *nonground_cloud) {
      livox_ros_driver2::msg::CustomPoint point;
      point.x = pcl_point.x;
      point.y = pcl_point.y;
      point.z = pcl_point.z;
      point.reflectivity = pcl_point.intensity;
      nonground_msg.points.push_back(point);
    }
    nonground_msg.header = livox_lidar_msg->header;

    // Publish ground and nonground messages
    pub_ground_->publish(ground_msg);
    pub_nonground_->publish(nonground_msg);
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
