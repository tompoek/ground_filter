#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar_pointcloud2", 10, std::bind(&GroundFilterNode::filterGround, this, _1));
    pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nonground", 10);
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 10);
  }

private:
  double distanceThreshold_;

  void filterGround(sensor_msgs::msg::PointCloud2::SharedPtr livox_lidar_msg)
  {
    // Convert ROS2 PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr livox_lidar_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*livox_lidar_msg, *livox_lidar_cloud);

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

    // Convert filtered PCL clouds back to ROS2 PointCloud2
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = livox_lidar_msg->header;
    sensor_msgs::msg::PointCloud2 nonground_msg;
    pcl::toROSMsg(*nonground_cloud, nonground_msg);
    nonground_msg.header = livox_lidar_msg->header;

    // Publish ground and nonground messages
    pub_ground_->publish(ground_msg);
    pub_nonground_->publish(nonground_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilterNode>());
  rclcpp::shutdown();
  return 0;
}
