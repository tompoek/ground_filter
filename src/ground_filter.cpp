// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class GroundFilter : public rclcpp::Node
{
public:
  GroundFilter()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&GroundFilter::topic_callback, this, _1));
    pub_nonground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/nonground", 10);
    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground", 10);
  }

private:
  sensor_msgs::msg::PointCloud2 nonground;
  sensor_msgs::msg::PointCloud2 ground;

  sensor_msgs::msg::PointCloud2 initPoints(sensor_msgs::msg::PointCloud2 points_in)
  { // initialize new point clouds by reusing input point clouds except timestamp and data values
    sensor_msgs::msg::PointCloud2 points_out;
    points_out.header.frame_id = points_in.header.frame_id;
    points_out.height = points_in.height;
    points_out.width = 0;
    points_out.point_step = points_in.point_step; // 18
    points_out.row_step = 0;
    points_out.fields = points_in.fields;
    points_out.is_dense = true;
    return points_out;
  }
  
  void filterGround(sensor_msgs::msg::PointCloud2 points_in)
  {
    nonground = points_in;
    ground = points_in;
    nonground.data.clear();
    ground.data.clear();
    for (unsigned int i=0; i<points_in.width; ++i) {
      uint8_t* point_ptr = &points_in.data[i * points_in.point_step];
      // Extract point data
      float x, y, z, intensity;
      uint8_t tag, line;
      std::memcpy(&x, point_ptr + 0, sizeof(float));
      std::memcpy(&y, point_ptr + 4, sizeof(float));
      std::memcpy(&z, point_ptr + 8, sizeof(float));
      std::memcpy(&intensity, point_ptr + 12, sizeof(float));
      tag = *(point_ptr + 16);
      line = *(point_ptr + 17);
      // Serialize point data
      std::vector<uint8_t> point_data;
      point_data.insert(point_data.end(), reinterpret_cast<uint8_t*>(&x), reinterpret_cast<uint8_t*>(&x) + sizeof(x));
      point_data.insert(point_data.end(), reinterpret_cast<uint8_t*>(&y), reinterpret_cast<uint8_t*>(&y) + sizeof(y));
      point_data.insert(point_data.end(), reinterpret_cast<uint8_t*>(&z), reinterpret_cast<uint8_t*>(&z) + sizeof(z));
      point_data.insert(point_data.end(), reinterpret_cast<uint8_t*>(&intensity), reinterpret_cast<uint8_t*>(&intensity) + sizeof(intensity));
      point_data.push_back(tag);
      point_data.push_back(line);
      // Assign to nonground or ground based on z value
      if (z > 0.3 /*TODO: make the threshold (in metre) tunable*/) {
        nonground.data.insert(nonground.data.end(), point_data.begin(), point_data.end());
      } else {
        ground.data.insert(ground.data.end(), point_data.begin(), point_data.end());
      };
    }
    // Update the number of points in nonground and ground
    nonground.width = nonground.data.size() / nonground.point_step;
    nonground.row_step = nonground.point_step * nonground.width;
    ground.width = ground.data.size() / ground.point_step;
    ground.row_step = ground.point_step * ground.width;
    ground.header.stamp = this->now();
    nonground.header.stamp = this->now();
  }
  
  void topic_callback(const sensor_msgs::msg::PointCloud2 & livox_lidar)
  {
    filterGround(livox_lidar);
    pub_nonground_->publish(nonground);
    pub_ground_->publish(ground);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_nonground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundFilter>());
  rclcpp::shutdown();
  return 0;
}
