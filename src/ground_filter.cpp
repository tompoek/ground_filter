#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <sstream>

class GroundFilter
{
private:
    sensor_msgs::PointCloud2 ground;
    sensor_msgs::PointCloud2 nonground;
    ros::Publisher pub_ground;
    ros::Publisher pub_nonground;
    ros::Subscriber sub;
    ros::NodeHandle n;

    std::vector< std::tuple<float,float,float,float,uint8_t,uint8_t> > readPoints(auto vector_of_chars) {
        std::vector< std::tuple<float,float,float,float,uint8_t,uint8_t> > vector_of_tuples;
        // TOFIX: Converting 8 bytes (unsigned chars) to 1 float always lose resolution to 1m...
        for (int i=0; i<vector_of_chars.size(); i+=18) {

            unsigned char bufferX[4] = {vector_of_chars[i],vector_of_chars[i+1],vector_of_chars[i+2],vector_of_chars[i+3]};
            unsigned char bufferY[4] = {vector_of_chars[i+4],vector_of_chars[i+5],vector_of_chars[i+6],vector_of_chars[i+7]};
            unsigned char bufferZ[4] = {vector_of_chars[i+8],vector_of_chars[i+9],vector_of_chars[i+10],vector_of_chars[i+11]};
            unsigned char bufferIntensity[4] = {vector_of_chars[i+12],vector_of_chars[i+13],vector_of_chars[i+14],vector_of_chars[i+15]};
            float x; float y; float z; float intensity;
            memcpy((char*)&x, bufferX, 4);
            memcpy((char*)&y, bufferY, 4);
            memcpy((char*)&z, bufferZ, 4);
            memcpy((char*)&intensity, bufferIntensity, 4);

            uint8_t tag = vector_of_chars[16];
            uint8_t line = vector_of_chars[17];
            auto t = std::make_tuple(x,y,z,intensity,tag,line);
            vector_of_tuples.push_back(t);
        }
        return vector_of_tuples;
    }

    int get10Percentile(auto v) { // for debug
        size_t n = v.size() / 10;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    int get90Percentile(auto v) { // for debug
        size_t n = v.size() * 9 / 10;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    int getMedian(auto v) { // for debug
        size_t n = v.size() / 2;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    int get1Qtile(auto v) { // for debug
        size_t n = v.size() / 4;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    int get3Qtile(auto v) { // for debug
        size_t n = v.size() * 3 / 4;
        nth_element(v.begin(), v.begin()+n, v.end());
        return v[n];
    }

    int getMax(auto v) { // for debug
        auto max=-100000;
        auto n = v.size();
        for (int i=0; i<n; ++i) {if (v[i]>max) {max=v[i];}}
        return max;
    }

    int getMin(auto v) { // for debug
        auto min=100000;
        auto n = v.size();
        for (int i=0; i<n; ++i) {if (v[i]!=0 && v[i]<min) {min=v[i];}}
        return min;
    }

    int getMean(auto v) { // for debug
        auto mean = 0;
        auto n = v.size();
        for (int i=0; i<n; ++i) {mean += v[i];}
        mean /= n;
        return mean;
    }

    void printStats(auto vector, auto tuple_element_index) // for debug
    {
        std::vector<float> v;
        for (int i=0; i<vector.size(); ++i) {
            float x; float y; float z; float intensity; uint8_t tag; uint8_t line;
            std::tie(x, y, z, intensity, tag, line) = vector[i];
            if (tuple_element_index==0) {v.push_back(x);}
            else if (tuple_element_index==1) {v.push_back(y);}
            else if (tuple_element_index==2) {v.push_back(z);}
            else {v.push_back(intensity);}
        }
        std::stringstream ss;
        std_msgs::String msg;
        ss.str("");
        ss << "Mean=" << (getMean(v)) << ", ";
        ss << "Min=" << (getMin(v)) << ", ";
        ss << "Median=" << (getMedian(v)) << ", ";
        ss << "3Qtile=" << (get3Qtile(v)) << ", ";
        ss << "Max=" << (getMax(v)) << ".";
        msg.data = ss.str();
        ROS_INFO(msg.data.c_str());
    }

    sensor_msgs::PointCloud2 initPoints(sensor_msgs::PointCloud2 points_in)
    {
        sensor_msgs::PointCloud2 points_out;
        points_out.header.frame_id = points_in.header.frame_id;
        points_out.header.seq = points_in.header.seq;
        points_out.height = points_in.height;
        points_out.width = 0;
        points_out.point_step = points_in.point_step; // 18
        points_out.row_step = 0;
        points_out.fields = points_in.fields;
        points_out.is_dense = true;
        return points_out;
    }
    
    void filterGround(sensor_msgs::PointCloud2 points_in)
    {
        auto points = readPoints(points_in.data);
        for (int i=0; i<points.size(); ++i) {
            int x; int y; int z; int intensity; int tag; int line;
            bool is_ground = true; // assume ground by default
            std::tie(x, y, z, intensity, tag, line) = points[i];

            if (z > 0) {is_ground = false;}

            if (is_ground) {
                for (int j=0; j<points_in.point_step; ++j) {
                    if (j==16) {ground.data.push_back('255');} // tag a colour for ground in rviz
                    else {ground.data.push_back(points_in.data[points_in.point_step*i + j]);}
                }
                ground.width += 1;
            } else {
                for (int j=0; j<points_in.point_step; ++j) {nonground.data.push_back(points_in.data[points_in.point_step*i + j]);}
                nonground.width += 1;
            };
        }
        ground.row_step = ground.point_step * ground.width;
        nonground.row_step = nonground.point_step * nonground.width;
        ground.header.stamp = ros::Time::now();
        nonground.header.stamp = ros::Time::now();
    }

    void callback(const sensor_msgs::PointCloud2 livox_lidar)
    {
        // auto points = readPoints(livox_lidar.data); // for debug
        // printStats(points, 0); // for debug, 0:x, 1:y, 2:z, 3:intensity
        ground = initPoints(livox_lidar);
        nonground = initPoints(livox_lidar);
        filterGround(livox_lidar);
        pub_ground.publish(ground);
        pub_nonground.publish(nonground);
    }

public:
    GroundFilter()
    {
        sub = n.subscribe<sensor_msgs::PointCloud2> ("/livox/lidar", 10, &GroundFilter::callback, this);
        pub_nonground = n.advertise<sensor_msgs::PointCloud2> ("/nonground", 100);
        pub_ground = n.advertise<sensor_msgs::PointCloud2> ("/ground", 100);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_pub");
    GroundFilter ground_filter;

    ros::spin();

    return 0;
}
