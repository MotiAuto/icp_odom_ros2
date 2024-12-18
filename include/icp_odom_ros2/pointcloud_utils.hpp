#ifndef POINTCLOUD_UTILS_HPP_
#define POINTCLOUD_UTILS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

namespace icp_odom_ros2
{
    void findNeighbor(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud2 pointcloud);
}

#endif