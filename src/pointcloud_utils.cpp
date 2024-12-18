#include "icp_odom_ros2/pointcloud_utils.hpp"

namespace icp_odom_ros2
{
    void findNeighbor(geometry_msgs::msg::Point32 p, sensor_msgs::msg::PointCloud2 pointcloud)
    {
        sensor_msgs::msg::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(pointcloud, cloud);

        auto min_dist =10000.0;
        auto nearest_point = geometry_msgs::msg::Point32();

        for(const auto &other_p : cloud.points)
        {
            const auto dx = p.x - other_p.x;
            const auto dy = p.y - other_p.y;
            
            const auto distance = dx*dx + dy*dy;

            if(distance < min_dist)
            {
                min_dist = distance;
                nearest_point = other_p;
            }
        }
    }
}