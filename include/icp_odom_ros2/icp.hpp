#ifndef ICP_HPP_
#define ICP_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <limits>

namespace icp_odom_ros2
{
    class ICP
    {
        public:
        ICP();

        void setSource(const sensor_msgs::msg::PointCloud2::SharedPtr source);
        void setPosture(const geometry_msgs::msg::Vector3::SharedPtr euler);
        tf2::Vector3 compute(const sensor_msgs::msg::PointCloud2::SharedPtr target);

        private:
        sensor_msgs::msg::PointCloud source_;
        tf2::Matrix3x3 posture;
    };

    sensor_msgs::msg::PointCloud parsePointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    tf2::Vector3 toTF2Vector(const geometry_msgs::msg::Point32& p);
}

#endif