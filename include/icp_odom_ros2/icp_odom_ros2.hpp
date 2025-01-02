#ifndef ICP_ODOM_ROS2_HPP_
#define ICP_ODOM_ROS2_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "icp.hpp"

using std::placeholders::_1;

namespace icp_odom_ros2
{
    class IcpOdomROS2 : public rclcpp::Node
    {
        public:
        explicit IcpOdomROS2(const rclcpp::NodeOptions& option=rclcpp::NodeOptions());

        void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void euler_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

        std::string frame_id_;
        std::shared_ptr<ICP> icp_;
        bool get_source_pc_;
        geometry_msgs::msg::Vector3::SharedPtr euler_;
    };
}

#endif