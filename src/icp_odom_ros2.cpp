#include "icp_odom_ros2/icp_odom_ros2.hpp"

namespace icp_odom_ros2
{
    IcpOdomROS2::IcpOdomROS2(const rclcpp::NodeOptions& option) : Node("IcpOdomROS2", option)
    {
        pc2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IcpOdomROS2::pointcloud_callback, this, _1)
        );

        euler_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/euler",
            rclcpp::SystemDefaultsQoS(),
            std::bind(&IcpOdomROS2::euler_callback, this, _1)
        );

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current", rclcpp::SystemDefaultsQoS());

        this->declare_parameter("frame_id", "camera_link");
        this->get_parameter("frame_id", frame_id_);
        p.header.frame_id = frame_id_;
        p.pose.position.z = 0.0;

        icp_ = std::make_shared<ICP>();
        get_source_pc_ = false;
        euler_ = nullptr;

        RCLCPP_INFO(this->get_logger(), "Start IcpOdomROS2");
    }

    void IcpOdomROS2::euler_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        icp_->setPosture(msg);
        euler_ = msg;
    }

    void IcpOdomROS2::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if(!get_source_pc_)
        {
            icp_->setSource(msg);
            get_source_pc_ = true;
            RCLCPP_INFO(this->get_logger(), "Set Source PointCloud.");

            return;
        }

        auto translation = icp_->compute(msg);
        RCLCPP_INFO(this->get_logger(), "x:%lf, y:%lf, z:%lf", translation.x(), translation.y(), translation.z());

        tf2::Quaternion q;
        if(euler_ == nullptr)
        {
            q.setEuler(0.0, 0.0, 0.0);
        }
        else
        {
            q.setEuler(euler_->x, euler_->y, euler_->z);
        }
        p.pose.position.y = -1.0*translation.x();
        p.pose.position.x = translation.z();
        p.pose.orientation.w = q.w();
        p.pose.orientation.x = q.x();
        p.pose.orientation.y = q.y();
        p.pose.orientation.z = q.z();

        pose_pub_->publish(p);
        // get_source_pc_ = false;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(icp_odom_ros2::IcpOdomROS2)