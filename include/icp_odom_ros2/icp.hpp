#ifndef ICP_HPP_
#define ICP_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <limits>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_;
        tf2::Matrix3x3 posture;
    };

    tf2::Vector3 toTF2Vector(const pcl::PointXYZ& p);
}

#endif