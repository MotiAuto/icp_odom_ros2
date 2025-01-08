#include "icp_odom_ros2/icp.hpp"

namespace icp_odom_ros2
{
    ICP::ICP() : source_(new pcl::PointCloud<pcl::PointXYZ>),posture(tf2::Matrix3x3(tf2::Quaternion(0.0, 0.0, 0.0, 1.0)))
    {

    }

    void ICP::setSource(const sensor_msgs::msg::PointCloud2::SharedPtr source)
    {
        pcl::fromROSMsg(*source, *source_);

        return;
    }

    void ICP::setPosture(const geometry_msgs::msg::Vector3::SharedPtr euler)
    {
        tf2::Quaternion q;
        q.setEuler(euler->x, euler->y, euler->z);

        posture = tf2::Matrix3x3(q);
    }

    tf2::Vector3 ICP::compute(const sensor_msgs::msg::PointCloud2::SharedPtr target)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*target, *target_);

        tf2::Vector3 translation;
        const auto source_size = source_->size();
        if(source_size == 0 || target_->size() == 0)
        {
            return translation;
        }

        tf2::Vector3 source_centroid, target_centroid;
        for(const auto& src_p : *source_)
        {
            auto min_dist = std::numeric_limits<float>::max();
            pcl::PointXYZ nearest_p;

            for(const auto& target_p : *target_)
            {
                const auto dx = src_p.x - target_p.x;
                const auto dy = src_p.y - target_p.y;
                const auto dz = src_p.z - target_p.z;
                const auto dist = dx*dx + dy*dy + dz*dz;

                if(dist < min_dist)
                {
                    min_dist = dist;
                    nearest_p = target_p;
                }
            }

            source_centroid += toTF2Vector(src_p);
            target_centroid += toTF2Vector(nearest_p);
        }

        source_centroid /= source_size;
        target_centroid /= source_size;

        translation = source_centroid - posture * target_centroid;

        return translation;
    }

    tf2::Vector3 toTF2Vector(const pcl::PointXYZ& p)
    {
        return tf2::Vector3(p.x, p.y, p.z);
    }
}