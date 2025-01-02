#include "icp_odom_ros2/icp.hpp"

namespace icp_odom_ros2
{
    ICP::ICP() : source_(sensor_msgs::msg::PointCloud()),posture(tf2::Matrix3x3(tf2::Quaternion(0.0, 0.0, 0.0, 1.0)))
    {

    }

    void ICP::setSource(const sensor_msgs::msg::PointCloud2::SharedPtr source)
    {
        auto parsed_source = parsePointCloud2(source);

        source_ = parsed_source;

        return ;
    }

    void ICP::setPosture(const geometry_msgs::msg::Vector3::SharedPtr euler)
    {
        tf2::Quaternion q;
        q.setEuler(euler->x, euler->y, euler->z);

        posture = tf2::Matrix3x3(q);
    }

    tf2::Vector3 ICP::compute(const sensor_msgs::msg::PointCloud2::SharedPtr target)
    {
        auto target_ = parsePointCloud2(target);

        tf2::Vector3 translation;
        const auto source_size = source_.points.size();
        if(source_size == 0 || target_.points.size() == 0)
        {
            return translation;
        }

        tf2::Vector3 source_centroid, target_centroid;
        for(const auto& src_p : source_.points)
        {
            auto min_dist = std::numeric_limits<float>::max();
            geometry_msgs::msg::Point32 nearest_p;

            for(const auto& target_p : target_.points)
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

        source_centroid /= static_cast<float>(source_size);
        target_centroid /= static_cast<float>(source_size);

        translation = target_centroid - posture * source_centroid;

        return translation;
    }

    sensor_msgs::msg::PointCloud parsePointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::PointCloud output;

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            geometry_msgs::msg::Point32 p;
            p.x = *iter_x;
            p.y = *iter_y;
            p.z = *iter_z;
            output.points.push_back(p);
        }

        return output;
    }

    tf2::Vector3 toTF2Vector(const geometry_msgs::msg::Point32& p)
    {
        return tf2::Vector3(p.x, p.y, p.z);
    }
}