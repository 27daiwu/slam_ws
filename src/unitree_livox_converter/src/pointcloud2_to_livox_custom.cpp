#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

class PointCloud2ToLivoxCustom
{
public:
    PointCloud2ToLivoxCustom(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("input_topic", input_topic_, std::string("/livox/lidar_raw"));
        pnh.param<std::string>("output_topic", output_topic_, std::string("/livox/lidar"));
        pnh.param<std::string>("frame_id", frame_id_, std::string("livox_frame"));
        pnh.param<int>("lidar_id", lidar_id_int_, 0);
        pnh.param<bool>("use_msg_stamp_as_timebase", use_msg_stamp_as_timebase_, true);
        pnh.param<bool>("time_is_in_seconds", time_is_in_seconds_, true);

        sub_ = nh.subscribe(input_topic_, 5, &PointCloud2ToLivoxCustom::cloudCallback, this);
        pub_ = nh.advertise<livox_ros_driver::CustomMsg>(output_topic_, 5);

        ROS_INFO("PointCloud2 -> livox_ros_driver/CustomMsg converter started.");
        ROS_INFO("input_topic:  %s", input_topic_.c_str());
        ROS_INFO("output_topic: %s", output_topic_.c_str());
        ROS_INFO("frame_id:     %s", frame_id_.c_str());
        ROS_INFO("lidar_id:     %d", lidar_id_int_);
        ROS_INFO("time_is_in_seconds: %s", time_is_in_seconds_ ? "true" : "false");
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    int lidar_id_int_;
    bool use_msg_stamp_as_timebase_;
    bool time_is_in_seconds_;

    static uint8_t clampToUint8(float v)
    {
        if (!std::isfinite(v)) return 0;
        if (v < 0.0f) return 0;
        if (v > 255.0f) return 255;
        return static_cast<uint8_t>(std::lround(v));
    }

    static uint8_t clampRingToUint8(uint16_t ring)
    {
        if (ring > 255) return 255;
        return static_cast<uint8_t>(ring);
    }

    static uint32_t convertTimeToOffset(float t, bool time_is_in_seconds)
    {
        if (!std::isfinite(t) || t < 0.0f) {
            return 0;
        }

        if (time_is_in_seconds) {
            double us = static_cast<double>(t) * 1e6;
            if (us < 0.0) return 0;
            if (us > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
                return std::numeric_limits<uint32_t>::max();
            }
            return static_cast<uint32_t>(us);
        } else {
            if (t > static_cast<float>(std::numeric_limits<uint32_t>::max())) {
                return std::numeric_limits<uint32_t>::max();
            }
            return static_cast<uint32_t>(t);
        }
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (!msg) return;

    const size_t point_count =
        static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (point_count == 0) {
        ROS_WARN_THROTTLE(1.0, "Received empty PointCloud2.");
        return;
    }

    livox_ros_driver::CustomMsg out_msg;
    out_msg.header = msg->header;
    out_msg.header.frame_id = frame_id_;
    out_msg.timebase = static_cast<uint64_t>(msg->header.stamp.toNSec());
    out_msg.lidar_id = static_cast<uint8_t>(std::max(0, std::min(255, lidar_id_int_)));
    out_msg.rsvd[0] = 0;
    out_msg.rsvd[1] = 0;
    out_msg.rsvd[2] = 0;
    out_msg.points.reserve(point_count);

    std::vector<float> times;
    times.reserve(point_count);

    float min_time = std::numeric_limits<float>::infinity();
    float max_time = -std::numeric_limits<float>::infinity();

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");
        for (size_t i = 0; i < point_count; ++i, ++iter_time) {
            float t = *iter_time;
            times.push_back(t);
            if (std::isfinite(t)) {
                if (t < min_time) min_time = t;
                if (t > max_time) max_time = t;
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Failed to parse time field: %s", e.what());
        return;
    }

    if (!std::isfinite(min_time) || !std::isfinite(max_time)) {
        ROS_ERROR_THROTTLE(1.0, "Invalid time field in PointCloud2.");
        return;
    }

    // 自动判断 time 字段单位
    // 常见情况：
    // 1) 相对时间（秒）: 一帧内跨度通常 < 0.2
    // 2) 相对时间（毫秒）/（微秒）: 一帧内跨度几十到几万
    // 3) 绝对时间: 数值很大，但一帧内 max-min 很小，先减 min_time 再换算
    double time_scale_to_us = 1.0;

    double span = static_cast<double>(max_time - min_time);

    if (span < 1.0) {
        // 认为单位是秒
        time_scale_to_us = 1e6;
    } else if (span < 1000.0) {
        // 认为单位是毫秒
        time_scale_to_us = 1e3;
    } else {
        // 认为单位已经接近微秒
        time_scale_to_us = 1.0;
    }

    try {
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");

        for (size_t i = 0; i < point_count;
             ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring) {
            livox_ros_driver::CustomPoint p;
            p.x = *iter_x;
            p.y = *iter_y;
            p.z = *iter_z;
            p.reflectivity = clampToUint8(*iter_intensity);
            p.tag = 0;
            p.line = clampRingToUint8(*iter_ring);

            double rel_t = static_cast<double>(times[i] - min_time);
            if (!std::isfinite(rel_t) || rel_t < 0.0) {
                rel_t = 0.0;
            }

            double offset_us = rel_t * time_scale_to_us;
            if (offset_us < 0.0) offset_us = 0.0;
            if (offset_us > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
                offset_us = static_cast<double>(std::numeric_limits<uint32_t>::max());
            }

            p.offset_time = static_cast<uint32_t>(offset_us);
            out_msg.points.push_back(p);
        }
    } catch (const std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Failed to parse PointCloud2 fields: %s", e.what());
        return;
    }

    out_msg.point_num = static_cast<uint32_t>(out_msg.points.size());

    ROS_INFO_THROTTLE(1.0,
        "CustomMsg time stats: min=%.9f max=%.9f span=%.9f scale_to_us=%.1f first_offset=%u last_offset=%u",
        min_time, max_time, span, time_scale_to_us,
        out_msg.points.empty() ? 0 : out_msg.points.front().offset_time,
        out_msg.points.empty() ? 0 : out_msg.points.back().offset_time);

    pub_.publish(out_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud2_to_livox_custom");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PointCloud2ToLivoxCustom node(nh, pnh);
    ros::spin();
    return 0;
}