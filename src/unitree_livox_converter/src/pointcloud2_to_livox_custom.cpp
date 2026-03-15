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
#include <vector>
#include <sstream>

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

        // 时间模式：
        // 0: INDEX_LINEAR_100MS
        //    完全不信任 PointCloud2.time，按点序线性分配 0~scan_period_us
        // 1: RELATIVE_SECONDS
        //    PointCloud2.time 已是“帧内相对时间（秒）”
        // 2: RELATIVE_MICROSECONDS
        //    PointCloud2.time 已是“帧内相对时间（微秒）”
        // 3: ABSOLUTE_SECONDS_MINUS_FIRST
        //    PointCloud2.time 是绝对时间（秒），减去首点后转微秒
        // 4: ABSOLUTE_MICROSECONDS_MINUS_FIRST
        //    PointCloud2.time 是绝对时间（微秒），减去首点后直接用
        pnh.param<int>("time_mode", time_mode_, 0);

        // 10Hz Mid-360 默认 100ms
        pnh.param<int>("scan_period_us", scan_period_us_, 100000);

        // 合法时间跨度阈值
        pnh.param<int>("min_valid_span_us", min_valid_span_us_, 1000);
        pnh.param<int>("max_valid_span_us", max_valid_span_us_, 200000);

        // 调试输出前几个点
        pnh.param<int>("debug_print_points", debug_print_points_, 8);

        sub_ = nh.subscribe(input_topic_, 5, &PointCloud2ToLivoxCustom::cloudCallback, this);
        pub_ = nh.advertise<livox_ros_driver::CustomMsg>(output_topic_, 5);

        ROS_INFO("PointCloud2 -> livox_ros_driver/CustomMsg converter started.");
        ROS_INFO("input_topic:  %s", input_topic_.c_str());
        ROS_INFO("output_topic: %s", output_topic_.c_str());
        ROS_INFO("frame_id:     %s", frame_id_.c_str());
        ROS_INFO("lidar_id:     %d", lidar_id_int_);
        ROS_INFO("time_mode:    %d", time_mode_);
        ROS_INFO("scan_period_us: %d", scan_period_us_);
        ROS_INFO("valid_span_us: [%d, %d]", min_valid_span_us_, max_valid_span_us_);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;

    int lidar_id_int_;
    bool use_msg_stamp_as_timebase_;

    int time_mode_;
    int scan_period_us_;
    int min_valid_span_us_;
    int max_valid_span_us_;
    int debug_print_points_;

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

    static uint32_t clampOffsetUs(double us)
    {
        if (!std::isfinite(us) || us < 0.0) return 0;
        if (us > static_cast<double>(std::numeric_limits<uint32_t>::max())) {
            return std::numeric_limits<uint32_t>::max();
        }
        return static_cast<uint32_t>(us);
    }

    double convertRawTimeToUs(float t_raw, float t0_raw) const
    {
        switch (time_mode_) {
        case 1: // RELATIVE_SECONDS
            return static_cast<double>(t_raw) * 1e6;

        case 2: // RELATIVE_MICROSECONDS
            return static_cast<double>(t_raw);

        case 3: // ABSOLUTE_SECONDS_MINUS_FIRST
            return static_cast<double>(t_raw - t0_raw) * 1e6;

        case 4: // ABSOLUTE_MICROSECONDS_MINUS_FIRST
            return static_cast<double>(t_raw - t0_raw);

        case 0: // INDEX_LINEAR_100MS
        default:
            return 0.0; // 该模式不走这里
        }
    }

    bool needRawTimeField() const
    {
        return time_mode_ != 0;
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
        out_msg.timebase = use_msg_stamp_as_timebase_
                         ? static_cast<uint64_t>(msg->header.stamp.toNSec())
                         : 0ULL;
        out_msg.lidar_id = static_cast<uint8_t>(std::max(0, std::min(255, lidar_id_int_)));
        out_msg.rsvd[0] = 0;
        out_msg.rsvd[1] = 0;
        out_msg.rsvd[2] = 0;
        out_msg.points.reserve(point_count);

        std::vector<float> times;
        float first_time = 0.0f;
        float min_time = std::numeric_limits<float>::infinity();
        float max_time = -std::numeric_limits<float>::infinity();

        if (needRawTimeField()) {
            try {
                sensor_msgs::PointCloud2ConstIterator<float> iter_time(*msg, "time");
                times.reserve(point_count);
                for (size_t i = 0; i < point_count; ++i, ++iter_time) {
                    float t = *iter_time;
                    times.push_back(t);
                    if (i == 0) first_time = t;
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

                if (time_mode_ == 0) {
                    // INDEX_LINEAR_100MS：按点序均匀展开
                    double ratio = (point_count <= 1) ? 0.0
                                  : static_cast<double>(i) / static_cast<double>(point_count - 1);
                    p.offset_time = clampOffsetUs(ratio * static_cast<double>(scan_period_us_));
                } else {
                    double offset_us = convertRawTimeToUs(times[i], first_time);
                    p.offset_time = clampOffsetUs(offset_us);
                }

                out_msg.points.push_back(p);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "Failed to parse PointCloud2 fields: %s", e.what());
            return;
        }

        out_msg.point_num = static_cast<uint32_t>(out_msg.points.size());

        if (needRawTimeField()) {
            uint32_t first_offset = out_msg.points.empty() ? 0 : out_msg.points.front().offset_time;
            uint32_t last_offset  = out_msg.points.empty() ? 0 : out_msg.points.back().offset_time;
            double span_us = 0.0;

            switch (time_mode_) {
            case 1: span_us = static_cast<double>(max_time - min_time) * 1e6; break;
            case 2: span_us = static_cast<double>(max_time - min_time); break;
            case 3: span_us = static_cast<double>(max_time - min_time) * 1e6; break;
            case 4: span_us = static_cast<double>(max_time - min_time); break;
            default: break;
            }

            ROS_INFO_THROTTLE(1.0,
                "CustomMsg time stats: mode=%d min=%.9f max=%.9f span_us=%.3f first_offset=%u last_offset=%u",
                time_mode_, min_time, max_time, span_us, first_offset, last_offset);

            if (span_us < static_cast<double>(min_valid_span_us_) ||
                span_us > static_cast<double>(max_valid_span_us_)) {
                ROS_ERROR_THROTTLE(1.0,
                    "Abnormal per-frame time span: %.3f us, expected about %d us for a 10Hz lidar. "
                    "Drop this frame. Raw PointCloud2.time is likely not in the selected time_mode.",
                    span_us, scan_period_us_);
                return;
            }

            if (debug_print_points_ > 0) {
                std::ostringstream oss;
                oss << "time samples: ";
                int n = std::min<int>(debug_print_points_, static_cast<int>(point_count));
                for (int i = 0; i < n; ++i) {
                    oss << "[" << i
                        << ": raw=" << times[i]
                        << ", off=" << out_msg.points[i].offset_time << "] ";
                }
                ROS_INFO_THROTTLE(1.0, "%s", oss.str().c_str());
            }
        } else {
            ROS_INFO_THROTTLE(1.0,
                "CustomMsg time stats: mode=%d (INDEX_LINEAR), first_offset=%u last_offset=%u point_num=%u",
                time_mode_,
                out_msg.points.empty() ? 0 : out_msg.points.front().offset_time,
                out_msg.points.empty() ? 0 : out_msg.points.back().offset_time,
                out_msg.point_num);
        }

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