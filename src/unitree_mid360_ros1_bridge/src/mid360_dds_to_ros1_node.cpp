#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/common/thread/thread.hpp>

#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>

#include <mutex>
#include <string>
#include <vector>
#include <cstring>
#include <cmath>

#define LIDAR_TOPIC "rt/utlidar/cloud_livox_mid360"
#define IMU_TOPIC   "rt/utlidar/imu_livox_mid360"

using namespace unitree::robot;
using namespace unitree::common;

class Mid360DDSBridge
{
public:
    Mid360DDSBridge(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<std::string>("lidar_topic_out", lidar_topic_out_, std::string("/livox/lidar"));
        pnh.param<std::string>("imu_topic_out", imu_topic_out_, std::string("/livox/imu"));
        pnh.param<std::string>("lidar_frame_id", lidar_frame_id_, std::string("livox_frame"));
        pnh.param<std::string>("imu_frame_id", imu_frame_id_, std::string("livox_imu"));
        pnh.param<bool>("use_ros_now_for_stamp", use_ros_now_for_stamp_, false);

        lidar_pub_ = nh.advertise<sensor_msgs::PointCloud2>(lidar_topic_out_, 10);
        imu_pub_   = nh.advertise<sensor_msgs::Imu>(imu_topic_out_, 200);

        ChannelFactory::Instance()->Init(0);

        lidar_sub_.reset(
            new ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>(LIDAR_TOPIC));
        imu_sub_.reset(
            new ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>(IMU_TOPIC));

        lidar_sub_->InitChannel(
            std::bind(&Mid360DDSBridge::LidarCallback, this, std::placeholders::_1), 1);
        imu_sub_->InitChannel(
            std::bind(&Mid360DDSBridge::ImuCallback, this, std::placeholders::_1), 1);

        ROS_INFO("Mid360 DDS -> ROS1 bridge started.");
        ROS_INFO("Publish lidar to: %s", lidar_topic_out_.c_str());
        ROS_INFO("Publish imu   to: %s", imu_topic_out_.c_str());
        ROS_INFO("Lidar frame_id: %s", lidar_frame_id_.c_str());
        ROS_INFO("IMU   frame_id: %s", imu_frame_id_.c_str());
        ROS_INFO("use_ros_now_for_stamp: %s", use_ros_now_for_stamp_ ? "true" : "false");
    }

private:
    ros::Publisher lidar_pub_;
    ros::Publisher imu_pub_;

    std::shared_ptr<ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>> lidar_sub_;
    std::shared_ptr<ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>> imu_sub_;

    std::string lidar_topic_out_;
    std::string imu_topic_out_;
    std::string lidar_frame_id_;
    std::string imu_frame_id_;
    bool use_ros_now_for_stamp_;

    std::mutex lidar_mutex_;
    std::mutex imu_mutex_;

private:
    ros::Time ConvertStamp(int32_t sec, uint32_t nanosec) const
    {
        if (use_ros_now_for_stamp_) {
            return ros::Time::now();
        }

        if (sec <= 0) {
            return ros::Time::now();
        }

        return ros::Time(sec, nanosec);
    }

    sensor_msgs::PointField ConvertPointField(
        const sensor_msgs::msg::dds_::PointField_& dds_field) const
    {
        sensor_msgs::PointField ros_field;
        ros_field.name = dds_field.name();
        ros_field.offset = dds_field.offset();
        ros_field.datatype = dds_field.datatype();
        ros_field.count = dds_field.count();
        return ros_field;
    }

    void LidarCallback(const void* msg)
    {
        std::lock_guard<std::mutex> lock(lidar_mutex_);

        const auto* dds_pc =
            static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(msg);
        if (!dds_pc) {
            ROS_WARN_THROTTLE(1.0, "Received null lidar DDS message.");
            return;
        }

        sensor_msgs::PointCloud2 ros_pc;

        // header
        ros_pc.header.stamp = ConvertStamp(
            dds_pc->header().stamp().sec(),
            dds_pc->header().stamp().nanosec());

        // 强制改成我们需要的 frame_id，避免沿用 DDS 里的异常值
        ros_pc.header.frame_id = lidar_frame_id_;

        // basic layout
        ros_pc.height = dds_pc->height();
        ros_pc.width = dds_pc->width();

        ros_pc.is_bigendian = dds_pc->is_bigendian();
        ros_pc.point_step = dds_pc->point_step();
        ros_pc.row_step = dds_pc->row_step();
        ros_pc.is_dense = dds_pc->is_dense();

        // fields
        ros_pc.fields.clear();
        ros_pc.fields.reserve(dds_pc->fields().size());
        for (const auto& f : dds_pc->fields()) {
            ros_pc.fields.push_back(ConvertPointField(f));
        }

        // data
        ros_pc.data.resize(dds_pc->data().size());
        if (!dds_pc->data().empty()) {
            std::memcpy(ros_pc.data.data(),
                        dds_pc->data().data(),
                        dds_pc->data().size());
        }

        lidar_pub_.publish(ros_pc);

        ROS_DEBUG_THROTTLE(1.0,
                           "Published lidar: width=%u height=%u fields=%zu data=%zu",
                           ros_pc.width, ros_pc.height,
                           ros_pc.fields.size(), ros_pc.data.size());
    }

    void ImuCallback(const void* msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);

        const auto* dds_imu =
            static_cast<const sensor_msgs::msg::dds_::Imu_*>(msg);
        if (!dds_imu) {
            ROS_WARN_THROTTLE(1.0, "Received null imu DDS message.");
            return;
        }

        sensor_msgs::Imu ros_imu;

        // header
        ros_imu.header.stamp = ConvertStamp(
            dds_imu->header().stamp().sec(),
            dds_imu->header().stamp().nanosec());

        ros_imu.header.frame_id = imu_frame_id_;

        // orientation
        ros_imu.orientation.x = dds_imu->orientation().x();
        ros_imu.orientation.y = dds_imu->orientation().y();
        ros_imu.orientation.z = dds_imu->orientation().z();
        ros_imu.orientation.w = dds_imu->orientation().w();

        for (int i = 0; i < 9; ++i) {
            ros_imu.orientation_covariance[i] = dds_imu->orientation_covariance()[i];
        }

        // angular velocity
        ros_imu.angular_velocity.x = dds_imu->angular_velocity().x();
        ros_imu.angular_velocity.y = dds_imu->angular_velocity().y();
        ros_imu.angular_velocity.z = dds_imu->angular_velocity().z();

        for (int i = 0; i < 9; ++i) {
            ros_imu.angular_velocity_covariance[i] =
                dds_imu->angular_velocity_covariance()[i];
        }

        // linear acceleration
        ros_imu.linear_acceleration.x = dds_imu->linear_acceleration().x();
        ros_imu.linear_acceleration.y = dds_imu->linear_acceleration().y();
        ros_imu.linear_acceleration.z = dds_imu->linear_acceleration().z();

        for (int i = 0; i < 9; ++i) {
            ros_imu.linear_acceleration_covariance[i] =
                dds_imu->linear_acceleration_covariance()[i];
        }

        imu_pub_.publish(ros_imu);

        ROS_DEBUG_THROTTLE(1.0,
                           "Published imu: gyro=(%.4f %.4f %.4f) acc=(%.4f %.4f %.4f)",
                           ros_imu.angular_velocity.x,
                           ros_imu.angular_velocity.y,
                           ros_imu.angular_velocity.z,
                           ros_imu.linear_acceleration.x,
                           ros_imu.linear_acceleration.y,
                           ros_imu.linear_acceleration.z);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mid360_dds_to_ros1_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Mid360DDSBridge bridge(nh, pnh);

    ros::spin();
    return 0;
}