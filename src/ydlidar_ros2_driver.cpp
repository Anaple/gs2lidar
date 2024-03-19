#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "std_srvs/srv/empty.hpp"
#include "src/CYdLidar.h"
#include "ydlidar_config.h"
#include <limits>
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <math.h>
#include <chrono>
#include <memory>
#define SDKROSVerision "1.0.2"

// bool stop_scan(std_srvs::srv::Empty::Request &req,
//                std_srvs::srv::Empty::Response &res)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Stop scan");
//     return laser.turnOff();
// }

// bool start_scan(std_srvs::srv::Empty::Request &req,
//                 std_srvs::srv::Empty::Response &res)
// {
//     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Start scan");
//     return laser.turnOn();
// }

int main(int argc, char **argv)
{

    CYdLidar laser;
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "YDLIDAR ROS Driver Version: %s", SDKROSVerision);
    auto node = std::make_shared<rclcpp::Node>("ydlidar_ros_driver");

    auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1);
    auto pc_pub = node->create_publisher<sensor_msgs::msg::PointCloud>("point_cloud", 1);

    rclcpp::NodeOptions node_options;
    auto private_node = std::make_shared<rclcpp::Node>("ydlidar_ros_driver_private", node_options);
    std::string str_optvalue = "/dev/ydlidar";

    private_node->declare_parameter("port", str_optvalue);
    private_node->get_parameter("port", str_optvalue);
    laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

    /// 忽略数组
    private_node->declare_parameter("ignore_array", str_optvalue);
    private_node->get_parameter("ignore_array", str_optvalue);

    laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(),
                      str_optvalue.size());

    std::string frame_id = "laser_frame";
    private_node->declare_parameter("frame_id", frame_id);
    private_node->get_parameter("frame_id", frame_id);

    //////////////////////int property/////////////////
    /// 激光雷达波特率
    int optval = 230400;
    private_node->declare_parameter("baudrate", optval);
    private_node->get_parameter("baudrate", optval);
    laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));

    /// TOF激光雷达类型
    optval = TYPE_TRIANGLE;
    private_node->declare_parameter("lidar_type", optval);
    private_node->get_parameter("lidar_type", optval);
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));

    /// 设备类型
    optval = YDLIDAR_TYPE_SERIAL;
    private_node->declare_parameter("device_type", optval);
    private_node->get_parameter("device_type", optval);
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));

    /// 采样率
    optval = 9;
    private_node->declare_parameter("sample_rate", optval);
    private_node->get_parameter("sample_rate", optval);
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));

    /// 异常点检测次数
    optval = 4;
    private_node->declare_parameter("abnormal_check_count", optval);
    private_node->get_parameter("abnormal_check_count", optval);
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
    // 强度位数
    optval = 10;
    private_node->declare_parameter("intensity_bit", optval);
    private_node->get_parameter("intensity_bit", optval);
    laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// 固定角度分辨率
    bool b_optvalue = false;
    private_node->declare_parameter("resolution_fixed", b_optvalue);
    private_node->get_parameter("resolution_fixed", b_optvalue);
    laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// 旋转180度
    private_node->declare_parameter("reversion", b_optvalue);
    private_node->get_parameter("reversion", b_optvalue);
    laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// 逆时针方向
    private_node->declare_parameter("inverted", b_optvalue);
    private_node->get_parameter("inverted", b_optvalue);
    laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    private_node->declare_parameter("auto_reconnect", b_optvalue);
    private_node->get_parameter("auto_reconnect", b_optvalue);
    laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// 单通道通信
    b_optvalue = false;
    private_node->declare_parameter("isSingleChannel", b_optvalue);
    private_node->get_parameter("isSingleChannel", b_optvalue);
    laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// 强度
    b_optvalue = false;
    private_node->declare_parameter("intensity", b_optvalue);
    private_node->get_parameter("intensity", b_optvalue);
    laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// 电机DTR
    b_optvalue = false;
    private_node->declare_parameter("support_motor_dtr", b_optvalue);
    private_node->get_parameter("support_motor_dtr", b_optvalue);
    laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// 单位: °
    float f_optvalue = 180.0f;
    private_node->declare_parameter("angle_max", f_optvalue);
    private_node->get_parameter("angle_max", f_optvalue);
    laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.0f;
    private_node->declare_parameter("angle_min", f_optvalue);
    private_node->get_parameter("angle_min", f_optvalue);
    laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// 单位: m
    f_optvalue = 16.f;
    private_node->declare_parameter("range_max", f_optvalue);
    private_node->get_parameter("range_max", f_optvalue);
    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.1f;
    private_node->declare_parameter("range_min", f_optvalue);
    private_node->get_parameter("range_min", f_optvalue);
    laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// 单位: Hz
    f_optvalue = 10.f;
    private_node->declare_parameter("frequency", f_optvalue);
    private_node->get_parameter("frequency", f_optvalue);
    laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    bool invalid_range_is_inf = false;
    private_node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
    private_node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

    bool point_cloud_preservative = false;
    private_node->declare_parameter("point_cloud_preservative", point_cloud_preservative);
    private_node->get_parameter("point_cloud_preservative", point_cloud_preservative);

    bool ret = laser.initialize();
    if (ret)
    {
        ret = laser.turnOn();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
    }

    // auto stop_scan_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan);
    // auto start_scan_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan);

    auto stop_scan_service =
        [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                 std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
    {
        return laser.turnOff();
    };

    auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_service);

    auto start_scan_service =
        [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                 std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
    {
        return laser.turnOn();
    };

    auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_service);

    rclcpp::WallRate loop_rate(30);

    while (ret && rclcpp::ok())
    {
        LaserScan scan;

        if (laser.doProcessSimple(scan))
        {

            // auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
            // auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud>();

            sensor_msgs::msg::LaserScan scan_msg;
            sensor_msgs::msg::PointCloud pc_msg;

            scan_msg.header.stamp.sec = RCL_NS_TO_S(scan.stamp);
            scan_msg.header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg.header.stamp.sec);
            scan_msg.header.frame_id = frame_id;
            pc_msg.header = scan_msg.header;

            scan_msg.angle_min = (scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);

            int size = (scan.config.max_angle - scan.config.min_angle) /
                           scan.config.angle_increment +
                       1;
            scan_msg.ranges.resize(size,
                                   invalid_range_is_inf ? std::numeric_limits<float>::infinity() : 0.0);
            scan_msg.intensities.resize(size);
            pc_msg.channels.resize(2);
            int idx_intensity = 0;
            pc_msg.channels[idx_intensity].name = "intensities";
            int idx_timestamp = 1;
            pc_msg.channels[idx_timestamp].name = "stamps";

            for (size_t i = 0; i < scan.points.size(); i++)
            {
                int index = std::ceil((scan.points[i].angle - scan.config.min_angle) /
                                      scan.config.angle_increment);

                if (index >= 0 && index < size)
                {
                    if (scan.points[i].range >= scan.config.min_range)
                    {
                        scan_msg.ranges[index] = scan.points[i].range;
                        // RCLCPP_INFO(rclcpp::get_logger("scan"), "%f", scan.points[i].range);
                        // RCLCPP_INFO(rclcpp::get_logger("scan_msg"), "%f", scan_msg.ranges[index]);

                        scan_msg.intensities[index] = scan.points[i].intensity;
                    }
                }

                if (point_cloud_preservative ||
                    (scan.points[i].range >= scan.config.min_range &&
                     scan.points[i].range <= scan.config.max_range))
                {
                    geometry_msgs::msg::Point32 point;
                    point.x = scan.points[i].range * cos(scan.points[i].angle);
                    point.y = scan.points[i].range * sin(scan.points[i].angle);
                    point.z = 0.0;
                    pc_msg.points.push_back(point);
                    pc_msg.channels[idx_intensity].values.push_back(scan.points[i].intensity);
                    pc_msg.channels[idx_timestamp].values.push_back(i * scan.config.time_increment);
                }
            }

            // for (int i = 0; i < scan_msg.ranges.size(); i++)
            // {
            //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%d] %f", i,scan_msg.ranges[i]);
            // }

            scan_pub->publish(scan_msg);
            pc_pub->publish(pc_msg);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get Lidar Data");
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    laser.turnOff();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
    laser.disconnecting();
    rclcpp::shutdown();
    return 0;
}
