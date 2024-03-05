#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::placeholders;

class CmdVelToWheelAngle : public rclcpp::Node
{
public:
    CmdVelToWheelAngle()
        : Node("cmd_vel_to_wheel_angle")
    {
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            10,
            std::bind(&CmdVelToWheelAngle::cmdVelCallback, this, _1));
        wheel_angle_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "wheel_angle",
            10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            10,
            std::bind(&CmdVelToWheelAngle::scanCallback, this, _1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        current_angle = msg->angular.z * 180 / M_PI;
    }
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_data)
    {
        // 获取当前角度

        // 计算 5 度范围内的索引
        int start_index = (current_angle - 2.5) / scan_data->angle_increment;
        int end_index = (current_angle + 2.5) / scan_data->angle_increment;

        // 遍历 5 度范围内的雷达数据
        for (int i = start_index; i <= end_index; ++i)
        {
            // 检查范围是否有效
            if (i < 0 || i >= scan_data->ranges.size())
            {
                continue;
            }

            // 获取距离值
            float distance = scan_data->ranges[i];

            // 打印距离值
            RCLCPP_INFO(this->get_logger(), "Distance at angle %.2f degrees: %.2f meters",
                        current_angle + i * scan_data->angle_increment, distance);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wheel_angle_pub_;

    float current_angle = 0.0;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CmdVelToWheelAngle>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
