//ROS
#include "rclcpp/rclcpp.hpp"
#include <tf2_kdl/tf2_kdl.h>
#include <tf2/convert.h>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
// Sigma
#include <dhdc.h>
#include <drdc.h>
#include <sensor_msgs/msg/joy.hpp>


class SigmaDevice {
public:
    SigmaDevice(rclcpp::Node::SharedPtr n, const std::string name_space);

    void WrenchCallback( const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    int ReadMeasurementsFromDevice();
    void PublishPoseTwistButtonPedal();
    void HandleWrench();

private:

    int CalibrateDevice();

private:

    static int id;
    bool enable_gripper_button=0;


    bool lock_orient=0;

    double locked_orient[3]={0., 0., 0.};

    geometry_msgs::msg::PoseStamped pose_msg;
    geometry_msgs::msg::TwistStamped twist_msg;
    geometry_msgs::msg::WrenchStamped wrench;
    bool new_wrench_msg;
    std_msgs::msg::Float32 gripper_angle;

    int buttons_state[2];
    int buttons_previous_state[2];

    sensor_msgs::msg::Joy buttons_msg;

    int pedal_previous_state;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_gripper;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_buttons;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench;
};