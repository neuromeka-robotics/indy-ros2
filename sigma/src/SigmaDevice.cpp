#include "SigmaDevice.hpp"
#include "rclcpp/clock.hpp"

int SigmaDevice::id = -1;

SigmaDevice::SigmaDevice(rclcpp::Node::SharedPtr n, const std::string ns)
        : new_wrench_msg(false)
{
    id++;

    pub_pose = n->create_publisher<geometry_msgs::msg::PoseStamped>(ns+"/pose", 1);
    pub_twist = n->create_publisher<geometry_msgs::msg::TwistStamped>(ns+"/twist", 1);
    pub_gripper = n->create_publisher<std_msgs::msg::Float32>(ns+"/gripper_angle", 1);
    pub_buttons = n->create_publisher<sensor_msgs::msg::Joy>(ns+"/buttons", 1);

    // std::string wrench_topic("/sigma/force_feedback");
    std::string wrench_topic(ns+"/force_feedback");
    n->get_parameter("wrench_topic", wrench_topic);
    sub_wrench	= n->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_topic, 1, std::bind(&SigmaDevice::WrenchCallback, this, std::placeholders::_1));

    n->get_parameter<bool>("enable_gripper_button", enable_gripper_button);
    n->get_parameter<bool>("lock_orientation", lock_orient);

    if(CalibrateDevice() == -1)
        rclcpp::shutdown();

    buttons_msg.buttons.push_back(0);
    buttons_msg.buttons.push_back(0);
}

void SigmaDevice::WrenchCallback(
        const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    //newDataDirect = true;
    wrench.wrench = msg->wrench;
    new_wrench_msg = true;
}


int SigmaDevice::CalibrateDevice() {

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Calibrating device " << id << " ...");

    if (drdOpenID ((char)id) < 0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "No device " << id << " found. dhd says: " << dhdErrorGetLastStr());
        dhdSleep (2.0);
        drdClose ((char)id);
        return -1;
    }

    if(drdIsInitialized((char)id)){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device " << id << " is already calibrated.");
    }
    else if(drdAutoInit((char)id)<0) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Initialization of device " << id << " failed. dhd says: (" << dhdErrorGetLastStr() << ")");
        dhdSleep(2.0);
    }

    drdStop(true, (char)id);

    dhdEnableForce (DHD_ON, (char)id);

    dhdSleep (0.2);
    if(enable_gripper_button)
        dhdEmulateButton(DHD_ON, (char)id);

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Device " << id << " ready.");
    return 0;
}

int SigmaDevice::ReadMeasurementsFromDevice() {
    double p[3];
    double orient_m[3][3];
    dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orient_m, (char)id);

    geometry_msgs::msg::Pose pose;
    pose.position.x = p[0];
    pose.position.y = p[1];
    pose.position.z = p[2];

    pose.orientation.w = sqrt(1.0 + orient_m[0][0] + orient_m[1][1] + orient_m[2][2]) / 2.0;
    double w4 = (4.0 * pose.orientation.w);
    pose.orientation.x = (orient_m[2][1] - orient_m[1][2]) / w4;
    pose.orientation.y = (orient_m[0][2] - orient_m[2][0]) / w4;
    pose.orientation.z = (orient_m[1][0] - orient_m[0][1]) / w4;

    pose_msg.pose = pose;
    pose_msg.header.stamp = rclcpp::Clock().now();

    double v[6];

    dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
    dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
    twist_msg.twist.linear.x = v[0];
    twist_msg.twist.linear.y = v[1];
    twist_msg.twist.linear.z = v[2];
    twist_msg.twist.angular.x = v[3];
    twist_msg.twist.angular.y = v[4];
    twist_msg.twist.angular.z = v[5];
    twist_msg.header.stamp = rclcpp::Clock().now();

    double temp;
    dhdGetGripperAngleRad(&temp);
    gripper_angle.data = (float)temp;

    for (int i = 0; i < 2; ++i) {
        buttons_previous_state[i] = buttons_state[i];
        buttons_state[i] = dhdGetButton(i, (char)id);
    }

    return 0;
}


void SigmaDevice::PublishPoseTwistButtonPedal() {

    pub_pose->publish(pose_msg);
    pub_twist->publish(twist_msg);

    pub_gripper->publish(gripper_angle);

    if((buttons_state[0] != buttons_previous_state[0]) ||
            (buttons_state[1] != buttons_previous_state[1]) ){

        buttons_msg.buttons[0] = buttons_state[0];
        buttons_msg.buttons[1] = buttons_state[1];
        pub_buttons->publish(buttons_msg);
    }
}

void SigmaDevice::HandleWrench() {

    // should we use new_wrench_msg?
    if(buttons_state[1] == 1) {
        if (dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.x,
                                                wrench.wrench.force.y,
                                                wrench.wrench.force.z,
                                                wrench.wrench.torque.x,
                                                wrench.wrench.torque.y,
                                                wrench.wrench.torque.z,
                                                0.0, (char)id) < DHD_NO_ERROR){
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }
        dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
    }
    else if (lock_orient){
        drdRegulatePos  (false);
        drdRegulateRot  (true);
        drdRegulateGrip (false);
        drdStart();
        drdMoveToRot (locked_orient[0], locked_orient[1],locked_orient[2]);
        drdStop(true);
    }
    else{
        if (dhdSetForceAndTorqueAndGripperForce(.0, .0, .0, .0, .0, .0, 0.,
                                                (char)id) < DHD_NO_ERROR)
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }

}







