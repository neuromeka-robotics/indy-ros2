#include <sstream>
#include "SigmaDevice.hpp"
#include <chrono>
#include <thread>

void CheckAvailableDevices(int &devs);

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sigma");
    int devs = 0;
    CheckAvailableDevices(devs);
    std::vector<std::unique_ptr<SigmaDevice>> sigma(devs);
    for (int i=0; i<devs; i++){
        std::stringstream dev_names;
        dev_names << "sigma"<< i;
        sigma[i] = std::make_unique<SigmaDevice>(node, dev_names.str());
    }
    double rate;
    node->get_parameter_or("frequency", rate, 100.0); // 1000.0
    RCLCPP_INFO(node->get_logger(), "Set frequency: %f", rate);
    rclcpp::Rate loop_rate(rate);

    RCLCPP_INFO(node->get_logger(), "Initialization done.");

    while (rclcpp::ok()) {

        for (int i=0; i<devs; i++){

            sigma[i]->ReadMeasurementsFromDevice();

            sigma[i]->PublishPoseTwistButtonPedal();
            
            sigma[i]->HandleWrench();
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Ending Session...\n");
    for (int i=0; i<devs; i++){
        if (dhdClose ((char)i) < 0)
            RCLCPP_ERROR(node->get_logger(), " %s\n", dhdErrorGetLastStr ());
        else
            RCLCPP_INFO(node->get_logger(), "Closed device %i" ,i );
        sigma[i].reset();
    }

    rclcpp::shutdown();
    return 0;
}

void CheckAvailableDevices(int &devs) {

    while(rclcpp::ok() && devs==0) {
        for (int i = 0; i < 2; i++) {
            if (drdOpenID((char) i) > -1)
                devs = i+1;
        }
        std::chrono::milliseconds timespan(500);
        std::this_thread::sleep_for(timespan);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Looking for connected devices...");
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Found %i Device(s)", devs);
};