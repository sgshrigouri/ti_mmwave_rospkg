#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <serial/serial.h>
#include "ti_mmwave_rospkg/srv/mm_wave_c_l_i.hpp"

class MMWaveQuickConfig : public rclcpp::Node {
public:
  MMWaveQuickConfig() : Node("mmWaveQuickConfig") {
    std::string config_file = this->declare_parameter("config_file", "");
    std::string user_port = this->declare_parameter("user_port", "/dev/ttyACM1");
    int user_baud = this->declare_parameter("user_baud", 115200);

    serial::Serial serial_port;
    serial_port.setPort(user_port);
    serial_port.setBaudrate(user_baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    serial_port.setTimeout(to);

    try {
      serial_port.open();
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open User serial port: %s", e.what());
      return;
    }

    if (serial_port.isOpen()) {
      RCLCPP_INFO(this->get_logger(), "User Serial Port initialized");
    } else {
      RCLCPP_ERROR(this->get_logger(), "User Serial Port failed to open");
      return;
    }

    std::ifstream file(config_file);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open config file: %s", config_file.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      if (!line.empty() && line[0] != '%') {
        serial_port.write(line + "\n");
        std::string response = serial_port.readline();
        RCLCPP_INFO(this->get_logger(), "Sent: %s, Received: %s", line.c_str(), response.c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }

    file.close();
    serial_port.close();
    RCLCPP_INFO(this->get_logger(), "Configuration completed");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MMWaveQuickConfig>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}