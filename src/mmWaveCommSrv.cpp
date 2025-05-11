#include "mmWaveCommSrv.hpp"

mmWaveCommSrv::mmWaveCommSrv(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mmWaveCommSrv", options) {
  std::string user_port = this->declare_parameter("user_port", "/dev/ttyACM0");
  int user_baud = this->declare_parameter("user_baud", 115200);

  mmWavePort_ = std::make_shared<serial::Serial>();
  mmWavePort_->setPort(user_port);
  mmWavePort_->setBaudrate(user_baud);
  mmWavePort_->setTimeout(serial::Timeout::simpleTimeout(1000));
  try {
    mmWavePort_->open();
  } catch (serial::IOException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open User serial port: %s", e.what());
  }

  server_ = this->create_service<ti_mmwave_rospkg::srv::MmWaveCLI>(
    "ti_mmwave/mmWaveCLI",
    std::bind(&mmWaveCommSrv::commSrv, this, std::placeholders::_1, std::placeholders::_2));
}

bool mmWaveCommSrv::commSrv(const std::shared_ptr<ti_mmwave_rospkg::srv::MmWaveCLI::Request> req,
                            std::shared_ptr<ti_mmwave_rospkg::srv::MmWaveCLI::Response> res) {
  if (mmWavePort_->isOpen()) {
    mmWavePort_->write(req->command + "\n");
    res->response = mmWavePort_->readline();
    return true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Serial port not open");
    res->response = "Error: Serial port not open";
    return false;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mmWaveCommSrv)