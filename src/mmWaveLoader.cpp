#include <rclcpp/rclcpp.hpp>
#include "DataHandlerClass.h"

class MMWaveLoader : public rclcpp::Node {
public:
  MMWaveLoader() : Node("mmWaveLoader") {
    data_handler_ = std::make_shared<DataUARTHandler>(this->shared_from_this());
    data_handler_->setMaxAllowedElevationAngleDeg(this->declare_parameter("max_elevation_angle_deg", 90.0));
    data_handler_->setMaxAllowedAzimuthAngleDeg(this->declare_parameter("max_azimuth_angle_deg", 90.0));
    data_handler_->start();

    RCLCPP_INFO(this->get_logger(), "mmWaveLoader initialized");
  }

private:
  std::shared_ptr<DataUARTHandler> data_handler_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MMWaveLoader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}