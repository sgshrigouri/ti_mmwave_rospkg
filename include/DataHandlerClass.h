#ifndef DATA_HANDLER_CLASS_H_
#define DATA_HANDLER_CLASS_H_

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <thread>
#include <boost/shared_ptr.hpp>
#include "ti_mmwave_rospkg/msg/radar_scan.hpp"

class DataUARTHandler {
public:
  explicit DataUARTHandler(std::shared_ptr<rclcpp::Node> node);
  ~DataUARTHandler();
  void setPublisher(rclcpp::Publisher<ti_mmwave_rospkg::msg::RadarScan>::SharedPtr pub);
  void setMaxAllowedElevationAngleDeg(float maxElevationAngleDeg);
  void setMaxAllowedAzimuthAngleDeg(float maxAzimuthAngleDeg);
  void start();
  void readIncomingData();

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<ti_mmwave_rospkg::msg::RadarScan>::SharedPtr DataUARTHandler_pub;
  boost::shared_ptr<serial::Serial> mySerialPort_;
  std::thread readThread_;
  float maxAllowedElevationAngleDeg_;
  float maxAllowedAzimuthAngleDeg_;
};

#endif // DATA_HANDLER_CLASS_H_