#include "DataHandlerClass.h"
#include <rclcpp/rclcpp.hpp>
#include <boost/bind.hpp>
#include <stdio.h>
#include <math.h>

DataUARTHandler::DataUARTHandler(std::shared_ptr<rclcpp::Node> node) : node_(node), mySerialPort_(new serial::Serial) {
  DataUARTHandler_pub = node_->create_publisher<ti_mmwave_rospkg::msg::RadarScan>("ti_mmwave/radar_scan", 100);
  maxAllowedElevationAngleDeg_ = 90;
  maxAllowedAzimuthAngleDeg_ = 90;
}

void DataUARTHandler::setPublisher(rclcpp::Publisher<ti_mmwave_rospkg::msg::RadarScan>::SharedPtr pub) {
  DataUARTHandler_pub = pub;
}

void DataUARTHandler::setMaxAllowedElevationAngleDeg(float maxElevationAngleDeg) {
  maxAllowedElevationAngleDeg_ = maxElevationAngleDeg;
}

void DataUARTHandler::setMaxAllowedAzimuthAngleDeg(float maxAzimuthAngleDeg) {
  maxAllowedAzimuthAngleDeg_ = maxAzimuthAngleDeg;
}

void DataUARTHandler::start() {
  std::string data_port = node_->declare_parameter("data_port", "/dev/ttyACM0");
  int data_rate = node_->declare_parameter("data_rate", 921600);

  try {
    mySerialPort_->setPort(data_port);
    mySerialPort_->setBaudrate(data_rate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    mySerialPort_->setTimeout(to);
    mySerialPort_->open();
  } catch (serial::IOException &e) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to open Data serial port: %s", e.what());
    return;
  }

  if (mySerialPort_->isOpen()) {
    RCLCPP_INFO(node_->get_logger(), "Data Serial Port initialized");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Data Serial Port failed to open");
    return;
  }

  readThread_ = std::thread(&DataUARTHandler::readIncomingData, this);
}

DataUARTHandler::~DataUARTHandler() {
  mySerialPort_->close();
  if (readThread_.joinable()) {
    readThread_.join();
  }
}

void DataUARTHandler::readIncomingData() {
  unsigned char *buffer = new unsigned char[4096];
  ti_mmwave_rospkg::msg::RadarScan radarscan;
  radarscan.header.frame_id = node_->declare_parameter("frame_id", "ti_mmwave");
  int point_id = 0;

  while (rclcpp::ok()) {
    try {
      size_t numBytes = mySerialPort_->read(buffer, 4096);
      if (numBytes > 0) {
        for (size_t i = 0; i < numBytes; i += 16) {
          if (numBytes - i < 16) break;

          radarscan.header.stamp = node_->get_clock()->now();
          radarscan.point_id = point_id++;

          float x = *(float *)(buffer + i);
          float y = *(float *)(buffer + i + 4);
          float z = *(float *)(buffer + i + 8);
          float intensity = *(float *)(buffer + i + 12);

          float range = sqrt(x * x + y * y + z * z);
          float velocity = 0.0; // Simplified, update based on SDK
          float doppler_bin = 0.0;
          float bearing = atan2(y, x) * 180.0 / M_PI;
          float elevation = atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI;

          if (elevation <= maxAllowedElevationAngleDeg_ && bearing <= maxAllowedAzimuthAngleDeg_) {
            radarscan.x = x;
            radarscan.y = y;
            radarscan.z = z;
            radarscan.range = range;
            radarscan.velocity = velocity;
            radarscan.doppler_bin = doppler_bin;
            radarscan.bearing = bearing;
            radarscan.intensity = intensity;

            DataUARTHandler_pub->publish(radarscan);
          }
        }
      }
    } catch (serial::IOException &e) {
      RCLCPP_ERROR(node_->get_logger(), "Error reading from Data serial port: %s", e.what());
    }
    rclcpp::Rate(100).sleep();
  }
  delete[] buffer;
}