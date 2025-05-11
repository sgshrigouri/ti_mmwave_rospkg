#include "ParameterParser.h"
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <string>

ParameterParser::ParameterParser(std::shared_ptr<rclcpp::Node> node) : node_(node) {
  frame_id_ = node_->declare_parameter("frame_id", "ti_mmwave");
  max_points_ = node_->declare_parameter("max_points", 300);
  max_range_ = node_->declare_parameter("max_range", 50.0);
  max_elevation_angle_deg_ = node_->declare_parameter("max_elevation_angle_deg", 90.0);
  max_azimuth_angle_deg_ = node_->declare_parameter("max_azimuth_angle_deg", 90.0);
}

void ParameterParser::parseConfigFile(const std::string &config_file) {
  std::ifstream file(config_file);
  if (!file.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to open config file: %s", config_file.c_str());
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty() && line[0] != '%') {
      RCLCPP_INFO(node_->get_logger(), "Parsed config: %s", line.c_str());
      // Add parsing logic for specific parameters if needed
    }
  }

  file.close();
}

std::string ParameterParser::getFrameID() { return frame_id_; }
int ParameterParser::getMaxPoints() { return max_points_; }
float ParameterParser::getMaxRange() { return max_range_; }
float ParameterParser::getMaxElevationAngleDeg() { return max_elevation_angle_deg_; }
float ParameterParser::getMaxAzimuthAngleDeg() { return max_azimuth_angle_deg_; }