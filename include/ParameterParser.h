#ifndef PARAMETER_PARSER_H_
#define PARAMETER_PARSER_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

class ParameterParser {
public:
  explicit ParameterParser(std::shared_ptr<rclcpp::Node> node);
  void parseConfigFile(const std::string &config_file);
  std::string getFrameID();
  int getMaxPoints();
  float getMaxRange();
  float getMaxElevationAngleDeg();
  float getMaxAzimuthAngleDeg();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string frame_id_;
  int max_points_;
  float max_range_;
  float max_elevation_angle_deg_;
  float max_azimuth_angle_deg_;
};

#endif // PARAMETER_PARSER_H_