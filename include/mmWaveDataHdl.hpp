#ifndef MMWAVE_DATA_HDL_HPP_
#define MMWAVE_DATA_HDL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ti_mmwave_rospkg/msg/radar_scan.hpp"
#include "DataHandlerClass.h"

class mmWaveDataHdl : public rclcpp::Node {
public:
  explicit mmWaveDataHdl(const rclcpp::NodeOptions & options);

private:
  void cloudCB(const ti_mmwave_rospkg::msg::RadarScan::SharedPtr msg);
  void paramCB();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr maxRangePub_;
  rclcpp::Subscription<ti_mmwave_rospkg::msg::RadarScan>::SharedPtr cloudSub_;
  boost::shared_ptr<DataUARTHandler> DataHandler_;
  int maxPoints_;
  float maxRange_;
};

#endif // MMWAVE_DATA_HDL_HPP_