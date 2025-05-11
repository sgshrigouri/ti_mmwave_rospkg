#include "mmWaveDataHdl.hpp"

mmWaveDataHdl::mmWaveDataHdl(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mmWaveDataHdl", options) {
  maxPoints_ = this->declare_parameter("max_points", 300);
  maxRange_ = this->declare_parameter("max_range", 50.0);

  DataHandler_ = boost::make_shared<DataUARTHandler>(this->shared_from_this());
  DataHandler_->setMaxAllowedElevationAngleDeg(this->declare_parameter("max_elevation_angle_deg", 90));
  DataHandler_->setMaxAllowedAzimuthAngleDeg(this->declare_parameter("max_azimuth_angle_deg", 90));
  DataHandler_->start();

  cloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ti_mmwave/radar_scan_pcl", 100);
  maxRangePub_ = this->create_publisher<std_msgs::msg::Float32>("ti_mmwave/max_range", 100);
  cloudSub_ = this->create_subscription<ti_mmwave_rospkg::msg::RadarScan>(
    "ti_mmwave/radar_scan", 100,
    std::bind(&mmWaveDataHdl::cloudCB, this, std::placeholders::_1));

  paramCB();
}

void mmWaveDataHdl::cloudCB(const ti_mmwave_rospkg::msg::RadarScan::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.header.frame_id = this->declare_parameter("frame_id", "ti_mmwave");
  cloud.header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  cloud.width = maxPoints_;
  cloud.height = 1;
  cloud.points.resize(maxPoints_);

  pcl::PointXYZI point;
  point.x = msg->x;
  point.y = msg->y;
  point.z = msg->z;
  point.intensity = msg->intensity;
  cloud.push_back(point);

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header = msg->header;
  cloudPub_->publish(cloud_msg);
}

void mmWaveDataHdl::paramCB() {
  std_msgs::msg::Float32 range;
  range.data = maxRange_;
  maxRangePub_->publish(range);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mmWaveDataHdl)