#include "preprocess.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
sensor_msgs::PointCloud2::Ptr message(
    const pcl::PointCloud<velodyne_ros::Point> &points, bool with_time = true) {
  sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(points, *msg);
  if (!with_time)
    msg->fields.erase(std::remove_if(msg->fields.begin(), msg->fields.end(),
        [](const sensor_msgs::PointField &f) { return f.name == "time"; }), msg->fields.end());
  return msg;
}

velodyne_ros::Point point(float x, float y, float z, uint16_t ring, float time) {
  velodyne_ros::Point p{};
  p.x = x; p.y = y; p.z = z;
  p.ring = ring; p.time = time; p.intensity = 10;
  return p;
}

class VelodynePreprocess : public ::testing::Test {
protected:
  Preprocess pre;
  PointCloudXYZI::Ptr output{new PointCloudXYZI};
  void SetUp() override {
    ros::Time::init();
    pre.set(false, VELO16, 1.0, 1);
    pre.N_SCANS = 16;
  }
};

TEST_F(VelodynePreprocess, KeepsAllSixteenBeamsAndSortsTimes) {
  pcl::PointCloud<velodyne_ros::Point> points;
  for (int i = 0; i < 16; ++i)
    points.push_back(point(10, 0, 10 * std::tan((-15 + 2 * i) * M_PI / 180),
                           i, (15 - i) * 0.005f));
  pre.process(message(points), output);
  ASSERT_EQ(16u, output->size());
  EXPECT_GT(output->front().z, 0);
  EXPECT_FLOAT_EQ(0, output->front().curvature);
  EXPECT_NEAR(75, output->back().curvature, 1e-4);
  for (size_t i = 1; i < output->size(); ++i)
    EXPECT_LE((*output)[i - 1].curvature, (*output)[i].curvature);
}

TEST_F(VelodynePreprocess, FiltersInvalidRangeRingAndTime) {
  pcl::PointCloud<velodyne_ros::Point> points;
  points.push_back(point(2, 0, 0, 0, 0.01));
  points.push_back(point(0.5, 0, 0, 0, 0));
  points.push_back(point(2, 0, 0, 16, 0));
  points.push_back(point(std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0));
  points.push_back(point(2, 0, 0, 0, -0.01));
  points.push_back(point(2, 0, 0, 0, std::numeric_limits<float>::infinity()));
  pre.process(message(points), output);
  ASSERT_EQ(1u, output->size());
  EXPECT_NEAR(10, output->front().curvature, 1e-4);
}

TEST_F(VelodynePreprocess, AppliesDecimation) {
  pcl::PointCloud<velodyne_ros::Point> points;
  for (int i = 0; i < 6; ++i)
    points.push_back(point(2, 0, 0, 0, i * 0.01));
  pre.point_filter_num = 2;
  pre.process(message(points), output);
  ASSERT_EQ(3u, output->size());
  EXPECT_NEAR(40, output->back().curvature, 1e-4);
}

TEST_F(VelodynePreprocess, EstimatesMissingTimeAcrossAzimuthWrap) {
  pcl::PointCloud<velodyne_ros::Point> points;
  for (double degrees : {-170.0, 100.0, 10.0, -80.0}) {
    double yaw = degrees * M_PI / 180;
    points.push_back(point(10 * std::cos(yaw), 10 * std::sin(yaw), 0, 0, 0));
  }
  pre.scan_rate = 10;
  pre.process(message(points, false), output);
  ASSERT_EQ(4u, output->size());
  for (size_t i = 0; i < 4; ++i)
    EXPECT_NEAR(i * 25, (*output)[i].curvature, 1e-4);
  pre.scan_rate = 20;
  pre.process(message(points, false), output);
  EXPECT_NEAR(37.5, output->back().curvature, 1e-4);
}

TEST_F(VelodynePreprocess, RejectsMissingRingAndClearsPreviousOutput) {
  pcl::PointCloud<velodyne_ros::Point> points;
  points.push_back(point(2, 0, 0, 0, 0));
  auto msg = message(points);
  pre.process(msg, output);
  ASSERT_EQ(1u, output->size());
  msg->fields.erase(std::remove_if(msg->fields.begin(), msg->fields.end(),
      [](const sensor_msgs::PointField &f) { return f.name == "ring"; }), msg->fields.end());
  pre.process(msg, output);
  EXPECT_TRUE(output->empty());
}

TEST_F(VelodynePreprocess, HandlesEmptyInputAndInvalidParameters) {
  pcl::PointCloud<velodyne_ros::Point> points;
  pre.process(message(points), output);
  EXPECT_TRUE(output->empty());
  points.push_back(point(2, 0, 0, 0, 0));
  pre.point_filter_num = 0;
  pre.process(message(points), output);
  EXPECT_TRUE(output->empty());
}
} // namespace
