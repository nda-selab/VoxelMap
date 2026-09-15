#include "preprocess.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <limits>
#include <sensor_msgs/point_cloud2_iterator.h>

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1) {
  N_SCANS = 6;
  given_offset_time = false;
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
  switch (lidar_type) {
  case L515:
    l515_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::avia_handler(
    const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  int plsize = msg->point_num;
  std::vector<bool> is_valid_pt(plsize, false);

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for (int i = 0; i < N_SCANS; i++) {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  for (uint i = 1; i < plsize; i++) {
    if ((msg->points[i].line < N_SCANS) &&
        ((msg->points[i].tag & 0x30) == 0x10 ||
         (msg->points[i].tag & 0x30) == 0x00)) {
      valid_num++;
      if (i % point_filter_num == 0) {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature =
            msg->points[i].offset_time /
            float(1000000); // use curvature as time of each laser points

        if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
            (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
            (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) &&
                (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                     pl_full[i].z + pl_full[i].z >
                 blind * blind)) {
          is_valid_pt[i] = true;
        }
      }
    }
  }

  for (uint i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      pl_surf.points.push_back(pl_full[i]);
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);

  double time_stamp = msg->header.stamp.toSec();
  // cout << "===================================" << endl;
  // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
  for (int i = 0; i < pl_orig.points.size(); i++) {
    if (i % point_filter_num != 0)
      continue;

    double range = pl_orig.points[i].x * pl_orig.points[i].x +
                   pl_orig.points[i].y * pl_orig.points[i].y +
                   pl_orig.points[i].z * pl_orig.points[i].z;

    if (range < blind)
      continue;

    Eigen::Vector3d pt_vec;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
    if (yaw_angle >= 180.0)
      yaw_angle -= 360.0;
    if (yaw_angle <= -180.0)
      yaw_angle += 360.0;

    added_pt.curvature = pl_orig.points[i].t / 1e6;

    pl_surf.points.push_back(added_pt);
  }
}

void Preprocess::l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pl_surf.clear();
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  // pl_surf.reserve(plsize);
  for (int i = 0; i < pl_orig.size(); i++) {
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    if (i % point_filter_num == 0) {
      pl_surf.push_back(added_pt);
    }
  }
}

// Velodyne XYZIRT: time is seconds relative to the cloud header stamp.
void Preprocess::velodyne_handler(
    const sensor_msgs::PointCloud2::ConstPtr &msg) {
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  if (msg->width == 0 || msg->height == 0)
    return;

  const auto field = [&](const std::string &name, uint8_t type) {
    return std::any_of(msg->fields.begin(), msg->fields.end(),
                       [&](const sensor_msgs::PointField &f) {
                         return f.name == name && f.datatype == type && f.count == 1;
                       });
  };
  if (!field("x", sensor_msgs::PointField::FLOAT32) ||
      !field("y", sensor_msgs::PointField::FLOAT32) ||
      !field("z", sensor_msgs::PointField::FLOAT32) ||
      !field("intensity", sensor_msgs::PointField::FLOAT32) ||
      !field("ring", sensor_msgs::PointField::UINT16)) {
    ROS_ERROR_THROTTLE(5.0, "Velodyne requires float32 x/y/z/intensity and uint16 ring.");
    return;
  }
  const bool has_time = std::any_of(msg->fields.begin(), msg->fields.end(),
      [](const sensor_msgs::PointField &f) { return f.name == "time"; });
  if (has_time && !field("time", sensor_msgs::PointField::FLOAT32)) {
    ROS_ERROR_THROTTLE(5.0, "Velodyne time must be float32 seconds relative to header.stamp.");
    return;
  }
  if (N_SCANS <= 0 || point_filter_num <= 0 ||
      !std::isfinite(blind) || blind < 0 ||
      (!has_time && (!std::isfinite(scan_rate) || scan_rate <= 0))) {
    ROS_ERROR_THROTTLE(5.0, "Invalid Velodyne preprocessing parameters.");
    return;
  }
  given_offset_time = has_time;
  if (!has_time)
    ROS_WARN_THROTTLE(5.0, "Velodyne has no time field: estimating from azimuth and preprocess/scan_rate (Hz). Assumes a clockwise full scan starting at the first valid point; use measured point times for accurate deskew.");

  pcl::PointCloud<pcl::PointXYZI> input;
  pcl::fromROSMsg(*msg, input);
  sensor_msgs::PointCloud2ConstIterator<uint16_t> ring(*msg, "ring");
  std::unique_ptr<sensor_msgs::PointCloud2ConstIterator<float>> time;
  if (has_time)
    time.reset(new sensor_msgs::PointCloud2ConstIterator<float>(*msg, "time"));
  pl_surf.reserve(input.size());
  bool have_start = false;
  double start_yaw = 0;
  for (size_t i = 0; i < input.size(); ++i, ++ring) {
    const double seconds = has_time ? **time : 0.0;
    if (has_time)
      ++(*time);
    const auto &p = input[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(p.intensity) || *ring >= N_SCANS)
      continue;
    const double yaw = std::atan2(p.y, p.x);
    if (!have_start) {
      start_yaw = yaw;
      have_start = true;
    }
    const double range2 = double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z;
    if (range2 <= blind * blind || i % point_filter_num != 0)
      continue;
    double offset_ms = seconds * 1000.0;
    if (!has_time) {
      double angle = start_yaw - yaw;
      if (angle < 0)
        angle += 2.0 * M_PI;
      offset_ms = angle / (2.0 * M_PI * scan_rate) * 1000.0;
    }
    if (!std::isfinite(offset_ms) || offset_ms < 0 ||
        offset_ms > std::numeric_limits<float>::max()) {
      ROS_WARN_THROTTLE(5.0, "Discarding Velodyne points with invalid relative time.");
      continue;
    }
    PointType added_pt{};
    added_pt.x = p.x;
    added_pt.y = p.y;
    added_pt.z = p.z;
    added_pt.intensity = p.intensity;
    added_pt.normal_x = added_pt.normal_y = added_pt.normal_z = 0;
    added_pt.curvature = offset_ms;
    pl_surf.push_back(added_pt);
  }
  // Synchronization uses the last point's time as the scan end time.
  std::stable_sort(pl_surf.begin(), pl_surf.end(),
      [](const PointType &a, const PointType &b) { return a.curvature < b.curvature; });
}
