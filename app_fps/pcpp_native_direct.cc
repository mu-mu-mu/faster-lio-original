#include <cassert>
#include <math.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include "ros/serialization.h"
#include "ros/ros.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include "laser_mapping.h"
#include "utils.h"

#include "pubsub.h"
#include "pubsub_rcb_template.h"

int point_filter_num_ = 6;
double blind_ = 4.0;
bool feature_enabled_ = false;
bool given_offset_time_ = false;
int num_scans_ = 6;
float time_scale_ = 1e-3;

__attribute__((__visibility__("default")))
bool rcb_init()
{
  return true;
}

__attribute__((__visibility__("default")))
void rcb_callback(const struct rcb_context *ctx,
                  const void *_data, size_t len)
{
  assert(len == sizeof(void*)*2);
  void *data = const_cast<void*>(_data);

  const pcl::PointCloud<velodyne_ros::Point> pl_orig =
    *reinterpret_cast<const pcl::PointCloud<velodyne_ros::Point>*>(((void**)data)[0]);

  const std_msgs::Header hd =
    *reinterpret_cast<const std_msgs::Header*>(((void**)data)[1]);

  pcl::PointCloud<pcl::PointXYZINormal> cloud_out_;

  int plsize = pl_orig.points.size();
  cloud_out_.reserve(plsize);

  double omega_l = 3.61;  // scan angular velocity
  std::vector<bool> is_first(num_scans_, true);
  std::vector<double> yaw_fp(num_scans_, 0.0);    // yaw of first scan point
  std::vector<float> yaw_last(num_scans_, 0.0);   // yaw of last scan point
  std::vector<float> time_last(num_scans_, 0.0);  // last offset time

  if (pl_orig.points[plsize - 1].time > 0) {
      given_offset_time_ = true;
  } else {
      given_offset_time_ = false;
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
      double yaw_end = yaw_first;
      int layer_first = pl_orig.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--) {
          if (pl_orig.points[i].ring == layer_first) {
              yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
              break;
          }
      }
  }

  for (int i = 0; i < plsize; i++) {
    pcl::PointXYZINormal added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_scale_;  // curvature unit: ms

      if (!given_offset_time_) {
          int layer = pl_orig.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first[layer]) {
              yaw_fp[layer] = yaw_angle;
              is_first[layer] = false;
              added_pt.curvature = 0.0;
              yaw_last[layer] = yaw_angle;
              time_last[layer] = added_pt.curvature;
              continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_fp[layer]) {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
          } else {
              added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
          }

          if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num_ == 0) {
          if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind_ * blind_)) {
              cloud_out_.points.push_back(added_pt);
          }
      }
  }

  sensor_msgs::PointCloud2 pc2;
  ros::SerializedMessage m;

  pcl::toROSMsg(cloud_out_, pc2);

  //TODO
  pc2.header = hd; //for time stamp

  m = ros::serialization::serializeMessage
                            <sensor_msgs::PointCloud2>(pc2);

  rcb_send(ctx, m.message_start, m.num_bytes);
}
