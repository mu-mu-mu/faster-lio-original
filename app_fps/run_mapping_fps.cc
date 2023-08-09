//
// Created by xiang on 2021/10/9.
//

#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>
#include <iostream>


#include "laser_mapping.h"
#include "utils.h"

#include "pubsub.h"
#include "pubsub_rcb.h"

std::shared_ptr<faster_lio::LaserMapping> laser_mapping;

struct ps_context *ctx;

/// run faster-LIO in offline mode

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
DEFINE_string(rcb_mode, "PC2", "Internal (PC) or PC2");
DEFINE_string(addr, "127.0.0.1", "addr to manager");
DEFINE_string(port, "9090", "port to manager");
DEFINE_string(rcb_path, "./build/devel/lib/libpcpp_native_direct.so", "path to rcb");

static bool pcl_none_cb(void *arg, void *data, size_t len)
{
  assert(arg != NULL);
  struct publisher *pub = (struct publisher *)arg;
  int seq;

  sensor_msgs::PointCloud2::Ptr point_cloud_msg(new sensor_msgs::PointCloud2);

  ros::SerializedMessage m;
  m.message_start = (uint8_t*)data;
  m.num_bytes = len;
  m.type_info = &typeid(sensor_msgs::PointCloud2);

  std::cout << "start: " << std::endl;

  ros::serialization::deserializeMessage(m, *point_cloud_msg);

  seq = point_cloud_msg->header.seq;


  faster_lio::Timer::Evaluate(
      [&point_cloud_msg]() {
          laser_mapping->StandardPCLCallBack(point_cloud_msg);
          laser_mapping->Run();
      },
      "Laser Mapping Single Run");

  ps_publish(pub, &seq, sizeof(seq));

  return true;
}

static bool pcl_pc_pc2_cb(void *arg, void *data, size_t len)
{
  struct publisher *pub = (struct publisher *)arg;
  int seq;

  sensor_msgs::PointCloud2::Ptr point_cloud_msg(new sensor_msgs::PointCloud2);

  ros::SerializedMessage m;
  m.message_start = (uint8_t*)data;
  m.num_bytes = len;
  m.type_info = &typeid(sensor_msgs::PointCloud2);

  ros::serialization::deserializeMessage(m, *point_cloud_msg);

  seq = point_cloud_msg->header.seq;

  faster_lio::Timer::Evaluate(
      [&point_cloud_msg]() {
          laser_mapping->FPSCallBack(point_cloud_msg);
          laser_mapping->Run();
      },
      "Laser Mapping Single Run");

  ps_publish(pub, &seq, sizeof(seq));
  return true;
}

static bool imu_cb(void *arg, void *data, size_t len)
{
  struct publisher *pub = (struct publisher *)arg;
  sensor_msgs::Imu::Ptr imu_msg(new sensor_msgs::Imu);
  ros::SerializedMessage m;
  int seq = 0;

  m.message_start = (uint8_t*)data;
  m.num_bytes = len;
  m.type_info = &typeid(sensor_msgs::Imu);

  ros::serialization::deserializeMessage(m, *imu_msg);

  laser_mapping->IMUCallBack(imu_msg);

  ps_publish(pub, &seq, sizeof(seq));
  return true;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    const std::string config_file = FLAGS_config_file;
    const std::string rcb_mode = FLAGS_rcb_mode;
    const std::string addr = FLAGS_addr;
    const int port = std::atoi(FLAGS_port.c_str());
    const std::string rcb_path = FLAGS_rcb_path;

    laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }


    ctx = ps_init(addr.c_str(), port);
    if (! (rcb_mode == "none" || rcb_mode == "pc" || rcb_mode == "pc2")) {
      LOG(INFO) << "mode invalid (none/pc/pc2)";
      return 1;
    }

    usleep(2*1000*1000);

    struct publisher *imu_pub = ps_create_publisher(ctx, "topic/imu_rt");
    ps_register_subscriber(ctx, imu_cb, imu_pub, "topic/faster_lio_imu");

    if (rcb_mode == "none") {
      struct publisher *pub = ps_create_publisher(ctx, "topic/pcl_none_rt");
      ps_register_subscriber(ctx, pcl_none_cb, pub, "topic/faster_lio_pcl_none");

    } else if (rcb_mode == "pc") {
      struct publisher *pub = ps_create_publisher(ctx, "topic/pcl_pc_rt");
      struct ps_rcb *rcb = ps_create_rcb(RCB_x86, rcb_path.c_str(), pcl_pc_pc2_cb, pub);

      ps_register_subscriber_rcb(ctx, rcb,  pcl_none_cb, NULL, "topic/faster_lio_pcl_pc");

    } else if (rcb_mode == "pc2") {
      struct publisher *pub = ps_create_publisher(ctx, "topic/pcl_pc2_rt");
      struct ps_rcb *rcb = ps_create_rcb(RCB_x86, rcb_path.c_str(), pcl_pc_pc2_cb, pub);

      ps_register_subscriber_rcb(ctx, rcb,  pcl_none_cb, NULL, "topic/faster_lio_pcl_pc2");
    }


    usleep(2*1000*1000);

    ps_spin(ctx);

    LOG(INFO) << "finishing mapping";

    laser_mapping->Finish();

    /// print the fps
    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.);
    LOG(INFO) << "Faster LIO average FPS: " << fps;

    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    faster_lio::Timer::PrintAll();
    faster_lio::Timer::DumpIntoFile(FLAGS_time_log_file);

    return 0;
}
