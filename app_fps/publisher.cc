//
// (Original) Created by xiang on 2021/10/9.
// mu-mu-mu (keisuke nishimura)
//

#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>
#include <stdlib.h>
#include <cassert>
#include <time.h>
#include <unistd.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "ros/serialization.h"

#include "laser_mapping.h"
#include "utils.h"

#include "pubsub.h"
#include "pubsub_rcb.h"

#define PC_NUM_MAX (10000)

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
DEFINE_string(bag_file, "/home/xiang/Data/dataset/fast_lio2/avia/2020-09-16-quick-shack.bag", "path to the ros bag");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
DEFINE_string(rcb_mode, "PC2", "Internal (PC) or PC2");
DEFINE_string(addr, "127.0.0.1", "addr to manager");
DEFINE_string(port, "9091", "port to manager");

struct ps_context *ctx;
struct publisher *pub;

struct timespec ts[PC_NUM_MAX+2];


//TODO XXX remove
void rcb_callback(const struct rcb_context *ctx,
                  const void *data, size_t len)
{
  assert(len == sizeof(data));
  const pcl::PointCloud<velodyne_ros::Point> pl_orig =
      *reinterpret_cast<const pcl::PointCloud<velodyne_ros::Point>*>(data);

  int plsize = pl_orig.points.size();
  LOG(INFO) << "aa: " << pl_orig.points.size();

  sensor_msgs::PointCloud2 pcl2;
  pcl::toROSMsg(pl_orig, pcl2);

  // /ros/roscpp_core/roscpp_serialization
  ros::SerializedMessage m;
  m = ros::serialization::serializeMessage<sensor_msgs::PointCloud2>(pcl2);

  ros::SerializedMessage m2; //(m.buf, m.num_bytes);

  m2.message_start = m.message_start; //m.buf.get();
  //m2.buf = m.buf;
  m2.num_bytes = m.num_bytes;
  m2.type_info =  &typeid(sensor_msgs::PointCloud2);
  //m2.message = m.message;

  //m2.message_start = m.message_start;

  sensor_msgs::PointCloud2 pcl__;

  sensor_msgs::PointCloud2::Ptr point_cloud_msg(new sensor_msgs::PointCloud2);
  //ros::serialization::deserializeMessage(m2, pcl__);
  ros::serialization::deserializeMessage(m2, *point_cloud_msg);
  LOG(INFO) << "ccc: " << point_cloud_msg->height << " " <<point_cloud_msg->width;
}


// ref; https://stackoverflow.com/questions/53708076/what-is-the-proper-way-to-use-clock-gettime
struct timespec sub_timespec(int count)
{
  struct timespec t1 = ts[count];
  struct timespec t2;
  struct timespec td;

  clock_gettime(CLOCK_REALTIME, &t2);

  td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
  td.tv_sec  = t2.tv_sec - t1.tv_sec;
  if (td.tv_sec > 0 && td.tv_nsec < 0) {
    td.tv_nsec += 1000*1000*1000;
    td.tv_sec--;
  }
  else if (td.tv_sec < 0 && td.tv_nsec > 0)
  {
    td.tv_nsec -= 1000*1000*1000;
    td.tv_sec++;
  }
  return td;
}

static inline void start_time_count(int count)
{
  clock_gettime(CLOCK_REALTIME, &ts[count]);
}

static bool imu_cb(void *arg, void *data, size_t len)
{
  assert(len == sizeof(int));
  int seq = *(int*)data;

  assert(seq == 0);

  return true;
}

static bool pcl_cb(void *arg, void *data, size_t len)
{
  assert(len == sizeof(int));
  int seq = *(int*)data;

  struct timespec ts = sub_timespec(seq);

  std::cout << "pcl cb: " << seq  << " " << ts.tv_nsec / 1000 << " us" << std::endl;
  return true;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    const std::string bag_file = FLAGS_bag_file;
    const std::string config_file = FLAGS_config_file;
    const std::string rcb_mode = FLAGS_rcb_mode;
    const std::string pcl_addr = FLAGS_addr;
    const int port = std::atoi(FLAGS_port.c_str());

    struct publisher *pcl_pub = NULL;
    struct publisher *imu_pub = NULL;

    ctx = ps_init(pcl_addr.c_str(), port);

    LOG(INFO) << "mode " << rcb_mode;
    if (! (rcb_mode == "none" || rcb_mode == "pc" || rcb_mode == "pc2")) {
      LOG(INFO) << "mode invalid (none/pc/pc2)";
      return 1;
    }

    usleep(500*1000); // 0.5 s

    imu_pub = ps_create_publisher(ctx, "topic/faster_lio_imu");
    ps_register_subscriber(ctx, imu_cb, NULL, "topic/imu_rt");

    if (rcb_mode == "none") {
      pcl_pub = ps_create_publisher(ctx, "topic/faster_lio_pcl_none");
      ps_register_subscriber(ctx, pcl_cb, NULL, "topic/pcl_none_rt");
    } else if (rcb_mode == "pc") {
      pcl_pub = ps_create_publisher_rcb(ctx, "topic/faster_lio_pcl_pc", RCB_x86);
      ps_register_subscriber(ctx, pcl_cb, NULL, "topic/pcl_pc_rt");
    } else if (rcb_mode == "pc2") {
      pcl_pub = ps_create_publisher_rcb(ctx, "topic/faster_lio_pcl_pc2", RCB_x86);
      ps_register_subscriber(ctx, pcl_cb, NULL, "topic/pcl_pc2_rt");
    }

    usleep(1*1000*1000); // 1 s

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag";
    rosbag::Bag bag(FLAGS_bag_file, rosbag::bagmode::Read);

    LOG(INFO) << "Go!";

    int count = 0;
    for (const rosbag::MessageInstance &m : rosbag::View(bag)) {

      // Point Cloud
      auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (point_cloud_msg) {
        pcl::PointCloud<velodyne_ros::Point> pc;
        pcl::fromROSMsg(*point_cloud_msg, pc);
        point_cloud_msg->header.seq = count;
        std_msgs::Header hd = point_cloud_msg->header;

        if (rcb_mode == "none") {
          //Start
          start_time_count(count);

          sensor_msgs::PointCloud2 pc2;
          ros::SerializedMessage m;
          pcl::toROSMsg(pc, pc2);

          pc2.header = point_cloud_msg->header; //for time stamp

          m = ros::serialization::serializeMessage
                                    <sensor_msgs::PointCloud2>(pc2);

          ps_publish(pcl_pub, m.message_start, m.num_bytes);

        } else if (rcb_mode == "pc") {
          //Start
          start_time_count(count);

          pcl::fromROSMsg(*point_cloud_msg, pc);

          sensor_msgs::PointCloud2 pc2;
          ros::SerializedMessage m;
          pcl::toROSMsg(pc, pc2);

          m = ros::serialization::serializeMessage
                                    <sensor_msgs::PointCloud2>(pc2);

          ps_publish(pcl_pub, m.message_start, m.num_bytes);

          //rcb_callback(NULL, &pc, sizeof(&pc));

        } else if (rcb_mode == "pc2") {
          //Start
          start_time_count(count);

          void *data[2];
          data[0] = (void*)&pc;
          data[1] = (void*)&hd;

          ps_publish(pcl_pub, &data, sizeof(data));
        } else {
          assert(false);
        }
        count ++;
        if (count > PC_NUM_MAX)
          break;

        usleep(50*1000); // 20 ms
      }

      // IMU
      auto imu_msg = m.instantiate<sensor_msgs::Imu>();
      if (imu_msg) {
        ros::SerializedMessage m;
        m = ros::serialization::serializeMessage
                                  <sensor_msgs::Imu>(*imu_msg);
        ps_publish(imu_pub, m.message_start, m.num_bytes);
        usleep(2*1000); // 2 ms
      }

      if (faster_lio::options::FLAG_EXIT) {
          break;
      }

    }

    usleep(5 * 1000*1000); // 100 ms

    return 0;
}
