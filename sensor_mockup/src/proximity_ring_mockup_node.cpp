#include "ros/ros.h"
#include <limits>
#include <cmath>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/UInt16.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub;
static double max_distance_m;
const size_t numSensors = 8;

void callback(const sensor_msgs::PointCloud2::ConstPtr msg0,
              const sensor_msgs::PointCloud2::ConstPtr msg1,
              const sensor_msgs::PointCloud2::ConstPtr msg2,
              const sensor_msgs::PointCloud2::ConstPtr msg3,
              const sensor_msgs::PointCloud2::ConstPtr msg4,
              const sensor_msgs::PointCloud2::ConstPtr msg5,
              const sensor_msgs::PointCloud2::ConstPtr msg6,
              const sensor_msgs::PointCloud2::ConstPtr msg7) {

  const sensor_msgs::PointCloud2::ConstPtr msgs[numSensors] = {msg0, msg1, msg2, msg3, msg4, msg5, msg6, msg7};

  amiro_msgs::UInt16MultiArrayStamped values;
  values.array.data.resize(size(msgs));
  values.header = msgs[0]->header;
  values.header.frame_id = "";

  sensor_msgs::PointCloud cloud;
  for (std::size_t idx = 0; idx < numSensors; ++idx) {
    sensor_msgs::convertPointCloud2ToPointCloud(*(msgs[idx]), cloud);

    // TODO Respect the color as well
    double dist = 0.0;
    for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it) {
      const double distTmp = sqrt(it->x * it->x +
                                  it->y * it->y +
                                  it->z * it->z);
      dist += distTmp > max_distance_m || std::isnan(distTmp) ? 0.0 : max_distance_m - distTmp;
    }
    // Normalize
    const double distNormalized = dist / (max_distance_m * cloud.points.size());

    // Normalize to unsigned short
    values.array.data.at(idx) = uint16_t(distNormalized * double(std::numeric_limits<unsigned short>::max()));

    // Get the most current timestamp
    if (values.header.stamp < msgs[idx]->header.stamp) {
      values.header.stamp = msgs[idx]->header.stamp;
    }
  }
  pub.publish(values);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "proximity_ring_mockup_node");
  ros::NodeHandle n("~");

  ROS_INFO("Started proximity_ring_mockup_node");

  std::string topic_out, topic_in_suffix, topic_in_prefix;

  n.param<std::string>("topic_in_suffix", topic_in_suffix, "/amiro1/proximity_ring_");
  n.param<std::string>("topic_in_prefix", topic_in_prefix, "/points");
  n.param<std::string>("topic_out", topic_out, "/amiro1/proximity_ring/values");
  n.param<double>("max_distance_m", max_distance_m, 0.25);

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> syncPolicy;

  message_filters::Subscriber<sensor_msgs::PointCloud2> sub0(n, topic_in_suffix + "0" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(n, topic_in_suffix + "1" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(n, topic_in_suffix + "2" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub3(n, topic_in_suffix + "3" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub4(n, topic_in_suffix + "4" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub5(n, topic_in_suffix + "5" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub6(n, topic_in_suffix + "6" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub7(n, topic_in_suffix + "7" + topic_in_prefix, 1);

  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub0, sub1, sub2, sub3, sub4, sub5, sub6, sub7);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, _7, _8));

  pub = n.advertise<amiro_msgs::UInt16MultiArrayStamped>(topic_out, 1);

  ros::spin();

  return 0;
}
