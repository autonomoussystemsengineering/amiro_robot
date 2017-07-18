#include "ros/ros.h"
#include <limits>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/UInt16.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <cmath>

ros::Publisher pub;
static double max_distance_m;

void callback(const sensor_msgs::PointCloud2::ConstPtr msg) {

  sensor_msgs::PointCloud cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud);

  // TODO Respect the color as well
  double dist = 0.0;
  for (std::size_t idx = 0; idx < cloud.points.size(); ++idx) {

    const double distTmp = sqrt(cloud.points.at(idx).x * cloud.points.at(idx).x +
                                cloud.points.at(idx).y * cloud.points.at(idx).y +
                                cloud.points.at(idx).z * cloud.points.at(idx).z);
    dist += distTmp > max_distance_m || std::isnan(distTmp) ? 0.0 : max_distance_m - distTmp;
  }
  // Normalize
  dist /= cloud.points.size();

  // Normalize to unsigned short and send
  std_msgs::UInt16 value;
  value.data = uint16_t((dist / max_distance_m) * double(std::numeric_limits<unsigned short>::max()));
  pub.publish(value);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "proximity_mockup_node");
  ros::NodeHandle n("~");

  ROS_INFO("Started proximity_mockup_node");

  std::string topic_out,topic_in;

  n.param<std::string>("topic_in", topic_in, "/amiro1/proximity_ring_1/points");
  n.param<std::string>("topic_out", topic_out, "/amiro1/proximity_ring_1/value");
  n.param<double>("max_distance_m", max_distance_m, 0.25);

  ros::Subscriber sub = n.subscribe(topic_in, 1, callback);
  pub = n.advertise<std_msgs::UInt16>(topic_out, 1);

  ros::spin();

  return 0;
}
