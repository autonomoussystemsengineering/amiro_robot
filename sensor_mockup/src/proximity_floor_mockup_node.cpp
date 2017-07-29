#include "ros/ros.h"
#include <limits>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt16.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <amiro_msgs/UInt16MultiArrayStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

ros::Publisher pub;
const size_t numSensors = 4;
std::random_device rd;
std::mt19937 gen(rd());

inline double gray2mean(const double gray) {
  const double a = 11010.7641914599;
  const double b = 0.0610333423539444;
  const double c = 21783.9217626851;
  const double d = -7.94411704377273;
  return a*atan(b*gray+d)+c;
}

inline double gray2std(const double gray) {
  const double a = 1108.83338439758;
  const double b = 0.0223713977792142;
  const double c = 1503.24485506827;
  const double d = -2.08504831316051;
  return a*tanh(b*gray+d)+c;
}

void callback(const sensor_msgs::Image::ConstPtr msg0,
              const sensor_msgs::Image::ConstPtr msg1,
              const sensor_msgs::Image::ConstPtr msg2,
              const sensor_msgs::Image::ConstPtr msg3) {

  const sensor_msgs::Image::ConstPtr msgs[numSensors] = {msg0, msg1, msg2, msg3};

  amiro_msgs::UInt16MultiArrayStamped values;
  values.array.data.resize(size(msgs));
  values.header = msgs[0]->header;
  values.header.frame_id = "";

  for (std::size_t idx = 0; idx < numSensors; ++idx) {

    // Integrate over all pixel values to get the mean gray value
    size_t grayIntegrated = 0;
    for(auto it = msgs[idx]->data.begin(); it != msgs[idx]->data.end(); ++it) {
      grayIntegrated += size_t(*it);
    }
    // Normalize to 0 .. 255
    const double gray = double(grayIntegrated) / msgs[idx]->data.size();

    // Parameterize the normal distribution to sample
    std::normal_distribution<> distribution(gray2mean(gray),gray2std(gray));

    // Sample the sensor value
    const double sensorValue = distribution(gen);

    // Truncate the sampled value
    if (sensorValue < double(std::numeric_limits<unsigned short>::min())) {
      values.array.data.at(idx) = std::numeric_limits<unsigned short>::min();
    } else if (sensorValue > double(std::numeric_limits<unsigned short>::max())) {
      values.array.data.at(idx) = std::numeric_limits<unsigned short>::max();
    } else {
      values.array.data.at(idx) = std::round(sensorValue);
    }

    // Get the most current timestamp for the whole message
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

  n.param<std::string>("topic_in_suffix", topic_in_suffix, "/amiro1/proximity_floor_");
  n.param<std::string>("topic_in_prefix", topic_in_prefix, "/image_raw");
  n.param<std::string>("topic_out", topic_out, "/amiro1/proximity_floor/values");

  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::Image,
      sensor_msgs::Image,
      sensor_msgs::Image> syncPolicy;

  message_filters::Subscriber<sensor_msgs::Image> sub0(n, topic_in_suffix + "0" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub1(n, topic_in_suffix + "1" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub2(n, topic_in_suffix + "2" + topic_in_prefix, 1);
  message_filters::Subscriber<sensor_msgs::Image> sub3(n, topic_in_suffix + "3" + topic_in_prefix, 1);

  message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub0, sub1, sub2, sub3);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  pub = n.advertise<amiro_msgs::UInt16MultiArrayStamped>(topic_out, 1);

  ros::spin();

  return 0;
}
