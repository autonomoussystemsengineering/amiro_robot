#include "ros/ros.h"
#include <limits>
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt16.h"

ros::Publisher pub;

void callback(const sensor_msgs::Image::ConstPtr msg) {

  size_t sum = 0;
  // Integrate over all pixel values
  for(auto it = msg->data.begin(); it != msg->data.end(); ++it) {
    sum += size_t(*it);
  }
  // Normalize
  sum /= msg->data.size();

  // Normalize to unsigned short and send
  std_msgs::UInt16 value;
  value.data = uint16_t((sum / 255.0) * double(std::numeric_limits<unsigned short>::max()));
  pub.publish(value);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "proximity_floor_mockup_node");
  ros::NodeHandle n("~");

  ROS_INFO("Started proximity_floor_mockup_node");

  std::string topic_out,topic_in;

  n.param<std::string>("topic_in", topic_in, "/amiro1/proximity_floor_1/image_raw");
  n.param<std::string>("topic_out", topic_out, "/amiro1/proximity_floor_1/value");

  ros::Subscriber sub = n.subscribe(topic_in, 1, callback);
  pub = n.advertise<std_msgs::UInt16>(topic_out, 1);

  ros::spin();

  return 0;
}
