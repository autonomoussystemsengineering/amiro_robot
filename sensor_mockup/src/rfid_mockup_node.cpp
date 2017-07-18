#include "ros/ros.h"
#include <tuple>
#include "xmlrpcpp/XmlRpc.h"
#include <boost/algorithm/string.hpp>
#include "nav_msgs/Odometry.h"
#include "std_msgs/UInt32MultiArray.h"

ros::Publisher pub;
static std::vector<std::tuple<double, double, double, uint32_t>> tag_xyzi;
enum tag {
    x  = 0,
    y  = 1,
    z  = 2,
    id = 3
};
static double max_tag_distance_m;


void getTags(ros::NodeHandle &n) {

  ROS_INFO_STREAM("Start tag parsing");
  std::string parsePrefix("tag_");
  std::vector<std::string> keys;
  n.getParamNames(keys);
  for (auto key = keys.begin(); key < keys.end(); ++key) {

    // Continue if the parameter is not in our namespace
    try {
      if (key->compare(0, n.getNamespace().size(), n.getNamespace()) != 0) {
        ROS_DEBUG_STREAM(
            *key << " is not a parameter in the " << n.getNamespace()
                << " namespace => continue");
        continue;
      }
    } catch (...) {
      continue;
    }
    // Get rid of the namespace and check for the correct prefix
    std::vector<std::string> strs;
    boost::split(strs, *key, boost::is_any_of("/"));
    if (strs.back().size() < parsePrefix.size()) {
      ROS_DEBUG_STREAM(*key << " to short");
      continue;
    } else {
      ROS_DEBUG_STREAM(*key << " matches in length");
    }

    // Parse the parameters
    if (strs.back().compare(0, parsePrefix.size(), parsePrefix) == 0) {
      try {
        ROS_DEBUG_STREAM(*key << " matches");
        XmlRpc::XmlRpcValue parsedList;
        n.getParam(*key, parsedList);
        ROS_ASSERT(parsedList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(parsedList.size() == 4);  // s.t. 4 is the number of possible entries [x, y, z, id]
        ROS_ASSERT(parsedList[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(parsedList[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(parsedList[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(parsedList[3].getType() == XmlRpc::XmlRpcValue::TypeInt);
        tag_xyzi.push_back(std::make_tuple<double, double, double, uint32_t>(
            static_cast<double>(parsedList[0]),
            static_cast<double>(parsedList[1]),
            static_cast<double>(parsedList[2]),
            static_cast<int>(parsedList[3])));
        ROS_INFO_STREAM("Load " << *key << "with xyzi: " <<
                        std::get<tag::x>(*(tag_xyzi.begin())) << " " <<
                        std::get<tag::x>(*(tag_xyzi.begin())) << " " <<
                        std::get<tag::y>(*(tag_xyzi.begin())) << " " <<
                        std::get<tag::z>(*(tag_xyzi.begin())) << " ");
      } catch (XmlRpc::XmlRpcException a) {
        std::cerr << "XmlRpc exception: " << a.getMessage() << std::endl;
      }
    } else {
      ROS_DEBUG_STREAM(*key << " does not match");
    }
  }
  ROS_INFO_STREAM("End parsing");
}

void callback(const nav_msgs::Odometry msg) {

  // Iterate through all given tags
  std_msgs::UInt32MultiArray tags;
  for(auto it = tag_xyzi.begin(); it != tag_xyzi.end(); ++it) {
    const double x = msg.pose.pose.position.x - std::get<tag::x>(*it);
    const double y = msg.pose.pose.position.y - std::get<tag::y>(*it);
    const double z = msg.pose.pose.position.z - std::get<tag::z>(*it);
    if (sqrt(x*x+y*y+z*z) <= max_tag_distance_m) {
      tags.data.push_back(std::get<tag::id>(*it));
    }
  }

  pub.publish(tags);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rfid_mockup_node");
  ros::NodeHandle n("~");

  ROS_INFO("Started rfid_mockup_node");

  std::string topic_out,topic_in;

  n.param<std::string>("topic_in", topic_in, "/amiro1/odom");
  n.param<std::string>("topic_out", topic_out, "/amiro1/rfid_tag_list");
  n.param<double>("max_tag_distance_m", max_tag_distance_m, 0.1);

  getTags(n);
  ros::Subscriber sub = n.subscribe(topic_in, 1, callback);
  pub = n.advertise<std_msgs::UInt32MultiArray>(topic_out, 1);

  ros::spin();

  return 0;
}
