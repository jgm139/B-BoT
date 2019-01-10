#ifndef _INTROSPECTION_ONLINE_UTILS_
#define _INTROSPECTION_ONLINE_UTILS_

#include <ros/ros.h>
#include <dynamic_introspection/dynamic_introspection_utils.h>
#include <pal_statistics_msgs/Statistics.h>
#include <boost/scoped_ptr.hpp>
#include <ros/callback_queue.h>

namespace dynamic_introspection
{
class IntrospectionOnlineReader : public IntrospectionBagReader
{
public:
  IntrospectionOnlineReader(const std::string &topic = "/introspection_data");

  virtual ~IntrospectionOnlineReader();

  void start();

  void stop();

  void dumpRosBag(const std::string &bag_name);

private:
  ros::NodeHandle nh_;
  ros::CallbackQueue cb_queue_;
  boost::scoped_ptr<ros::AsyncSpinner> spinner_;
  ros::Subscriber sub_;

  void introspectionCB(const pal_statistics_msgs::StatisticsConstPtr &msg);

  std::vector<pal_statistics_msgs::Statistics> read_messages_;
};
}

#endif
