#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <string>

namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle& nodeHandle) :
  nodeHandle_(nodeHandle)
{
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  ROS_INFO("Successfully launched node.");
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 10, &SmbHighlevelController::topicCallback, this);
}

SmbHighlevelController::~SmbHighlevelController()
{
}

bool SmbHighlevelController::readParameters()
{
  if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void SmbHighlevelController::topicCallback(const sensor_msgs::LaserScan& msg)
{
  float smallest_distance = *std::min_element(msg.ranges.begin(), msg.ranges.end());
  ROS_INFO("Smallest distance: %f", smallest_distance);
}

} /* namespace */
