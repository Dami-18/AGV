#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String& msg){
    ROS_INFO("I heard: [%s]", msg.data.c_str());
}
int main(int argc, char* argv[]){
        ros::init(argc, argv, "listener_node");
        ros::NodeHandle nh2;
        ros::Subscriber firstsubscriber =
        nh2.subscribe("greetings",10,chatterCallback);
        ros::spin();
        return 0;
}

