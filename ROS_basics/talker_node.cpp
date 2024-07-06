#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "talker_node");
    ros::NodeHandle nh;
    ros::Publisher firstpublisher = nh.advertise<std_msgs::String>("greetings",10);
    ros::Rate loop_rate(10);
    unsigned int count = 0;
    while(ros::ok()){
        std_msgs::String message;
        message.data = "Hello peeps! " + std::to_string(count);
        ROS_INFO_STREAM(message.data);
        firstpublisher.publish(message);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}