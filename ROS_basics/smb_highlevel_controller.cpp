#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>

class SmbController {
private:
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber scan_sub_;
    float kp_ = 100.0; //Got the value of pid gain from smb_control config.yaml file 

public:
    SmbController() {
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        scan_sub_ = nh_.subscribe("/scan", 1, &SmbController::topicCallback, this);
    }

    void topicCallback(const sensor_msgs::LaserScan& msg) {
        
        float smallest_distance = *std::min_element(msg.ranges.begin(), msg.ranges.end());
        ROS_INFO("Smallest distance: %f", smallest_distance);
        int pillar_idx = -1;      //Initially setting index of that point in ranges array which is nearest to robot (directly in front of robot) -1
        float min = msg.range_max;
        for (int i = 0; i < msg.ranges.size(); i++) { //Applying loop on ranges array and finding the min distance and its index
            if (msg.ranges[i] < min) {                //Can also directly use smallest_distance as minimum distance
                min = msg.ranges[i]; 
                pillar_idx = i;
            }
        }
        float pillar_angle = msg.angle_min + pillar_idx * msg.angle_increment; //error = pillar_angle left to be covered from -90 degree
        geometry_msgs::Twist cmd_vel;
        cmd_vel.angular.z = kp_ * pillar_angle;   //Twisting amount decided by kp * error
        cmd_vel.linear.x = 0.6; //Here also we can apply cmd_vel_.linear.x = kp_*(minimum distance to pillar)
        vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "smb_controller");
    SmbController controller;
    ros::spin();
    return 0;
}

