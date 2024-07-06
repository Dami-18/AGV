import rospy
import rosbag
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
import numpy as np

def add_gaussian_noise(velocity, mean=0, std_dev=0.1):
    noisy_vel = Twist()
    noisy_vel.linear.x = velocity.linear.x + np.random.normal(mean, std_dev)
    noisy_vel.linear.y = velocity.linear.y
    noisy_vel.linear.z = velocity.linear.z
    noisy_vel.angular.z = velocity.angular.z + np.random.normal(mean, std_dev)
    noisy_vel.angular.y = velocity.angular.y
    noisy_vel.angular.x = velocity.angular.x
    return noisy_vel

def main():
    rospy.init_node('noisy_vel_pub')

    # Read bag file
    bag = rosbag.Bag('/home/vboxuser/catkin_ws/turtlebot_messages_2024-05-11-18-53-48.bag')

    # Create a publisher for the noisy velocity
    pub = rospy.Publisher('/noisy_vel', Twist, queue_size=10)

    for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
        # Add Gaussian noise to the velocity
        noisy_velocity = add_gaussian_noise(msg)

        # Publish the noisy velocity
        pub.publish(noisy_velocity)
        rospy.loginfo("Published Noisy Velocity: {}".format(noisy_velocity))

        # Sleep for a short duration if needed
        rospy.sleep(0.1)

    bag.close()

if __name__ == '__main__':
    main()
