#!/usr/bin/env python

import numpy as np
import rospy
import cv2
from cv2 import aruco
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import csv

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()
        self.current_pose = None
        self.waypoints = []
        self.waypoint_file = open('waypoints.csv', mode='w', newline='')
        self.writer = csv.writer(self.waypoint_file)
        self.writer.writerow(['Marker_ID', 'X', 'Y', 'Theta'])
           
    def pose_callback(self, data):
        self.current_pose = data.pose.pose

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                if self.current_pose is not None:
                    x = self.current_pose.position.x
                    y = self.current_pose.position.y
                    theta = self.euler_from_quaternion(self.current_pose.orientation)
                    self.waypoints.append([marker_id, x, y, theta])
                    self.writer.writerow([marker_id, x, y, theta])
                    print(self.waypoints)
                    print("Marker detected!")
        else:
            print("Marker not detected!")

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        return [roll_x, pitch_y, yaw_z]

    def shutdown(self):
        self.waypoint_file.close()

if __name__ == '__main__':
    rospy.init_node('aruco_detector_node', anonymous=True)
    aruco_detector = ArucoDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    aruco_detector.shutdown()
