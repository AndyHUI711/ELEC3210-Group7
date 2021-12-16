#!/usr/bin/env python

import numpy as np
import cv2
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

bridge = CvBridge()

laser_scan_on = True

def auto_mode_callback(msg):
    global laser_scan_on
    laser_scan_on = msg.data
    
def image_callback(msg):
    global cv2_img
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2_img = cv2.flip(cv2_img, 1)
        
        hsv = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        #find ball 
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) < 1 or laser_scan_on:
            return

        #locate  ball
        for c in contours:
            M = cv2.moments(c)
            cX = float(M["m10"]/M["m00"])
            cY = float(M["m01"]/M["m00"])
            rX = int(M["m10"]/M["m00"])
            rY = int(M["m01"]/M["m00"])
            radius = int(math.sqrt(cv2.contourArea(c)/math.pi))


        h,w = cv2_img.shape[:2]
        (ideal_X, ideal_Y) = (w/2, h-(20 + radius))
        verticle_diff = cY-ideal_Y
        angle_diff = cX-ideal_X
        
        pub = rospy.Publisher('/vrep/cmd_vel', Twist, queue_size=10)
        
        twist = Twist()
        #linear
        if verticle_diff <= -50:
            twist.linear.x = 1.1
        elif (verticle_diff > -50) & (verticle_diff < 0):
            twist.linear.x = 0.5
        elif verticle_diff >= 20:
            twist.linear.x = -0.6
        elif (verticle_diff <20) & (verticle_diff > 5):
            twist.linear.x = -0.3
        else:
            twist.linear.x = 0    
        #angular
        if angle_diff >= 30:
            twist.angular.z = -1
        elif (angle_diff < 30) & (angle_diff > 10):
            twist.angular.z = -0.5
        elif angle_diff <= -30:
            twist.angular.z = 1
        elif (angle_diff > -30) & (angle_diff < -10):
            twist.angular.z = 0.5
        else:
            twist.angular.z = 0
        
        pub.publish(twist)
        
        
        copy_img = cv2_img.copy()
        cv2.drawContours(copy_img, contours, -1, (0, 0, 255), 2)
        cv2.circle(copy_img, (rX, rY), 3, (255, 0, 0), -1)
        cv2.putText(copy_img, "centroid", (rX - 25, rY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)



    except CvBridgeError as err:
        print(err)

def main():
    rospy.init_node('ball', anonymous=True)
    rospy.Subscriber('/vrep/laser_switch', Bool, auto_mode_callback)
    rospy.Subscriber('/vrep/image', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
