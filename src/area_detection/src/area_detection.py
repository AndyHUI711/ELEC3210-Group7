#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

AREA = {
    "A": ((5, 0), (-3.5, -3.5)),
    "B": ((5, -3.5), (-3.5, -7)),
    "C": ((5, -7), (-3.5, -14)),
    "D": ((10.5, 0), (5, -14)),
}

class Areadetection:
    def __init__(self):
        self.publisher = rospy.Publisher("/area", String, queue_size=1)
        self.subscriber = rospy.Subscriber("/slam_out_pose", PoseStamped, self.callback, queue_size=1)
        self.last_area = "Unknown"

    def callback(self, msg_in):
        if int(msg_in.pose.position.x) <= 15 and int (msg_in.pose.position.y) <=15 and int(msg_in.pose.position.x) >= -15 and int (msg_in.pose.position.y)>= -15:
            global x
            global y
            x = msg_in.pose.position.x
            y = msg_in.pose.position.y
            area = self.last_area

        for a, coor in AREA.items():
        #UnboundLocalError: local variable 'x' referenced before assignment

            if self._in_area(x, y, coor):
                area = a

        if area == self.last_area:
            return

        self.last_area = area
        self.publisher.publish(area)

    def _in_area(self, x, y, coor):
        return coor[0][0] >= x and x >= coor[1][0] and coor[0][1] >= y and y >= coor[1][1]

def main():
    rospy.init_node("area_detection", anonymous=True)
    node = Areadetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupted")

if __name__ == '__main__':
    main()
