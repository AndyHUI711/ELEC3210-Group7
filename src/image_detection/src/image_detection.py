#!/usr/bin/env python
#reference:
#https://vimsky.com/zh-tw/examples/detail/python-method-visualization_msgs.msg.Marker.html
import rospy
import math
import time
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
##from visualization_msgs.msg.Marker import action

class ImageDetection:
    def __init__(self):
        self.last_id = 0
        self.subscriber = rospy.Subscriber("/vrep/image", Image, self.callback, queue_size=100)
        self.publisher = rospy.Publisher("image_marker", Marker, queue_size=100)
        self.publisher2 = rospy.Publisher("/id", String, queue_size=1)
	
        self.bridge = CvBridge()

        # Load the image
        path = "/home/user/catkin_ws/src/image_detection/image/"
        pictures_name = ["pic001", "pic002", "pic003", "pic004", "pic005"]
        self.pictures = [cv2.imread(path+name+".jpg") for name in pictures_name]
        #pictures_name = ["pic001"]
        #self.pictures = [cv2.imread(path+name+".jpg") for name in pictures_name]
	
        self.orb_cnt = [0] * len(self.pictures)
        self.square_cnt = [0] * len(self.pictures)
        self.marked = [False] * len(self.pictures)

        self.bf = cv2.BFMatcher()
        self.orb = cv2.ORB_create(nfeatures=500)
        self.keypoints, self.descriptors = [], []
        ##okprint('test hello world1')
        
        for i, picture in enumerate(self.pictures):
            size = (400, 400)
            ##okprint('test hello world',i)
            if picture is None:
                print('Wrong path:', path)
            else:
                picture = cv2.resize(picture, size, interpolation = cv2.INTER_AREA)
                
            # find the keypoints and descriptors with ORB
            kp = self.orb.detect(picture, None)
            kp, des = self.orb.compute(picture, kp)
            # some pictures in env map is flipped
            picture = cv2.flip(picture, 1)
            kp2 = self.orb.detect(picture, None)
            kp2, des2 = self.orb.compute(picture, kp2)
            picture = cv2.flip(picture, 0)
            kp3 = self.orb.detect(picture, None)
            kp3, des3 = self.orb.compute(picture, kp3)
            
            self.keypoints.append((kp,kp2,kp3))
            self.descriptors.append((des, des2,des3))
            
        self.markers = [Marker() for i in range(len(pictures_name))]
        ##okprint('test hello worldxx1')
        
        ##self.markers = MarkerArray()
        self.markers = []
        ##okprint('test hello worldxx2')
        
        for i in range(len(pictures_name)): ##arrray size picture_name = 5
            marker = Marker()
            marker.header.frame_id = "camera_link"
            #marker.header.stamp = rospy.Time.now()
            marker.ns = "picture"
            #marker.id = 0
            #marker.type = Marker.CUBE
            #marker.pose.orientation.x = 1.0
            #marker.pose.orientation.y = 1.0
            #marker.pose.orientation.z = 1.0
            
            
            marker.type = marker.TEXT_VIEW_FACING
            marker.text = pictures_name[i]
            marker.action = marker.ADD

            marker.pose.position.x = 0.0
            marker.pose.position.y = 1.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 1
            marker.scale.y = 1
            marker.scale.z = 1
            marker.id = i 

            marker.color.r = 0
            marker.color.g = 1
            marker.color.b = 1
            marker.color.a = 1
            
            marker.lifetime = rospy.Duration()
            self.markers.append(marker)
            #print(self.markers) ##works
            
    def callback(self, img):
        img = self.bridge.imgmsg_to_cv2(img, "bgr8")
        id, cnt = self._best_fit(img)
        print(cnt)
        ##best_id, best
        br = CvBridge()
        _id = self.last_id

        if id == -1:
            print("return")
            return

        if (cnt < 60): #120 #40 for test
            
            self.orb_cnt[id] = 0
        else:
           
            self.orb_cnt[id] += 1

        if(self.orb_cnt[id] >= 10 and not self.marked[id]):
            print(self.orb_cnt[id])
            self._mark(id, img)
            _id=id+1

            if _id == self.last_id:
                return

            self.last_id = _id
            
            self.publisher2.publish(str(_id))    
            print('id:',id)  ## work   

      
    #### callback call _best_fit          

    def _best_fit(self, img):
        try:
            best_id = -1
            best = -1
            
            keypoint = self.orb.detect(img, None)
            keypoint, descriptor = self.orb.compute(img, keypoint)

            for i, des in enumerate(self.descriptors):
                cur_cnt = 0
                

                for dd in des:
                    # BFMatcher with default params
                    bf = cv2.BFMatcher()
                    matches = self.bf.knnMatch(dd, descriptor, k=2)
                    
                    #print("test",cur_cnt,best) #work
                    #print(dd)
                    # Apply ratio test
                    for m, n in matches:
                        if m.distance < 0.75 *n.distance:
      
                            cur_cnt += 1

                if cur_cnt > best:
                    best = cur_cnt
                    best_id = i

            return best_id, best        
        except Exception as err:
            #print("test ") #work
            return -1, -1
                    
    def _mark(self, id, img):
        print('test hello world def mark')## work
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img_BW = cv2.threshold(img_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        contours, hierarchy = cv2.findContours(img_BW, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        ratio = float(w)/h

        if ratio > 1.1 and ratio < 0.5:
            self.square_cnt[id] = 0
        else:
            self.square_cnt[id] += 1

        if (self.square_cnt[id] >= 15):
            self.markers[id].pose.position.x = 1 / math.tan(math.pi / 8 * h / img.shape[1]) * 0.5
            self.markers[id].pose.position.y = (x + w/2) / img.shape[1]
            self.markers[id].pose.position.z = 0
            #print(self.markers[id].pose.position)
            #print("here")
            #print(self.markers[id])
            #print(id)  till here is work

            self.publisher.publish(self.markers[id])
            print(self.markers[id])
            print("sent") 
            self.marked[id] = True
            #return
            




def main():
#Case with multiple nodes publishing on a topic
#Simply set the node as anonymous
    rospy.init_node("image_detection", anonymous=True)
    node = ImageDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupted")
        
if __name__ == "__main__":
    main()
