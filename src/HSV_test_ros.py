#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('realsense_camera')
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

BLUR = 3
THRESHOLD_KERNEL = 11 # 11
THRESHOLD_PARAMETER = 3 # 3
DILATE_ITER = 5
ERODE_ITER = 2


def nothing(x):
    pass

def getLine(image, tapeType='r'):
        # isFind = False
        # select type of tape
        # default white/red tape

        color = [0, 0, 255]
        lower1 = np.array([160, 70, 0]) #706BA5
        upper1 = np.array([180, 255, 255]) #D3D7DF
        lower2 = np.array([0, 100, 0]) #706BA5
        upper2 = np.array([15, 255, 255]) #D3D7DF
        # lower = np.array([165, 107, 112]) #706BA5
        # upper = np.array([211, 255, 255]) #D3D7DF

        # or select
        if tapeType == 'y':
            color = [0, 200, 200]
            lower = np.array([165, 107, 112])
            upper = np.array([211, 215, 223])
        elif tapeType == 'b':
            color = [255, 0, 0]
            lower = np.array([165, 107, 112])
            upper = np.array([211, 215, 223])

        h, w, _ = image.shape

        # filtering and lentochka selection in bin image
        raw = cv2.blur(image, (5, 5))
        hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        res = cv2.bitwise_or(mask1, mask2)

        return res

def getHSV(image,hsv):
    image[:,:] = hsv
    rgb_color = cv2.cvtColor(image,cv2.COLOR_HSV2BGR)
    return rgb_color


def centroid(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx, cy

# cv2.namedWindow('Bars')
# cv2.createTrackbar('hue', 'Bars', 180, 255, nothing)
# cv2.createTrackbar('saturation', 'Bars', 215, 255, nothing)
# cv2.createTrackbar('value', 'Bars', 200, 255, nothing)

class image_converter:
  
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
        self.image_pub1 = rospy.Publisher("see_tape1", Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("see_tape2", Image, queue_size=1)

    def callback(self,data):
        
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        mask = getLine(image)

        # v = np.median(mask)
        # sigma = 0.33
        # canny_low = int(max(0, (1 - sigma) * v))
        # canny_high = int(min(255, (1 + sigma) * v))
        # th = cv2.Canny(mask, 1, 250)
        # mask = cv2.dilate(mask, None, iterations=2)
        # mask = cv2.erode(mask, None, iterations=2)
        # kernel = np.array([[0,0,1,1,1,0,0],[0,1,1,1,1,1,0],[1,1,1,1,1,1,1],[1,1,1,1,1,1,1],[1,1,1,1,1,1,1],[0,1,1,1,1,1,0],[0,0,1,1,1,0,0]],np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations= 3)

        # find contours centres of mass
        raw = cv2.blur(image, (5, 5))
        hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

        # cv2.imshow("frame", image)
        # cv2.imshow("edge", edged)
        # cv2.imshow("canny", canny)


        image1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        try:
           self.image_pub1.publish(self.bridge.cv2_to_imgmsg(image1,"bgr8"))
        except CvBridgeError as e:
            print(e)

        try:
           self.image_pub2.publish(self.bridge.cv2_to_imgmsg(hsv,"bgr8"))
        except CvBridgeError as e:
            print(e)

        # cv2.waitKey(1)

def main():
    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, e:
        print("HSV.py: " + e.message)
