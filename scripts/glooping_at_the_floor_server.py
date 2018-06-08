import rospy
import tf
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from math import sin, cos, atan2, sqrt

from libs.utils import getCVImage, getMsgImage
from libs.line_finder import LineFinder
from libs.geometry import unitVector, angleBetween

from red_cvnavi.srv import CameraFloor

# FOVS = (43.78, 54.35, 65.47)    # vertical, horizontal, diagonal angles. Logitech 920FullHD (640, 480)
FOVS = (41.5, 53.6, 64.5)          # not very precise fovs for Intel RealSense SR300 (4:3)

class GloopingAtTheFloorServer:

    def __init__(self, cameraTopic, frame, alpha, n):
        self.alpha = alpha
        self.n = n

        self.cameraInfo = dict()

        self.cameraInfo['shape'] = None # w, h, d == x, y, d
        self.cameraInfo['ratios'] = None  # ration [mm] / [px]
        self.cameraInfo['fovs'] = {'v': FOVS[0], 'h': FOVS[1], 'd': FOVS[2]}

        self.name = 'GloopingAtTheFloor' + str(self)

        self.cameraTopic = cameraTopic

        self.imageInfo = dict()
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None  # ration [mm] / [px]

        self.subCamera = None
        self.cameraGloopingServer = None

        self.points = None

        self.pubView = rospy.Publisher('see_tape', Image, queue_size=1)
        self.pubLineRed = rospy.Publisher('red_tape', LaserScan, queue_size=1)
        self.pubLineYellow = rospy.Publisher('yellow_tape', LaserScan, queue_size=1)
        self.rate = rospy.Rate(10.0)

        self.translationFromTF = True
        self.refFrame = frame
        self.cameraFrame = '/realsense_camera'
        self.baseFrame = '/base_link'
        self.listener = tf.TransformListener()
        self.t = tf.TransformerROS(True)
        self.br = tf.TransformBroadcaster()

        self.visualisation = True

    def __del__(self):
        del(self.subCamera)
        del(self.cameraGloopingServer)
        del(self.pubView)

    def callback(self, imageMsg):
        cvImage, shape = getCVImage(imageMsg)
        self.cameraInfo['shape'] = shape
        if self.translationFromTF:
            try:
                (trans,rot) = self.listener.lookupTransform(self.refFrame , self.cameraFrame, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo('cvnavi TF error')

            # TM - Transform matrix
            TM = self.t.fromTranslationRotation(trans, rot)

            # find alpha as angle between direction of Z axis of camera frame and vector directed to the floor
            z_target = np.array([TM[0,2],TM[1,2],TM[2,2]])
            z_ref = np.array([0, 0, -1])
            # z_ref = unitVector(np.array([z_target[0], z_target[1], 0]))

            dist_to_floor = 0.3 # distance from arm1 to floor
            self.n = trans[2] + dist_to_floor
            self.alpha = angleBetween(z_target, z_ref)


        fl = LineFinder(self.cameraInfo, self.alpha, self.n)

        # tape_colors = ['r', 'y'] # find red and yellow lines
        tape_colors = ['r'] # find only red lines
        for tType in tape_colors:
            self.points, imageEx, isFind = fl.getLine(cvImage, tapeType=tType, self.visualisation)
            if isFind:
                msg = self.getLaserScanMsg(self.points)
                if tType = 'r':
                    self.pubLineRed.publish(msg)
                else:
                    self.pubLineYellow.publish(msg)

            # publish decorated image
            if self.visualisation:
                imageMsgEx = getMsgImage(imageEx)
                self.pubView.publish(imageMsgEx)
        self.rate.sleep()

    def handler(self, request):

        if request.switch:
            self.subCamera = rospy.Subscriber(self.cameraTopic, Image, self.callback)
            
            rospy.loginfo('Work was started! For more detail see topics /see_tape and /camera_tape')
            rospy.loginfo('Working...')
        else:
            if self.subCamera is not None:
                self.subCamera.unregister()
                self.subCamera = None
                rospy.loginfo('Work was stoped!')
            else:
                rospy.loginfo('Work dont started, please call with switch: true')
        return {}

    def getLaserScanMsgOld(self, points, TM):
        # angle between camera and x axis of refFrame
        dgamma = atan2(TM[1,2], TM[0,2])

        # print(dgamma)
        minAngle = -1.5 + dgamma
        maxAngle = 1.5 + dgamma
        incrementAngle = 0.05
        scanTime = 0.01
        minRange = 0.05
        maxRange = 8

        msg = LaserScan()
        msg.header.frame_id = self.refFrame
        msg.header.stamp = rospy.Time()
        msg.angle_min = minAngle
        msg.angle_max = maxAngle
        msg.angle_increment = incrementAngle
        msg.scan_time = scanTime
        msg.range_min = minRange
        msg.range_max = maxRange

        laser_points = []
        try:
                (trans,rot) = self.listener.lookupTransform(self.refFrame , self.cameraFrame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('cvnavi TF error')

        for point in points:

            dx = trans[0]
            dy = trans[1]

            x_cam = sqrt(point[1]**2 + point[2]**2 - self.n**2)
            y_cam = -point[0]

            Sin = sin(dgamma)
            Cos = cos(dgamma)

            x = x_cam * Cos + y_cam * Sin + dx
            y = x_cam * Sin + y_cam * Cos + dy

            # print('x_cam = ', x_cam, "   y_cam = ", y_cam)
            # print('xd = ', dx, "   yd = ", dy)
            # print('x = ', x, "   y = ", y)
            # print(point)
            # print(trans)

            gamma = atan2(y, x)
            k = sqrt(x*x + y*y)
            # print(gamma, k)
            laser_points.append([gamma,k])

        for i in range(int((maxAngle - minAngle)/incrementAngle)):
            cur_angle = i * incrementAngle + minAngle
            for laser_point in laser_points:
                # print("lp = ",laser_point[0], "   cur = ", cur_angle)
                if ((laser_point[0] > cur_angle) and (laser_point[0] < cur_angle + incrementAngle)):
                    msg.ranges.append(laser_point[1])
                    print(i, cur_angle)
                else:
                    msg.ranges.append(maxRange)
        print()
        return msg

    def getLaserScanMsg(self, points):

        # parameters of laserscan message
        minAngle = -1.5
        maxAngle = 1.5
        incrementAngle = 0.05
        scanTime = 0.1
        minRange = 0.05
        maxRange = 8

        # make new frame projection of arm_link1 to Oxy plane of base_link 
        try:
            (trans,rot) = self.listener.lookupTransform(self.refFrame , self.baseFrame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('cvnavi TF error')


        tapeFrame = "/tape_laser_scan"
        self.br.sendTransform((0.0, 0.0, trans[2]),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         tapeFrame,
                         self.refFrame)

        msg = LaserScan()
        msg.header.frame_id = tapeFrame
        msg.header.stamp = rospy.Time()
        msg.angle_min = minAngle
        msg.angle_max = maxAngle
        msg.angle_increment = incrementAngle
        msg.scan_time = scanTime
        msg.range_min = minRange
        msg.range_max = maxRange


        try:
            (trans,rot) = self.listener.lookupTransform(self.refFrame , self.cameraFrame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo('cvnavi TF error')

        laser_points = [] 

        for point in points:
            # transform 3d coordinates in camera frame to 2d coordinates in tapeFrame
            x = sqrt(point[1]**2 + point[2]**2 - self.n**2) + trans[0]
            y = -point[0] + trans[1]

            # tranform 2d coordinates to polar
            theta = atan2(y, x)
            r = sqrt(x*x + y*y)

            laser_points.append([theta,r])

        # make ranges of laserscan
        # if theta ~ angle of laserscan then: range = r
        # else: range = max range # ROS discard max rage values?
        for i in range(int((maxAngle - minAngle)/incrementAngle)):
            cur_angle = i * incrementAngle + minAngle
            flag = True
            for laser_point in laser_points:
                if ((laser_point[0] > cur_angle) and (laser_point[0] < cur_angle + incrementAngle)):
                    msg.ranges.append(laser_point[1])
                    flag = False
                    break       
            if flag:
                msg.ranges.append(maxRange)

        return msg


    def startServer(self):
        self.cameraGloopingServer = rospy.Service('camera_floor', CameraFloor, self.handler)
        rospy.loginfo('GloopingAtTheFloor start!')