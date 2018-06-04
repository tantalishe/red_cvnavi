import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from libs.utils import getCVImage, getMsgImage
from libs.line_finder import LineFinder

from red_cvnavi.srv import CameraFloor

# FOVS = (43.78, 54.35, 65.47)    # vertical, horizontal, diagonal angles. Logitech 920FullHD (640, 480)
FOVS = (41.5, 53.6, 64.5)          # not very precise fovs for Intel RealSense SR300 (4:3)

class GloopingAtTheFloorServer:

    def __init__(self, cameraTopic, alpha, n):
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
        self.pubLine = rospy.Publisher('camera_tape', PoseArray, queue_size=1)

    def __del__(self):
        del(self.subCamera)
        del(self.cameraGloopingServer)
        del(self.pubView)

    def callback(self, imageMsg):
        cvImage, shape = getCVImage(imageMsg)
        self.cameraInfo['shape'] = shape
        fl = LineFinder(self.cameraInfo, self.alpha, self.n)

        # tape_colors = ['r', 'y'] # find red and yellow lines
        tape_colors = ['r'] # find only red lines
        for tType in tape_colors:
            self.points, imageEx, isFind = fl.getLine(cvImage, tapeType=tType)
            if isFind:
                msg = PoseArray()
                msg.header.frame_id = 'camera'
                msg.header.stamp = rospy.Time()
                if tType == 'r':
                    msg.header.seq = 0
                elif tType == 'y':
                    msg.header.seq = 1

                for point in self.points:
                    p = Pose()
                    p.position.x = point[0]
                    p.position.y = point[1]
                    p.position.z = point[2]
                    msg.poses.append(p)

                self.pubLine.publish(msg)

            # publish decorated image
            imageMsgEx = getMsgImage(imageEx)
            self.pubView.publish(imageMsgEx)

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

    def startServer(self):
        self.cameraGloopingServer = rospy.Service('camera_floor', CameraFloor, self.handler)
        rospy.loginfo('GloopingAtTheFloor start!')
