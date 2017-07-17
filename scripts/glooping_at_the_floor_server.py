import rospy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

from libs.utils import getCVImage, getMsgImage
from libs.line_finder import LineFinder

from red_msgs.srv import CameraFloor

FOVS = (43.78, 54.35, 65.47)    # vertical, horizontal, diagonal angles. Logitech 920FullHD (640, 480)


class GloopingAtTheFloorServer:

    def __init__(self, cameraTopic, alpha, n):
        self.alpha = alpha
        self.n = n

        self.cameraInfo = dict()

        self.cameraInfo['shape'] = None # w, h, d == x, y, d
        self.cameraInfo['ratios'] = None  # ration [mm] / [px]
        self.cameraInfo['fovs'] = {'v': FOVS[0], 'h': FOVS[1], 'd': FOVS[2]}

        self.name = 'GloopingArTheFloor' + str(self)

        self.cameraTopic = cameraTopic

        self.imageInfo = dict()
        self.imageInfo['shape'] = None
        self.imageInfo['ratio'] = None  # ration [mm] / [px]

        self.subCamera = None
        self.cameraGloopingServer = None

        self.left = None
        self.right = None

        self.pubView = rospy.Publisher('see_type', Image, queue_size=1)
        self.pubLine = rospy.Publisher('camera_type', PoseArray, queue_size=1)

    def __del__(self):
        del(self.subCamera)
        del(self.cameraGloopingServer)
        del(self.pubView)

    def callback(self, imageMsg):
        cvImage, shape = getCVImage(imageMsg)
        self.cameraInfo['shape'] = shape
        fl = LineFinder(self.cameraInfo, self.alpha, self.n)
        self.left, self.right, imageEx = fl.getLine(cvImage)

        msg = PoseArray()
        msg.header.frame_id = 'camera'
        msg.header.stamp = rospy.Time()

        # leftest point
        p = Pose()
        p.position.x = self.left[0]
        p.position.y = self.left[1]
        p.position.z = self.left[2]
        msg.poses.append(p)
        # rightest point
        p.position.x = self.right[0]
        p.position.y = self.right[1]
        p.position.z = self.right[2]
        msg.poses.append(p)

        self.pubLine.publish(msg)

        imageMsgEx = getMsgImage(imageEx)
        self.pubView.publish(imageMsgEx)

    def handler(self, request):
        if request.switch:
            self.subCamera = rospy.Subscriber(self.cameraTopic, Image, self.callback)
            rospy.loginfo('Work was started! For more detail see topic /see_type and /camera_type')
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
