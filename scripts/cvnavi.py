#!/usr/bin/env python2
import argparse
import rospy
from math import pi

from glooping_at_the_floor_server import GloopingAtTheFloorServer

CAMERA_TOPICS = ['/usb_cam/image_rect', '/usb_cam/image_raw', '/camera/rgb/image_color']


def getArgs():
    topicNum = 2
    ap = argparse.ArgumentParser()
    ap.add_argument('-n', '--normal', required=False,
                    default=0.52,
                    help='norm length from camera to floor in [m]')
    ap.add_argument('-a', '--alpha', required=False,
                    default=18.3,
                    help='angle between camera optical axis and norm to floor in [deg]')

    ap.add_argument('-t', '--topic', required=False,
                    help='camera topic num')

    args = vars(ap.parse_args())
    
    alpha = float(args['alpha']) * pi / 180
    alpha  = 51 * pi / 180
    n = float(args['normal'])
    n = 0.47

    if args['topic'] is not None:
        topicNum = int(args['topic'])
    return alpha, n, topicNum


def cvnavi():
    """ the main launching function """
    alpha, n, topicNum = getArgs()
    rospy.init_node('cv_tape_dtc', anonymous=False)
    rospy.loginfo("Camera's topic is " + CAMERA_TOPICS[topicNum])
    glooping = GloopingAtTheFloorServer(CAMERA_TOPICS[topicNum], alpha, n)
    glooping.startServer()
    rospy.loginfo('Spining...')
    rospy.spin()
    del(glooping)

if __name__ == '__main__':
    try:
        cvnavi()
    except rospy.ROSInterruptException, e:
        print("cvnavi.py: " + e.message)
