#!/usr/bin/env python
# coding: utf-8
import cv2
import time
import numpy as np
from math import sin, cos, tan, atan2, pi
import scipy as sp
from utils import *
from scipy import matrix

RESOLUTION = (640, 480)         # Width, Height
FOVS = (43.78, 54.35, 65.47)    # vertical, horizontal, diagonal angles. Logitech 920FullHD (640, 480)
VIDEO_DEV = 1                   # /dev/video1


class LineFinder:

    def __init__(self, cameraInfo, alpha=0.6, n=0.6):
        self.n = n                    # [m] length of normal line from camera to floor
        self.alpha = alpha            # [rad] angle between X-axis of camera and floor
        self.cameraInfo = cameraInfo

        # image size
        x, y, _ = cameraInfo['shape']
        self.xy0 = (x, y)               # длины осей

        # center RF
        self.CRF = (y / 2, x / 2)       # поворот системы координат кадра на pi/2 и перенос в центр кадра

        # self.cam = cv2.VideoCapture(VIDEO_DEV)
        # self.tests()

    def pxToM(self, value, key='v'):
        return self.cameraInfo['ratios'][key] * value

    def getPoint3d(self, kv, kh):
        """
            input: kv, kh in [px]
            returns: x, y, z in [m] in CameraRF
        """
        L = self.n / cos(self.alpha)
        # print("L={0}".format(L))
        R = self.n * tan(self.alpha)
        r0 = self.n * tan(self.alpha - self.cameraInfo['fovs']['v'] / 2)
        r = R - r0
        l = r * cos(pi/2 - self.alpha)
        h = 2 * r * sin(pi/2 - self.alpha)
        l0 = L - l

        imDims = getDimImage(l0, self.cameraInfo['fovs'])  # dims plane of frame in [m]
        vRatio, hRatio, dRatio = getRatio(self.cameraInfo['shape'], imDims) # [m]/[px]
        self.cameraInfo['ratios'] = {'v': vRatio, 'h': hRatio, 'd': dRatio} # comfortable representation

        # from [px] to [m] coordinates of frame
        kv = self.pxToM(kv, 'v')
        kh = self.pxToM(kh, 'h')

        z = tan(pi/2 - self.alpha) * L / (tan(pi/2 - self.alpha) - kv / l0)
        x = kv * z / l0
        y = kh * z / l0
        return (x, y, z)

    def convertToCenterRF(self, (x, y)):
        objGRF = (x, y)
        objCRF = (self.CRF[0] - objGRF[0], self.CRF[1] - objGRF[1])
        return objCRF

    def tests(self):
        """ ONLY FOR TEST ON LOCAL COMPUTER WITHOUT ROS!!!"""
        t = matrix([[   cos(self.alpha), 0, sin(self.alpha), 0],
                      [ 0, -1, 0, 0],
                      [ sin(self.alpha), 0, -cos(self.alpha), self.n],
                      [ 0, 0, 0, 1]])
        print(t)
        while True:
            _, raw = self.cam.read()
            h, w, _ = raw.shape
            blank_image = np.zeros_like(raw)
            raw = cv2.blur(raw, (5  ,5))
            hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)
            lower = np.array([165, 107, 112])
            upper = np.array([211, 215, 223])
            mask = cv2.inRange(hsv, lower, upper)
            res = cv2.bitwise_and(raw, raw, mask=mask)

            v = np.median(mask)
            sigma = 0.33
            canny_low = int(max(0, (1 - sigma) * v))
            canny_high = int(min(255, (1 + sigma) * v))

            th = cv2.Canny(mask, 1, 250)
            # th = cv2.dilate(edges, None, iterations=2)
            # th = cv2.erode(edges, None, iterations=2)

            cnts, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            x = []
            y = []
            points = []
            for cnt in cnts:
                if cv2.contourArea(cnt) > 100:
                    M = cv2.moments(cnt)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    x.append(cx)
                    y.append(cy)
                    points.append([cx, cy])
                    cv2.circle(raw, (cx, cy), 2, (0, 255, 0), 2)

            points = np.array(points)

            # x = np.transpose(x)
            # y = np.transpose(y)
            # A = np.vstack([x, np.ones(len(x))]).T
            # m, c = np.linalg.lstsq(A, y)[0]

            rows, cols = raw.shape[:2]
            if len(points) > 0:
                [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_LABEL_PIXEL, 0, 0.01, 0.01)

                x_min = min(points[:,0])
                x_max = max(points[:,0])
                y_min = min(points[:,1])
                y_max = max(points[:,1])

                lefty = (int(x_min), int([y for x, y in points if x == x_min][0]))
                righty = (int(x_max), int([y for x, y in points if x == x_max][0]))

                cv2.line(raw, lefty, righty, (0, 255, 0), 2)

                # objGRF = lefty
                # objCRF = (self.CRF[0] - objGRF[0], self.CRF[1] - objGRF[1])

                objCRF = self.convertToCenterRF(lefty)
                x, y, z = self.getPoint3d(objCRF[1], objCRF[0])
                floorPoint = t * np.transpose(matrix([x, y, z, 1]))
                # print(x,y,z)
                print(np.transpose(floorPoint))
            #
            # point in a center frame
            cv2.circle(raw, (int(self.CRF[0]), int(self.CRF[1])), 5, (0, 200, 200), 2)
            # cross in a center frame
            cv2.line(raw, (0, self.CRF[1]), (self.xy0[1], self.CRF[1]), (0, 200, 200), 1)
            cv2.line(raw, (self.CRF[0], 0), (self.CRF[0], self.xy0[0]), (0, 200, 200), 1)

            cv2.imshow("raw", raw)
            # cv2.imshow("th_hsv", mask)
            # cv2.imshow('bitwise', res)
            # cv2.imshow('canny', th)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cam.release()
        cv2.destroyAllWindows()

    def getLine(self, image, tapeType='r'):
        isFind = False
        # select type of tape
        # default white/red tape
        color = [0, 0, 255]
        lower = np.array([165, 107, 112])
        upper = np.array([211, 215, 223])
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

        mask = cv2.inRange(hsv, lower, upper)
        # res = cv2.bitwise_and(raw, raw, mask=mask)

        v = np.median(mask)
        sigma = 0.33
        canny_low = int(max(0, (1 - sigma) * v))
        canny_high = int(min(255, (1 + sigma) * v))
        th = cv2.Canny(mask, 1, 250)
        # th = cv2.dilate(edges, None, iterations=2)
        # th = cv2.erode(edges, None, iterations=2)

        # find contours centres of mass
        cnts, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # make array consists all centres of mass on our image
        points = []
        for cnt in cnts:
            if cv2.contourArea(cnt) > 100:
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                points.append([cx, cy])
                cv2.circle(image, (cx, cy), 2, (0, 255, 0), 1)

        # convert that our array to numpy type
        points = np.array(points)

        # try to find leftest and rightest points
        xyzL, xyzR = (0, 0, 0), (0, 0, 0)
        if len(points) > 1:
            isFind = True
            [vx, vy, x, y] = cv2.fitLine(points, cv2.DIST_LABEL_PIXEL, 0, 0.01, 0.01)

            x_min = min(points[:, 0])
            x_max = max(points[:, 0])

            lefty = (int(x_min), int([y for x, y in points if x == x_min][0]))
            righty = (int(x_max), int([y for x, y in points if x == x_max][0]))

            cv2.line(image, lefty, righty, color, 2)

            # lefty convert RF
            objCRF = self.convertToCenterRF(lefty)
            xyzL = self.getPoint3d(objCRF[1], objCRF[0])

            # righty convert RF
            objCRF = self.convertToCenterRF(righty)
            xyzR = self.getPoint3d(objCRF[1], objCRF[0])

        # draw point in a center frame
        cv2.circle(image, (int(self.CRF[0]), int(self.CRF[1])), 5, (0, 200, 200), 2)
        # cross in a center frame
        cv2.line(image, (0, self.CRF[1]), (self.xy0[1], self.CRF[1]), (0, 200, 200), 1)
        cv2.line(image, (self.CRF[0], 0), (self.CRF[0], self.xy0[0]), (0, 200, 200), 1)

        return xyzL, xyzR, image, isFind


if __name__ == '__main__':
    # imageDim = getDimImage(l0, FOVS[0], FOVS[1], FOVS[2])
    # imageInfo['ratio'] = getRatio(imageInfo['shape'], imageDim)

    cam = cv2.VideoCapture(VIDEO_DEV)
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, RESOLUTION[0])
    cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, RESOLUTION[1])
    _, frame = cam.read()
    shape = frame.shape
    shape = (shape[0], shape[1], math.sqrt(shape[0] ** 2 + shape[1] ** 2))


    cameraInfo = dict()
    cameraInfo['shape'] = shape  # w, h, d == x, y, d
    cameraInfo['ratios'] = None  # ration [mm] / [px]
    cameraInfo['fovs'] = {'v':FOVS[0], 'h':FOVS[1], 'd':FOVS[2]}

    cam.release()

    print("Resolution [w,h,_] = {}".format(frame.shape))

    LineFinder(cameraInfo, 18*pi/180, 0.52)