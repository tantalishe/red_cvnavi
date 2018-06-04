import numpy as np
import cv2
import math
import time

BLUR = 5
THRESHOLD_KERNEL = 7 # 11
THRESHOLD_PARAMETER = 4 # 3
DILATE_ITER = 1
ERODE_ITER = 1
FPS_VISUALISATION = False
FPS_VISUALISATION_DELAY = 3 # [sec]

def nothing(x):
    pass

def getLine(image, tapeType='r'):
        # isFind = False
        # select type of tape
        # default white/red tape
        color = [0, 0, 255]
        lower = np.array([165, 107, 112]) #706BA5
        upper = np.array([211, 215, 223]) #D3D7DF
        # lower = np.array([165, 0, 0]) #706BA5
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

        mask = cv2.inRange(hsv, lower, upper)
        # res = cv2.bitwise_and(raw, raw, mask=mask)
        return mask
        # return hsv

def getHSV(image,hsv):
    image[:,:] = hsv
    # image[:,:, 0] = hsv[0]
    # image[:,:, 1] = hsv[1]
    # image[:,:, 2] = hsv[2]
    rgb_color = cv2.cvtColor(image,cv2.COLOR_HSV2BGR)
    return rgb_color


def centroid(contour):
    M = cv2.moments(contour)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    return cx, cy

start_time = time.time()
frame_counter = 0
cam = cv2.VideoCapture(1)
number = 1

cv2.namedWindow('Bars')
cv2.createTrackbar('hue', 'Bars', 180, 255, nothing)
cv2.createTrackbar('saturation', 'Bars', 215, 255, nothing)
cv2.createTrackbar('value', 'Bars', 200, 255, nothing)

while True:
    _, frame = cam.read()

    image = getLine(frame)
    # image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # reanding and filterinf image
    # image = cv2.equalizeHist(image)
    # image = cv2.medianBlur(image, BLUR)

    # thres = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,THRESHOLD_KERNEL,THRESHOLD_PARAMETER)
    # thres = cv2.bitwise_not(thres)

    # kernel1 = np.array([[0,0,0,1,1,1,0,0,0],[0,0,1,1,1,1,1,0,0],[0,1,1,1,1,1,1,1,0],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[1,1,1,1,1,1,1,1,1],[0,1,1,1,1,1,1,1,0],[0,0,1,1,1,1,1,0,0],[0,0,0,1,1,1,0,0,0]])
    # kernel2 = np.ones((5,5))

    # edged = cv2.dilate(thres, kernel2, iterations=ERODE_ITER)
    # edged = cv2.bitwise_not(edged)

    # v = np.median(image)
    # sigma = 0.33
    # canny_low = int(max(0, (1 - sigma) * v))
    # canny_high = int(min(255, (1 + sigma) * v))
    # canny = cv2.Canny(edged, canny_low, canny_high)
    # # print('l', canny_low, '   h', canny_high)

    # dilated = cv2.dilate(canny, kernel1, iterations=DILATE_ITER)
    # eroded = cv2.erode(dilated, kernel1, iterations=ERODE_ITER)

    # _, contours, hierarchy = cv2.findContours(eroded.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # for cnt in contours:
    #     area = cv2.contourArea(cnt)
    #     # print(area)
    #     if area > 500:

    #         x, y = centroid(cnt)
    #         text_pos = (x, y)
    #         cv2.drawContours(frame, cnt, -1, (0, 255, 0),2)
    #         # # cv2.putText(frame, text, text_pos,
    #         #                 cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
    #         #                 color=(0, 255, 0), thickness=2)
    # frame_counter += 1    
    # if FPS_VISUALISATION and (time.time() - start_time) > FPS_VISUALISATION_DELAY:
    #     print("FPS:", frame_counter / (time.time() - start_time))
    #     start_time = time.time()
    #     frame_counter = 0

    # color_hsv = np.uint8([[[211, 215, 223]]])
    # rgb_color = cv2.cvtColor(color_hsv,cv2.COLOR_HSV2BGR)
    # print(rgb_color)
    hsv = [cv2.getTrackbarPos('hue', 'Bars'), cv2.getTrackbarPos('saturation', 'Bars'), cv2.getTrackbarPos('value', 'Bars')]
    hsv_color = getHSV(frame.copy(), hsv)
    
    cv2.imshow("image", image)
    cv2.imshow("HSV", hsv_color)
    cv2.imshow("frame", frame)
    # cv2.imshow("eroded", eroded)
    # cv2.imshow("canny", canny)
    # cv2.imshow("thres", thres)
    # cv2.imshow("edge", edged)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cam.release()
cv2.destroyAllWindows()     