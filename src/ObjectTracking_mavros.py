#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from std_msgs.msg import Int32
from std_msgs.msg import String

# ------------------camera calibration parameter-------------------------#
mtx = np.array([[1.14061619e+03, 0, 5.61516225e+02],
                [0, 1.06635618e+03, 3.83868686e+02],
                [0, 0, 1]])
dist = np.array([1.48226651e-01, -5.20981369e-01, 8.13119113e-03, 2.99522792e-02, 2.66890015])
# _______________________________________________________________________#

# ------------------Parameter HSV---------------------------------------#
max_value = 255
max_value_H = 360 // 2
low_H = 7
low_S = 91
low_V = 120
high_H = 110
high_S = 250
high_V = 255
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

# ------------------Parameter Pixell for 640x480----------------#
centerBuffer = 30
x_center = 400
y_center = 300
leftbound = int(x_center - centerBuffer)
rightbound = int(x_center + centerBuffer)
upperbound = int(y_center - centerBuffer)
lowerbound = int(y_center + centerBuffer)

# ______________________PID parameter___________________________#
kp = 0.5
ki = 0.1
kd = 0
x_prev_error = 0
x_sum_error = 0
y_prev_error = 0
y_sum_error = 0
delta_x = 0
delta_y = 0
ki_sampletime = 0.0001  # 0.1ms sampling
kd_sampletime = 0.0001  # 0.1ms sampling

##########################INITIAL SETUP############


pilih1 = None


###### INISIASI SPEEED ###############

# netral value
netralspeed = 1500
# default value for forward
defaultspeed = 1600

# Medium speed
naikmedium = 1600
turunmedium = 1400
majumedium = 1600
mundurmedium = 1400
kirimedium = 1600
kananmedium = 1400
mkirimedium = 1600
mkananmedium = 1400

#################INISIASI CHANNEL##############
fwbw_channel = 4  # forward/backward
lateral = 5  # pitch movement
throttle_channel = 2  # throttle up/down movement
yaw_channel = 3  # yaw movement
# ga dipake
useless1 = 0
useless2 = 1
useless3 = 6
useless4 = 7


####____________________Initial GUI OpenCV_________________________#########

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H - 1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)


def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H + 1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)


def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S - 1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)


def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S + 1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)


def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V - 1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)


def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V + 1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)


def mapObjectPosition(x, y, r):
    print("[INFO] Object Center coordenates at X0 = {0} and Y0 =  {1} and radius {2}".format(x, y, r))

cv2.namedWindow(window_detection_name)
cv2.createTrackbar(low_H_name, window_detection_name, low_H, max_value_H, on_low_H_thresh_trackbar)
cv2.createTrackbar(high_H_name, window_detection_name, high_H, max_value_H, on_high_H_thresh_trackbar)
cv2.createTrackbar(low_S_name, window_detection_name, low_S, max_value, on_low_S_thresh_trackbar)
cv2.createTrackbar(high_S_name, window_detection_name, high_S, max_value, on_high_S_thresh_trackbar)
cv2.createTrackbar(low_V_name, window_detection_name, low_V, max_value, on_low_V_thresh_trackbar)
cv2.createTrackbar(high_V_name, window_detection_name, high_V, max_value, on_high_V_thresh_trackbar)
font = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture(
    "udpsrc port=5001 ! application/x-rtp,media=video,payload=26,clock-rate=90000,encoding-name=JPEG,framerate=30/1 ! rtpjpegdepay ! jpegdec ! videoconvert ! appsink",
    cv2.CAP_GSTREAMER)

# cam = cv2.VideoCapture(0)

##### initial data subcriber####

def callback1(data1):
    global pilih1
    pilih1 = data1.data




#########################################################



def mode():
    # Set Mode mavros
    print("SET MODE")
    rospy.wait_for_service('/mavros/set_mode')
    try:
        modeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        modeResponse = modeService(0, 'ALT_HOLD')
        rospy.loginfo(modeResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def arm():
    print("ARM")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armResponse = armService(True)
        rospy.loginfo(armResponse)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def disarm():
    print("DISARM")
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def motionsearching():
    print("Searching object motion")
    #################___start_________Motion_______________#####################
    # puter dikit
    msg.channels[throttle_channel] = netralspeed
    msg.channels[yaw_channel] = kananmedium
    msg.channels[fwbw_channel] = netralspeed
    msg.channels[lateral] = netralspeed
    ### ga di ubah yg d bawah
    msg.channels[useless1] = netralspeed
    msg.channels[useless2] = netralspeed
    msg.channels[useless3] = netralspeed
    msg.channels[useless4] = netralspeed
    rospy.loginfo(msg)
    pub.publish(msg)
    r.sleep()

###### INITIAL ROS#####

### publisher#########
print("Bismillah")
print("Starting Publisher..")
rospy.init_node('tech_sas_node', anonymous=True)
pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=10)
r = rospy.Rate(20)  # 20hz clock
msg = OverrideRCIn()
#####_________________________Subcriber________________##########
print("Starting Subcriber..")
rospy.Subscriber("button_start", String, callback1)
rospy.sleep(3)  # delay 3s to get ready
mode()
arm()
try:
    while not rospy.is_shutdown():  # looping
        ret, frame = cap.read()

        if frame is None:
            break
        h, w = frame.shape[:2]
        newCameraMtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
        ### tambah undistor
        dst = cv2.undistort(frame, mtx, dist, None, newCameraMtx)
        ### Undistord
        # blur = cv2.GaussianBlur(dst, (5, 5), 0) #biar smooth

        frame_HSV = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)  # konversi
        frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
        frame_threshold = cv2.erode(frame_threshold, None, iterations=2)
        frame_threshold = cv2.dilate(frame_threshold, None, iterations=2)
        output = cv2.bitwise_and(frame, frame, mask=frame_threshold)
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 3, 500, minRadius=10, maxRadius=200, param1=100, param2=60)
        # deteksigaris
        # lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, maxLineGap=50)
        # contur
        # contours, hierarchy = cv2.findContours(frame_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        xpixel = 0
        ypixel = 0
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, radius) in circles:

                cv2.circle(output, (x, y), radius, (0, 255, 0), 4)
                # mapObjectPosition(int(x), int(y), int(radius))
                if radius > 10:
                    xpixel = x
                    ypixel = y
                else:
                    xpixel = 0
                    ypixel = 0

        # No object detection

        if xpixel == 0 and ypixel == 0:
            print("no Object")
            errorx = 0
            errory = 0
            if pilih1 == 'start':
                motionsearching()
            else:
                print("Stanby!")
        elif (xpixel < leftbound) or (xpixel > rightbound) or (ypixel < upperbound) or (ypixel > lowerbound):
            # PID control for x and y
            errorx = x_center - xpixel
            errory = y_center - ypixel
            x_proportional = (errorx * kp)
            y_proportional = (errory * kp)
            x_out_value = int((netralspeed + x_proportional)  + (x_sum_error * ki) + (delta_x * kd))
            y_out_value = int((netralspeed + y_proportional) + (y_sum_error * ki) + (delta_y * kd) )
            time.sleep(ki_sampletime)
            x_sum_error += errorx
            y_sum_error += errory
            time.sleep(kd_sampletime)
            delta_x = errorx - x_prev_error
            delta_y = errory - y_prev_error
            x_prev_error = errorx
            y_prev_error = errory
            # print("x out="+str(x_out_value)+" y out="+str(y_out_value))
            if pilih1 == 'start':
                print("GO!")

                #################### Output ROS ###################
                msg.channels[throttle_channel] = y_out_value  # nilai PID y
                msg.channels[yaw_channel] = x_out_value  # nilai PID x
                msg.channels[fwbw_channel] = netralspeed
                msg.channels[lateral] = netralspeed
                msg.channels[useless1] = netralspeed
                msg.channels[useless2] = netralspeed
                msg.channels[useless3] = netralspeed
                msg.channels[useless4] = netralspeed
                rospy.loginfo(msg)
                pub.publish(msg)
                r.sleep()
                ###################### end of trial ###################
            else:
                print("Object Detected!!! Stanby!")
        else:
            if pilih1 == 'start':
                if (radius < 150):
                    print("Go! Get it !")
                    #################### Output ROS ###################
                    msg.channels[throttle_channel] = netralspeed
                    msg.channels[yaw_channel] = netralspeed
                    msg.channels[fwbw_channel] = defaultspeed
                    msg.channels[lateral] = netralspeed
                    msg.channels[useless1] = netralspeed
                    msg.channels[useless2] = netralspeed
                    msg.channels[useless3] = netralspeed
                    msg.channels[useless4] = netralspeed
                    rospy.loginfo(msg)
                    pub.publish(msg)
                    r.sleep()
                    ###################### end of trial ###################
                else:
                    print("Mission Done!")
                    disarm()
            else:
                print("Object Detected!!! Stanby!!")


        # frame_r = cv2.resize(frame, (640,480))
        cv2.imshow('Output', output)
        # frame_undistor = cv2.resize(dst, (640,480))
        cv2.imshow('undistortion', dst)
        # detection = cv2.resize(frame_threshold, (640,480))
        cv2.imshow(window_detection_name, frame_threshold)

        key = cv2.waitKey(1)
        if KeyboardInterrupt == True:
            break
except rospy.ROSInterruptException:
    pass
