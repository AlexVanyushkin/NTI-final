import rospy
import math

from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import srv
from std_srvs.srv import Trigger



from math import sqrt

import csv
from rosgraph_msgs.msg import Clock

from mavros_msgs.srv import CommandBool

import cv2 as cv
import numpy as np
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped

from clover.srv import SetLEDEffect

import tf2_ros
import tf2_geometry_msgs

from aruco_pose.msg import MarkerArray


import cv2

from PIL import ImageDraw

rospy.init_node('fly_with_QR')

#Proxys
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
fly_to = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)


tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
bridge = CvBridge()


#!QR_data of 1 QR code!
#Change for your cordes of 1st QR

popit = 10

file = open('Report.txt', 'w')
file.close()

def save_telem():
	file = open('Report.txt', 'a+')
	telemetry = get_telemetry()
	file.write(str(telemetry.x) + ' ' + str(telemetry.y) + '\n')
	file.close()

#Read QR code 1 times
def check_for_QR():
    global QR_data
    flag_find_QR = False
    cv_image = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    barcodes = pyzbar.decode(cv_image)
    for barcode in barcodes:
        cont()
        rospy.sleep(1)
        cont()
        rospy.sleep(1)
        cont()
        rospy.sleep(1)
        QR_data = barcode.data.decode("utf-8")
        flag_find_QR = True
    return flag_find_QR

def check_read(xc, yc):
    flag_of_result = False
    fly_and_wait(x=xc, y=yc, z=1, speed=1.0, frame_id='aruco_map')
    rospy.sleep(0.5)
    save_telem()
    rospy.sleep(0.5)
    for height in heights:
        fly_and_wait(x=xc, y=yc, z=height, speed=1.0, frame_id='aruco_map')
        rospy.sleep(0.5)
        save_telem()
        rospy.sleep(2)
        for i in range(popit):
            flag_find_QR = check_for_QR()
            if flag_find_QR == True:
                flag_of_result = True
                break
        if flag_of_result == True:
            break
    return flag_of_result

#correct fly
def fly_and_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    fly_to(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    save_telem()
    while not rospy.is_shutdown():
        save_telem()
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

#Correct land
def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)


#Max height
z_decline = 1.1
#Minimum height
z_minimum = 0.3
#Number of attempts to read the QR code
attempts = 7
#Change height for a new attempt
z_attempts = (z_decline - z_minimum) / attempts

#Creating a list of height values for reading QR codes
heights = []
for i in range(attempts + 1):
    new_height = z_decline - i * z_attempts
    heights.append(new_height)

image_pub = rospy.Publisher('Detect', Image)

def cont():
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    colors = {'black': [0, 0, 0, 180, 255, 50, [255, 105, 180]]}
    colors_name = ['black']

    for i in range(len(colors_name)):
        hsv_color = colors[colors_name[i]]
        hsv_min = np.array((hsv_color[0], hsv_color[1], hsv_color[2]), np.uint8)
        hsv_max = np.array((hsv_color[3], hsv_color[4], hsv_color[5]), np.uint8)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        obrab_hsv = cv2.inRange(hsv, hsv_min, hsv_max)
        none, conturs, none2 = cv.findContours(obrab_hsv.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        
        for cn in conturs:
            rect = cv.minAreaRect(cn)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 25:
                cv.drawContours(frame, [box], 0, (hsv_color[6][0], hsv_color[6][1], hsv_color[6][2]), 2)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

#Up on body
fly_and_wait(x=0, y=0, z=1, speed=1.0, frame_id='body', auto_arm=True)
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)

#Binding to aruco_map
fly_and_wait(x=0, y=0, z=1.3, yaw=0, speed=1.0, frame_id='aruco_map')
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
rospy.sleep(0.5)
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)


def decode_data(s):
    v = list(s)
    v1 = list(s)
    for j in v:
        num = True
        try:
            c = int(j)
        except ValueError:
            num = False
        if not num and (j != '.'):
            v1.remove(j)
    s = ''
    for i in range(len(v1)):
        s += v1[i]
    return s

# Process of reading 3 QR codes and flying to points
#Flying to the QR code
flag_of_result = check_read(0.4, 0.8)
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
if flag_of_result == False:
    print('Didnt find any QR')
else:
    #Info of decoded QR
    info_qr = QR_data
    dd = list(map(str, info_qr.split('\n')))

    Column_area = dd[0]
    Navigation_area = dd[1]
    Order_number = dd[2]

    Column_area = list(map(str, Column_area.split(' ')))
    Navigation_area = list(map(str, Navigation_area.split(' ')))
    Order_number = list(map(str, Order_number.split(' ')))

    Column_area_d = []
    for i in Column_area:
        if i != '':
            Column_area_d.append(decode_data(i))

    Navigation_area_d = []
    for i in Navigation_area:
        if i != '':
            Navigation_area_d.append(decode_data(i))

    Order_number_d = []
    for i in Order_number:
        if i != '':
            Order_number_d.append(decode_data(i))

    fff = 0
    for i in range(len(Column_area_d) / 2):
        print('Column area x=' + str(Column_area_d[fff]) + ', y=' + str(Column_area_d[fff + 1]))
        fff += 2

    print('Navigation area x=' + str(Navigation_area_d[0]) + ', y=' + str(Navigation_area_d[1]))

    print('Order number: ' + str(Order_number_d[0]))
    Column_area_d.append('0')
    Column_area_d.append('0')
    Column_area_d.append('0')
    Column_area_d.append('0')
    Column_area_d.append('0')
    Column_area_d.append('0')
    Column_area_d.append('0')

def equation(x1, y1, x2, y2, X1=0, Y1=0, X2=0, Y2=0, X3=0, Y3=0,):
    
    k = (y2 - y1) / (x2 - x1)
    b = y1 - (k * x1)
    
    if (Y1 >= k * X1 + b - 0.7) and (Y1 <= k * X1 + b + 0.7):
        return True
    if (Y2 >= k * X2 + b - 0.7) and (Y2 <= k * X2 + b + 0.7):
        return True
    if (Y3 >= k * X3 + b - 0.7) and (Y3 <= k * X3 + b + 0.7):
        return True
    else:
        return False


fly_and_wait(x=0, y=0, z=1.3, speed=1.0, frame_id='aruco_map')
xxx = 0
yyy = 0
let = 0
while let != 1:
    kk = equation(xxx, yyy, float(Navigation_area_d[0]), float(Navigation_area_d[1]), float(Column_area_d[0]), float(Column_area_d[1]), float(Column_area_d[2]), float(Column_area_d[3]), float(Column_area_d[4]), float(Column_area_d[5]))
    print(kk)
    if kk == False:
        let = 1
    else:
        if xxx >= 4:
            let = 1
        else:
            xxx += 0.3
            rospy.sleep(0.5)
            save_telem()
            rospy.sleep(0.5)
            fly_and_wait(x=xxx, y=0, z=1.6, yaw=0, speed=1.0, frame_id='aruco_map')
            rospy.sleep(0.5)
            save_telem()
            rospy.sleep(0.5)

fly_and_wait(x=float(Navigation_area_d[0]), y=float(Navigation_area_d[1]), yaw=0, z=1.6, speed=1.0, frame_id='aruco_map')
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.15):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

image_pub = rospy.Publisher('Detect', Image)
image_pub_black = rospy.Publisher('black', Image)
image_pub_blue = rospy.Publisher('blue', Image)
image_pub_yellow = rospy.Publisher('yellow', Image)
image_pub_red = rospy.Publisher('red', Image)


def obvod():
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')

    colors = {'blue': [110, 55, 68, 125, 125, 100, [0, 0, 255]], 'yellow': [25, 110, 110, 35, 170, 170, [255, 255, 0]], 'red': [170, 120, 120, 179, 200, 170, [255, 0, 0]], 'red1': [1, 120, 120, 10, 200, 170, [255, 0, 0]], 'black': [0, 0, 0, 180, 255, 55, [255, 105, 180]]}
    colors_name = ['blue', 'yellow', 'red', 'red1', 'black']

    for i in range(len(colors_name)):
        hsv_color = colors[colors_name[i]]
        hsv_min = np.array((hsv_color[0], hsv_color[1], hsv_color[2]), np.uint8)
        hsv_max = np.array((hsv_color[3], hsv_color[4], hsv_color[5]), np.uint8)

        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        obrab_hsv = cv2.inRange(hsv, hsv_min, hsv_max)
        if colors_name[i] == 'black':
            image_pub_black.publish(bridge.cv2_to_imgmsg(obrab_hsv, 'mono8'))
        if colors_name[i] == 'blue':
            image_pub_blue.publish(bridge.cv2_to_imgmsg(obrab_hsv, 'mono8'))
        if colors_name[i] == 'yellow':
            image_pub_yellow.publish(bridge.cv2_to_imgmsg(obrab_hsv, 'mono8'))
        if colors_name[i] == 'red' or colors_name[i] == 'red1':
            image_pub_red.publish(bridge.cv2_to_imgmsg(obrab_hsv, 'mono8'))
        none, conturs, none2 = cv.findContours(obrab_hsv.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for cn in conturs:
            rect = cv.minAreaRect(cn)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 30:
                cv.drawContours(frame, [box], 0, (hsv_color[6][0], hsv_color[6][1], hsv_color[6][2]), 2)
                set_effect(r=hsv_color[6][0], g=hsv_color[6][1], b=hsv_color[6][2])
                print(colors_name[i])
                return hsv_min, hsv_max
        return np.array((0, 0, 0), np.uint8), np.array((0, 0, 0), np.uint8)
    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

def getT(original):
    hsv_min = np.array((0, 0, 0), np.uint8)
    hsv_max = np.array((0, 0, 0), np.uint8)
    hsv_min, hsv_max = obvod()
    hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
    threshold = cv2.inRange(hsv, hsv_min, hsv_max)
    thresh_new = cv.GaussianBlur(threshold, (5, 5), 2)
    img = np.zeros((512,512,3), np.uint8)
    height, width = thresh_new.shape
    upper_left = (width // 4, height // 4)
    bottom_right = (width * 3 // 4, height * 3 // 4)
    rect_img = thresh_new[upper_left[1]: bottom_right[1] + 1, upper_left[0]: bottom_right[0] + 1]
    image_pu.publish(bridge.cv2_to_imgmsg(rect_img, 'mono8'))
    _, contours, hierarchy = cv.findContours( rect_img.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    #cv.drawContours( img, contours, -1, (255,0,0), 3, cv.LINE_AA, hierarchy, 1 )
    for cnt in contours:
        rect = cv.minAreaRect(cnt)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        if sqrt((box[0][0] - box[2][0])**2 + (box[0][1] - box[2][1])**2) > 50:
            
            min_x = box[:, 0].min()
            max_x = box[:, 0].max()
            min_y = box[:, 1].min()
            max_y = box[:, 1].max()

            new_min_y = min_y-20 if min_y-20 >= 0 else 0
            new_max_y = max_y+20 if max_y+20 >= 0 else 0
            new_min_x = min_x-20 if min_x-20 >= 0 else 0
            new_max_x = max_x+20 if max_x+20 >= 0 else 0

            thresh_new = rect_img[new_min_y:new_max_y, new_min_x:new_max_x]

            return thresh_new
def main():
    original = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    gray = getT(original)
    try:
        if gray.shape[1] > gray.shape[0]:
            n = 1
        else:
            n = 0
        imgg = np.zeros((gray.shape[0],gray.shape[1],3), np.uint8)
        imgg[:] = (255,255,255)
        _, contours, hierarchy = cv.findContours( gray.copy(), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours( imgg, contours, -1, (0,0,0), 3, cv.LINE_AA, hierarchy, 1 )


        img = imgg
        #img = img[:, 10:img.shape[1]-10]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        minLineLength = 10 #img.shape[1] - 300
        lines = cv2.HoughLinesP(image=edges, rho=0.02, theta=np.pi / 4,     threshold=10, lines=np.array([]), minLineLength=minLineLength, maxLineGap=2)
        a, b, c = lines.shape
        imgg = np.zeros((img.shape[0],img.shape[1],3), np.uint8)
        imgg[:] = (255,255,255)
        for i in range(a):
            cv2.line(imgg, (lines[i][0][0], lines[i][0][1]), (lines[i][0][2],  lines[i][0][3]), (0, 0, 0), 2, cv2.LINE_AA)
        im = imgg
        if n == 0:
            M = im.shape[0]//2
            N = im.shape[1]//1
        else:
            M = im.shape[0]//1
            N = im.shape[1]//2
        tiles = [im[x:x+M,y:y+N] for x in range(0,im.shape[0],M) for y in range(0,im.shape[1],N)]
        p = 0
        p1 = 0
        vis2 = cv2.cvtColor(tiles[1], cv2.COLOR_BGR2GRAY)
        vis3 = cv2.cvtColor(tiles[0], cv2.COLOR_BGR2GRAY)
        for row in vis2:
            for col in row:
                p += col
        for row in vis3:
            for col in row:
                p1 += col
        if n == 0:
            if p < p1:
                print('Sector A required')
            else:
                print('Sector B required')
        else:
            if p < p1:
                print('Sector D required')
            else:
                print('Sector C required')
    except Exception as e:
        print('Sector C required')

image_pub = rospy.Publisher('color_debug', Image)
image_pu = rospy.Publisher('detect', Image)

navigate_wait(x=float(Navigation_area_d[0]), y=float(Navigation_area_d[1]), yaw=0, z=1.6, speed=1.0, frame_id='aruco_map')
rospy.sleep(0.5)
cont()
rospy.sleep(0.5)

navigate_wait(x=float(Navigation_area_d[0]), y=float(Navigation_area_d[1]), yaw=0, z=0.5, speed=1.0, frame_id='aruco_map')
rospy.sleep(3)
main()

rospy.sleep(0.5)
cont()
rospy.sleep(0.5)
cont()
save_telem()
rospy.sleep(2)
cont()
rospy.sleep(0.5)
cont()
rospy.sleep(0.5)
cont()
rospy.sleep(0.5)



########################################

land_wait()
rospy.sleep(5)

rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
navigate_wait(x=0, y=0, z=1.0, speed=1.0, frame_id='body', auto_arm=True)
print('up')
rospy.sleep(1.0)

rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
navigate_wait(x=3.6, y=0, z=1.0, speed=1.0, frame_id='aruco_map')
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
navigate_wait(x=0, y=0, z=1.0, speed=1.0, frame_id='aruco_map')
rospy.sleep(0.5)
save_telem()
rospy.sleep(0.5)
rospy.sleep(1)
land_wait()
