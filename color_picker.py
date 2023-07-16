from skimage import io
import cv2
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge
import numpy as np


def nothing(x):
    pass


cv2.namedWindow('Trackbars')

cv2.createTrackbar('H', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('S', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('V', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('mH', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('mS', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('mV', 'Trackbars', 0, 255, nothing)

cv2.setTrackbarPos('mH', 'Trackbars', 180)
cv2.setTrackbarPos('mS', 'Trackbars', 255)
cv2.setTrackbarPos('mV', 'Trackbars', 255)

# url = input()
# frame = io.imread(url)
bridge = CvBridge()

raw_image = np.zeros((100, 100, 3), dtype=np.uint8)


def image_callback(msg):
    global raw_image
    raw_image = bridge.imgmsg_to_cv2(msg, 'bgr8')


rospy.init_node("hi")
sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)


# frame = cv2.imread('data/snapshot(2).jpeg')
# mask = cv2.erode(mask, kernel=(10, 10), iterations=3)
# frame = cv2.erode(frame, kernel=(2, 2, 3), iterations=10)
# frame = cv2.dilate(frame, kernel=(2, 2, 3), iterations=10)
rainbow = cv2.imread('data/rainbow2.jpg')

while True:
    hsv = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)
    rainbow_hsv = cv2.cvtColor(rainbow, cv2.COLOR_BGR2HSV)

    H = cv2.getTrackbarPos('H', 'Trackbars')
    S = cv2.getTrackbarPos('S', 'Trackbars')
    V = cv2.getTrackbarPos('V', 'Trackbars')
    mH = cv2.getTrackbarPos('mH', 'Trackbars')
    mS = cv2.getTrackbarPos('mS', 'Trackbars')
    mV = cv2.getTrackbarPos('mV', 'Trackbars')

    mask = cv2.inRange(hsv, (H, S, V), (mH, mS, mV))
    rainbow_mask = cv2.inRange(rainbow_hsv, (H, S, V), (mH, mS, mV))

    # show = cv2.bitwise_and(raw_image, raw_image, mask=mask)
    cv2.imshow('Trackbars', mask)

    rainbow_show = cv2.bitwise_and(rainbow, rainbow, mask=rainbow_mask)
    cv2.imshow('Rainbow', rainbow_show)

    if cv2.waitKey(1) == ord('q'):
        break


