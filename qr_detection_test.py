import rospy
import cv2
import numpy as np
from math import nan
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge
from clover import srv
import tf2_ros
import tf2_geometry_msgs

rospy.init_node('qr', disable_signals=True)

# img = cv2.imread('data/cal.jpg')
# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

mask_pub = rospy.Publisher('~mask', Image, queue_size=1)
point_pub = rospy.Publisher('~debug_cords', String, queue_size=1)

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
bridge = CvBridge()

camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

qcd = cv2.QRCodeDetector()
qr_points_cloud = []


def img_xy_to_point(xy, dist):
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]

    # Shift points to center
    xy -= camera_info.width // 2, camera_info.height // 2

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)


def find_qr_center_on_image(points: (list, tuple)):
    x = [float(i[0]) for i in points]
    y = [float(i[1]) for i in points]
    precision = 5
    k1 = round((x[0] - x[2]) / (y[0] - y[2]), precision)
    k2 = round((x[3] - x[1]) / (y[3] - y[1]), precision)
    m1 = round(x[0] - k1 * y[0], precision)
    m2 = round(x[3] - k2 * y[3], precision)
    y_ = round((m2 - m1) / (k1 - k2), 1)
    x_ = round(k1 * y_ + m1, 1)
    return x_, y_


def find_centers(adress, points, msg):
    # position relative to the image
    xy = find_qr_center_on_image(points)
    if xy is None:
        return

    telem = get_telemetry()

    # position relative to the drone
    xy3d = img_xy_to_point(xy, telem.z)

    # position relative to the aruco_map
    target = PointStamped(header=msg.header, point=xy3d)
    setpoint = tf_buffer.transform(target, 'map', timeout=rospy.Duration(0.2))

    qr_points_cloud.append((round(setpoint.point.x, 3), round(setpoint.point.y, 3)))


adress = '0'


def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    ret_qr, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(img)
    if ret_qr and len(decoded_info) == 1 and decoded_info[0] == adress:
        find_centers(adress, points[0], msg)


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
adress = input()

rospy.spin()

# find_centers(img_hsv)

cv2.destroyAllWindows()
