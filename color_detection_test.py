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

rospy.init_node('cv2', disable_signals=True)

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


# color_points_cloud = []


def img_xy_to_point(xy, dist):
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]

    # Shift points to center
    xy -= camera_info.width // 2, camera_info.height // 2

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)


def get_center_of_mass(mask):
    M = cv2.moments(mask)
    if M['m00'] == 0:
        return None
    return M['m10'] // M['m00'], M['m01'] // M['m00']


def set_led_color(color_name):
    effect = 'fill'
    if color_name == 'red':
        set_effect(effect=effect, r=255, g=0, b=0)  # red
    elif color_name == 'orange':
        set_effect(effect=effect, r=255, g=128, b=0)  # orange
    elif color_name == 'yellow':
        set_effect(effect=effect, r=255, g=255, b=0)  # yellow
    elif color_name == 'green':
        set_effect(effect=effect, r=0, g=255, b=0)  # green
    elif color_name == 'cyan':
        set_effect(effect=effect, r=0, g=128, b=255)  # cyan
    elif color_name == 'blue':
        set_effect(effect=effect, r=0, g=0, b=255)  # blue
    elif color_name == 'magenta':
        set_effect(effect=effect, r=255, g=0, b=255)  # 'magenta'
    elif color_name == 'none':
        set_effect(effect=effect, r=255, g=255, b=255)  # 'white'
    else:
        set_effect(effect=effect, r=255, g=0, b=0)  # red


def find_centers(img_hsv, msg, mins=110, minv=120, maxs=245, maxv=255):  # mins=90, minv=120, maxs=235, maxv=235
    mask_red1 = cv2.inRange(img_hsv, (0, mins, minv), (8, maxs, maxv))
    mask_red2 = cv2.inRange(img_hsv, (160, mins, minv), (180, maxs, maxv))

    mask_red = (cv2.bitwise_or(mask_red1, mask_red2), 'red')
    mask_yellow = (cv2.inRange(img_hsv, (12, mins, minv), (50, maxs, maxv)), 'yellow')
    mask_blue = (cv2.inRange(img_hsv, (70, 50, 130), (150, 230, 250)), 'blue')

    # if mask_pub.get_num_connections() > 0:
    #     mask_pub.publish(bridge.cv2_to_imgmsg(mask_yellow[0], 'mono8'))

    coord_pub_list = []
    led_flag = False

    for mask, color in (mask_red, mask_blue, mask_yellow, ):  # (mask_red, mask_yellow, mask_blue)

        if mask_pub.get_num_connections() > 0:
            mask_pub.publish(bridge.cv2_to_imgmsg(mask, 'mono8'))

        # position relative to the image
        xy = get_center_of_mass(mask)
        if xy is None:
            return

        telem = get_telemetry()

        # position relative to the drone
        xy3d = img_xy_to_point(xy, telem.z)

        # position relative to the aruco_map
        target = PointStamped(header=msg.header, point=xy3d)
        setpoint = tf_buffer.transform(target, 'map', timeout=rospy.Duration(0.2))

        coord_pub_list.append([f'{color} {setpoint.point.x} {setpoint.point.y}'])

        # color_points_cloud.append((round(setpoint.point.x, 3), round(setpoint.point.y, 3), color))

        if ((telem.x - setpoint.point.x) ** 2 + (telem.y - setpoint.point.y) ** 2) ** 0.5 <= 0.25:
            set_led_color(color_name=color)
            led_flag = True

    if not led_flag:
        set_led_color(color_name='none')

    for i in coord_pub_list:
        point_pub.publish(i)


def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img = cv2.erode(img, kernel=(2, 2, 3), iterations=10)
    img = cv2.dilate(img, kernel=(2, 2, 3), iterations=10)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    find_centers(img_hsv, msg)


image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
rospy.spin()

# find_centers(img_hsv)

cv2.destroyAllWindows()
