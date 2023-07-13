import rospy
from clover import srv, long_callback
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point

import tf2_ros
import tf2_geometry_msgs

import cv2
from cv_bridge import CvBridge

from sklearn.cluster import KMeans
import numpy as np
import math


class COEX:
    def __init__(self):
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
        self.set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.SetLEDEffect = rospy.ServiceProxy('SetLEDEffect', srv.SetLEDEffect)

    def navigate_wait(self, x=0, y=0, z=0, yaw=math.nan, speed=0.5, frame_id='body', tolerance=0.1, auto_arm=False):
        res = self.navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

        if not res.success:
            return res

        while not rospy.is_shutdown():
            telem = self.get_telemetry(frame_id='navigate_target')
            if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
                return res
            rospy.sleep(0.2)

    def land_wait(self):
        self.land()
        while self.get_telemetry().armed:
            rospy.sleep(0.2)


class CameraStream:
    def __init__(self):
        # self.mask_pub = rospy.Publisher('~mask', Image, queue_size=1)
        # self.point_pub = rospy.Publisher('~debug_cords', PointStamped, queue_size=1)
        self.stream_sub = rospy.Subscriber('main_camera/image_raw', Image, self.image_callback, queue_size=1)

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_effect = rospy.ServiceProxy('led/set_effect', srv.SetLEDEffect)
        self.bridge = CvBridge()

        self.camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
        self.camera_matrix = np.float64(self.camera_info.K).reshape(3, 3)
        self.distortion = np.float64(self.camera_info.D).flatten()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.color_points_cloud = []

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # convert to HSV to work with color hue
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        self.find_centers(img_hsv=img_hsv, msg=msg)

    def find_centers(self, img_hsv, msg, mins=90, minv=120, maxs=235, maxv=255):
        mask_red1 = cv2.inRange(img_hsv, (0, mins, minv), (14, maxs, maxv))
        mask_red2 = cv2.inRange(img_hsv, (160, mins, minv), (180, maxs, maxv))

        mask_red = (cv2.bitwise_or(mask_red1, mask_red2), 'red')
        mask_yellow = (cv2.inRange(img_hsv, (15, mins, minv), (50, maxs, maxv)), 'yellow')
        mask_blue = (cv2.inRange(img_hsv, (90, mins, minv), (150, maxs, maxv)), 'blue')

        led_flag = False

        for mask, color in (mask_red, mask_yellow, mask_blue):
            mask = cv2.erode(mask, kernel=(10, 10), iterations=3)
            mask = cv2.erode(mask, kernel=(5, 5), iterations=3)
            mask = cv2.dilate(mask, kernel=(2, 2), iterations=3)

            # if self.mask_pub.get_num_connections() > 0:
            #     self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))

            # position relative to the image
            xy = self.get_center_of_mass(mask)
            if xy is None:
                return

            telem = self.get_telemetry()

            # position relative to the drone
            xy3d = self.img_xy_to_point(xy, telem.z)

            # position relative to the aruco_map
            target = PointStamped(header=msg.header, point=xy3d)
            setpoint = self.tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(0.2))

            # self.point_pub.publish(target)

            self.color_points_cloud.append((round(setpoint.point.x, 3), round(setpoint.point.y, 3), color))

            if ((telem.x - setpoint.point.x) ** 2 + (telem.y - setpoint.point.y) ** 2) ** 0.5 <= 0.25:
                self.set_led_color(color_name=color)
                led_flag = True

        if not led_flag:
            self.set_led_color(color_name='none')

    def img_xy_to_point(self, xy, dist):
        xy = cv2.undistortPoints(xy, self.camera_matrix, self.distortion, P=self.camera_matrix)[0][0]

        # Shift points to center
        xy -= self.camera_info.width // 2, self.camera_info.height // 2

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)

    def get_center_of_mass(self, mask):
        M = cv2.moments(mask)
        if M['m00'] == 0:
            return None
        return M['m10'] // M['m00'], M['m01'] // M['m00']

    def set_led_color(self, color_name):
        effect = 'fill'
        if color_name == 'red':
            self.set_effect(effect=effect, r=255, g=0, b=0)  # red
        elif color_name == 'orange':
            self.set_effect(effect=effect, r=255, g=128, b=0)  # orange
        elif color_name == 'yellow':
            self.set_effect(effect=effect, r=255, g=255, b=0)  # yellow
        elif color_name == 'green':
            self.set_effect(effect=effect, r=0, g=255, b=0)  # green
        elif color_name == 'cyan':
            self.set_effect(effect=effect, r=0, g=128, b=255)  # cyan
        elif color_name == 'blue':
            self.set_effect(effect=effect, r=0, g=0, b=255)  # blue
        elif color_name == 'magenta':
            self.set_effect(effect=effect, r=255, g=0, b=255)  # 'magenta'
        elif color_name == 'none':
            self.set_effect(effect=effect, r=255, g=255, b=255)  # 'white'
        else:
            self.set_effect(effect=effect, r=255, g=0, b=0)  # red


class Points:
    def __init__(self, array_of_points: list = []):
        self.points: list = []
        self._points_reached = []
        self._all_points = []
        self.altitude = 1
        self.size = 0

        self.x: float = None
        self.y: float = None
        self.z: float = None

        self.points.extend(array_of_points)
        self.size += len(array_of_points)

        self.next()

    def update(self, additional_points_array: list) -> None:
        self.points.extend(additional_points_array)

        self.size += len(additional_points_array)

    def set(self, new_points_array: list) -> None:
        self.points = new_points_array

        self.size = len(new_points_array)

    def update_altitude(self, new_alt) -> None:
        self.altitude = new_alt
        if self.points:
            for ind, obj in enumerate(self.points):
                self.points[ind][2] = self.altitude

    def next(self) -> None:
        if self.points:
            point = self.points.pop(0)
            self.x, self.y, self.z = point
            self._points_reached.append(point)
        else:
            pass

    def make_snake(self, len_x, len_y, z=1, step_x=0.25, step_y=1):
        if not self.points:
            len_x = int(len_x // step_x - 1 / step_x + 1)
        len_y = int(len_y // step_y - 1 / step_y + 1)
        flag = True
        for x in range(len_x):
            if flag:
                for y in range(len_y):
                    self.points.append([round(x * step_x, 3), round(y * step_y, 3), z])
                    self.size += 1
                flag = not flag
            else:
                for y in range(len_y - 1, -1, -1):
                    self.points.append([round(x * step_x, 3), round(y * step_y, 3), z])
                    self.size += 1
                flag = not flag


# [x, y, {color_name}]
def get_color_centers(color_arr):
    for_fit = [[i[0], i[1]] for i in color_arr]
    kmeans = KMeans(n_clusters=3)
    kmeans.fit(for_fit)
    centers = kmeans.cluster_centers_
    res = {}
    for center in centers:
        for color_point in color_arr:
            if ((color_point[0] - center[0]) ** 2 + (color_point[1] - center[1]) ** 2) ** 0.5 <= 0.2:
                res[f"{center[0], center[1]}"] = color_point[2]
    return res


def main():
    rospy.init_node('flight', disable_signals=True)
    coex = COEX()
    cam = CameraStream()

    altitude = 1
    velocity = 0.3

    points = Points()
    points.make_snake(2, 3)
    points.update_altitude(altitude)
    for i in points.points:
        print(i)

    points.next()

    print(f'Moving up  {points.x}, {points.y}, {points.z}')
    coex.navigate_wait(x=0, y=0, z=altitude, frame_id='body', auto_arm=True)

    for point in range(points.size):
        print(f'Moving to {points.x}, {points.y}, {points.z}')
        coex.navigate_wait(x=points.x, y=points.y, z=points.z, speed=velocity, frame_id='aruco_map')
        points.next()

    print("Moving to zero point...")
    coex.navigate_wait(x=0, y=0, z=altitude, speed=velocity, frame_id='aruco_map')

    print("Landing...")
    coex.land_wait()

    color_arr = cam.color_points_cloud
    color_clustered = get_color_centers(color_arr)
    for i in color_clustered.items():
        print(f'{i[1]} in {i[0]}')


if __name__ == '__main__':
    main()
