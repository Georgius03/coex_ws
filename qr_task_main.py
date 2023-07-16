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

        self.qcd = cv2.QRCodeDetector()

        self.drone_find_qr_with_adress = False
        self.adress = '0'
        self.interest_points = []

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        ret_qr, decoded_info, points, straight_qrcode = self.qcd.detectAndDecodeMulti(img)
        if ret_qr and len(decoded_info) == 1 and decoded_info[0] == self.adress:
            self.find_centers(self.adress, points[0], msg)

    def update_adress(self, new_adress='0'):
        self.adress = new_adress

    def find_centers(self, adress, points, msg):
        # position relative to the image
        xy = self.find_qr_center_on_image(points)
        if xy is None:
            return

        telem = self.get_telemetry()

        # position relative to the drone
        xy3d = self.img_xy_to_point(xy, telem.z)

        # position relative to the aruco_map
        target = PointStamped(header=msg.header, point=xy3d)
        setpoint = self.tf_buffer.transform(target, 'map', timeout=rospy.Duration(0.2))

        self.interest_points.append([round(setpoint.point.x, 3), round(setpoint.point.y, 3)])
        self.drone_find_qr_with_adress = True
        print(round(setpoint.point.x, 3), round(setpoint.point.y, 3))

    def img_xy_to_point(self, xy, dist):
        xy = cv2.undistortPoints(xy, self.camera_matrix, self.distortion, P=self.camera_matrix)[0][0]

        # Shift points to center
        xy -= self.camera_info.width // 2, self.camera_info.height // 2

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]

        return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)

    def find_qr_center_on_image(self, points: (list, tuple)):
        x = [float(i[0]) for i in points]
        y = [float(i[1]) for i in points]
        precision = 5
        k1 = round((x[0] - x[2]) / (y[0] - y[2]), precision)
        k2 = round((x[3] - x[1]) / (y[3] - y[1]), precision)
        m1 = round(x[0] - k1 * y[0], precision)
        m2 = round(x[3] - k2 * y[3], precision)
        y_ = round((m2 - m1) / (k1 - k2), 2)
        x_ = round(k1 * y_ + m1, 2)
        return x_, y_


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

def calculate_center(arr):
    x, y = 0, 0
    s = len(arr)
    for xx, yy in arr:
        x += xx
        y += yy
    x /= s
    y /= s
    return x, y

def main():
    rospy.init_node('flight_qr', disable_signals=True)
    coex = COEX()
    cam = CameraStream()

    altitude = 1
    velocity = 0.3

    points = Points()
    points.make_snake(2, 3, step_x=0.25, step_y=0.5)
    points.update_altitude(altitude)
    for i in points.points:
        print(i)

    points.next()

    adress = input('Put here destination/adress point (1 or 2 or 3) : ')
    cam.update_adress(adress)

    print(f'Moving up  0, 0, {altitude}')
    coex.navigate_wait(x=0, y=0, z=altitude, frame_id='body', auto_arm=True)

    for point in range(points.size):
        print(f'Moving to {points.x}, {points.y}, {points.z}')
        coex.navigate_wait(x=points.x, y=points.y, z=points.z, speed=velocity, frame_id='aruco_map')
        points.next()
    if cam.drone_find_qr_with_adress:
        x, y = calculate_center(cam.interest_points)
        print(f'Moving to {x}, {y}, {altitude}')
        coex.navigate_wait(x=x, y=y, z=altitude, frame_id='aruco_map')
        print("Landing...")
        coex.land_wait()
        rospy.sleep(5)
        print(f'Moving up  0, 0, {altitude} relative to drone')
        coex.navigate_wait(x=0, y=0, z=altitude, frame_id='body', auto_arm=True)


    print("Moving to zero point...")
    coex.navigate_wait(x=0, y=0, z=altitude, speed=velocity, frame_id='aruco_map')

    print("Landing...")
    coex.land_wait()


if __name__ == '__main__':
    main()
