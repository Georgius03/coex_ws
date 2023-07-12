import math
import rospy
import cv2
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
from clover.srv import SetLEDEffect
from sklearn.cluster import KMeans

color_cloud = []


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
        self.pub_cord = rospy.Publisher('color_detect_coords', String, queue_size=10)
        self.pub_img = rospy.Publisher('color_detect_image', Image, queue_size=10)

        self.sub = rospy.Subscriber('main_camera/image_raw', Image, self.image_callback, queue_size=1)

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
        self.bridge = CvBridge()

        self.current_color = None
        self.prev_color = None
        self.color_points_cloud = []

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # convert to HSV to work with color hue
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # cut out a central square
        w = img.shape[1]
        h = img.shape[0]
        r = 5
        center = img_hsv[h // 2 - r:h // 2 + r, w // 2 - r:w // 2 + r]

        # compute and print the average hue
        mean_hue = center[:, :, 0].mean()
        mean_saturatuion = center[:, :, 1].mean()
        mean_value = center[:, :, 2].mean()

        self.current_color = self.get_color_name(mean_hue, mean_saturatuion, mean_value)
        if self.current_color != 'none':
            self.set_led_color(self.current_color)

            print(f'{self.current_color}:  hue={mean_hue}, saturation={mean_saturatuion}, value={mean_value}')

            telem = self.get_telemetry()
            self.pub_cord.publish(f'{telem.x:.3} {telem.y:.3} {telem.z:.3}')
            self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, 'bgr8'))

            self.color_points_cloud.append([round(telem.x, 3), round(telem.y, 3), self.current_color])

        self.prev_color = self.current_color

    def get_color_name2(self, hue):
        if hue < 15:
            return 'red'
        elif hue < 30:
            return 'orange'
        elif hue < 60:
            return 'yellow'
        elif hue < 90:
            return 'green'
        elif hue < 120:
            return 'cyan'
        elif hue < 150:
            return 'blue'
        elif hue < 170:
            return 'magenta'
        else:
            return 'red'

    def get_color_name(self, hue, saturation, value):
        if (50 <= saturation <= 240) and (50 <= value <= 240):
            if (160 <= hue <= 180) or (0 <= hue <= 12):
                return 'red'
            elif (35 <= hue < 110):
                return 'green'
            elif (110 < hue < 150):
                return 'blue'
            else:
                return 'none'
        else:
            return 'none'

    def set_led_color(self, hue_name):
        effect = 'flash'
        if hue_name == 'red':
            self.set_effect(effect=effect, r=255, g=0, b=0)  # red
        elif hue_name == 'orange':
            self.set_effect(effect=effect, r=255, g=128, b=0)  # orange
        elif hue_name == 'yellow':
            self.set_effect(effect=effect, r=255, g=255, b=0)  # yellow
        elif hue_name == 'green':
            self.set_effect(effect=effect, r=0, g=255, b=0)  # green
        elif hue_name == 'cyan':
            self.set_effect(effect=effect, r=0, g=128, b=255)  # cyan
        elif hue_name == 'blue':
            self.set_effect(effect=effect, r=0, g=0, b=255)  # blue
        elif hue_name == 'magenta':
            self.set_effect(effect=effect, r=255, g=0, b=255)  # 'magenta'
        elif hue_name == 'none':
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
            len_x = int(len_x // step_x)
            len_y = int(len_y // step_y)
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
    kmeans = KMeans(n_clusters=4)
    kmeans.fit(for_fit)
    centers = kmeans.cluster_centers_
    res = {}
    for center in centers:
        for color_point in color_arr:
            if ((color_point[0] - center[0]) ** 2 + (color_point[1] - center[1]) ** 2) ** 0.5 <= 0.7:
                res[f"{center[0], center[1]}"] = color_point[2]
    return res


def main():
    rospy.init_node('cv')
    coex = COEX()
    cam = CameraStream()

    points = Points()
    points.make_snake(3, 4, step_x=0.5)
    points.update_altitude(1)
    for i in points.points:
        print(i)

    points.next()

    print(f'Moving up  {points.x}, {points.y}, {points.z}')
    coex.navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)
    velocity = 0.5
    for point in range(points.size):
        print(f'Moving to {points.x}, {points.y}, {points.z}')
        coex.navigate_wait(x=points.x, y=points.y, z=points.z, speed=velocity, frame_id='aruco_map')
        points.next()

    print("Landing...")
    coex.land_wait()

    color_arr = cam.color_points_cloud
    color_clustered = get_color_centers(color_arr)
    for i in color_clustered.items():
        print(f'{i[1]} in {i[0]}')


if __name__ == '__main__':
    main()
