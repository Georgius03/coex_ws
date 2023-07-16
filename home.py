import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from std_msgs.msg import String


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

    def make_snake(self, len_x, len_y, z=1) -> None:
        if not self.points:
            flag = True
            for x in range(len_x):
                if flag:
                    for y in range(len_y + 1):
                        self.points.append([x, y, z])
                        self.size += 1
                    flag = not flag
                else:
                    for y in range(len_y, -1, -1):
                        self.points.append([x, y, z])
                        self.size += 1
                    flag = not flag
            self.next()


def main():
    rospy.init_node('cv')
    coex = COEX()

    print(f'Moving up 1 meter')
    coex.navigate_wait(x=0, y=0, z=1, frame_id='body', auto_arm=True)

    print(f'Moving to zero point - Home')
    coex.navigate_wait(x=0, y=0, z=1, frame_id='aruco_map', auto_arm=True)

    print("Landing...")
    coex.land_wait()


if __name__ == '__main__':
    main()
