import rospy
from std_msgs.msg import Int16, Float32

class Numbers(object):
    def __init__(self):
        self.pub = rospy.Publisher("/result", Float32, queue_size=10)
        self.sub = rospy.Subscriber("/numbers", Int16, self.innum)
        self.counter = 0
        self.arr = [0, 0, 0, 0, 0]
        self.mean = 0
        self.num = 0

    def innum(self, in_num):
        self.num = in_num.data

    def pubmean(self):
        self.arr[self.counter] = int(self.num)
        self.counter += 1
        if not (self.counter % 5):
            self.counter %= 5
            self.mean = sum(self.arr) / 5
            self.pub.publish(self.mean)
    def run(self):
        while not rospy.is_shutdown():
            self.pubmean()
            rospy.sleep(0.1)

rospy.init_node("numbers_sub_node")
numberist = Numbers()
numberist.run()
