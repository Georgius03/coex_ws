import rospy
from std_msgs.msg import Int16
from random import randint

rospy.init_node("numbers")

pub = rospy.Publisher("/numbers", Int16, queue_size=10)

while not rospy.is_shutdown():
    s = randint(1, 99)
    pub.publish(s)
    rospy.sleep(0.2)
