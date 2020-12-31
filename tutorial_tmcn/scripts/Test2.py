import rospy
from nav_msgs.msg import Odometry

class Echo:

    def __init__(self):
        self.value = 0

        rospy.init_node('Test2')

        self.pub = rospy.Publisher('mytopic',Odometry,latch=True)
        rospy.Subscriber('odom',Odometry,self.update_value)


    def update_value(self,msg):
        self.value = msg.twist.twist.angular.x

    def run(self):
        rate =rospy.Rate(10)

        while not rospy.is_shutdown():
            self.pub.publish(self.value)
            rate.sleep()