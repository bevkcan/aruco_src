#! /usr/bin/env python
import message_filters
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry



class MySubscriber(object):
    
    def __init__(self):

        self.odom=Odometry()
        # Odometry Class'dan object olusturur

        self.twist = Twist()
        # Twist Class'dan object olusturur

        rospy.Subscriber('cmd_vel',Twist,self.callbackVel)
        # cmd_vel topic'ine subscriber atar ve callbackVel function'a baglar

        rospy.Subscriber('odom',Odometry,self.callbackPose)
        # odom topic'ine sunscriber atar ve callbackPose function'a baglar

        self.pub = rospy.Publisher('myTopic',Odometry,queue_size=10)
        # myTopic'e publisher

    def callbackVel(self,velData):
        # cmd_vel topic'ine msg geldigi zaman bu function'i yapar

        # yukarda self.twist object'i olusturmustuk
        # self.twist'in linear ve angular gibi ozellikleri var bunu ogrenmek icin
        # rostopic info geometry_msgs/Twist komutunu calistir ordaki bilgilere bak
        self.twist.linear.x = velData.linear.x
        self.twist.linear.y = velData.linear.y
        self.twist.linear.z = velData.linear.z
        self.twist.angular.x = velData.angular.x
        self.twist.angular.y = velData.angular.y
        self.twist.angular.z = velData.angular.z
        


    def callbackPose(self,poseData):
        # odom topic'ine msg geldigi zaman bu function'i yapar

        # yukarda self.twist object'i olusturmustuk
        # self.twist'in linear ve angular gibi ozellikleri var bunu ogrenmek icin
        # rostopic info nav_msgs/Odometry komutunu calistir ordaki bilgilere bak
        self.odom.pose.pose.position.x = poseData.pose.pose.position.x
        self.odom.pose.pose.position.y = poseData.pose.pose.position.y
        self.odom.pose.pose.position.z = poseData.pose.pose.position.z

        self.odom.pose.pose.orientation.x = poseData.pose.pose.orientation.x
        self.odom.pose.pose.orientation.y = poseData.pose.pose.orientation.y
        self.odom.pose.pose.orientation.z = poseData.pose.pose.orientation.z
        self.odom.pose.pose.orientation.w = poseData.pose.pose.orientation.w

        self.odom.twist.twist.linear.x = poseData.twist.twist.linear.x
        self.odom.twist.twist.linear.y = poseData.twist.twist.linear.y
        self.odom.twist.twist.linear.z = poseData.twist.twist.linear.z

        self.odom.twist.twist.angular.x = poseData.twist.twist.angular.x
        self.odom.twist.twist.angular.y = poseData.twist.twist.angular.y
        self.odom.twist.twist.angular.z = poseData.twist.twist.angular.z

        self.pub.publish(self.odom)
        # myTopic publish eder.


    def loop(self):
        rospy.logwarn("Starting loop...")
        rospy.spin()
        # rospy.spin herhangi bir message alinmadigi zaman bile bu script surekli calisir



if __name__ == "__main__":
    rospy.init_node('Test3',anonymous=True,log_level=rospy.WARN)
    # Test3.py script'ini rospy tanimasi icin gerekli satir

    mySubscriber= MySubscriber()
    # Yukarda olusturdugumuz MySubscriber Class'indan object olusturur
    # Ve her object olusturdugumuz zaman Class'in constructor function otomatik olarak calistirir.
    # Ama diger function'lari sadece cagirdigimiz zaman calistirir.

    mySubscriber.loop()
    # Object'in loop function'i calistirir


