#! /usr/bin/env python3

import rospy
import time

from geometry_msgs.msg import Twist

class MoveRobot():
    def __init__(self):
        rospy.init_node("MoveRobot", anonymous=True)

        rospy.loginfo("CTRL + C to stop the turtlebot")
        rospy.on_shutdown(self.shutdown)

        self.new_velocity_sub = rospy.Subscriber('/new_vel', Twist, self.callback_new_velocity)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        self.vel = Twist()
        self.new_velocity_pub = rospy.Publisher('/change', Twist, queue_size=1)

        self.rate = rospy.Rate(5) #times per second

    def callback_new_velocity(self, msg):
        rospy.loginfo("Received velocity [linear x]%5.2f [angular z]%5.2f", msg.linear.x, msg.angular.z)

        self.vel.linear.x = msg.linear.x
        self.vel.angular.z = msg.angular.z

        self.new_velocity_pub.publish(self.vel)

    def send_velocity_cmd(self):
        self.vel_pub.publish(self.vel)

    def shutdown(self):
        rospy.loginfo("Shutdown triggered")

        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0

        self.vel_pub.publish(self.vel)

        rospy.sleep(1)

    
if __name__ == "__main__":
    try:
        controller = MoveRobot()

        while not rospy.is_shutdown():
            controller.send_velocity_cmd()
            controller.rate.sleep()
    except:
        rospy.loginfo("Move robot node terminated")


    

