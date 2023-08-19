#!/usr/bin/env python3

import rospy
import random

from geometry_msgs.msg import Twist

class RandomVelocityGen():
    def __init__(self):
        
        rospy.init_node('random_velocity')

        self.vel_pub = rospy.Publisher('/new_vel', Twist, queue_size=1)
        self.vel = Twist()

        self.vel.linear.x = 0.1 # m/s
        self.vel.angular.z = 0.005 # rad/s

        self.vel_pub.publish(self.vel)
        rospy.loginfo('Iniital velocities: [%5.3f, %5.3f]', 
                      self.vel.linear.x, 
                      self.vel.angular.z)

        self.linear_vel_x_max = 1.2
        self.angular_vel_z_max = 0.5

        self.max_interval = 100
        print("Max velocities {}, {} - Max interval {}".format(self.linear_vel_x_max, 
                                                               self.angular_vel_z_max, 
                                                               self.max_interval))
        
    def generate_random_velocities(self):
        while not rospy.is_shutdown():

            x_forward = random.choice((-1, 1))
            z_counterclock = random.choice((-1, 1))

            self.vel.linear.x = (x_forward * random.uniform(0, self.linear_vel_x_max))
            self.vel.angular.z = (z_counterclock * random.uniform(0, self.angular_vel_z_max))

            self.vel_pub.publish(self.vel)

            now = rospy.get_rostime()
            print("Time now ", now.secs)

            next = (random.randint(1, self.max_interval))

            rospy.loginfo("Twist: [%5.3f, %5.3f], next change in %i secs - ",
                          self.vel.linear.x,
                          self.vel.angular.z,
                          next)
            rospy.sleep(next)

if __name__ == '__main__':
    try:
        generator = RandomVelocityGen()
        generator.generate_random_velocities()
    except rospy.ROSInterruptException:
        pass