#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

import tf

from gazebo_msgs.srv import GetModelState

class PoseMonitor():

    def __init__(self):
        rospy.init_node("pose_monitor", anonymous=True)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_odometry)
        self.vel_change_sub = rospy.Subscriber('/change', Twist, self.callback_velocity_change)

        self.report_pose = False

        print("Wait for service")
        rospy.wait_for_service("gazebo/get_model_state")

        print("Got it!")
        self.get_ground_truth = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)

        self.robot_model = 'turtlebot3_waffle'

    def callback_velocity_change(self, msg):
        rospy.loginfo("Velocity has cnanged, now %5.3f, %5.3f",
                      msg.linear.x, msg.angular.z)
        rospy.sleep(0.75)
        self.report_pose = True

    def callback_odometry(self, msg):
        if self.report_pose == True:
            print("Position: (%5.2f, %5.2f, %5.2f)" % (msg.pose.pose.position.x, 
                                                       msg.pose.pose.position.y,
                                                       msg.pose.pose.position.z))
            self.quatarnion_to_euler(msg)
            print("Linear twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.linear.x,
                                                           msg.twist.twist.linear.y,
                                                           msg.twist.twist.linear.z))
            print("Angular twist: (%5.2f, %5.2f, %5.2f)" % (msg.twist.twist.angular.x,
                                                           msg.twist.twist.angular.y,
                                                           msg.twist.twist.angular.z))
            print("Ground Truth: ", self.get_ground_truth(self.robot_model, "world"))
            self.report_pose = False

    def quaterinon_to_euler(self, msg):
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        print("Roll: %5.2f Pitch: %5.2f Yaw %5.2f" % (roll, pitch, yaw))

if __name__ == "__main__":
    try:
        PoseMonitor()
        while not rospy.is_shutdown():
            rospy.spin()
    except:
        rospy.loginfo("Pose monitor terminated")

    