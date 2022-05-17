#!/usr/bin/env python3

import rospy
import numpy as np
from pid_controller import PIDController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class AngController(object):


    def __init__(self):
        rospy.init_node('ang_controller')
        self.variables_init()
        self.connections_init()

    def variables_init(self):

        # orientation and goal orientation
        self.ang = 0.0
        self.goal_ang = 0.0

        # last orienantation and delta ang
        self.last_ang = 0.0
        self.delta_ang = 0.0

        # message sending frequency: 10Hz
        self.hz = 20
        self.rate_obj = rospy.Rate(self.hz)

        # msg type: Twist
        self.speed_msg = Twist()

        # controller topic, in launch "ns = topic"
        self.topic = "robot_ang"

    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        #  angule PID controller
        self.ang_PID_controller = PIDController(self.topic)

        # Odometry
        rospy.Subscriber('/odom', Odometry, self.set_odom)

        # set point
        rospy.Subscriber('/goal_ang', Float64, self.run)

    def set_odom(self, odom_data):
        pose_data = odom_data.pose.pose

        quaternion = (pose_data.orientation.x,
                        pose_data.orientation.y,
                        pose_data.orientation.z,
                        pose_data.orientation.w)

        row, pitch, self.ang = euler_from_quaternion(quaternion)

        #self.delta_ang = self.goal_ang - self.last_ang
        #self.last_ang = self.ang

        self.ang_PID_controller.pub_state(self.ang)

    def run(self, goal_ang):
        rospy.loginfo(f"llego_goal_ang:{goal_ang.data}")
        self.goal_ang = goal_ang.data
        self.delta_ang = self.goal_ang - self.last_ang
        self.last_ang = self.ang

        # while erro > 3 degrees and delta_ang > 2 degrees 
        #while np.abs(self.ang) < 0.9*np.abs(self.goal_ang):
        while np.abs(self.delta_ang) > 0.05:
            self.ang_PID_controller.pub_set_point(self.goal_ang)
            self.delta_ang = self.goal_ang - self.last_ang
            self.last_ang = self.ang
            self.speed_msg.angular.z = self.ang_PID_controller.speed
            rospy.loginfo(f"self.speed_msg.angular.z {self.speed_msg.angular.z}")
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()
            #self.rate_obj.sleep()
        else:
            self.speed_msg.angular.z = 0.0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

        rospy.loginfo("ready")

if __name__ == '__main__':

  controller = AngController()
  print( '\nCreé clase "AngController"\n' )
  rospy.spin() 