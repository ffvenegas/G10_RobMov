#!/usr/bin/env python3

import rospy
import numpy as np
from pid_controller import PIDController
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class PosController(object):


    def __init__(self):
        rospy.init_node('pos_controller')
        self.variables_init()
        self.connections_init()

    def variables_init(self):

        # orientation and goal orientation
        self.x = 0.0
        self.goal_pos_x = 0.0

        self.y = 0.0
        self.goal_pos_y = 0.0

        # last orienantation and delta ang
        self.last_pos_x = 0.0
        self.delta_pos_x = 0.0

        self.last_pos_y = 0.0
        self.delta_pos_y = 0.0

        self.state= Float64()

        # message sending frequency: 10Hz
        self.hz = 20
        self.rate_obj = rospy.Rate(self.hz)

        # msg type: Twist
        self.speed_msg = Twist()

        # controller topic, in launch "ns = topic"
        self.topic = "robot_pos"

    def connections_init(self):
        # Turtlebot connections
        self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)

        #  angule PID controller
        self.pos_PID_controller = PIDController(self.topic)

        # Odometry
        rospy.Subscriber('/odom', Odometry, self.set_odom)

        # set point
        rospy.Subscriber('/goal_pos', Point, self.run)



    def set_odom(self, odom_data):
        pose_data = odom_data.pose.pose

        self.x, self.y, z = (pose_data.position.x, pose_data.position.y, pose_data.position.z)


        #self.pos_PID_controller.pub_state(self.x)

    def run(self, goal_pos):
        rospy.loginfo(f"llego goal_pos x:{goal_pos.x}, y:{goal_pos.y}, z:{goal_pos.z}")
        self.goal_pos_x, self.goal_pos_y, z= goal_pos.x, goal_pos.y, goal_pos.z

        #self.goal_pos_x = float(self.goal_pos_x)
        #self.goal_pos_y = float(self.goal_pos_y)

        self.delta_pos_x = self.goal_pos_x - self.last_pos_x
        #if np.abs(self.delta_pos_x) > 0.1:
        #    self.pos_PID_controller.pub_state(self.x)
        self.last_pos_x = self.x

        self.delta_pos_y = self.goal_pos_y - self.last_pos_y
        #if np.abs(self.delta_pos_y) > 0.1:
        #    self.pos_PID_controller.pub_state(self.y)
        self.last_pos_y = self.y

        # while erro > 3 degrees and delta_ang > 2 degrees 
        while np.abs(self.delta_pos_x) > 0.05:
            self.pos_PID_controller.pub_state(self.x)
            self.pos_PID_controller.pub_set_point(self.goal_pos_x)
            self.delta_pos_x = self.goal_pos_x - self.last_pos_x
            self.last_pos_x = self.x
            self.speed_msg.linear.x = np.abs(self.pos_PID_controller.speed)
            rospy.loginfo(f"self.speed_msg.linear.x {self.speed_msg.linear.x}")
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()
        else:
            self.speed_msg.linear.x = 0.0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

        while np.abs(self.delta_pos_y) > 0.05:
            self.pos_PID_controller.pub_state(self.y)
            self.pos_PID_controller.pub_set_point(self.goal_pos_y)
            self.delta_pos_y = self.goal_pos_y - self.last_pos_y
            self.last_pos_y = self.y
            self.speed_msg.linear.x = np.abs(self.pos_PID_controller.speed)
            rospy.loginfo(f"self.speed_msg.linear.y {self.speed_msg.linear.x}")
            self.cmd_vel_mux_pub.publish(self.speed_msg)
            self.rate_obj.sleep()
        else:
            self.speed_msg.linear.x = 0.0
            self.cmd_vel_mux_pub.publish(self.speed_msg)

        rospy.loginfo("ready")

if __name__ == '__main__':

  controller = PosController()
  print( '\nCreé clase "PosController"\n' )
  rospy.spin() 