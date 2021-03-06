#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
 
from pid_controller import PIDController
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class FollowThePath(object):

  def __init__(self):
    rospy.init_node('follow_the_path')
    self.variables_init()
    self.connections_init()
    rospy.spin()
    
  def variables_init(self):

    # value and goal goal value
    self.value = 0.0
    self.goal_value = 0.001
    
    # last orienantation and delta ang
    self.last_value = 0.0
    self.delta_value = 0.0

    # message sending frequency: 10Hz
    self.hz = 10
    self.rate_obj = rospy.Rate(self.hz)

    # msg type: Twist
    self.speed_msg = Twist()

    # line speed
    self.speed_msg.linear.x = 0.2

    # controller topic, in launch "ns = topic"
    self.topic = "robot_ang"

    self.bridge = CvBridge()
    
  def connections_init(self):
    # Turtlebot connections
    self.cmd_vel_mux_pub = rospy.Publisher('/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10)
    
    #  angule PID controller
    self.ang_PID_controller = PIDController(self.topic)
    
    # depth sensor 
    rospy.Subscriber( '/camera/depth/image_raw', Image , self.depth_image_cb )

    # set point
    rospy.Subscriber('/goal_value', Float64, self.run)
  
  def depth_image_cb( self, msg ):

    try:
      self.depth_image_np = self.bridge.imgmsg_to_cv2( msg )

      img2display = self.depth_image_np - self.depth_image_np.min()
      img2display = ( img2display * ( 255.0/img2display.max() ) ).astype( np.uint8 )
      img2display = 255 - img2display
      img2display = cv2.applyColorMap( img2display, cv2.COLORMAP_HOT)
      img2display = cv2.flip(img2display, 0)
      
      img_y, img_x = img2display.shape[0:2]
      left_img = img2display[ :, : (img_x // 3) ]
      right_img = img2display[ :, (2 * img_x // 3) :  ]
      
      if self.depth_image_np is not None:
      	#column_sample_left = self.depth_image_np[ img_y//2:, : (2 * img_x//3) ]
      	#column_sample_right = self.depth_image_np[ img_y//2:, (img_x // 3) : ]
      	
      	column_sample_left_ext = self.depth_image_np[ :, : ( img_x // 3) ]
      	column_sample_right_ext = self.depth_image_np[ :, (2 * img_x // 3) : ]

      	#column_sample_left = np.where( np.isnan( column_sample_left ), 0.0, column_sample_left )
      	#column_sample_right = np.where( np.isnan( column_sample_right ), 0.0, column_sample_right )
      	
      	column_sample_left_ext = np.where( np.isnan( column_sample_left_ext ), 0.0, column_sample_left_ext )
      	column_sample_right_ext = np.where( np.isnan( column_sample_right_ext ), 0.0, column_sample_right_ext )
      
      #value_left = np.mean(column_sample_left)
      #value_right = np.mean(column_sample_right)
      
      value_left_ext = np.mean(column_sample_left_ext)
      value_right_ext = np.mean(column_sample_right_ext)
      	
      
      if value_left_ext < value_right_ext:
      	if value_left_ext < 2.57:
      	  value_right_ext = (6.0 - value_left_ext) #* ( value_left_ext / 3.0 )
      	else:
      	  value_right_ext = (6.0 - value_left_ext) #* ( value_left_ext / 3.0 )
      	rospy.loginfo( 'izq_ext: %f | delta: %f' % ( value_left_ext, self.value ) )	  
      
      else:
      	if value_right_ext < 1.71: # muy cerca de chocar con la derecha
      	  value_left_ext = (6.0 - value_right_ext) #* ( value_right_ext / 3.0 ) # no achicamos
      	else:
      	  value_left_ext = (6.0 - value_right_ext) #* ( value_right_ext / 3.0 ) # no achicamos
      	rospy.loginfo( 'der_ext: %f | delta: %f' % ( value_right_ext, self.value ) )
      
      if value_left_ext > 0.1 and value_right_ext > 0.1: # chocar con el borde de la interfaz
      	self.speed_msg.linear.x = 0.2
      else:
      	self.speed_msg.linear.x = 0.0
      	
      self.value = value_right_ext - value_left_ext
      
      self.ang_PID_controller.pub_state(self.value)
      #rospy.loginfo( 'izq: %f | der %f | delta: %f' % ( value_left, value_right, self.value ) )
      #rospy.loginfo( 'izq_ext: %f | der_ext %f | delta: %f' % ( value_left_ext, value_right_ext, self.value ) )
      
      cv2.imshow('Depth Sensor', img2display)
      cv2.waitKey( 1 )

    except CvBridgeError as e:
      rospy.logerr( e )

  def run(self, goal):
    self.goal_value = goal.data
    self.ang_PID_controller.pub_set_point(self.goal_value)
 
    while True:
      self.speed_msg.angular.z = self.ang_PID_controller.speed
      self.cmd_vel_mux_pub.publish(self.speed_msg)
      self.rate_obj.sleep()

if __name__ == '__main__':
  follow_the_path = FollowThePath()
  rospy.spin()
