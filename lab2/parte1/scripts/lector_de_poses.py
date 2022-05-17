#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from os.path import join

class LectorPoses( object ):

    def __init__( self ):
        rospy.init_node( 'lector_goal_poses' )
        self.goal_list_pub = rospy.Publisher( '/goal_list', PoseArray, queue_size=10 )
        self.goal_list = []
        self.rate_hz = 0.5
        self.rate_obj = rospy.Rate( self.rate_hz )
        
    
    def leer_archivo( self ):
    # Lee lista de poses desde un archivo
        #ruta_poses = join( 'poses.csv' )
        ruta_poses = '/home/robotica/RoboticaMovil/src/simulador/include/poses1.csv'
        
        with open( ruta_poses, 'rt' ) as archivo:
            self.goal_list = archivo.readlines()
        
        self.goal_list = [ linea.strip().split(',') for linea in self.goal_list ]
        print( self.goal_list )
    
    
    def publicar_goal_list( self ):
    # Crea msg tipo PoseArray para cada pose y envía lista con estos
        pose_array = PoseArray()
        
        for data in self.goal_list:
            x_goal = float( data[0] )
            y_goal = float( data[1] )
            z_goal = 0.0
            qt_x, qt_y, qt_z, qt_w = quaternion_from_euler( 0.0, 0.0, float( data[2] ) )
            
            my_point = Point()
            
            my_point.x = x_goal
            my_point.y = y_goal
            my_point.z = z_goal
            
            my_quaternion = Quaternion()
            my_quaternion.x = qt_x
            my_quaternion.y = qt_y
            my_quaternion.z = qt_z
            my_quaternion.w = qt_w
            
            my_pose = Pose()
           
            my_pose.position = my_point
            my_pose.orientation = my_quaternion
            
            pose_array.poses.append( my_pose )
            
            #print( f'pose_array.poses:\n' )
            #print( pose_array.poses, '\n' )
        
        self.rate_obj.sleep()    
        self.goal_list_pub.publish( pose_array )
        
        print( '\nPoseArray publicado' )
        

if __name__ == '__main__':

  lector = LectorPoses()
  lector.leer_archivo()
  lector.publicar_goal_list()
  rospy.spin()
  
  #rostopic pub /goal_list geometry_msgs/PoseArray "{header: {frame_id: 'base_frame'}, poses: [{position: {x: 2.0, y: 1.0, z: 0.0},    orientation: {x: 0.0, y: 0.0, z: 0.7071788, w: 0.7070348}}, {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}"
  
  
  
             
