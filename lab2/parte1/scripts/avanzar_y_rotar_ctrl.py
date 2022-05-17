#! /usr/bin/env python3


import rospy
import numpy as np
import matplotlib.pyplot as plt
from pid_controller import PIDController
from ang_controller import AngController
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray, Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Float64
from sound_play.libsoundplay import SoundClient
from tf.transformations import euler_from_quaternion


class MoveTurtlebot( object ):
    
    def __init__( self ):
        rospy.init_node( 'avanzar_y_rotar_ctrl' )
        self.cmd_vel_mux_pub = rospy.Publisher( '/yocs_cmd_vel_mux/input/navigation', Twist, queue_size=10 )
        self.odom_sub = rospy.Subscriber( '/odom', Odometry, self.odom_cb )
        self.goal_list_sub = rospy.Subscriber( '/goal_list', PoseArray, self.mover_robot_a_destino_ctrl )
        self.occupancy_state_sub = rospy.Subscriber( '/occupancy_state', String, self.obstacle_cb )
        self.goal_ang_pub = rospy.Publisher( '/goal_ang', Float64, queue_size = 10 )
        self.goal_pos_pub = rospy.Publisher( '/goal_pos', Point, queue_size = 10 )
        self.obstacle = 'FREE'
        self.printeo = False
        
        self.x = 1.0
        self.y = 1.0
        self.yaw = 0.0

        self.delta_pos = 0.0
        self.delta_ang = 0.0

        self.rate_hz = 0.2
        self.periodo = 1 / self.rate_hz
        self.rate_obj = rospy.Rate( self.rate_hz )
        
            
    def guardar_pose( self, pose ):
        ruta_poses = '/home/robotica/RoboticaMovil/src/simulador/include/real_poses_guardadass_exp1.csv'
        #ruta_pose = '/home/ffvenegas/ffvenegas_ws/src/lab1/scripts/odom_poses_guardadas.csv'
        
        with open( ruta_poses, 'r+' ) as archivo:
            archivo.read()
            archivo.write('\n' + pose)
    
    
    def obstacle_cb( self, msg ):
    # Guarda el mensaje enviado por el nodo detector de obstáculos
        self.obstacle = msg.data
    
    def graficar(self, lista_x, lista_y):
        longitud = np.array(lista_x)
        latitud = np.array(lista_y)
        plt.plot(longitud, latitud, c = 'red') #genera el grafico del mov del robot
        #esperado = plt.plot(longitud_objetivos, altura_objetivos, c= 'red') #genera el grafico del path
        print(f"x final: {longitud[-1]}, y final:{latitud[-1]}") #Termina de calcular el error cuadratico medio
        #plt.annotate("{.3f:.3f}".format(longitud[-1],latitud[-1]), (longitud[-1],latitud[-1]), textcoords= "offset points")
        plt.text(longitud[-1],latitud[-1], f"{longitud[-1]},{latitud[-1]}")
        plt.show()
    
    def odom_cb( self, msg ):
        pose = msg.pose.pose.position
        orient = msg.pose.pose.orientation
       
        self.x, self.y, z = pose.x, pose.y, pose.z

        roll, pitch, self.yaw = euler_from_quaternion( ( orient.x, orient.y, orient.z, orient.w ) )
        rospy.loginfo(f"self.x: {self.x}, self.y: {self.y}")
        rospy.loginfo(f"self.yaw {self.yaw}")

        #self.guardar_pose( f'{self.x},{self.y}')
       
        #rospy.loginfo( 'Current pose - lin: (%f, %f, %f) ang: (%f, %f, %f)' % ( self.x, self.y, z, roll, pitch, self.yaw ) )
            
            
    def mover_robot_a_destino_ctrl(self, goal_pose):
        self.last_ang= 0.0
        self.last_pos= 0.0
        id = 1
        rospy.loginfo("mover_robot_a_destino_ctrl")
        lista_x = []
        lista_y = []
        for pose in goal_pose.poses:
            
            # Pedimos específicamente x, y, theta
            x_goal, y_goal = pose.position.x, pose.position.y
            roll, pitch, yaw_goal = euler_from_quaternion( ( pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ) )

            
            print( f'self.x: {self.x}' )
            print( f'self.y: {self.y}' )
            print( f'self.yaw: {self.yaw}\n' )
            
            print( f'x_goal: { x_goal }' )
            print( f'y_goal: { y_goal }' )
            print( f'yaw_goal: { yaw_goal }\n' )

            lista_x.append(self.x)
            lista_y.append(self.y)
        
            # Entregamos los vectores a la función que aplica la velocidad
            if np.abs(yaw_goal - self.yaw) > 0.1:
                self.giro_controlado(yaw_goal)
                self.rate_obj.sleep()
            elif np.abs(x_goal - self.x) > 0.1 or np.abs(y_goal - self.y) > 0.1:
                self.desplazamiento_controlado(x_goal, y_goal)
                self.rate_obj.sleep()

            rospy.loginfo( f'Posición {id} lista\n' )
            id += 1
        self.graficar(lista_x, lista_y)

    def giro_controlado(self, theta_ref):
        rospy.loginfo("giro_controlado")
        self.goal_ang = Float64()
        self.goal_ang.data = theta_ref
        self.delta_ang = self.goal_ang.data - self.yaw
        self.last_ang = self.yaw

        self.goal_ang.data = self.goal_ang.data

        # while erro > 3 degrees and delta_ang > 2 degrees 
        #while abs(self.goal_ang.data - self.last_ang) > np.pi/120 or self.delta_ang > np.pi/180:
        #while self.yaw < 0.9*self.goal_ang.data:
        self.goal_ang_pub.publish( self.goal_ang )
        self.rate_obj.sleep()

        rospy.loginfo("ready")

    def desplazamiento_controlado(self, pos_ref_x, pos_ref_y):
        rospy.loginfo("desplazamiento_controlado")
        self.goal_pos = Point()
        self.goal_pos.x = pos_ref_x
        self.goal_pos.y = pos_ref_y
        self.goal_pos.z = 0.0
        while self.obstacle != 'FREE':
            if not self.printeo:
                print( self.obstacle, '\n' )
                self.printeo = True
        self.printeo = False

        #self.delta_pos = self.goal_pos.data - self.x
        #self.last_pos = self.x

        #while abs(pos_ref - self.last_pos) > 0.3:
        #while self.x < 0.9*np.abs(self.goal_pos.data):
        self.goal_pos_pub.publish( self.goal_pos )
        self.rate_obj.sleep()

        rospy.loginfo("ready")
            
   
if __name__ == '__main__':

  mov = MoveTurtlebot()
  print( '\nCreé clase "MoveTurtlebot"\n' )
  rospy.spin()     
    
    
    
    
    
    
    
    
    
    
    
      
  
           
    
    
