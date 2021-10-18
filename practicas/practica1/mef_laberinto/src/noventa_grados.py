#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    vision[0] = msg.ranges[0:20]
    vision[1] = msg.ranges[80:101]
    vision[2] = msg.ranges[-20,-1]

rospy.init_node("maquina_estados")
sub = rospy.Subscriber("/robot/laser/scan",LaserScan,callback)
pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=1)
vision = [[0],[0]]
rate = rospy.Rate(2)

while vision[0]==[0]:
    rate.sleep()
direccion = Twist()

def giro90(sentido=False):  #FALSE=DERECHA
    direccion.linear.x = .0
    direccion.angular.z = np.pi/2.
    for i in range(2):    
        pub.publish(direccion)
        rate.sleep()

estados = [(.6,0.0),   (0.2,-0.6),     (0.,0.6)] #RECTO   #GIRO DERECHA   #GIRO IZQUIERDA

estado_actual = 0
medidas = [0,0]

while not rospy.is_shutdown():              #PARED 1    VACIO 0         MEDIDAS = [V1,V2]
    
    medidas[0] = 1 if min(vision[0]) < 1.3 else 0
    medidas[1] = 1 if min(vision[1]) < 2.0 else 0
    
    if estado_actual == 0:
        if medidas == [0,0]:
            estado_actual = 1
        if medidas == [1,1]:
            estado_actual = 2
        if medidas == [0,1]:
            estado_actual = 2
        

    elif estado_actual == 1:
        if medidas == [1,0]:
            estado_actual = 0
        if medidas == [1,1]:
            estado_actual = 2
    
    elif estado_actual == 2:
        if medidas == [1,0]:
            estado_actual = 0
        if medidas == [0,1]:
            estado_actual = 1
    
    direccion.linear.x, direccion.angular.z = estados[estado_actual]
    #print(min(vision[0]))
    #"""
    pub.publish(direccion)
    print(min(vision[0]))
    print(min(vision[1]))
    print(estado_actual)
    #"""
    rate.sleep()
