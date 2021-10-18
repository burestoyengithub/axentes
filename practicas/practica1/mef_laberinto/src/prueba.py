#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    vision[0] = msg.ranges[0:40]
    vision[1] = msg.ranges[40:80]
    vision[2] = msg.ranges[80:101]

rospy.init_node("maquina_estados")
sub = rospy.Subscriber("/robot/laser/scan",LaserScan,callback)
pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=1)
vision = [[0],[0], [0]]
rate = rospy.Rate(2)

while vision[0]==[0]:
    rate.sleep()
direccion = Twist()

estados_nopared = [(0.,0.5),    (.5,1.) ,    (0., 0.)] #Pared de frente   #busqueda pared mas cercana
estados_pared = [(.6,0.0),   (0.2,-0.5),     (0.,0.5)] #RECTO   #GIRO DERECHA   #GIRO IZQUIERDA

estado_actual = 0
medidas = [0,0]

while not rospy.is_shutdown():              #PARED 1    VACIO 0         MEDIDAS = [V1,V2]
    
    pegado_pared = True if min(vision[0]) < 1.0 else False
    medidas[0] = 1 if min(vision[0]) < 1.3 else 0
    medidas[1] = 1 if min(vision[2]) < 2.0 else 0
    
    if not pegado_pared:
        if min(vision[2] + vision[1])<1.0:
            estado_actual = 0
        
        elif (np.array(min(vision[0]+vision[1]+vision[2])) < 5).all():
            estados_nopared[1] = (0.5, np.argmin(np.array((vision[0]+vision[1]+vision[2])[::-1]))/ 100. * -0.5)
            print(estados_nopared)
            estado_actual=1
            
        else:
            estado_actual=2
            
        direccion.linear.x, direccion.angular.z = estados_nopared[estado_actual]
    
    else:
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
    
        direccion.linear.x, direccion.angular.z = estados_pared[estado_actual]
    #print(min(vision[0]))
    #"""
    pub.publish(direccion)
    print(min(vision[0]))
    print(min(vision[2]))
    print(str(pegado_pared)+ str(estado_actual))
    #"""
    rate.sleep()
