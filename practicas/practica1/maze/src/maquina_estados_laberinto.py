#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from smach import State,StateMachine
from time import time

class Giro_Izquierda(State):
    def __init__(self):
        State.__init__(self, outcomes=["11","10","0X"])

    def execute(self, userdata):
        print("Giro Izquierda")
        toma_medidas()
        direccion.linear.x, direccion.angular.z = 0.0, 0.5
        pub.publish(direccion)
        rate.sleep()
        if medidas == [1,1]:
            return "11"
        if medidas == [1,0]:
            return "10"
        if medidas[0] == 0:
            return "0X"
        
class Recto(State):
    def __init__(self):
        State.__init__(self, outcomes=["11","10","0X"])

    def execute(self, userdata):
        print("Recto")
        toma_medidas()
        direccion.linear.x, direccion.angular.z = 0.5, 0.
        pub.publish(direccion)
        rate.sleep()
        if medidas == [1,1]:
            return "11"
        if medidas == [1,0]:
            return "10"
        if medidas[0] == 0:
            return "0X"

class Giro_Derecha(State):
    def __init__(self):
        State.__init__(self, outcomes=["11","10","0X","end"])

    def execute(self, userdata):
        print("Giro Derecha")
        toma_medidas()
        direccion.linear.x = 0.5
        direccion.angular.z =  (np.argmin(np.array((vision[0])[::-1]))+18.)/ 40. * -.75
        pub.publish(direccion)
        rate.sleep()
        if medidas == [1,1]:
            return "11"
        if medidas == [1,0]:
            return "10"
        if medidas[0] == 0:
            if (np.array((vision[0]+vision[1]))>10).all():
                direccion.linear.x, direccion.angular.z = 0., 0.
                pub.publish(direccion)
                print("Tardo ->",time()-empiezo," segundos")
                return "end"
            else:
                return "0X"
        


def callback(msg):
    vision[0] = msg.ranges[0:45]     #derecha
    vision[1] = msg.ranges[80:100]   #centro

def toma_medidas():
    global medidas
    medidas[0] = 1 if min(vision[0]) < 1.4 else 0
    medidas[1] = 1 if min(vision[1]) < 1.85 else 0

if __name__== "__main__":
    empiezo=time()
    rospy.init_node("maquina_estados")
    sub = rospy.Subscriber("/robot/laser/scan",LaserScan,callback)
    pub = rospy.Publisher("/robot/cmd_vel",Twist,queue_size=1)
    vision = [[0],[0]]
    rate = rospy.Rate(6)

    while vision[0]==[0]:
        rate.sleep()
    direccion = Twist()


    medidas = [0,0]


    toma_medidas()

    laberinto = StateMachine(outcomes=["success"])
    with laberinto:
        StateMachine.add("Derecha", Giro_Derecha(), transitions={"11":"Izquierda", "10":"Recto", "0X":"Derecha","end":"success"})
        StateMachine.add("Izquierda", Giro_Izquierda(), transitions={"11":"Izquierda", "10":"Recto", "0X":"Derecha"})
        StateMachine.add("Recto", Recto(), transitions={"11":"Izquierda", "10":"Recto", "0X":"Derecha",})
           
        
    laberinto.execute()
