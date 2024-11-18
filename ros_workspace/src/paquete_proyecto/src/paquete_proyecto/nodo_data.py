import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String, Int16
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import yaml

captura_datos = False
data_list = []

def cb_movements(data: Int16) -> None:
    if data.data == 1:
        captura_datos = True
    else:
        captura_datos = False

def cb_data_capture(data: List) -> None: # Este tipo de dato esta mal pero bueno
    if captura_datos == True:
        data_list.append(data.data)
    
    if len(data_list) > 0:
        # Queda aqui hacer todo lo de subir a InfluxDB 

        data_list = []




## Logica a seguir ##

# Este nodo va a estar suscrito a 2 topics --> 1 topic es para decir si tiene que guardar datos si o no
#                                           --> Otro topic es el que te devuelve la posicion, velodidad y esfuerzo del robot /joint_states


if __name__ == '__main__':
    rospy.Subscriber("/movimientos", Int16, cb_movements)
    rospy.Subscriber("/joint_states", Int16, cb_data_capture)


