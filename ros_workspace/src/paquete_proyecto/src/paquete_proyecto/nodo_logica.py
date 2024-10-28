#!/usr/bin/python3

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


class Nodo_logica:
    def __init__(self, cartas) -> None:
        # cartas = 2 dimensional array ['carta', PosicionCarta (x,y)]
        self.cartas = cartas

    def game_1(self, reversed_bool=False) -> None:
        valores = {'A': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, 'J': 11, 'Q': 12, 'K': 13}
   
        # Ordenar cartas en función de su valor en el diccionario
        cartas_ordenadas = sorted(self.cartas, key=lambda carta: valores[carta], reverse=reversed_bool)
        return cartas_ordenadas

cartas = ['A', 'K', '10', 'Q']
logica = Nodo_logica(cartas)
print(logica.game_1())