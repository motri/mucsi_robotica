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

        pass

    def game_1(self) -> None:
        catas_ordenadas = sorted(cartas)


