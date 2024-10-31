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
        self.parejas = parejas

    def updateCartas(self, cartas) -> None:
        self.cartas = cartas

    def ordenarCartas(self, reversed_bool=False) -> List:
        valores = {'A': 1, '2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10, 'J': 11, 'Q': 12, 'K': 13}
   
        # Ordenar cartas en función de su valor en el diccionario
        cartas_ordenadas = sorted(self.cartas, key=lambda carta: valores[carta], reverse=reversed_bool)
        return cartas_ordenadas
    
    def jugarPoker(self) -> List:
        # Definimos los valores de las cartas
        valores = {'2': 2, '3': 3, '4': 4, '5': 5, '6': 6, '7': 7, '8': 8, '9': 9, '10': 10,
                'J': 11, 'Q': 12, 'K': 13, 'A': 14}

        # Función auxiliar para obtener el valor y tipo de cada pareja
        def evaluar_pareja(self.pareja):
            carta1, carta2 = self.pareja
            if carta1 == carta2:  # Pareja verdadera
                return (carta1*10 + carta2*10)
            else:  # Combinación de cartas diferentes
                return (carta1 + carta2)

        # Evaluamos las parejas
        valor_pareja1 = evaluar_pareja(pareja1)
        valor_pareja2 = evaluar_pareja(pareja2)
        valor_pareja3 = evaluar_pareja(pareja3)

        # Comparar las evaluaciones y devolver la mejor pareja
        mejor = max(valor_pareja1, valor_pareja2, valor_pareja3)

        # Devolver la pareja correspondiente al valor máximo
        if mejor == valor_pareja1:
            return pareja1
        elif mejor == valor_pareja2:
            return pareja2
        else:
            return pareja3
    
    def jugarBlackjack(self) -> List:
        return None

cartas = ['A', 'K', '10', 'Q']
logica = Nodo_logica(cartas)
print(logica.ordenarCartas())