import sys
import copy
import rospy
import moveit_commander
from math import pi
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib import SimpleActionClient
from time import sleep

# Clase para controlar las funciones del robot
class ControlRobot:

    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        self.add_floor()

    def move_motors(self, joint_goal: List[float], wait: bool = True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def move_to_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)
        if fraction != 1.0:
            return False
        return self.move_group.execute(plan, wait=wait)

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        return self.gripper_action_client.get_result().reached_goal

    def abrir_pinza(self):
        self.mover_pinza(100.0, 5.0)
        sleep(2)

    def cerrar_pinza(self):
        self.mover_pinza(60.0, 5.0)
        sleep(2)

    def bajar_pinza(self):
        pose_actual = self.get_pose()
        pose_actual.position.z -= 0.07  # Ajustar a la medida real
        self.move_trajectory([pose_actual])
        sleep(2)

    def subir_pinza(self):
        pose_actual = self.get_pose()
        pose_actual.position.z += 0.07  # Ajustar a la medida real
        self.move_trajectory([pose_actual])
        sleep(2)

    def mover_a_configuracion(self, configuracion: List[float]):
        self.move_motors(configuracion)


# Función para ordenar las cartas
#array_sin_ordenar = [2, 1, 3, 4, 0]
#array_ordenado = [4, 3, 2, 1, 0]

def ordenar_array(control: ControlRobot, array_ordenado: List[int], array_sin_ordenar: List[int], posiciones: List[Pose]):
    posicion_vacia = len(array_ordenado) - 1

    while array_sin_ordenar != array_ordenado:
        for i in range(len(array_sin_ordenar)):
            if array_sin_ordenar[i] != array_ordenado[i]:
                # Mover la carta incorrecta al hueco vacío
                control.move_to_pose(posiciones[i])
                control.bajar_pinza()
                control.cerrar_pinza()
                control.subir_pinza()

                # Mover al espacio vacío
                control.move_to_pose(posiciones[posicion_vacia])
                control.bajar_pinza()
                control.abrir_pinza()
                control.subir_pinza()

                # Actualizar los arrays y posición vacía
                array_sin_ordenar[posicion_vacia] = array_sin_ordenar[i]
                array_sin_ordenar[i] = 0
                posicion_vacia = i


# Configuraciones predeterminadas del robot
def configuraciones_iniciales():
    return [
        [0.54, -1.96, -0.69, -2.04, 1.57, 1.48],  # Configuración 1 #Carta 2 
        [0.15, -2.09, -0.46, -2.14, 1.57, 1.09],  # Configuración 2 # Carta 1
        [-0.22, -2.11, -0.48, -2.11, 1.57, 0.78],  # Configuración 3 # Carta 3
        [-0.61, -1.94, -0.54, -2.23, 1.57, 0.31]   # Configuración 4 #Carta 4
    ]


if __name__ == '__main__':
    try:
        control = ControlRobot()

        # Inicializar posiciones y arrays
        array_sin_ordenar = [2, 1, 3, 4, 0]
        array_ordenado = [4, 3, 2, 1, 0]
        posiciones = [
            control.get_pose(),  # Pose para la carta 2
            control.get_pose(),  # Pose para la carta 1
            control.get_pose(),  # Pose para la carta 3
            control.get_pose(),  # Pose para la carta 4
            control.get_pose()   # Pose para el espacio vacío
        ]

        # Ordenar usando el robot
        ordenar_array(control, array_ordenado, array_sin_ordenar, posiciones)
        print("Cartas ordenadas correctamente.")

    except rospy.ROSInterruptException:
        print("Programa interrumpido")

    finally:
        moveit_commander.roscpp_shutdown()
        print("Programa finalizado")
