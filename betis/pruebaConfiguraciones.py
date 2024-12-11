# Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import yaml
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient
from time import sleep

# Clase para controlar las funciones del robot
class ControlRobot:

    # Constructor de la clase para controlar las funciones del robot
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)

    # Devolvemos una lista de los angulos de los motores
    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    # Movemos los motores del robot a la posicion que le indicamos
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    # Obtenemos la pose del efector final del robot
    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    # Mover el efector final del robot a una pose indicada, devuelve True al finalizar
    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def mover_a_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    # Añadimos un obstaculo, en este caso una caja
    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    # Añadir una caja basada en una configuración
    def add_box_from_config(self, config_name: str, name: str, tamaño: tuple = (.05, .05, .05)) -> None:
        configuraciones = self.cargar_configuraciones()
        if config_name not in configuraciones:
            raise ValueError(f"La configuración '{config_name}' no existe en el archivo.")
        
        # Crear la pose basada en la configuración
        joint_values = configuraciones[config_name]
        pose_caja = Pose()
        pose_caja.position.x = joint_values[0]  # Usa los valores que correspondan
        pose_caja.position.y = joint_values[1]
        pose_caja.position.z = joint_values[2]
        print(pose_caja.position.z)

        # Crear y añadir la caja
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

        print("Objeto Creado...")

    # Añadimos el suelo
    def add_floor(self) -> None:
        print("Añadiendo suelo...")
        pose_suelo = Pose()
        self.add_box_to_planning_scene(pose_suelo, "suelo", (1.3, 1.3,.055)) # Las medidas del suelo son 0,6m x 0.6m y un grosor 5,5cm

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)
    
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()        
        return result.reached_goal
    
    def abrir_pinza(self):
        print("Abriendo la pinza...")
        control.mover_pinza(100.0, 5.0)  # Abrir a 10cm con 5N de fuerza
        sleep(2)

    def cerrar_pinza(self):
        print("Cerrando la pinza...")
        control.mover_pinza(60.0, 5.0)  # Cerrar a 6cm con 5N de fuerza
        sleep(4)
    
    def bajar_pinza(self):
        print("Bajando la pinza...")
        pose_actual = control.pose_actual()
        pose_actual.position.z -= 0.07 # Ajustar a la medida real
        control.move_trajectory([pose_actual])
        sleep(2)

    def subir_pinza(self):
        print("Subiendo la pinza...")
        pose_actual = control.pose_actual()
        pose_actual.position.z += 0.07   # Ajustar a la medida real
        control.move_trajectory([pose_actual])
        sleep(2)

    def cargar_configuraciones(self, archivo: str = "/home/laboratorio/ros_workspace/src/pruebas/configuraciones.yaml") -> dict:
        try:
            with open(archivo, 'r') as file:
                return yaml.safe_load(file)["configuraciones"]
        except Exception as e:
            raise ValueError(f"Error al cargar el archivo de configuraciones: {str(e)}")
    
    def mover_configuracion(self, nombre_config: str, wait: bool = True) -> bool:
        configuraciones = self.cargar_configuraciones()
        if nombre_config not in configuraciones:
            raise ValueError(f"La configuración '{nombre_config}' no existe en el archivo.")
        joint_goal = configuraciones[nombre_config]
        self.move_motors(joint_goal, wait=wait)
        print(f"Moviendo a la configuración '{nombre_config}'")
        return self.move_group.go(joint_goal, wait=wait)

if __name__ == '__main__':
    try:
        # Inicializar el control del robot
        control = ControlRobot()
        
        # Añadimos el suelo
        control.add_floor() # Quedaría por medir como es el suelo en la escena real 

        # Cargar configuraciones desde el archivo YAML
        configuraciones = control.cargar_configuraciones()

        # Lista de configuraciones para mover
        config_names = list(configuraciones.keys())

        # Mover de cada configuración a todas las demás
        for from_config in config_names:
            print(f"\n[INFO] Moviendo desde '{from_config}' hacia las demás configuraciones.\n")
            control.mover_configuracion(from_config)
            for to_config in config_names:
                if from_config != to_config:
                    print(f"[INFO] Moviendo desde '{from_config}' a '{to_config}'")
                    control.mover_configuracion(to_config)
        
        print("\n[INFO] Todas las transiciones entre configuraciones se realizaron correctamente.")
        
    except rospy.ROSInterruptException:
        print("Programa interrumpido")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        print("Programa finalizado")
        