#!/usr/bin/python3

import sys
import rospy
import yaml
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int16MultiArray, Int16
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib import SimpleActionClient
from typing import List
from time import sleep

class ControlRobot:

    def __init__(self, config_file="/home/laboratorio/ros_workspace/src/proyecto/src/proyecto/positions.yaml") -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        rospy.Subscriber("/movimiento_brazo", Int16, self.consumir_movimiento)

        # Load configurations from YAML
        with open(config_file, "r") as file:
            config = yaml.safe_load(file)

        self.joint_configuration = config["joint_configurations"]["positions"]
        self.base_configuration = config["joint_configurations"]["base_position"]

        rospy.Subscriber("/mueve_carta", Int16MultiArray, self.mover_carta_callback)
        self.pub_success = rospy.Publisher("/estado_movimiento", Int16, queue_size=10)
        self.pub_necesitar_lectura = rospy.Publisher("/necesitamos_lectura", Int16, queue_size=10)
        

        self.creacion_objetos()

    def get_motor_angles(self) -> List[float]:
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal: List[float], wait: bool = True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool = True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (0.1, 0.1, 0.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def moverConfig(self, joint_goal, wait: bool=True) -> bool:
        self.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
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
        self.mover_pinza(100.0, 5.0)  # Abrir a 10cm con 5N de fuerza
        sleep(2)


    def cerrar_pinza(self):
        print("Cerrando la pinza...")
        self.mover_pinza(60.0, 5.0)  # Cerrar a 6cm con 5N de fuerza
        sleep(4)

    def cerrar_pinza_entera(self):
        print("Cerrando la pinza...")
        self.mover_pinza(0.0, 7.0)  # Cerrar a 6cm con 5N de fuerza
        sleep(4)

    def bajar_pinza(self):
        pose_actual = self.pose_actual()
        pose_actual.position.z -= 0.07 # Ajustar a la medida real
        self.move_trajectory([pose_actual])
        sleep(2)

    def consumir_movimiento(self, msg):
        if msg.data == 0:
            self.pedir_carta()
            self.volver_base()
            self.pub_necesitar_lectura.publish(1)
        elif msg.data == 2:
            self.mover_configuracion("BASE")
            self.mover_configuracion("FINISHER")
            self.cerrar_pinza_entera()
        elif msg.data == 3:
            self.abrir_pinza()
            self.bajar_pinza()
            self.cerrar_pinza()
            self.subir_pinza()
            self.volver_base()
        else:
            self.move_trajectory([self.joint_configuration[-1]])

    def mover_configuracion(self, nombre_config: str, wait: bool = True) -> bool:
        configuraciones = self.cargar_configuraciones()
        if nombre_config not in configuraciones:
            raise ValueError(f"La configuración '{nombre_config}' no existe en el archivo.")
        joint_goal = configuraciones[nombre_config]
        self.move_motors(joint_goal, wait=wait)
        print(f"Moviendo a la configuración '{nombre_config}'")
        return self.move_group.go(joint_goal, wait=wait)

    def pedir_carta(self):
        pose_actual = self.pose_actual()
        self.cerrar_pinza_entera()
        pose_actual.position.z -= 0.04 # Bajar 1
        self.move_trajectory([pose_actual])
        pose_actual.position.z += 0.04 # Subir 1
        self.move_trajectory([pose_actual])
        pose_actual.position.z -= 0.04 # Bajar 2
        self.move_trajectory([pose_actual])
        pose_actual.position.z += 0.04 # Subir 2
        self.move_trajectory([pose_actual])
        self.abrir_pinza()
        sleep(2)

    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def subir_pinza(self) -> True:
        print("subiendo pinza")
        pose_actual = self.pose_actual()
        pose_actual.position.z += 0.07   # Ajustar a la medida real
        return self.move_trajectory([pose_actual])
    
    def mover_carta_callback(self, msg: Int16MultiArray):
        index1, index2 = msg.data
        print(f"estes es indice 1: {index1}")
        rospy.loginfo(f"estes es indice 1: {index1}")
        rospy.loginfo(f"este es 2: {index2}")
        self.moverConfig(self.joint_configuration[index1])
        self.abrir_pinza()
        self.bajar_pinza()
        self.cerrar_pinza()
        self.subir_pinza()

        self.moverConfig(self.joint_configuration[index2])
        self.bajar_pinza()
        self.abrir_pinza()
        if self.subir_pinza():
            self.pub_success.publish(1)
        else:
            self.pub_success.publish(0) 

    def volver_base(self):
        self.moverConfig(self.base_configuration)

    def offset_pose(self, z_offset: float) -> Pose:
        pose = self.get_pose()
        pose.position.z += z_offset
        return pose

    def return_to_base(self):
        self.move_motors(self.base_configuration)

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)
    
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
    

    def cargar_configuraciones(self, archivo: str = "/home/laboratorio/ros_workspace/src/proyecto/src/proyecto/configuraciones.yaml") -> dict:
        try:
            with open(archivo, 'r') as file:
                return yaml.safe_load(file)["configuraciones"]
        except Exception as e:
            raise ValueError(f"Error al cargar el archivo de configuraciones: {str(e)}")

    # Añadimos el suelo
    def add_floor(self) -> None:
        print("Añadiendo suelo...")
        pose_suelo = Pose()
        pose_suelo.position.y = 0.75
        self.add_box_to_planning_scene(pose_suelo, "suelo", (1.3, 1.3,.055)) # Las medidas del suelo son 0,6m x 0.6m y un grosor 5,5cm
    
    def creacion_objetos(self):
        # Añadimos el suelo
        self.add_floor() 

        # Añadimos los obtaculos
        self.add_box_from_config("OBJ1", "Caja_1")
        self.add_box_from_config("OBJ2", "Caja_2")
        self.add_box_from_config("OBJ3", "Caja_3")
        self.add_box_from_config("OBJ4", "Caja_4")
        self.add_box_from_config("OBJ5", "Caja_5")
    
    def run(self):
        """
        Mantén el nodo en funcionamiento.
        """
        rospy.spin()

if __name__ == "__main__":
    control_robot = ControlRobot()
    control_robot.run()
