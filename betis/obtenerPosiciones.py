# Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient
from time import sleep
import yaml

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
        self.add_floor()

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

    # Añadimos un obstaculo, en este caso una caja
    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    # Añadimos el suelo
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2,2,.05))

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)

    def moverConfig1(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.4869278112994593, -1.858640810052389, -1.228166103363037, -1.6406942806639613, 1.5973241329193115, 0.5287708640098572]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig2(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.23798686662782842, -2.1974054775633753, -0.5155571699142456, -1.978826185266012, 1.5638964176177979, 0.7119466066360474]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)

    def moverConfig3(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.2328263819217682, -2.180206914941305, -0.42928189039230347, -2.1217647991576136, 1.6015007495880127, 1.05226731300354]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfigBase(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.28942328691482544, -1.535173923974373, -0.9331789016723633, -2.233608385125631, 1.5858209133148193, 1.2375750541687012]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()        
        return result.reached_goal
    


    def cargar_configuraciones(self, archivo):
        with open(archivo, 'r') as file:
            return yaml.safe_load(file)["configuraciones"]

    def mover_a_configuracion(self, config_name, wait=True):
        if config_name not in self.configuraciones:
            raise ValueError(f"La configuración '{config_name}' no existe.")
        joint_goal = self.configuraciones[config_name]
        self.control.move_motors(joint_goal)
        return self.move_group.go(joint_goal, wait=wait)

if __name__ == '__main__':
    try:
        # Inicializar el control del robot
        control = ControlRobot()
        print(control.get_motor_angles())
        print(control.get_pose())

    except rospy.ROSInterruptException:
        print("Programa interrumpido")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        print("Programa finalizado")
        