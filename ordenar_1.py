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

    # Añadimos el suelo
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2,2,.05)) # Editar valores con medidas reales

    # Indicamos una trayectoria al robot, devuelve True al finalizar
    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)

    def moverConfig1(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.6189659277545374, -1.8606702289977015, -0.9071004986763, -1.9074179134764613, 1.5332612991333008, 0.3503589332103729]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfig2(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.11765557924379522, -1.8906337223448695, -0.8680437207221985, -1.9137011967101039, 1.5219460725784302, 0.856809675693512]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)

    def moverConfig3(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [0.6014341115951538, -1.6877361736693324, -1.1214488744735718, -1.8689586124815882, 1.5520339012145996, 1.565879225730896]
        control.move_motors(joint_goal)    
        return self.move_group.go(joint_goal, wait=wait)
    
    def moverConfigBase(self, wait: bool=True) -> bool:
        control = ControlRobot()
        joint_goal = [-0.9107516447650355, -1.1026586157134552, -1.7343271970748901, -1.8417707882323207, 1.570966124534607, 0.02579823136329651]
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
    
    def abrir_pinza(self):
        control = ControlRobot()
        print("Abriendo la pinza...")
        control.mover_pinza(100.0, 5.0)  # Abrir a 10cm con 5N de fuerza
        sleep(2)



    def cerrar_pinza(self):
        control = ControlRobot()
        print("Cerrando la pinza...")
        control.mover_pinza(60.0, 5.0)  # Cerrar a 6cm con 5N de fuerza
        sleep(4)

    
    def bajar_pinza(self):
        pose_actual = control.pose_actual()
        
        pose_actual.position.z -= 0.07 # Ajustar a la medida real
        control.mover_a_pose(pose_actual)
        sleep(2)

    
    def subir_pinza(self):
        print("subiendo pinza")
        pose_actual = control.pose_actual()
        pose_actual.position.z += 0.07   # Ajustar a la medida real
        control.mover_a_pose(pose_actual)
        sleep(2)
    

if __name__ == '__main__':
    try:
        # Inicializar el control del robot
        control = ControlRobot()

        control.add_floor() # Quedaría por medir como es el suelo en la escena real 

        #control.moverConfigBase()
        #control.abrir_pinza()

        control.moverConfigBase()
        control.subir_pinza()

        # Tenemos cartas en config 1 y 2 y usamos config3 como auxiliar
        # Vamos a mover asi:
        # Carta en config 1 --> config 3
        # Carta en config 2 --> config 1
        # Carta en config 3 --> config 2

        control.moverConfig1()
        print(control.get_motor_angles())
        control.abrir_pinza()
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig3()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig2()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig1()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        #control.moverConfigBase()
        #control.subir_pinza()

        control.moverConfig3()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        #control.moverConfigBase()

        control.moverConfig2()
        print(control.get_motor_angles())
        control.bajar_pinza()
        control.abrir_pinza()
        control.subir_pinza()

        control.moverConfigBase()
        print(control.get_motor_angles())

        # TODO
        # Registrar los objetos que hay en la escena, la base, los soportes de las cartas 
        # Coger las configuraciones encima de la carta, y bajar en y a por ella, y luego volver a subir
        # Utilizar un archivo yml para leer las configuraciones
        
    except rospy.ROSInterruptException:
        print("Programa interrumpido")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        print("Programa finalizado")
        
