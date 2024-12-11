
# Imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos, sin
from std_msgs.msg import String, Int32MultiArray
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

        self.joint_configuration = [[0.5408084988594055, -1.969069620172018, -0.690409779548645, -2.046171327630514, 1.5728977918624878, 1.482495903968811],
                                [0.15190760791301727, -2.0658303699889125, -0.5416991710662842, -2.0914517841734828, 1.5743842124938965, 1.090315580368042],
                                [-0.22816354433168584, -2.147924085656637, -0.3972216546535492, -2.1656438312926234, 1.5727119445800781, 0.7838241457939148],
                                [-0.614532772694723, -1.8291450939574183, -0.8850789070129395, -1.9965487919249476, 1.573143720626831, 0.3192458152770996],
                                [0.06626737117767334, -1.4222855877927323, -1.3193367719650269, -1.9700690708556117, 1.573316216468811, 0.988867998123169],
                                [-0.47379905382265264, -1.639226575891012, -1.761652946472168, -1.3131235402873536, 4.715549945831299, -6.207685377691881]]
    
        self.base_configuration = [-1.8235538641559046, -1.505070039337017, -0.7248993515968323, -2.4801069698729457, 1.5716415643692017, 0.05282888934016228]

        self.move_successful = False

        rospy.Subscriber("/moveCard", Int32MultiArray, self.mover_carta)

        self.pub_success = rospy.Publisher("/moveStatus", int, queue_size=10)

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
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, True)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait = wait)
    
    
    def moverConfig(self, joint_goal, wait: bool=True, ) -> bool:
        control = ControlRobot()
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
        control.move_trajectory([pose_actual])
        sleep(2)

    
    def subir_pinza(self) -> True:
        print("subiendo pinza")
        pose_actual = control.pose_actual()
        pose_actual.position.z += 0.07   # Ajustar a la medida real
        return control.move_trajectory([pose_actual])
        

    def mover_carta(self, positions):
        index1 = positions[0]
        index2 = positions[1]
        control.moverConfig(self.joint_configuration[index1])
        control.abrir_pinza()
        control.bajar_pinza()
        control.cerrar_pinza()
        control.subir_pinza()

        control.moverConfig(self.joint_configuration[index2])
        control.bajar_pinza()
        control.abrir_pinza()
        if control.subir_pinza():
            self.pub_success(1)
        else:
            self.pub_success(0) 

    def volver_base(self):
        control.moverConfig(self.base_configuration)

    def finisher(self):
        control.moverConfig(self.joint_configuration[-1])
    

if __name__ == '__main__':
    control = ControlRobot()
    rospy.spin()
        
