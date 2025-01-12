#!/usr/bin/python3

import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient

class ControlRobot:
    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        self.añadir_suelo()

    def articulaciones_actuales(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def mover_articulaciones(self, joint_goal: List[float], wait: bool= True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)
    
    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def pose_a_stamped(self, pose: Pose) -> PoseStamped:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = pose
        return pose_stamped
    
    def mover_a_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def añadir_caja_a_escena_de_planificacion(self, pose_caja: Pose, name: str,
                                  tamaño: tuple = (.1,.1,.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    def mover_trayectoria(self, poses: List[Pose], wait: bool = True) -> bool:
        poses_aux = copy.deepcopy(poses)
        poses_aux.insert(0, self.pose_actual())
            
        (plan, fraction) = self.move_group.compute_cartesian_path(poses_aux, 0.01)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait=wait)

    def añadir_suelo(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.añadir_caja_a_escena_de_planificacion(pose_suelo,"suelo",(2,2,.05))
        
    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        
        return result.reached_goal

if __name__ == '__main__':
    # Crear el objeto de tipo robot
    control = ControlRobot()
    
    pi_medios = pi/2
    # Mover el robot a articulaciones iniciales
    control.mover_articulaciones([0,-pi_medios,-pi_medios,-pi_medios,pi_medios,0])
    
    # Mover el robot a una pose
    pose_actual = control.pose_actual()
    pose_actual.position.z -= 0.1
    control.mover_a_pose(pose_actual)
    
    # Mover el efector final del robot en línea recta a través de varias poses
    poses = [] # Lista de poses que va a recorrer
    
    # Pose 1
    pose_actual = control.pose_actual()
    pose_actual.position.z += 0.1
    poses.append(copy.deepcopy(pose_actual))
    
    # Pose 2
    pose_actual.position.y += 0.1
    poses.append(copy.deepcopy(pose_actual))
    
    # Pose 3
    pose_actual.position.x += 0.1
    poses.append(copy.deepcopy(pose_actual))
    
    # Pose 4
    pose_actual.position.x -= 0.1
    pose_actual.position.y -= 0.1
    pose_actual.position.z -= 0.1
    poses.append(copy.deepcopy(pose_actual))
    
    control.mover_trayectoria(poses)