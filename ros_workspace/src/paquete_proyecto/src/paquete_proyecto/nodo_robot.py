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

class Nodo_robot:
    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.init_pose = self.get_pose()
        self.init_angles = self.get_motor_angles()
        self.configs_dict = {}
        #self.add_floor()
    
    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait = wait)
    
    def get_pose(self)-> Pose:
        return self.move_group.get_current_pose().pose
    
    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def add_box_to_planing_scene(self, name: str, pose_caja: Pose, tamaño: tuple = (.1,.1,.1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)
        
    def add_floor(self)-> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planing_scene("suelo", pose_suelo, (2,2,.05))
    
    def move_trayectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01)

        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait=wait)

    def reset_pos(self, wait: bool = True) -> bool:
        return self.move_to_pose(self.init_pose, wait=wait)
    
if __name__ == '__main__':
    # Creamos el control y suscribimos el nodo al topic de /consignas
    control = Nodo_robot()

    control.add_floor()

    caja_pose = Pose()
    caja_pose.position.x = 0.5
    caja_pose.position.y = 0.5
    caja_pose.position.z = 0.05
    control.add_box_to_planing_scene("caja_1", caja_pose)

    carta_pose = Pose()
    carta_pose.position.x = -0.5
    carta_pose.position.y = 0.5
    carta_pose.position.z = 0.05
    control.add_box_to_planing_scene("carta_1", carta_pose, (0.09, 0.06, 0.02))


    