#!/usr/bin/env python3

import rospy
import actionlib
from subprocess import getstatusoutput
from typing import List, Tuple
from time import sleep, time
from GripperState import GripperJointStates
from control_msgs.msg import GripperCommandGoal, GripperCommandResult, GripperCommandFeedback, GripperCommandAction
import sys

class RG2ActionServer():
     
    def __init__(self, robot_ip: str) -> None:
        self.nodo_server = rospy.init_node("rg2_control_node", anonymous=True)
        
        self.gripper_action_server = actionlib.SimpleActionServer("rg2_action_server", GripperCommandAction, execute_cb=self.__execute_callback,auto_start=False)
        self.feedback_publish_rate = rospy.Rate(60)
        self.robot_ip=robot_ip
        self.griper_state = GripperJointStates(self.robot_ip)

    def mover_pinza(self,width: float,force: float,depth_compensation: bool=None) -> Tuple[int, str]:
        if width < 0.0 or width > 100.0:
            return ()
        
        if depth_compensation is not None:
            return NotImplementedError()
        
        return getstatusoutput(f"xmlrpc http://{self.robot_ip}:41414 rg_grip i/0 d/{width} d/{force}")
        
    def __execute_callback(self, goal: GripperCommandGoal) -> GripperCommandResult():        
        self.mover_pinza(width = goal.command.position, force = goal.command.max_effort)
        feedback = GripperCommandFeedback()
        
        # Bucle que acaba cuando acabe el grip
        init = time()
        timeout = 10
        sleep(0.1)

        while True:
            feedback.position, busy, feedback.stalled = self.griper_state.check_gripper_state()
            feedback.effort = goal.command.max_effort
            feedback.reached_goal = True if not busy else  False

            self.gripper_action_server.publish_feedback(feedback)
            if  (feedback.reached_goal==1 or time()-init>timeout):
                break
            self.feedback_publish_rate.sleep()
        

        result = GripperCommandResult()
        result.effort = goal.command.max_effort
        result.stalled = bool(feedback.stalled)
        result.position = feedback.position

        if abs(goal.command.position - feedback.position)< 2.0 or feedback.stalled :
            result.reached_goal = True
            self.gripper_action_server.set_succeeded(result, text="Objetivo alcanzado")
        else:
            result.reached_goal = False
            self.gripper_action_server.set_aborted(result, text="Objetivo NO alcanzado")

    def start_server(self) -> None:
        self.gripper_action_server.start()
        rospy.spin()

if __name__ == "__main__":
    robot_ip = sys.argv[1]

    s = RG2ActionServer(robot_ip)
    s.start_server()
