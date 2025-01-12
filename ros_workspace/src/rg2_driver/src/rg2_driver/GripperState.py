#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import rtde_receive
import sys
from typing import Tuple
from math import asin

class GripperJointStates():

    def __init__(self, robot_ip: str) -> None:
        if not rospy.core.is_initialized():
            nodo = rospy.init_node("rg2_joint_state_publisher", anonymous=True)
        self.robot_ip = robot_ip
        self.rtde_receive_ = rtde_receive.RTDEReceiveInterface(robot_ip, use_upper_range_registers = True)
        self.state_publisher  = rospy.Publisher("/rg2/joint_states", JointState, queue_size=10)
        self.grip_publisher  = rospy.Publisher("/rg2/grip_detected", Bool, queue_size=10)
        self.joints = JointState()
        self.joints.name = ["finger_joint","left_inner_knuckle_joint","left_inner_finger_joint",
                            "right_outer_knuckle_joint","right_inner_knuckle_joint","right_inner_finger_joint"]
        self.pub_rate = rospy.Rate(60)

    def check_gripper_state(self) -> Tuple[float, int, int]:
        width = self.rtde_receive_.getOutputDoubleRegister(36)
        busy = self.rtde_receive_.getOutputIntRegister(36)
        grip_detected = self.rtde_receive_.getOutputIntRegister(37)
        return [width, busy, True if grip_detected else False]
    
    def width_to_rad(self, width: float) -> float:
        return 0.855211333 - asin((7.720897365417915 + width) / 110.6835961023352)
    
    def start(self) -> None:
        while not rospy.is_shutdown():
            width, busy, grip_detected = self.check_gripper_state()
            rads = self.width_to_rad(width)
            self.joints.position = [rads, -rads, rads, -rads, -rads, rads]
            self.state_publisher.publish(self.joints)
            self.grip_publisher.publish(Bool(data=grip_detected))
            self.pub_rate.sleep()

if __name__ == "__main__":
    robot_ip = sys.argv[1]
    gripper_joint_state_publisher = GripperJointStates(robot_ip)
    gripper_joint_state_publisher.start()