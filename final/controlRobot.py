import sys
import rospy
import yaml
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int32MultiArray
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib import SimpleActionClient
from typing import List
from time import sleep

class ControlRobot:

    def __init__(self, config_file="robot_config.yaml") -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)

        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)

        # Load configurations from YAML
        with open(config_file, "r") as file:
            config = yaml.safe_load(file)

        self.joint_configuration = config["joint_configurations"]["positions"]
        self.base_configuration = config["joint_configurations"]["base_position"]

        rospy.Subscriber("/moveCard", Int32MultiArray, self.mover_carta_callback)
        self.pub_success = rospy.Publisher("/moveStatus", Int32MultiArray, queue_size=10)

        self.add_floor()

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

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "floor", (2, 2, 0.05))

    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        poses.insert(0, self.get_pose())
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01, 0.0)
        if fraction < 1.0:
            rospy.logwarn("Trajectory planning incomplete.")
            return False
        return self.move_group.execute(plan, wait=wait)

    def control_gripper(self, position: float, effort: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = effort
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        return result.reached_goal

    def open_gripper(self) -> bool:
        return self.control_gripper(0.1, 5.0)

    def close_gripper(self) -> bool:
        return self.control_gripper(0.06, 5.0)

    def mover_carta_callback(self, msg: Int32MultiArray):
        index1, index2 = msg.data

        self.move_motors(self.joint_configuration[index1])
        self.open_gripper()
        self.move_trajectory([self.offset_pose(-0.07)])
        self.close_gripper()
        self.move_trajectory([self.offset_pose(0.07)])

        self.move_motors(self.joint_configuration[index2])
        self.move_trajectory([self.offset_pose(-0.07)])
        self.open_gripper()

        success = self.move_trajectory([self.offset_pose(0.07)])
        self.pub_success.publish(Int32MultiArray(data=[1 if success else 0]))

    def offset_pose(self, z_offset: float) -> Pose:
        pose = self.get_pose()
        pose.position.z += z_offset
        return pose

    def return_to_base(self):
        self.move_motors(self.base_configuration)

if __name__ == "__main__":
    control_robot = ControlRobot()
    rospy.spin()
