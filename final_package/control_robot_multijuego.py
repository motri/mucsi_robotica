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

    def __init__(self, config_file="/home/laboratorio/ros_workspace/src/final_package/positions.yaml") -> None:
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
            self.pedir_carta
        else:
            self.move_trajectory([self.joint_configuration[-1]])


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
    
    def run(self):
        """
        Mantén el nodo en funcionamiento.
        """
        rospy.spin()

if __name__ == "__main__":
    control_robot = ControlRobot()
    control_robot.run()
