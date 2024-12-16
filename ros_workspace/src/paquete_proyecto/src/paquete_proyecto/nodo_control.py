import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, PoseArray 
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16

class ControlRobot:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.pub_state = rospy.Publisher('/movimientos', Int16, queue_size=10)
        
        self.add_floor()

        rospy.Subscriber('/mover_configuracion', JointState, self.mover_brazo_a_configuracion)
        rospy.Subscriber('/trayectoria_cartesiana', PoseArray, self.mover_brazo_en_trayectoria)
        

    def mover_brazo_a_configuracion(self, joint_state):

        rospy.loginfo("Moviendo el robot a la configuración específica recibida.")

        self.pub_state.publish(1)
        joint_goal = joint_state.position
        if self.move_motors(joint_goal):
            rospy.loginfo("El robot ha alcanzado la configuración objetivo.")
            self.pub_state.publish(0)
            print("Publicado un 0")
        else:
            self.pub_state.publish(2)
            rospy.logwarn("No se pudo alcanzar la configuración objetivo.")

    def mover_brazo_en_trayectoria(self, pose_array):
        rospy.loginfo("Moviendo el robot a lo largo de la trayectoria especificada.")
        self.pub_state.publish(1)
        poses = pose_array.poses
        if self.move_trajectory(poses):
            self.pub_state.publish(0)
            rospy.loginfo("El robot ha completado la trayectoria con éxito.")
        else:
            self.pub_state.publish(2)
            rospy.logwarn("No se pudo completar la trayectoria.")

    def get_motor_angles(self):
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal, wait=True):
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self):
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal, wait=True):
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_box_to_planning_scene(self, pose_caja, name, tamaño=(.1, .1, .1)):
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def add_floor(self):
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))

    def move_trajectory(self, poses, wait=True):
        poses.insert(0, self.get_pose())

        print(poses[0])
        (plan, fraction) = self.move_group.compute_cartesian_path(poses,0.005, True)
        
        if fraction != 1.0:
            return False
        
        return self.move_group.execute(plan, wait=wait)
    
if __name__ == '__main__':
    controlador = ControlRobot()
    rospy.spin()
