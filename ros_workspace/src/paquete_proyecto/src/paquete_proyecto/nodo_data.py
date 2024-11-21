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

captura_datos = False
aborted = False
data_list = []

rospy.init_node('joint_state_listener', anonymous=True)

def cb_movements(data: Int16) -> None:
    global captura_datos, aborted
    if data.data == 1:
        captura_datos = True
    elif data.data == 2:
        aborted = True
    else:
        captura_datos = False

def cb_data_capture(msg: JointState) -> None:
    global captura_datos, aborted, data_list
    if captura_datos:
        if not aborted:
            current_joint_state = {
                "name": msg.name,
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort)
            }
            data_list.append(current_joint_state)
    else:
        if len(data_list) > 0:
            # Here, process or upload the data to InfluxDB
            print(data_list)
            data_list = []

## Logic to follow ##
# This node subscribes to two topics:
# - One topic tells the node whether to start or stop capturing data.
# - The other topic provides the position, velocity, and effort of the robot joints (/joint_states).

rospy.sleep(0.5)
sleep_compensado = rospy.Rate(10)  # This allows publishing at 10 Hz

if __name__ == '__main__':
    rospy.Subscriber("/movimientos", Int16, cb_movements)
    rospy.Subscriber("/joint_states", JointState, cb_data_capture)

    while not rospy.is_shutdown():
        sleep_compensado.sleep()
