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

# InfluxDB configuration
INFLUXDB_URL = "http://localhost:8086"  # Replace with your InfluxDB URL
INFLUXDB_TOKEN = "your_influxdb_token"  # Replace with your InfluxDB token
INFLUXDB_ORG = "your_organization"  # Replace with your organization name
INFLUXDB_BUCKET = "robot_joint_data"  # Replace with your bucket name

# Initialize InfluxDB client
#client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
#write_api = client.write_api(write_options=SYNCHRONOUS)


def cb_movements(data: Int16) -> None:
    global captura_datos, aborted
    print(data)
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
            timestamp = msg.header.stamp.secs  # Use the timestamp from the message header
            # Ensure velocity and effort have the same length as name and position
            velocity = msg.velocity if msg.velocity else [0.0] * len(msg.name)
            effort = msg.effort if msg.effort else [0.0] * len(msg.name)
            joint_data = []
            for joint_name, position, velocity, effort in zip(
                msg.name, msg.position, velocity, effort
            ):
                joint_data.append({
                    "measurement": "joint_state",
                    "tags": {"joint_name": joint_name},
                    "fields": {
                        "position": position,
                        "velocity": velocity if velocity else 0.0,
                        "effort": effort if effort else 0.0,
                    },
                    "time": timestamp
                })
                #print(joint_data)
            data_list.extend(joint_data)

            """
            current_joint_state = {
                "name": msg.name,
                "position": list(msg.position),
                "velocity": list(msg.velocity),
                "effort": list(msg.effort),
                "timestamp": msg.header.stamp.secs
            }
            data_list.append(current_joint_state)
            """
    else:
        if len(data_list) > 0:
            # Here, process or upload the data to InfluxDB
            #write_api.write(bucket=INFLUXDB_BUCKET, record=data_list)
            print(f"Uploaded {len(data_list)} data points to InfluxDB.")
            print(data_list)
            data_list = []

## Logic to follow ##
# This node subscribes to two topics:
# - One topic tells the node whether to start or stop capturing data.
# - The other topic provides the position, velocity, and effort of the robot joints (/joint_states).

rospy.init_node('joint_state_listener', anonymous=True)

rospy.sleep(0.5)
sleep_compensado = rospy.Rate(10)  # This allows publishing at 10 Hz

if __name__ == '__main__':
    rospy.Subscriber("/movimientos", Int16, cb_movements)
    rospy.Subscriber("/joint_states", JointState, cb_data_capture)

    while not rospy.is_shutdown():
        sleep_compensado.sleep()
