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
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import time
from datetime import datetime, timezone

captura_datos = False
aborted = False
data_list = []

client = InfluxDBClient(
            url='http://host.docker.internal:8086/',
            token='h7lViZblbyLF4kG2kuXvFZlqwqxmIi-GRsBLt2kWT0Lq4y7iC6cTOFOZrpTLxjxRC_RfllJEhQ77XoaGxFwbEQ==',
            org="mucsi")


write_api = client.write_api(write_options=SYNCHRONOUS)

for i in range(1_000):
    punto=Point("dummy_measure").field("dummy_point", i).time(time.time_ns())
    write_api.write("bucket_1","mucsi",punto)
    time.sleep(0.1)