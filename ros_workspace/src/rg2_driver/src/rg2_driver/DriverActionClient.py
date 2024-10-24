#! /usr/bin/env python

import rospy
#import roslib
#roslib.load_manifest('driver_rg2')
import actionlib
from driver_rg2.msg import gripGoal, gripFeedback, gripResult, gripAction


def RG2_mover_pinza(width:float,force:float,depth_compensation:bool) -> gripResult():

    cliente = actionlib.SimpleActionClient("rg2_action_server",gripAction)
    
    cliente.wait_for_server()

    goal = gripGoal()
    goal.width = width
    goal.force = force
    goal.depth_compensation = depth_compensation

    cliente.send_goal(goal)

    cliente.wait_for_result(rospy.Duration.from_sec(10.0))
    print(cliente)
    return cliente.get_result()



if __name__ == "__main__":
    rospy.init_node("action_client")
    resultado = RG2_mover_pinza(width=50.0,force=40.0,depth_compensation=False)
    print(resultado)



#! /usr/bin/env python

#import roslib
#roslib.load_manifest('my_pkg_name')
#import rospy
#import actionlib
#
#from driver_rg2.msg import gripGoal, gripFeedback, gripResult, gripAction
#
#if __name__ == '__main__':
#    rospy.init_node('action_client')
#    client = actionlib.SimpleActionClient('rg2_action_server', gripAction)
#    client.wait_for_server()
#
#    goal = gripGoal()
#    # Fill in the goal here
#    client.send_goal(goal)
#    client.wait_for_result(rospy.Duration.from_sec(5.0))