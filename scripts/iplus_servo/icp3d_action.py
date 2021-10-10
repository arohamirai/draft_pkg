'''
Author: liufeng(AT iplusbot.com)
Date: 2021-09-22 15:16:05
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-10-11 05:50:21
Description: 
'''
import rospy
from math import pi,sin, cos
import json
import actionlib
from geometry_msgs.msg import Pose2D, Pose
from geometry_msgs.msg import Pose
from std_msgs.msg import *
from visual_servo_msgs.msg import IbvsConstrainedAction, IbvsConstrainedGoal
from visual_servo_msgs.srv import *


def send_icp3d_goal():
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    action_client = actionlib.SimpleActionClient('/ibvs_constrained_action', IbvsConstrainedAction)
    action_client.wait_for_server()

    goal2D = Pose2D()
    goal2D.x = 0
    goal2D.y = 0
    goal2D.theta = 0
    
    goal3D = Pose()
    goal3D.position.x = 0
    goal3D.position.y = 0
    goal3D.position.z = 0
    goal3D.orientation.x = 0
    goal3D.orientation.y = 0
    goal3D.orientation.z = 0
    goal3D.orientation.w = 1

    ref_base2map = Pose()
    ref_base2map.position.x = 37.6545202908
    ref_base2map.position.y = -24.0134673885
    ref_base2map.position.z = 1.07322036222
    ref_base2map.orientation.x = -0.0146289449714
    ref_base2map.orientation.y = -0.0177548769163
    ref_base2map.orientation.z = -0.705216820364
    ref_base2map.orientation.w = 0.70861837020

    goal = IbvsConstrainedGoal()
    goal.vs_type = 2
    goal.sensor_type = 6
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.ref_base2map = ref_base2map
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0, 'ref_pointCloud2_path':'/mnt/hgfs/sharefolder/dataset/sanyou_icp_big_error_20210918/5f8dadd8-8db2-46ae-97f4-68454bc17087'})
    action_client.send_goal(goal)

if __name__ == '__main__':
  send_icp3d_goal()
  print("main finished")
