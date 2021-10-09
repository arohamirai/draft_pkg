'''
Author: liufeng(AT iplusbot.com)
Date: 2021-09-22 15:13:10
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-09-28 20:44:29
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


def send_icp2d_goal():
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
    ref_base2map.position.x = 37.6547270649
    ref_base2map.position.y = -10.0524169978
    ref_base2map.position.z = 0.941849098031
    ref_base2map.orientation.x = -0.00732569520313
    ref_base2map.orientation.y = -0.00290644024797
    ref_base2map.orientation.z = -0.715076066847
    ref_base2map.orientation.w = 0.699002221326

    templateScan_baseInMap = Pose2D()
    templateScan_baseInMap.x = 4.37291685356
    templateScan_baseInMap.y = -2.09988595421
    templateScan_baseInMap.theta = 2.45287052759

    goal = IbvsConstrainedGoal()
    goal.vs_type = 2
    goal.sensor_type = 1
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.ref_base2map = ref_base2map
    goal.templateScan_baseInMap = templateScan_baseInMap
    goal.minCorner = [-1000,-1000]
    goal.maxCorner = [1000,1000]
    goal.useLessScan = False
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0, 
    'icp2d_path':'/home/lile/a923b1d2-4364-46f9-ac67-2e08c90f29cc','sensor_topic':'/scan_fola_nav_top'})
    action_client.send_goal(goal)

if __name__ == '__main__':
  send_icp2d_goal()
  print("main finished")
