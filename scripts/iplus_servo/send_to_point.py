'''
Author: liufeng(AT iplusbot.com)
Date: 2021-06-09 15:06:04
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-09-03 21:19:49
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


def send_servo_goal(x,y,yaw):
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    action_client = actionlib.SimpleActionClient('/ibvs_constrained_action', IbvsConstrainedAction)
    action_client.wait_for_server()

    goal2D = Pose2D()
    goal2D.x = x
    goal2D.y = y
    goal2D.theta = yaw
    
    goal3D = Pose()
    goal3D.position.x = x
    goal3D.position.y = y
    goal3D.position.z = 0
    goal3D.orientation.x = 0
    goal3D.orientation.y = 0
    goal3D.orientation.z = sin(yaw/2)
    goal3D.orientation.w = cos(yaw/2)

    goal = IbvsConstrainedGoal()
    goal.vs_type = 0
    goal.camera_type = 1
    goal.tag_type = 0
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0})
    action_client.send_goal(goal)

    print("send goal succeed.")

if __name__ == '__main__':
  print("input: ", sys.argv)
  if(len(sys.argv)) != 4:
    print("please input x, y, yaw.")
    exit(0)
  x = float(sys.argv[1])
  y = float(sys.argv[2])
  yaw = float(sys.argv[3])
  send_servo_goal(x,y,yaw)
  print("main finished")
