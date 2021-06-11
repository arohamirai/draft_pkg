'''
Author: liufeng(AT iplusbot.com)
Date: 2021-06-09 15:06:04
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-06-11 11:18:04
Description: 
'''
import rospy
from math import pi,sin, cos
import json
import actionlib
from geometry_msgs.msg import Pose2D, Pose
from geometry_msgs.msg import Pose
from visual_servo_msgs.msg import IbvsConstrainedAction, IbvsConstrainedGoal


def send_servo_goal(x,y,yaw):
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    action_client = actionlib.SimpleActionClient('/ibvs_constrained_action', IbvsConstrainedAction)
    action_client.wait_for_server()

    goal2D = Pose2D()
    goal2D.x = 25
    goal2D.y = 1000
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
    goal.vs_type = 1
    goal.camera_type = 1
    goal.tag_type = 1
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0})
    action_client.send_goal(goal)

    print("send goal succeed.")

if __name__ == '__main__':
  send_servo_goal(25,1000, -pi/2)
  print("main finished")
