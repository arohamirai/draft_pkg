'''
Author: liufeng(AT iplusbot.com)
Date: 2021-06-09 15:06:04
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-08-23 17:34:46
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

def icp3d_teaching(mode):
  rospy.init_node('ibvs_constrained_test_node', anonymous=False)
  rospy.wait_for_service('/ibvs_constrained/icp3d_teaching')
  try:
      srv = rospy.ServiceProxy('/ibvs_constrained/icp3d_teaching', visual_servo_msgs.srv.Laser3DTeaching)
      query = visual_servo_msgs.srv.Laser3DTeachingRequest()
      query.mode = mode
      query.configure_json_string = "{\"path\":\"\/home\/lile\"}"
      response = srv(query)
      print("response: " ,response)
      
  except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

def send_icp_goal():
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

    goal = IbvsConstrainedGoal()
    goal.vs_type = 2
    goal.sensor_type = 6
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.ref_base2map = ref_base2map
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0, 'ref_pointCloud2_path':'/home/lile/abc'})
    action_client.send_goal(goal)

if __name__ == '__main__':
  #send_servo_goal(25,1000, -pi/2)
  #icp3d_teaching(1)
  send_icp_goal()
  print("main finished")
