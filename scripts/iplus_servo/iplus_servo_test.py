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

def icp3d_teaching():
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    rospy.wait_for_service('/ibvs_constrained/icp3d_teaching')
    try:
        srv = rospy.ServiceProxy('/ibvs_constrained/icp3d_teaching', visual_servo_msgs.srv.Laser3DTeaching)
        query = visual_servo_msgs.srv.Laser3DTeachingRequest()
        query.mode = 2
        query.configure_json_string = "{\"ref_pointCloud2_path\":\"\/home\/lile\/bd5aadff-7d5d-405b-bbe3-8aef00e08a2a\", \"ref_base2map\":{\"orientation\":{\"w\":0.698992,\"x\":-0.007326,\"y\":-0.0029066,\"z\":-0.715086},\"position\":{\"x\":37.6556,\"y\":-10.052,\"z\":0.941893}}}"
        response = srv(query)
        print("response: " ,response)
        return response

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
  icp3d_teaching()
  print("main finished")
