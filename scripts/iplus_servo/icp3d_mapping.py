from math import pi,sin, cos
import json
import actionlib
from geometry_msgs.msg import Pose2D, Pose
from geometry_msgs.msg import Pose
from std_msgs.msg import *
from visual_servo_msgs.msg import IbvsConstrainedAction, IbvsConstrainedGoal
from visual_servo_msgs.srv import *
import sys
import rospy

def icp3d_mapping(mode):
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    rospy.wait_for_service('/ibvs_constrained/icp3d_teaching')
    try:
        srv = rospy.ServiceProxy('/ibvs_constrained/icp3d_teaching', visual_servo_msgs.srv.Laser3DTeaching)
        query = visual_servo_msgs.srv.Laser3DTeachingRequest()
        query.mode = mode
        query.configure_json_string = "{\"path\":\"\/home\/lile\"}"
        response = srv(query)
        print("response: " ,response)
        return response
          
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == '__main__':
  if(len(sys.argv)) != 2:
    print("please input mode.(0 for start mpping, 1 for stop mapping)")
  icp3d_mapping(int(sys.argv[1]))
  print("main finished")
