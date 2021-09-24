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
    templateScan_baseInMap.x = 37.6535943106
    templateScan_baseInMap.y = -24.7078520023
    templateScan_baseInMap.theta = -1.57395758173

    goal = IbvsConstrainedGoal()
    goal.vs_type = 2
    goal.sensor_type = 1
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.ref_base2map = ref_base2map
    goal.templateScan_baseInMap = templateScan_baseInMap
    goal.minCorner = [-1000,-1000]
    goal.maxCorner = [1000,1000]
    goal.useLessScan = True
    goal.reconfigure_json_string.data = json.dumps({'speed_level': 2, 'accuracy_level':0, 
    'icp2d_path':'/home/lile/c8d9b2ce-e89e-46b7-b8cf-c8e4fe004d09','sensor_topic':'/scan_front'})
    action_client.send_goal(goal)

if __name__ == '__main__':
  send_icp2d_goal()
  print("main finished")
