'''
Author: liufeng(AT iplusbot.com)
Date: 2021-09-28 19:16:40
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-09-28 20:38:53
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
from sensor_msgs.msg import *


def icp2d_teach_and_action():
    rospy.init_node('ibvs_constrained_test_node', anonymous=False)
    rospy.wait_for_service('/laser_scan_matcher_server')
    action_client = actionlib.SimpleActionClient('/ibvs_constrained_action', IbvsConstrainedAction)
    action_client.wait_for_server()
    try:
        srv = rospy.ServiceProxy('/laser_scan_matcher_server', visual_servo_msgs.srv.LaserScanMatcher)
        query = visual_servo_msgs.srv.LaserScanMatcherRequest()
        query.sensor_topic= '/scan_fola_nav_top'
        query.useLessScan = False
        query.isTeaching = True
        query.minCorner = [-1000,-1000]
        query.maxCorner = [1000,1000]
        query.configure_json_string = "{\"path\":\"\/home\/lile\"}"
        response = srv(query)
        print("response: " ,response.baseInMap)


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

        goal = IbvsConstrainedGoal()
        goal.vs_type = 2
        goal.sensor_type = 1
        goal.targetInWorld = goal2D
        goal.targetInWorld_3d = goal3D
        goal.templateScan_baseInMap = response.baseInMap
        goal.minCorner = [-1000,-1000]
        goal.maxCorner = [1000,1000]
        goal.useLessScan = False
        reconfigure_json_string = json.loads(response.configure_json_string)
        reconfigure_json_string['speed_level'] = 2
        reconfigure_json_string['accuracy_level'] = 0
        reconfigure_json_string['sensor_topic'] = query.sensor_topic
        reconfigure_json_string['icp2d_path'] = '/home/lile/' + reconfigure_json_string['scan2d_id']

        goal.reconfigure_json_string.data = json.dumps(reconfigure_json_string)
        action_client.send_goal(goal)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    pass

if __name__ == '__main__':
  icp2d_teach_and_action()
  print("main finished")
