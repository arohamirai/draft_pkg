from math import pi,sin, cos
import json
import actionlib
from geometry_msgs.msg import Pose2D, Pose
from geometry_msgs.msg import Pose
from std_msgs.msg import *
from visual_servo_msgs.msg import IbvsConstrainedAction, IbvsConstrainedGoal
from visual_servo_msgs.srv import *

def icp3d_teach_and_action():
    icp3d_mapping(0)
    rospy.sleep(20)
    response = icp3d_mapping(1)
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


    goal = IbvsConstrainedGoal()
    goal.vs_type = 2
    goal.sensor_type = 6
    goal.targetInWorld = goal2D
    goal.targetInWorld_3d = goal3D
    goal.ref_base2map = response.ref_base2map

    reconfigure_json_string = dict()
    reconfigure_json_string['speed_level'] = 2
    reconfigure_json_string['accuracy_level'] = 0
    reconfigure_json_string['ref_pointCloud2_path'] = '/home/lile/' + response.ref_pointCloud2_id

    goal.reconfigure_json_string.data = json.dumps(reconfigure_json_string)
    action_client.send_goal(goal)

if __name__ == '__main__':
  icp3d_teach_and_action()
  print("main finished")
