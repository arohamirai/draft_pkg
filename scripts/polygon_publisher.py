'''
Author: liufeng(AT iplusbot.com)
Date: 2021-10-11 03:36:06
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-10-11 05:45:32
Description: 
'''
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point
# OpenCV2 for saving an image
import cv2
import sys
import tf



def main():
    rospy.init_node('polygon_publisher')
    # Define your image topic
    # Set up your subscriber and define its callback
    image_pub = rospy.Publisher("footprint",PolygonStamped, queue_size = 3)
    br = tf.TransformBroadcaster()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "car",
                "base_footprint")
        fp = PolygonStamped()
        fp.header.frame_id = "car"
        fp.header.stamp = rospy.Time.now()
        fp.polygon.points.append(Point(-2.13415932655,-0.95083129406, 0))
        fp.polygon.points.append(Point(-2.13415932655,0.95083129406, 0))
        fp.polygon.points.append(Point(2.13415932655,0.95083129406, 0))
        fp.polygon.points.append(Point(2.13415932655,-0.95083129406, 0))
        image_pub.publish(fp)
        r.sleep()

if __name__ == '__main__':
    main()