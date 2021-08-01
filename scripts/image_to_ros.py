'''
Author: liufeng(AT iplusbot.com)
Date: 2021-07-31 23:12:57
LastEditors: liufeng(AT iplusbot.com)
LastEditTime: 2021-07-31 23:34:48
Description: 
'''
### image saver: save image from ros topic every 1 second, image name is time stamp

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys

# Instantiate CvBridge
bridge = CvBridge()
image_format = ''
def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, image_format)
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        time = msg.header.stamp
        sec = time.secs
        nsec = time.nsecs
        cv2.imwrite(''+str(sec) +'.' + str(nsec)+'.bmp', cv2_img)
        print("image saved")
        #rospy.sleep(5)

def main():
    n = len(sys.argv)
    if n < 2:
        print("usage: python image_viewer.py [image_path] [image_topic]")
        sys.exit()

    rospy.init_node('image_to_ros')
    # Define your image topic
    image_path = str(sys.argv[1])
    image_topic = str(sys.argv[2])
    # Set up your subscriber and define its callback
    image_pub = rospy.Publisher(image_topic,Image, queue_size = 3)
    cv2_img = cv2.imread(image_path)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(cv2_img, encoding="passthrough")
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        image_pub.publish(image_message)
        print(image_path + " published in topic: " + image_topic)
        r.sleep()

if __name__ == '__main__':
    main()
