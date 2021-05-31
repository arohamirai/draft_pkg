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
        rospy.sleep(1)

def main():
    n = len(sys.argv)
    if n < 2:
        print("usage: python image_viewer.py [topic_name] [image_format]")
        sys.exit()

    rospy.init_node('image_saver')
    # Define your image topic
    image_topic = str(sys.argv[1])
    image_format = str(sys.argv[2])
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
