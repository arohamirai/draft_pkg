### image saver: save image from ros topic every 1 second, image name is time stamp

import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
import sys
import numpy as np

# Instantiate CvBridge
bridge = CvBridge()
image_format = ''
#cnt = 0
def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, 'mono8')
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        time = msg.header.stamp
        sec = time.secs
        nsec = time.nsecs
        #print("cnt", cnt)
	if msg.header.seq%300 == 0:
            cv2.imwrite(''+str(sec) +'.' + str(nsec)+'.bmp', cv2_img)

            height,width=cv2_img.shape
            dst=np.zeros((height,width,1),np.uint8)
            for i in range(height):
                for j in range(width):
                    dst[i,j]=255-cv2_img[i,j]

 
            cv2.imshow("demo", dst)
            cv2.waitKey(1000)
            print("image saved")
            #cnt = 0
        else:
            cv2.imshow("demo", cv2_img)
        cv2.waitKey(2)
        #cnt = cnt+1
        #print("cnt2 ", cnt)
        #rospy.sleep(5)

def main():
    n = len(sys.argv)
    if n < 1:
        print("usage: python image_viewer.py [topic_name]")
        sys.exit()

    rospy.init_node('image_saver')
    # Define your image topic
    image_topic = str(sys.argv[1])
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
