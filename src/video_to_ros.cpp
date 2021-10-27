/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-10-27 11:32:33
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-10-27 16:37:31
 * @Description: 
 */
/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-08 20:53:35
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-08 21:07:14
 * @Description: 
 */
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
    if(argc < 3)
        LOG(FATAL) << "Usage: video_to_ros video_path topic_name";

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;

    ros::init(argc, argv, "read_image_to_ros");
    ros::start();

    string image_path = string(argv[1]);
    string topic_name = string(argv[2]);
    LOG(INFO) << "image_path: " << image_path;
    LOG(INFO) << "topic_name: " << topic_name;
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic_name, 3);
    ros::Rate r(10);
    
    VideoCapture cap(image_path);
    if(!cap.isOpened())
        LOG(FATAL) << image_path << " Cant Opened.";

    while (ros::ok())
    {
        Mat frame;
        if (!cap.read(frame))
        {
            LOG(INFO) << "no video frame.";
            break;
        }

        Mat gray;
        if (frame.channels() == 3)
            cvtColor(frame, gray, CV_RGB2GRAY);
        else
            gray = frame;

        sensor_msgs::ImagePtr p_image_msg;
        p_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", gray).toImageMsg();
        p_image_msg->header.stamp = ros::Time::now();
        pub.publish(p_image_msg);

        LOG_EVERY_N(INFO, 10) << "Image published.";
        r.sleep();
    }

    google::ShutdownGoogleLogging();

    return 0;
}