/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-08 20:53:35
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-08 21:07:14
 * @Description: 
 */
#include <glog/logging.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  ros::init(argc, argv, "read_image_to_ros");
  ros::start();

  string image_path, topic_name;

  if (!ros::param::get("~image_path", image_path))
      LOG(FATAL) << "No image_path Given.";

  if (!ros::param::get("~topic_name", topic_name))
      LOG(FATAL) << "No topic_name Given.";

  Mat image = imread(image_path, CV_LOAD_IMAGE_GRAYSCALE);

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>(topic_name, 3);

  ros::Rate r(10);
  
  while (ros::ok())
  {
      sensor_msgs::ImagePtr p_image_msg;
      p_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
      p_image_msg->header.stamp = ros::Time::now();
      pub.publish(p_image_msg);

      LOG_EVERY_N(INFO, 10) << "Image published.";
      r.sleep();
  }

  google::ShutdownGoogleLogging();

  return 0;
}