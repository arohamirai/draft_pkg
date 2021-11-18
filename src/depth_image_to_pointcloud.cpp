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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
using namespace std;
using namespace cv;

ros::Subscriber image_sub;
ros::Publisher pc_pub;
void handleDepthImage(const sensor_msgs::ImageConstPtr &depth_image)
{
 LOG_EVERY_N(INFO, 10) << "depth_image Recieved.";
  cv_bridge::CvImageConstPtr p_depth_image;
  try
  {
    p_depth_image = cv_bridge::toCvShare(depth_image);
  }
  catch (cv_bridge::Exception &e)
  {
    LOG(ERROR) << "Some Error Occurred About Depth Image, Exception Info: "<< string(e.what());
    return;
  }
  cv::Mat depth_mat = p_depth_image->image;

  depth_mat.convertTo(depth_mat, CV_32F);
  depth_mat *= 0.001;

  int nr = depth_mat.rows;
  int nc = depth_mat.cols;
  Eigen::Matrix3f K, K_inv;
  K.matrix() << 474.08123779296875, 0.0, 310.0606689453125, 0.0, 474.08123779296875, 193.083984375, 0.0, 0.0, 1.0; 
  K_inv = K.inverse();

  sensor_msgs::PointCloud2 cloud_in_sensor;
  int point_cnt = 0;
  cloud_in_sensor.header = depth_image->header;
  cloud_in_sensor.height = 1;
  cloud_in_sensor.width = 5;
  sensor_msgs::PointCloud2Modifier modifier(cloud_in_sensor);
  modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32,
                                "ox", 1, sensor_msgs::PointField::FLOAT32,
                                "oy", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(nr * nc);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_in_sensor, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_in_sensor, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_in_sensor, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_ox(cloud_in_sensor, "ox");
  sensor_msgs::PointCloud2Iterator<float> iter_oy(cloud_in_sensor, "oy");
  for (int r = 0; r < nr; r++)
  {
    const float *data = depth_mat.ptr<float>(r);
    for (int c = 0; c < nc; c++)
    {
      const float &depth = data[c];

      Eigen::Vector3f vec;
      vec << depth * c, depth * r, depth;
      Eigen::Vector3f point_in_sensor = K_inv * vec;

      *iter_x = point_in_sensor(0);
      *iter_y = point_in_sensor(1);
      *iter_z = point_in_sensor(2);
      *iter_ox = c;
      *iter_oy = r;

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_ox;
      ++iter_oy;
      point_cnt++;
    }
  }
  modifier.resize(point_cnt);
  cloud_in_sensor.width = point_cnt;
  pc_pub.publish(cloud_in_sensor);
}
int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  ros::init(argc, argv, "depth_image_to_pointcloud");
  ros::start();

  ros::NodeHandle nh;
  image_sub = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 3, handleDepthImage);
  pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/points", 3);

  ros::Rate r(10);
  
ros::spin();

  google::ShutdownGoogleLogging();

  return 0;
}