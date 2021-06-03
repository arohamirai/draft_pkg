/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-03 12:06:17
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-03 13:30:54
 * @Description: 
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

 tf2::Matrix3x3 mat(tf2::Quaternion( -0.002, -0.620,  0.207, -0.757 ));
 double yaw, pitch, roll;
 mat.getEulerYPR(yaw, pitch, roll); // yaw(z) pitch(x) roll(y)

 LOG(INFO)
     << "yaw: " << yaw * 57.3;
 LOG(INFO) << "pitch: " << pitch * 57.3;
 LOG(INFO) << "roll: " << roll * 57.3;

 geometry_msgs::Pose pose;
 pose.orientation.x = -0.002;
 pose.orientation.y = -0.620;
 pose.orientation.z = 0.207;
 pose.orientation.w = -0.757;
 // sequential rotations about the Z, Y and X axes respectively
 tf2::getEulerYPR(pose.orientation, yaw, pitch, roll);

 LOG(INFO)
     << "111yaw: " << yaw*57.3;
 LOG(INFO) << "111pitch: " << pitch * 57.3;
 LOG(INFO) << "111roll: " << roll * 57.3;

 Eigen::Quaterniond q(-0.757, -0.002, -0.620, 0.207);

 LOG(INFO) << "e matrix: \n"
           << q.toRotationMatrix();

 LOG(INFO) << "euler: \n"
           << q.toRotationMatrix().eulerAngles(2,1,0).transpose() * 57.3;

 //tf2::c

//   ros::init(argc, argv, "CLASS_NAME");
//   ros::start();

//   ros::NodeHandle nh;
//   LOG(INFO) << "CLASS_NAME start!";

//   ros::spin();
//   //  ros::MultiThreadedSpinner spinner(2);
//   //  spinner.spin();

//   google::ShutdownGoogleLogging();

  return 0;
}