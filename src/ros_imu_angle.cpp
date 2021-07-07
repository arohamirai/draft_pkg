/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-07-07 15:07:27
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-07-07 15:25:00
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
#include <sensor_msgs/Imu.h>

void handleImu(const sensor_msgs::ImuConstPtr &imu)
{
    float gx, gy, gz;
    gx = imu->linear_acceleration.x;
    gy = imu->linear_acceleration.y;
    gz = imu->linear_acceleration.z;

    float theta = std::atan2(gz,gy);

    LOG(INFO) << "theta: " << theta * 57.3 << " degree.";
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;
    
      ros::init(argc, argv, "imu_test");
      ros::start();

      ros::NodeHandle nh;
      ros::Subscriber imu_sub;
      imu_sub = nh.subscribe<sensor_msgs::Imu>("/front/accel/sample", 4, &handleImu);

      ros::spin();

      //   ros::NodeHandle nh;
      //   LOG(INFO) << "CLASS_NAME start!";

      //   ros::spin();
      //   //  ros::MultiThreadedSpinner spinner(2);
      //   //  spinner.spin();

      //   google::ShutdownGoogleLogging();

      return 0;
}