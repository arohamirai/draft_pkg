#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
using namespace cv;

#define CLASS_NAME ProcessIfm

class CLASS_NAME
{
    public:
    CLASS_NAME(ros::NodeHandle& nh);
    ~CLASS_NAME();


    private:
    void handleImage(const boost::shared_ptr< const sensor_msgs::Image> sensor_msg);

    ros::NodeHandle&nh_;
    ros::Subscriber image_sub_;
    ros::Publisher pc_pub_;
};


CLASS_NAME::CLASS_NAME(ros::NodeHandle&nh)
:nh_(nh)
{
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pallet/camera/cloud_filtered", 3);
    image_sub_ = nh_.subscribe("/pallet/camera/xyz_image", 3, &CLASS_NAME::handleImage, this);
}

CLASS_NAME::~CLASS_NAME()
{

}
void CLASS_NAME::handleImage(const boost::shared_ptr< const sensor_msgs::Image> sensor_msg)
{
    LOG_EVERY_N(INFO, 4) << "Image Callback.";

    cv_bridge::CvImagePtr p_image = cv_bridge::toCvCopy(sensor_msg);

    Mat xyz_image = p_image->image;

    sensor_msgs::PointCloud2 pallet_pc;
    pallet_pc.header = sensor_msg->header;
    pallet_pc.height = 1;
    pallet_pc.width = 1;
    sensor_msgs::PointCloud2Modifier modifier(pallet_pc);

    modifier.setPointCloud2Fields(3, 
    "x", 1, sensor_msgs::PointField::FLOAT32,
    "y", 1, sensor_msgs::PointField::FLOAT32,
    "z", 1, sensor_msgs::PointField::FLOAT32);

    modifier.resize(xyz_image.rows* xyz_image.cols);

    sensor_msgs::PointCloud2Iterator<float> iter_x(pallet_pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pallet_pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pallet_pc, "z");

    int valid_cnt = 0;
    for(int r = 0; r< xyz_image.rows; r++)
        for(int c = 0; c < xyz_image.cols; c++)
        {
            const Vec3f& point = xyz_image.at<Vec3f>(r,c);
            *iter_x = point[0];
            *iter_y = point[1];
            *iter_z = point[2];

            ++iter_x;
            ++iter_y;
            ++iter_z;

            valid_cnt++;
        }
    
    modifier.resize(valid_cnt);
    pallet_pc.width = valid_cnt;

    pc_pub_.publish(pallet_pc);

}



int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;
  FLAGS_logbufsecs = 0;

  ros::init(argc, argv, "CLASS_NAME");
  ros::start();

  ros::NodeHandle nh;
  CLASS_NAME SG(nh);

  LOG(INFO) << "CLASS_NAME start!";

  ros::spin();
  //  ros::MultiThreadedSpinner spinner(2);
  //  spinner.spin();

  google::ShutdownGoogleLogging();

  return 0;
}