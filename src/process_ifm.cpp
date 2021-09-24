#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
using namespace std;
using namespace cv;

#define CLASS_NAME ProcessIfm

class CLASS_NAME
{
    public:
    CLASS_NAME(ros::NodeHandle& nh);
    ~CLASS_NAME();


    private:
    void handleImage(const boost::shared_ptr< const sensor_msgs::Image> sensor_msg);
    void handlePointCloud2(const boost::shared_ptr< const sensor_msgs::PointCloud2> sensor_msg);


    ros::NodeHandle&nh_;
    ros::Subscriber image_sub_, pc_sub_;
    ros::Publisher pc_pub_;
};


CLASS_NAME::CLASS_NAME(ros::NodeHandle&nh)
:nh_(nh)
{
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pallet/camera/cloud_filtered", 3);
    image_sub_ = nh_.subscribe("/pallet/camera/xyz_image", 3, &CLASS_NAME::handleImage, this);
    pc_sub_ = nh_.subscribe("/pointcloud_front", 3, &CLASS_NAME::handlePointCloud2, this);
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
  void CLASS_NAME::handlePointCloud2(const boost::shared_ptr< const sensor_msgs::PointCloud2> sensor_msg)
  {
      LOG_EVERY_N(INFO,10) << "handlePointCloud2";
    sensor_msgs::PointCloud2 pc_in = *sensor_msg; 
    std::map<string, sensor_msgs::PointField> pc_in_field_info, pc_out_field_info;
    for (auto it(pc_in.fields.begin()); it != pc_in.fields.end(); ++it)
    {
      pc_in_field_info[it->name] = *it;
    }
    if(pc_in_field_info.find("intensity") == pc_in_field_info.end())
    {
      LOG(ERROR) << "No intensity Chanel.";
      //return pc_in;
      return;
    }
  sensor_msgs::PointCloud2 pc_out;
  sensor_msgs::PointCloud2Modifier modifier(pc_out);
  int pc_out_offset = 0;
  for(auto field:pc_in_field_info)
  {
    sensor_msgs::PointField point_field;
    point_field.name = field.first;
    point_field.offset = pc_out_offset;
    point_field.datatype = sensor_msgs::PointField::FLOAT32;
    point_field.count = 1;
    pc_out_field_info[field.first] = point_field;
    pc_out_offset =  addPointField(pc_out, field.first, 1, sensor_msgs::PointField::FLOAT32, pc_out_offset);
    pc_out.point_step += pc_out_offset;
  }

  modifier.resize(pc_in.height * pc_in.width);
  int valid_cnt = 0;
    for (size_t row(0); row < pc_in.height; ++row)
    {
        const uint8_t *data_in_ptr(&pc_in.data[0] + pc_in.row_step * row); 
        for (size_t col(0); col < pc_in.width; ++col, data_in_ptr += pc_in.point_step)
        {
          uint8_t *data_out_ptr = &pc_out.data[0] + pc_out.row_step * row+valid_cnt* pc_out.point_step;
          int pc_in_intensity = 1;

          sensor_msgs::PointField pc_in_intensity_field = pc_in_field_info["intensity"];
           switch(pc_in_intensity_field.datatype)
            {
              case sensor_msgs::PointField::INT8:
                pc_in_intensity  = *reinterpret_cast<const int8_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::UINT8:
                pc_in_intensity  = *reinterpret_cast<const uint8_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::INT16:
                pc_in_intensity  = *reinterpret_cast<const int16_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::UINT16:
                pc_in_intensity  = *reinterpret_cast<const uint16_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::INT32:
                pc_in_intensity  = *reinterpret_cast<const int32_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::UINT32:
                pc_in_intensity  = *reinterpret_cast<const uint32_t *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::FLOAT32:
                pc_in_intensity  = *reinterpret_cast<const float *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
                case sensor_msgs::PointField::FLOAT64:
                pc_in_intensity  = *reinterpret_cast<const double *>(data_in_ptr + pc_in_intensity_field.offset);
                break;
            }

            if(pc_in_intensity < 100)
            continue;
            //LOG(INFO) << "pc_in_intensity: " << pc_in_intensity;
          for(auto it:pc_in_field_info)
          {
            string field_name = it.first;
            sensor_msgs::PointField pc_in_field = it.second;

            switch(it.second.datatype)
            {
                
              case sensor_msgs::PointField::INT8:
                *(reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset))  = *reinterpret_cast<const int8_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::UINT8:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const uint8_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::INT16:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const int16_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::UINT16:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const uint16_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::INT32:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const int32_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::UINT32:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const uint32_t *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::FLOAT32:
                *reinterpret_cast< float *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const float *>(data_in_ptr + pc_in_field.offset);
                break;
                case sensor_msgs::PointField::FLOAT64:
                *reinterpret_cast< double *>(data_out_ptr +pc_out_field_info[field_name].offset)  = *reinterpret_cast<const double *>(data_in_ptr + pc_in_field.offset);
                break;
            }
          }
          valid_cnt++;
        }
    }
    modifier.resize(valid_cnt);
    pc_out.width = valid_cnt;
    pc_out.height = 1;
    pc_out.header = pc_in.header;
    pc_pub_.publish(pc_out);
  //return pc_out;
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