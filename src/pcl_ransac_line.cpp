/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-01-23 14:55:04
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-05-31 10:09:27
 * @Description: 
 */
#include <boost/make_shared.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include "pcl/registration/correspondence_estimation_normal_shooting.h"
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/icp.h>

#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <Eigen/Core>
using namespace std;

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

  pc->points.push_back(pcl::PointXYZ(000.0460477, -0.00233957, -0.00504596));
  pc->points.push_back(pcl::PointXYZ(000.0917824, -0.00936998, -0.00716816));
  pc->points.push_back(pcl::PointXYZ(000.140165, -0.0117681, -0.0194017));
  pc->points.push_back(pcl::PointXYZ(000.185713, -0.0137506, -0.0233291));
  pc->points.push_back(pcl::PointXYZ(000.232771 ,-0.0159638, -0.0277897));
  pc->points.push_back(pcl::PointXYZ(000.281773, -0.0169641, -0.0316322));
  pc->points.push_back(pcl::PointXYZ(000.332069, -0.0158006, -0.0367756));
  pc->points.push_back(pcl::PointXYZ(000.381951, -0.0165671, -0.0313778));
  pc->points.push_back(pcl::PointXYZ(00.434708, -0.018548, -0.02953));
  pc->points.push_back(pcl::PointXYZ(000.482808, -0.0162605, -0.0201966));

  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
  model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pc));

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
  ransac.setDistanceThreshold(0.005);
  ransac.computeModel();

  Eigen::VectorXf model_coefficients;
  ransac.getModelCoefficients(model_coefficients);

  std::vector<int> inliers;
  ransac.getInliers(inliers);

  for(auto index: inliers)
  std::cout << index << std::endl;


  pcl::visualization::PCLVisualizer viewer("ICP demo");
  int v1(0);
  viewer.createViewPort(0.0, 0.0, 1, 1.0, v1);

  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(pc, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
                                                                                   (int)255 * txt_gray_lvl);
  viewer.addPointCloud(pc, cloud_in_color_h, "cloud_in_v1", v1);

  pcl::ModelCoefficients line_coeff;
  for(int i = 0; i < 6; i++)
  line_coeff.values.push_back(model_coefficients[i]);
  viewer.addLine(line_coeff);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024); // Visualiser window size

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }



}