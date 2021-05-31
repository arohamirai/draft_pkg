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
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

void point_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &src_pc,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &tgt_pc)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr src(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*src_pc, *src);
  pcl::PointCloud<pcl::PointNormal>::Ptr tgt(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud(*tgt_pc, *tgt);

  // pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_src;
  // norm_src.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
  // norm_src.setKSearch(30);
  // norm_src.setInputCloud(src);
  // norm_src.compute(*src);

  pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> norm_est;
  norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>));
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*src);


  norm_est.setInputCloud(tgt);
  norm_est.compute(*tgt);

  pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
  //typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> PointToPlane;
  typedef pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal> PointToPlane;

  boost::shared_ptr<PointToPlane> point_to_plane(new PointToPlane);
  icp.setTransformationEstimation(point_to_plane);
  //icp.setInputCloud(src);
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  // icp.setRANSACOutlierRejectionThreshold(0.1);
  // icp.setRANSACIterations(100);
  icp.setMaximumIterations(1000);
  icp.setTransformationEpsilon(1e-8);

  // pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr
  //     ce(new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>);
  // icp.setCorrespondenceEstimation(ce);

  // pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rej(new pcl::registration::CorrespondenceRejectorSurfaceNormal);
  // rej->setThreshold(0.1);
  // //Could be a lot of rotation -- just make sure they're at least within 0 degrees
  // icp.addCorrespondenceRejector(rej);

  pcl::PointCloud<pcl::PointNormal> output;
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f guess;
  guess.setIdentity();

  // float theta = -15 * M_PI / 180; // The angle of rotation in radians
  // guess(0, 0) = cos(theta);
  // guess(0, 1) = -sin(theta);
  // guess(1, 0) = sin(theta);
  // guess(1, 1) = cos(theta);
  //    (row, column)
  // icp.align(output, guess);
  std::cout<< "guess :\n"
            << guess.matrix() << endl;
  icp.align(output, guess);
  pcl::io::savePCDFile("output2.pcd", output);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << ", getEuclideanFitnessEpsilon: "
            << icp.getEuclideanFitnessEpsilon() <<std::endl;

  std::cout<< icp.getFinalTransformation() << std::endl;

  output_xyz->resize(output.points.size());
  for (size_t i = 0; i < output.points.size(); ++i)
  {
    output_xyz->points[i].x = output.points[i].x; //error
    output_xyz->points[i].y = output.points[i].y; //error
    output_xyz->points[i].z = output.points[i].z; //error
    //std::cout << output.points[i].x << ", " << output.points[i].y << ", " << output.points[i].z << std::endl;
  }

  pcl::visualization::PCLVisualizer viewer("ICP demo");
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(src_pc, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,
                                                                                   (int)255 * txt_gray_lvl);
  viewer.addPointCloud(src_pc, cloud_in_color_h, "cloud_in_v1", v1);
  // viewer.addPointCloud(src_pc, cloud_in_color_h, "cloud_in_v2", v2);

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tgt_color_h(tgt_pc, 20, 180, 20);
  viewer.addPointCloud(tgt_pc, cloud_tgt_color_h, "cloud_tgt_v1", v1);
  viewer.addPointCloud(tgt_pc, cloud_tgt_color_h, "cloud_tgt_v2", v2);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(output_xyz, 180, 20, 20);
  viewer.addPointCloud(output_xyz, cloud_icp_color_h, "cloud_icp_v2", v2);


  // Adding text descriptions in each viewport
  viewer.addText("White: src_pc\nGreen: tgt_pc", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
  viewer.addText("nGreen: tgt_pc\nRed: ICP aligned point cloud(src_pc_transformed)", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

  // Set background color
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
  viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

  // Set camera position and orientation
  viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize(1280, 1024); // Visualiser window size

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  }

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cur_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref_pc(new pcl::PointCloud<pcl::PointXYZ>);

    string cur_pc_name = std::string(argv[1]);
    string ref_pc_name = std::string(argv[2]);

  pcl::io::loadPCDFile<pcl::PointXYZ>(cur_pc_name, *cur_pc);
  pcl::io::loadPCDFile<pcl::PointXYZ>(ref_pc_name, *ref_pc);
  point_to_plane(cur_pc, ref_pc);
}