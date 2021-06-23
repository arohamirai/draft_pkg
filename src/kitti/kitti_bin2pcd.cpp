/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-23 19:31:08
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-23 19:57:03
 * @Description: 
 */
#include <boost/program_options.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_operators.h>
#include <pcl/common/io.h>
#include <pcl/search/organized.h>
#include <pcl/search/octree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>
 
#include <iostream>
#include <fstream>
 
using namespace pcl;
using namespace std;
 
namespace po = boost::program_options;

pcl::PointCloud<PointXYZI>::Ptr readKittiBinFile(string file)
{
	// load point cloud
	fstream input(file.c_str(), ios::in | ios::binary);
	if(!input.good()){
		cerr << "Could not read file: " << file << endl;
		exit(EXIT_FAILURE);
	}
	input.seekg(0, ios::beg);
 
	pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
 
	int i;
	for (i=0; input.good() && !input.eof(); i++) {
		PointXYZI point;
		input.read((char *) &point.x, 3*sizeof(float));
		input.read((char *) &point.intensity, sizeof(float));
		points->push_back(point);
	}
	input.close();
 
	cout << "Read KTTI point cloud with " << i << " points."<< endl;

	return points;
}
 
int main(int argc, char **argv){
	cout << "usage:./kitti_bin2pcd --input /home/lile/polex/KittiData/raw/0001_sync/velodyne_points/data/0000000020.bin --output ./20.pcd" << endl;
	///The file to read from.
	string infile;
	///The file to output to.
	string outfile;
	// Declare the supported options.
	po::options_description desc("Program options");
	desc.add_options()
		("input", po::value<string>(&infile)->required(), "the file to read a point cloud from")
		("output", po::value<string>(&outfile)->required(), "the file to write the DoN point cloud & normals to");
	// Parse the command line
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	// Print help
	if (vm.count("help"))
	{
		cout << desc << "\n";
		return false;
	}
 
	// Process options.
	po::notify(vm);
	pcl::PointCloud<PointXYZI>::Ptr points = readKittiBinFile(infile);

	pcl::PCDWriter writer;
 
    // Save DoN features
    writer.write<PointXYZI>(outfile, *points, false);
	cout << "KTTI point Saved to " << outfile << endl;
	return 0;
}