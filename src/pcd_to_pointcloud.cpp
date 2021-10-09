/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b pcd_to_pointcloud is a simple node that loads PCD (Point Cloud Data) files from disk and publishes them as ROS messages on the network.

 **/

// ROS core
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

// STL
#include <chrono>
#include <string>
#include <thread>
/* ---[ */
int main(int argc, char **argv)
{
    std::cout << "usage: " << argv[0] << " <file.pcd> <topic_name> <frame_id>]" << std::endl;

    std::string file_name, frame_id, topic_name;
    if (argc < 2)
    {
        std::cerr << "please input fild.pcd" << std::endl;
        return -1;
    }
    file_name = std::string(argv[1]);

    if (argc > 2)
        topic_name = std::string(argv[2]);
    else
        topic_name = "cloud_pcd";

    if (argc > 3)
        frame_id = std::string(argv[3]);
    else
        frame_id = "base_footprint";

    ros::init(argc, argv, "pcd_to_pointcloud");
    ros::start();

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 3);

    sensor_msgs::PointCloud2 cloud_;
    if (pcl::io::loadPCDFile(file_name, cloud_) == -1)
    {
        std::cerr << "read PCD failed." << std::endl;
        return -1;
    }

    int nr_points = cloud_.width * cloud_.height;
    std::string fields_list = pcl::getFieldsList(cloud_);
    std::cout << "Publishing data with " << nr_points << " points " << fields_list << " on topic /" << topic_name << " in frame /" << frame_id << "." << std::endl;
    ros::Rate r(1);
    while (ros::ok())
    {
        cloud_.header.frame_id = frame_id;
        cloud_.header.stamp = ros::Time::now();
        pub.publish(boost::make_shared<const sensor_msgs::PointCloud2>(cloud_));
        std::cout << "pointcloud pub succeed." << std::endl;
        r.sleep();
    }

    return 0;
}
/* ]--- */