/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-05-31 15:04:05
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-08-24 21:30:28
 * @Description: 
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include "nlohmann/json.hpp"
using namespace Eigen;
using namespace std;
std::string pose2json(std::string key, geometry_msgs::Pose pose)
{
    nlohmann::json j;

    j[key]["position"]["x"] = pose.position.x;
    j[key]["position"]["y"] = pose.position.y;
    j[key]["position"]["z"] = pose.position.z;
    j[key]["orientation"]["x"] = pose.orientation.x;
    j[key]["orientation"]["y"] = pose.orientation.y;
    j[key]["orientation"]["z"] = pose.orientation.z;
    j[key]["orientation"]["w"] = pose.orientation.w;
    return j.dump();
}
geometry_msgs::Pose json2pose(std::string key, std::string pose_string)
{
    geometry_msgs::Pose pose;
try{
    nlohmann::json j = nlohmann::json::parse(pose_string);
    pose.position.x = j[key]["position"]["x"];
    pose.position.y = j[key]["position"]["y"];
    pose.position.z = j[key]["position"]["z"];

    pose.orientation.x = j[key]["orientation"]["x"];
    pose.orientation.y = j[key]["orientation"]["y"];
    pose.orientation.z = j[key]["orientation"]["z"];
    pose.orientation.w = j[key]["orientation"]["w"];
    }
    catch(std::exception&e)
    {
        std::cout << "aaaa\n";
    }

    return pose;
}
int main()
{
    geometry_msgs::Pose pose;
    pose.position.x = 111;
    std::cout << pose2json("fuck", pose);
    std::cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaaa\n";
    std::cout << json2pose("fuck", pose2json("fuck1", pose));
    return 1;
    Eigen::Vector3d l1(1, 1, 3), l2(1, 1.00, 0);
    Eigen::Vector3d point_cross = l1.cross(l2);
    //point_cross /= point_cross[2];
    std::cout << "point_cross:  " << point_cross;


    int n = 10;
    // generate n points in the plane centered in p and spanned bu the u,v vectors.
    MatrixXd points_3D(3, n);
    Vector3d u = Vector3d::Random().normalized();
    Vector3d v = Vector3d::Random().normalized();
    Vector3d p = Vector3d::Random();
    points_3D = p.rowwise().replicate(n) + u * VectorXd::Random(n).transpose() + v * VectorXd::Random(n).transpose();
    MatrixXd initial_points = points_3D;

    Vector3d centroid = points_3D.rowwise().mean();
    points_3D.colwise() -= centroid;
    JacobiSVD<MatrixXd> svd(points_3D, ComputeFullU);
    Vector3d normal = svd.matrixU().col(2);
    double d = -normal.dot(centroid);

    cout << "Plane equation: " << normal.transpose() << " " << d << endl;
    cout << "Distances: " << (normal.transpose() * initial_points).array() + d << endl;
}