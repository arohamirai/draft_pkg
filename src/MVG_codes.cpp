/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-08-17 15:52:45
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-08-17 17:43:06
 * @Description: 
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

// 点到直线(两点式)的距离
double pointToLineDistance(Eigen::Vector3d line_point0,
                           Eigen::Vector3d line_point1,
                           Eigen::Vector3d point) {
  Eigen::Vector3d a = line_point0 - line_point1;
  Eigen::Vector3d b = point - line_point1;
  return a.cross(b).norm() / a.norm();
}

// 点投影到平面上的投影点
Eigen::Vector3d projectPointToPlane(const Eigen::Vector4d &plane, const Eigen::Vector3d &point)
{
    double A, B, C, D;
    A = plane[0];
    B = plane[1];
    C = plane[2];
    D = plane[3];

    double x0, y0, z0;
    x0 = point[0];
    y0 = point[1];
    z0 = point[2];

    Eigen::Vector3d norm_plane = Eigen::Vector3d(A, B, C);

    double ABC_2 = norm_plane.transpose() * norm_plane;
    CHECK(ABC_2 > 1e-8) << "plane normal vector is wrong, A*A + B*B + C*C = " << ABC_2;

    double t = (A * x0 + B * y0 + C * z0 + D) / ABC_2;
    return Eigen::Vector3d(x0 - A * t, y0 - B * t, z0 - C * t);
}

// svd拟合平面
Eigen::Vector4d fitPlaneSVD(Eigen::MatrixXd points_3D) {
  Eigen::Vector3d centroid = points_3D.rowwise().mean();
  points_3D.colwise() -= centroid;
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(points_3D, Eigen::ComputeFullU);
  Eigen::Vector3d normal = svd.matrixU().col(2);
  double d = -normal.dot(centroid);

  Eigen::Vector4d plane;
  plane.block(0, 0, 3, 1) = normal;
  plane[3] = d;
  return plane;
}

int main(int argc, char ** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;

    Eigen::Vector4d plane = Eigen::Vector4d(3, -2, 1, -2);
    Eigen::Vector3d point = Eigen::Vector3d(5, -6, 3);
    LOG(INFO)
        << projectPointToPlane(plane, point).transpose();  // desire to print (-1,-2, 1)
}