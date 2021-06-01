/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-05-31 15:04:05
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-05-31 15:19:50
 * @Description: 
 */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

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
