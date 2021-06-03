/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-03 19:54:13
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-03 20:45:17
 * @Description: 
 */
#include "opencv_cvui.h"
#include <glog/logging.h>

#define CVUI_IMPLEMENTATION
#include "cvui.h"

using namespace cv;
OpencvCvui::OpencvCvui(cv::Size window_size, std::string window_name)
    : window_size_(window_size), window_name_(window_name)
{
    window_ = cv::Mat(window_size, CV_8UC3);
    window_ = cv::Scalar(49, 52, 49);
    cvui::init(window_name_, 20);
}

OpencvCvui::~OpencvCvui()
{

}

bool OpencvCvui::pressButtonQuit()
{
    return true;
}

bool OpencvCvui::pressButtonShowImage()
{
    LOG(INFO) << "Show Image!";
    Mat img = cv::imread("/home/lile/wallpaper.jpg", 1);
    cv::resize(img, window_, window_size_);
    return true;
}

bool OpencvCvui::spin()
{
    while (true)
    {

        //cvui::text(window_, 40, 40, "To exit this app click the button below or press Q (shortcut for the button below).");
        if (cvui::button(window_, 1100, 80, "&LoadImage"))
        {
            pressButtonShowImage();
        }
        if (cvui::button(window_, 1200, 690, "&Quit"))
        {
            pressButtonQuit();
            break;
        }

        cvui::update();

        cv::imshow(window_name_, window_);
    }

    return true;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    FLAGS_logtostderr = true;
    FLAGS_colorlogtostderr = true;
    FLAGS_logbufsecs = 0;

    OpencvCvui ocv(cv::Size(1280, 720), "opencv_ui");

    LOG(INFO) << "opencv_ui Start.";
    ocv.spin();

    google::ShutdownGoogleLogging();
    return 0;
}