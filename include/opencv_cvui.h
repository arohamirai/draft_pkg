/*
 * @Author: liufeng(AT iplusbot.com)
 * @Date: 2021-06-03 19:55:24
 * @LastEditors: liufeng(AT iplusbot.com)
 * @LastEditTime: 2021-06-03 20:34:02
 * @Description: 
 */

#ifndef OPENCV_CVUI_H
#define OPENCV_CVUI_H

#include <opencv2/opencv.hpp>
#include <string>
class OpencvCvui
{
    public:
        OpencvCvui(cv::Size window_size, std::string window_name);
        ~OpencvCvui();

        bool pressButtonQuit();
        bool pressButtonShowImage();

        bool spin();

        cv::Mat window_;
        cv::Size window_size_;
        std::string window_name_;
};

#endif // !OPENCV_CVUI_H
