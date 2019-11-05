#pragma once

// #ifdef USE_NEW_CODE
// #undef USE_NEW_CODE
// #endif

// #ifndef USE_NEW_CODE
// #define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码
// #endif

// #ifdef DEBUG
// #undef DEBUG
// #endif

// #ifndef DEBUG
// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
// #endif

# include <chrono>
# include <opencv2/opencv.hpp>


#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{
    void drawText(cv::Mat& image);
    void drawText(cv::Mat& image)
{
    cv::putText(image, "Video",
    cv::Point(20,50),
    CV_FONT_HERSHEY_COMPLEX, 1,
    cv::Scalar(255,255,255), 1,
    LINE_MAX);
}
}

#else //旧代码在下面



#endif
