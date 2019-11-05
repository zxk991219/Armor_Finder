// #ifdef USE_NEW_CODE
// #undef USE_NEW_CODE
// #endif


// #ifdef DEBUG
// #undef DEBUG
// #endif

// #ifndef DEBUG
// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
// #endif

// #ifndef USE_NEW_CODE
// #define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码
// #endif

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#define BLOB_BLUE 0
#define BLOB_RED 1

# ifdef USE_NEW_CODE //新代码在下面

namespace sp
{

static uint8_t get_blob_color(const cv::Mat &src, const cv::RotatedRect &blobPos) 
{
    auto region = blobPos.boundingRect();
    region.x -= fmax(3, region.width * 0.1);
    region.y -= fmax(3, region.height * 0.05);
    region.width += 2 * fmax(3, region.width * 0.1);
    region.height += 2 * fmax(3, region.height * 0.05);
    region &= cv::Rect(0, 0, src.cols, src.rows);
    cv::Mat roi = src(region);
    int red_cnt = 0, blue_cnt = 0;
    for (int row = 0; row < roi.rows; row++) {
        for (int col = 0; col < roi.cols; col++) {
            red_cnt += roi.at<cv::Vec3b>(row, col)[2];
            blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
        }
    }
    if (red_cnt > blue_cnt) {
        return BLOB_RED;
    } else {
        return BLOB_BLUE;
    }
}
}
#else //旧代码在下面

#endif
