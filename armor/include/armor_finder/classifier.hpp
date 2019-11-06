#pragma once

#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "proportion_thresh.hpp"



# ifdef USE_NEW_CODE

namespace sp //使用命名空间sp
{
void classifier(const cv::Mat& src, std::string compare_filename) //src是从原图中截取mat，
{
	double thresh_binar = 0.02; //二值化取thresh_binar最亮部分
	cv::cvtColor(src, src, CV_RGB2GRAY); //转化截取图像为灰度
	sp::proportion_thresh(src, src, 255, thresh_binar); //二值化图像
	
	
}
}










# else

# endif
