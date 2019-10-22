#ifdef USE_NEW_CODE
#undef USE_NEW_CODE
#endif

#ifndef USE_NEW_CODE
#define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码
#endif

#ifdef DEBUG
#undef DEBUG
#endif

#ifndef DEBUG
// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
#endif

# include <iostream>
# include <opencv2/opencv.hpp>
# include "other/include/timer.hpp"
# include "other/include/drawText.hpp"


#ifdef USE_NEW_CODE //新代码在下面

int main()
{
    sp::timer timer;
    timer.reset(); //建立计时器

    cv::VideoCapture capture;
    capture.open("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0",CV_CAP_V4L);
    // capture.set(); //在网上查
    //capture.open(1);
    cv::Mat src;
    if(capture.isOpened())
    {
        for(;;)
        {
            capture >> src; 
            if(src.empty())
                break;
            sp::drawText(src);
            cv::imshow("image", src);
            if(cv::waitKey(10) >= 10)
                break;
        }
        #ifdef DEBUG
        std::cout<<"debug";
        #endif
    }
    else
    {
        std::cout << "No capture" << std::endl;
        src = cv::Mat::zeros(480,640,CV_8UC1);
        sp::drawText(src);
        imshow("image", src);
        cv::waitKey(0);

    }


    std::cout << "程序运行时间：" << timer.get() << "mm" << std::endl;
    return 0;
}


#else //旧代码在下面

int main()
{

    return 0;
}

#endif
