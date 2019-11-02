// #ifdef USE_NEW_CODE
// #undef USE_NEW_CODE
// #endif

// #ifdef USE_CAMERA
// #undef USE_CAMERA
// #endif

// #ifdef USE_VIDEO
// #define USE_VIDEO
// #endif

// #ifdef DEBUG
// #undef DEBUG
// #endif

#ifndef DEBUG
#define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
#endif

#ifndef USE_NEW_CODE
#define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码
#endif

#ifndef USE_CAMERA
#define USE_CAMERA
#endif

// #ifndef USE_VIDEO
// #define USE_VIDEO
// #endif



# include <iostream>
# include <opencv2/opencv.hpp>
# include "other/include/timer.hpp"
# include "other/include/drawText.hpp"
# include "armor/include/show_images/show_images.hpp"
# include "armor/include/armor_finder/C-color.hpp"
# include "armor/include/armor_finder/distance.hpp"
#ifdef USE_NEW_CODE //新代码在下面

int main()
{
    sp::timer timer;
    timer.reset(); //建立计时器

    cv::VideoCapture capture;

    #ifdef USE_CAMERA //使用摄像头
    capture.open("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0",CV_CAP_V4L);
    #endif

    #ifdef USE_VIDEO //使用录像
    capture.open("../Webcam/2019-10-25-172548.webm");
    #endif
    
    sp::capture_set(capture, 640,//WIDTH

                              480,//HEIGHT
                              30,//FPS
                             -64,//BRIGHTNESS,
                              64,//CONTRAST, 
                              128,//SATURATION
                              40,//HUE, const int 
                              70//EXPOSURE
                     );
    //capture.open(1)

    cv::Mat src;
    if(capture.isOpened())
    {
        for(;;)
        {
            capture >> src; 
            cv::resize(src,src,cv::Size(640,480),(0,0), (0,0), CV_INTER_AREA);
            if(src.empty())
                break;
            sp::drawText(src);
            cv::imshow("image", src);
            if(cv::waitKey(10) >= 10)
                break;
        }
    }
    else
    {
        std::cout << "No capture" << std::endl;
        src = cv::Mat::zeros(480,640,CV_8UC1);
        sp::drawText(src);
        cv::imshow("image", src);
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
