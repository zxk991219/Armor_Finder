#ifdef USE_NEW_CODE
#undef USE_NEW_CODE
#endif

#ifdef DEBUG
#undef DEBUG
#endif





// 开始define




// #define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 

#define USE_NEW_CODE //如果保留这一行,则使用新代码; 如果注释掉这一行,则使用旧代码

// #define USE_CAMERA
#define USE_VIDEO

#define USE_RED
// #define USE_BLUE
// #define USE_GREEN

# include <iostream>
# include <opencv2/opencv.hpp>
# include "other/include/timer.hpp"
# include "other/include/drawText.hpp"
# include "armor/include/show_images/show_images.hpp"
# include "armor/include/armor_finder/MSER.hpp"
// # include "armor/include/armor_finder/C-color.hpp"
// # include "armor/include/armor_finder/distance.hpp"

# ifdef USE_NEW_CODE //新代码在下面

int main()
{
    sp::timer timer; //建立计时器

    cv::VideoCapture capture;



    #ifdef USE_CAMERA //使用摄像头
    capture.open("/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1:1.0-video-index0",CV_CAP_V4L);
    #endif

    #ifdef USE_VIDEO //使用录像
    // capture.open("../Video/2019-10-28-222635.webm");
    capture.open("../Video/2019-10-28-223802.webm");
    // capture.open("../Video/2019-10-28-223826.webm");
    // capture.open("../Video/2019-10-28-223848.webm");
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
    cv::Mat src_real;

    if(capture.isOpened())
    {
        for(;;) //读取视频循环
        {
            capture >> src_real; 
            cv::resize(src_real,src_real,cv::Size(640,480),(0,0), (0,0), CV_INTER_AREA);
            
            std::vector<cv::Mat> channels;//定义Mat类型的向量
            cv::split(src_real, channels);//通道分离
            cv::Mat blue = channels.at(0);
            cv::Mat green = channels.at(1);
            cv::Mat red = channels.at(2);

            #ifdef USE_RED
            src = red;
            #endif

            #ifdef USE_BLUE
            src = blue;
            #endif

            #ifdef USE_GREEN
            src = green;
            #endif


            timer.reset(); // 开始计时

            if(src.empty())
                break;

            // cv::imshow("image_beforeMSER", src);
            src = sp::mser(src, src_real);
            // sp::drawText(src);
            
            #ifdef DEBUG
            cv::imshow("image", src);
            #endif
            
            cv::imshow("image_beforeMSER", src_real);

            std::cout << "程序运行时间：" << timer.get() << "ms" << std::endl; //结束计时

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

    return 0;
}


#else //旧代码在下面

int main()
{

    return 0;
}

#endif
