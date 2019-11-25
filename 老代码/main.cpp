#include <fstream>
#include <unistd.h>
#include <omp.h>  // openMP

#include "tools.hpp"
#include "detect_armor.hpp"
#include "camera.hpp"
#include "serial_port.hpp"
#include "Structure.h"

// 用本地txt记录来变化保存的视频名, 防止覆盖保存
void getVideoFileName(cv::String &filename);

// 取色用鼠标回调函数，鼠标右键按下获取某一点grey,b,r,h,s,v像素值
void on_mouse(int event, int x, int y, int flags, void *ustc); 

// #define record_video  // 是否一边录视频一边打

// #define FAST_CAMERA  // 100帧, 开server

// #define SERIAL

int main() {
    char portName[] = "/dev/ttyUSB0";
    CommunicatorC communicator;  // 通讯初始化
#ifdef SERIAL
    while (communicator.init(portName) != 0) {
        cv::waitKey(400);
        portName[11] = portName[11] == '0' ? '1' : '0'; //变换接口？？？？？？？？？
        printf("--- restart serial: %s, \n", portName);
    };
#endif

    omp_set_num_threads(2);  // 指定双线程
#pragma omp parallel sections
    {
#pragma omp section
        {
            // log工具模式选择, 0: 纯输出, 1: 窗口带最终结果, 2: 窗口详细输出 ////////////////////////////////////////////////////////
            is.setCostPrint(true);  // 耗时输出
            int mode = 1;
            is.setMode(mode);
            printf("--- Set mode = %d\n", mode);

            // 录视频初始化 /////////////////////////////////////////////////////////////////////////////////////////////////////
            cv::String filename;
            cv::VideoWriter vw;
#ifdef record_video
            getVideoFileName(filename);
        vw.open("../data/video/" + filename + ".avi", cv::VideoWriter::fourcc('x', '2', '6', '4'), 30.0,
            cv::Size(640, 480));
        printf("-- record video\n");
#endif

            // 初始化各模块 /////////////////////////////////////////////////////////////////////////////////////////////////////
            DetectArmor detect_armor;  // 装甲片检测模块初始化
            unsigned int timestamp = 0;  // 通讯包序号
            float yaw = 0.0;
            float pitch = 0.0;
            std::cout << "--- Detect Modules Init Successful" << std::endl;

            // 摄像头初始化 /////////////////////////////////////////////////////////////////////////////////////////////////////
            cameraInit();  // 摄像头参数初始化
            cv::VideoCapture cap;
            
#ifdef FAST_CAMERA
            cap.release();
#else
            int cameraIndex = 1;
            // cap.open(cameraIndex);
            cap.open("./a.mp4");
            while (!cap.isOpened()) {
                std::cout << "--- Try Open Camera Again..." << std::endl;
                // cap.open(cameraIndex);
                cap.open("../data/video/a.mp4");
                cameraIndex = cameraIndex == 0 ? 1 : 0;
                cv::waitKey(300);
            }
            cv::waitKey(1500);  // 等一会
            // 1. 分辨率设置
            cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);  // 1280
            cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);  // 720
            cv::waitKey(1500);  // 等一会
            // 2. 曝光设置
            cap.set(CV_CAP_PROP_EXPOSURE, 0.0043);   // 0.0032  0.0147

            // cv::waitKey(500);
            // cap.set(CV_CAP_PROP_AUTO_EXPOSURE, -7.0);
            // printf("CV_CAP_PROP_AUTO_EXPOSURE: %.2f", cap.get(CV_CAP_PROP_AUTO_EXPOSURE));

#endif
            std::cout << "-- Camera Set Complete" << std::endl;
            // 主循环 //////////////////////////////////////////////////////////////////////////////////////////////////////////
            cv::Mat frame;
            bool isOpen = true;
#ifndef FAST_CAMERA
            isOpen = cap.isOpened();
#endif
            while (isOpen) { //cap打开时进入循环
#ifndef FAST_CAMERA
                isOpen = cap.isOpened();//进入循环以后再次确认cap获取了图像
#endif
                printf("\n");
                float ttt = cv::getTickCount(); // 初始化计时器ttt

#ifdef FAST_CAMERA
                while (getImageFromMemory(frame) != 0) { }; // get frame
#else
                if (!cap.read(frame)) {  // end if (cap.read(frame)) //读取cap内的图像
                    perror("frame empty!!!");
                    exit(0);
                };
#endif
                is.updateImg(frame); //将frame的clock清零并建立
                // 取色用的窗口
                // cv::namedWindow("color", cv::WINDOW_GUI_NORMAL);
                // cv::setMouseCallback("color", on_mouse, &frame);
                // cv::imshow("color", frame);

                // 主要执行函数 /////////////////////////////////////////////////////////////////////////////////////////////
                float t = cv::getTickCount(); //初始化计时器t，detect_armor.runPlus计时
                int resultId = detect_armor.runPlus(frame, pitch, yaw);  //, zeroPt);

                printf("detect_armor: %.2f ms\n", 1000 * (cv::getTickCount() - t) / cv::getTickFrequency()); //打印detect_armor.runPlus计时结果
                printf("status: %d \n", resultId); //打印detect_armor.runPlus探测结果
                // -1 = 没找到, 也没的去瞄; 0 = 没找到, 但还是去瞄着; 6 = 找到了
                // 通讯 ////////////////////////////////////////////////////////////////////////////////////////////////////
                if (resultId >= 0) {  // 如果识别到, 历史值也算
                    unsigned int shootFlag = detect_armor.shootStrategy();  //通过判断yaw值得出是否击打 1是打, 0是不打
#ifdef SERIAL
                    communicator.send(++timestamp, -yaw, pitch, shootFlag, 0);
#endif
                }
                // 存视频 //////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef record_video
                vw.write(frame);
#endif
                // 以下画图的 ///////
                if (mode != 0) { //将几个yaw值输出在屏幕上
                    // char timeCost[50];
                    // sprintf(timeCost, "time cost: %.2f ms",
                    //         1000 * (cv::getTickCount() - t) / cv::getTickFrequency());
                    // is.addText(timeCost, true);
                    char angle1[50];
                    sprintf(angle1, "yaw: %.2f", yaw);
                    is.addText(angle1, true);
                    char angle2[50];
                    sprintf(angle2, "receive yaw: %.2f", current_global_yaw);
                    is.addText(angle2, true);
                    char angle3[50];
                    sprintf(angle3, "actualYaw: %.2f", current_global_yaw - yaw);
                    is.addText(angle3, true);
                }

                if (mode != 0) {
                    is.show();
                    int k = cv::waitKey(1);
                    is.keyEvent(k);
                    if (k == int('q'))
                        break;
                }
                printf("TOTAL cost: %.2f ms\n", 1000 * (cv::getTickCount() - ttt) / cv::getTickFrequency());
                is.clockPrint(1000.0f * (cv::getTickCount() - ttt) / cv::getTickFrequency());  // 打印时间统计，以毫秒为单位
            }  // end main loop

#ifndef FAST_CAMERA
            cap.release();
#endif
            cv::destroyAllWindows();
        }  // end omp section 0
#pragma omp section
        {
#ifdef SERIAL
            while (true)
                communicator.receive();
#endif
        }  // end omp section 1
    }  // end omp sections
    return 0;
}


// 取色鼠标回调函数
void on_mouse(int event, int x, int y, int flags, void *ustc) {
    cv::Mat frame = *(cv::Mat *) ustc; //将mat ustc指针的引用赋值给frame
    cv::Point p(x, y); //
    cv::Mat hsv, gray;
    std::vector<cv::Mat> bgr;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    if (event == cv::EVENT_LBUTTONDOWN) {   //右键按下
        printf("gray: %d \n", gray.at<cv::Vec3b>(p)[0]);
        printf("b: %d \n", frame.at<cv::Vec3b>(p)[0]);
        printf("r: %d \n", frame.at<cv::Vec3b>(p)[2]);
        printf("h: %d \n", hsv.at<cv::Vec3b>(p)[0]);
        printf("s: %d \n", hsv.at<cv::Vec3b>(p)[1]);
        printf("v: %d \n\n", hsv.at<cv::Vec3b>(p)[2]);
    }
}

// 用本地txt记录来变化保存的视频名, 防止覆盖保存
void getVideoFileName(cv::String &filename) {
    std::fstream f("../data/video/video_name.txt", std::ios::in);  // 打开文件，供读
    int record;
    f >> record;  // 读取数据
    f.close();
    filename += record;
    std::cout << "video file name: " << filename << std::endl;

    record += 1;
    f.open("../data/video/video_name.txt", std::ios::out);
    f << record;    // 写数据
    f.close();
}
