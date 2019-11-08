 #pragma once

# include <iostream>
# include <opencv2/opencv.hpp>
# include "../../../other/include/drawText.hpp"


# define HALF_WIDTH 7.10
# define HALF_HEIGHT 6.25

#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{

    void get_distance(cv::Mat frame, std::vector<cv::Rect> bboxes_armor)
{   
    cv::Mat cam;
    cv::Mat dis;

    cv::FileStorage fs("../out_camera_data.xml", cv::FileStorage::READ);
    fs["camera_matrix"] >> cam;
    fs["distortion_coefficients"] >> dis;//传入相机的内参和外参

    for(int i=0;i<bboxes_armor.size();++i)
 {
    cv::Rect rect=bboxes_armor[i];
    std::vector<cv::Point2f> pnts=std::vector<cv::Point2f>{
        cv::Point2f(rect.tl().x,rect.tl().y),
        cv::Point2f(rect.tl().x+rect.width,rect.tl().y),
        cv::Point2f(rect.br().x,rect.br().y),
        cv::Point2f(rect.br().x-rect.width,rect.br().y)
    };//设置像素坐标
    std::vector<cv::Point3f> obj=std::vector<cv::Point3f>{
            cv::Point3f(-HALF_WIDTH, -HALF_HEIGHT, 0),	//tl
            cv::Point3f(HALF_WIDTH, -HALF_HEIGHT, 0),	//tr
            cv::Point3f(HALF_WIDTH, HALF_HEIGHT, 0),	//br
            cv::Point3f(-HALF_WIDTH, HALF_HEIGHT, 0)	//bl
    };//设置世界坐标系中的坐标
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1);//init rvec
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1);//init tvec
    cv::solvePnP(obj,pnts,cam,dis,rVec,tVec,false,cv::SOLVEPNP_ITERATIVE);
     //利用solvepnp解算出平移向量和旋转向量
    double rm[3][3];
    cv::Mat rotMat(3,3,CV_64FC1,rm);//共享数据
    cv::Rodrigues(rVec,rotMat);
    float theta_z = atan2(rm[1][0], rm[0][0])*57.2958;

    float theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2]))*57.2958;

    float theta_x = atan2(rm[2][1], rm[2][2])*57.2958;//解算角度


    std::cout<<theta_z<<theta_y<<theta_z<<std::endl;

    std::vector<uchar>  buff;//将mat型转换为string
    cv::imencode(".bmp", tVec, buff);
    std::string text(reinterpret_cast<char*>(&buff[0]), buff.size());
    
    sp::drawText(frame,bboxes_armor[i],text );
  }

 }
}
//         void calAngle(cv::Mat cam,cv::Mat dis,int x,int y)
//     {
//     double fx=cam.at<double>(0,0);
//     double fy=cam.at<double>(1,1);
//     double cx=cam.at<double>(0,2);
//     double cy=cam.at<double>(1,2);
//     cv::Point2f pnt;
//     std::vector<cv::Point2f> in;
//     std::vector<cv::Point2f> out;
//     in.push_back(cv::Point2f(x,y));
//     //对像素点去畸变
//     undistortPoints(in,out,cam,dis,cv::noArray(),cam);
//     pnt=out.front();
//     //没有去畸变时的比值
//     double rx=(x-cx)/fx;
//     double ry=(y-cy)/fy;
//     //去畸变后的比值
//     double rxNew=(pnt.x-cx)/fx;
//     double ryNew=(pnt.y-cy)/fy;
//     //输出原来点的坐标和去畸变后点的坐标
//     std::cout<< "x: "<<x<<" xNew:"<<pnt.x<<std::endl;
//     std::cout<< "y: "<<y<<" yNew:"<<pnt.y<<std::endl;
//     //输出未去畸变时测得的角度和去畸变后测得的角度
//     std::cout<< "angx: "<<atan(rx)/CV_PI*180<<" angleNew:"<<atan(rxNew)/CV_PI*180<<std::endl;
//     std::cout<< "angy: "<<atan(ry)/CV_PI*180<<" angleNew:"<<atan(ryNew)/CV_PI*180<<std::endl;
//   }










 



#else //旧代码在下面

int main()
{

    return 0;
}

#endif
