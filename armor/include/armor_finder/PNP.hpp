# include <iostream>
# include <opencv2/opencv.hpp>
# include <string>
# define KNOWN_WIDTH 10 //占位数据
# define FOCI 10 //占位数据

#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{
    void get_distance()
    {
    
        cv::solvePnP(objPM,imagesPoint,cameraMatrix,distCoeffs,rvec,tvec,false,);
       
     
 
}

}
#else //旧代码在下面

int main()
{

    return 0;
}

#endif
