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





# include <iostream>
# include <opencv2/opencv.hpp>

#ifdef USE_NEW_CODE //新代码在下面

int main()
{
    
}


#else //旧代码在下面

int main()
{

    return 0;
}

#endif