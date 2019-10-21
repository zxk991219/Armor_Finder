#pragma once

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
#define DEBUG //在程序中用 #ifdef DEBUG 与 #endif 将debug代码块框起来,实现debug输出 
#endif


# include <chrono>


#ifdef USE_NEW_CODE //新代码在下面

namespace sp
{

class timer
{
public:
    using clk_t = std::chrono::high_resolution_clock;
    timer() : m_tp(clk_t::now()){}
    void reset()
    {
        m_tp = clk_t::now();
    }
    template <typename T = std::milli>
    double get() //ms
    {
        return std::chrono::duration<double, T>(clk_t::now() - m_tp).count;
    }
private:
    clk_t::time_point m_tp;
}; //class timer
}

#else //旧代码在下面



#endif
