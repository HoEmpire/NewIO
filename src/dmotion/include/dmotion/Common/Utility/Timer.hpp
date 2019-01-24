//
// Created by zjudancer on 17-11-27.
//

#ifndef dmotion_lib_MUTILITY_H
#define dmotion_lib_MUTILITY_H

#include <chrono>
#include <thread>
#include <iostream>

class timer
{
public:
    typedef std::chrono::steady_clock::time_point time_point;

    timer()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::steady_clock::now();
    }

    double toc()
    {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        std::cout << "**" << elapsed_seconds.count() * 1000 << "ms have passed**" << std::endl;
        return elapsed_seconds.count() * 1000;
    }

    static void delay_us(const int microseconds)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
    }

    static void delay_ms(const int msecs)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(msecs));
    }

    static void delay_s(const int seconds)
    {
        std::this_thread::sleep_for(std::chrono::seconds(seconds));
    }

    // rough clock
    static const time_point getCurrentSystemTime()
    {
        return std::chrono::steady_clock::now();
    }

    void smartDelay_ms(double ms)
    {
        end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        double time_past = elapsed_seconds.count() * 1000;
        if(time_past < ms)
        {
           delay_us((ms - time_past) * 1000);
        }
        else
        {
           std::cout << "smart ticks overflow:" << time_past << " ms > " << ms << " ms" << std::endl;
          // std::abort();
        }

    }

private:
    std::chrono::time_point<std::chrono::steady_clock> start, end;
};

#endif //dmotion_lib_MUTILITY_H
