//
// Created by zjudancer on 17-11-27.
//

#ifndef dmotion_lib_MUTILITY_H
#define dmotion_lib_MUTILITY_H

#include <chrono>
#include <thread>

class timer
{
public:
    typedef std::chrono::system_clock::time_point time_point;

    timer()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
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
        return std::chrono::system_clock::now();
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif //dmotion_lib_MUTILITY_H
