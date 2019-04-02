#pragma once

#include "Timer.hpp"
#include "Log.h"
#include "Math.hpp"
#include <iostream>
#include <iomanip>

#define DEBUG_OUTPUT true
inline void INFO(std::string output)
{
    if(DEBUG_OUTPUT)
        std::cout << "INFO:" << output << std::endl;
}
