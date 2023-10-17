/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

#define TIC_TOC_FUNCTIONAL (1U)
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
#if (TIC_TOC_FUNCTIONAL)
        start = std::chrono::system_clock::now();
        dt_ms = 0;
#endif
    }

    double toc()
    {
#if (TIC_TOC_FUNCTIONAL)
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        dt_ms = elapsed_seconds.count() * 1000;
        return dt_ms;
#else
        return 0.0;
#endif
    }

    double dt()
    {
        return dt_ms;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    double dt_ms;
};
