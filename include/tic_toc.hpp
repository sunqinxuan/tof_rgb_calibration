/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2022-10-28 09:21
#
# Filename: tic_toc.hpp
#
# Description:
#
************************************************/

#ifndef _TIC_TOC_HPP_
#define _TIC_TOC_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace ZJL {
class TicToc {
public:
  TicToc() { tic(); }
  TicToc(double min_dtime) : min_dtime_(min_dtime) { tic(); }

  void tic() {
#ifdef COMPILEDWITHC11
    start = std::chrono::steady_clock::now();
#else
    start = std::chrono::system_clock::now();
#endif
  }

  double toc() {
#ifdef COMPILEDWITHC11
    end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    start = std::chrono::steady_clock::now();
#else
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    start = std::chrono::system_clock::now();
#endif

    return elapsed_seconds.count();
  }
  double ave_toc() {
    double curr_dtime = toc();
    if (curr_dtime < min_dtime_)
      return ave_time_;

    ave_time_ = (ave_time_ * count_ + curr_dtime) / (count_ + 1);
    count_++;
    return ave_time_;
  }

private:
#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point start, end;
#else
  std::chrono::time_point<std::chrono::system_clock> start, end;
#endif
  double min_dtime_ = 0.0;
  double ave_time_ = 0.0;
  unsigned int count_ = 0;
};
} // namespace ZJL
#endif
