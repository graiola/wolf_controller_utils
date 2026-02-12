/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_COMMON_H
#define WOLF_CONTROLLER_UTILS_COMMON_H

#include <wolf_controller_utils/eigen_types.h>
#include <iostream>

// Some static stuff
namespace wolf_controller_utils
{
  static std::vector<std::string> _cartesian_names = {"x","y","z","roll","pitch","yaw"}; // This is our standard cartesian dofs order
  static std::vector<std::string> _xyz = {"x","y","z"};
  static std::vector<std::string> _rpy = {"roll","pitch","yaw"};
  static std::vector<std::string> _joints_prefix = {"haa","hfe","kfe"};
  static std::vector<std::string> _legs_prefix = {"lf","lh","rf","rh"};
}

// Definitions
#define WORLD_FRAME_NAME "world"
#define GRAVITY 9.81
#define FLOATING_BASE_DOFS 6
#define EPS 0.00001
#define N_LEGS 4 // Fixed number of legs supported in WoLF
#define N_ARMS 1 // Fixed number of arms supported in WoLF
#define THREADS_SLEEP_TIME_ms 4
#define THROTTLE_SEC 3.0


#include <iostream>
#include <chrono>

/* FOREGROUND */
#define RST   "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED  "\x1B[31m"
#define FGRN  "\x1B[32m"
#define FYEL  "\x1B[33m"
#define FBLU  "\x1B[34m"
#define FMAG  "\x1B[35m"
#define FCYN  "\x1B[36m"
#define FWHT  "\x1B[37m"

#define BOLD  "\x1B[1m"
#define UNDL  "\x1B[4m"

/* PRINTS */
#define PRINT_INFO(x) \
  do { \
  std::cout << FWHT << x << RST << std::endl; \
  } while (0)
#define PRINT_WARN(x) \
  do { \
  std::cout << FYEL << x << RST << std::endl; \
  } while (0)
#define PRINT_ERROR(x) \
  do { \
  std::cout << FRED << x << RST << std::endl; \
  } while (0)
#define PRINT_INFO_NAMED(x,y) \
  do { \
  std::cout << BOLD << "[" << x << "]" << RST << " " << FWHT << y << RST << std::endl; \
  } while (0)
#define PRINT_WARN_NAMED(x,y) \
  do { \
  std::cout << BOLD << "[" << x << "]" << RST << " " << FYEL << y << RST << std::endl; \
  } while (0)
#define PRINT_ERROR_NAMED(x,y) \
  do { \
  std::cout << BOLD << "[" << x << "]" << RST << " " << FRED << y << RST << std::endl; \
  } while (0)
#define PRINT_INFO_THROTTLE(THROTTLE_SEC, x) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << FWHT << x << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)
#define PRINT_WARN_THROTTLE(THROTTLE_SEC, x) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << FYEL << x << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)
#define PRINT_ERROR_THROTTLE(THROTTLE_SEC, x) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << FRED << x << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)
#define PRINT_INFO_THROTTLE_NAMED(THROTTLE_SEC, x, y) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << BOLD << "[" << x << "]" << RST << " " << FWHT << y << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)
#define PRINT_WARN_THROTTLE_NAMED(THROTTLE_SEC, x, y) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << BOLD << "[" << x << "]" << RST << " " << FYEL << y << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)
#define PRINT_ERROR_THROTTLE_NAMED(THROTTLE_SEC, x, y) \
  do { \
    static auto last_print_time = std::chrono::steady_clock::now(); \
    auto current_time = std::chrono::steady_clock::now(); \
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_print_time).count(); \
    if (duration >= THROTTLE_SEC) { \
      std::cout << BOLD << "[" << x << "]" << RST << " " << FRED << y << RST << std::endl; \
      last_print_time = current_time; \
    } \
  } while (0)


#endif
