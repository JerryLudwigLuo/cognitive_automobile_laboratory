 
/*
 * Copyright 2012. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors: 
 *  Henning Lategahn (henning.latgahn@kit.edu)
 *  and others
 */

#pragma once
#include <iostream>
#define GCC_VERSION (__GNUC__ * 10000                 \
                     + __GNUC_MINOR__ * 100           \
                     + __GNUC_PATCHLEVEL__)

#if GCC_VERSION >= 70000
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-attributes"
#include <Eigen/Dense>
#pragma GCC diagnostic pop
#else
#include <Eigen/Dense>
#endif

typedef Eigen::Matrix<double, 4, 1> cVector4;
typedef Eigen::Matrix<double, 3, 1> cVector3;
typedef Eigen::Matrix<double, 2, 1> cVector2;

typedef cVector4 cPoint4;
typedef cVector3 cPoint3;
typedef cVector2 cPoint2;

// Define fixed-width datatypes for Visual Studio projects
#ifndef _MSC_VER
  #include <stdint.h>
#else
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#endif

#if 1
#include <sys/time.h>
#define TIC(var_name) struct timeval var_##var_name;    gettimeofday(&var_##var_name, NULL);    double d_double_var##var_name = var_##var_name.tv_sec + (double) var_##var_name.tv_usec/1000000;
#define TOC(var_name, description) gettimeofday(&var_##var_name, NULL);   std::cout << "Time elapsed " << description << " : " << (var_##var_name.tv_sec + (double) var_##var_name.tv_usec/1000000 - d_double_var##var_name) * 1000 << " ms\n";
#else
#define TIC(var_name) ;
#define TOC(var_name, description) ;
#endif
