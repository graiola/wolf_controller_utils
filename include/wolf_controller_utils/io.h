/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) Gennaro Raiola
 */

#ifndef WOLF_CONTROLLER_UTILS_IO_H
#define WOLF_CONTROLLER_UTILS_IO_H

#include <Eigen/Core>

#include <fstream>
#include <iostream>
#include <string>

namespace wolf_controller_utils
{

inline void writeTxtFile(const std::string& filename, const Eigen::VectorXd& values) {
  std::ofstream myfile(filename.c_str());
  const std::size_t nb_rows = values.size();
  if (myfile.is_open())
  {
    for(std::size_t row = 0; row < nb_rows; ++row) {
      myfile << values(row) << "\n";
    }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ] "<<std::endl;
  }
  else{
    std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
  }
  myfile.close();
}

inline void writeTxtFile(const std::string& filename, const Eigen::MatrixXd& values) {
  std::ofstream myfile(filename.c_str());
  const std::size_t nb_rows = values.rows();
  const std::size_t nb_cols = values.cols();

  if (myfile.is_open())
  {
    for(std::size_t row = 0; row < nb_rows; ++row) {
      for(std::size_t col = 0; col < nb_cols; ++col) {
        myfile << values(row,col) << " ";
      }
      myfile << "\n";
    }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ]["<<nb_cols<<" cols ] "<<std::endl;
  }
  else{
    std::cerr << "Unable to open file : ["<<filename<<"]"<<std::endl;
  }
  myfile.close();
}

}; // namespace

#endif
