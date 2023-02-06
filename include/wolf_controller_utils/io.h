/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 *
 * original code and license notice here: http://wiki.ros.org/explore_lite
*/

#ifndef WOLF_CONTROLLER_UTILS_IO_H
#define WOLF_CONTROLLER_UTILS_IO_H

namespace wolf_controller_utils
{

inline void writeTxtFile(const std::string filename, Eigen::VectorXd& values) {
  std::ofstream myfile (filename.c_str());
  std::size_t row = 0;
  std::size_t nb_rows = values.size();
  if (myfile.is_open())
  {
    while(row < nb_rows) {
      myfile << values(row) << "\n";
      row++;
    }
    std::cout << "File ["<<filename<<"] write with success  ["<<nb_rows<<" rows ] "<<std::endl;
  }
  else{
    std::cout << "Unable to open file : ["<<filename<<"]"<<std::endl;
  }
  myfile.close();
}

inline void writeTxtFile(const std::string filename, Eigen::MatrixXd& values) {
  std::ofstream myfile (filename.c_str());
  std::size_t row = 0;
  std::size_t nb_rows = values.rows();
  std::size_t col = 0;
  std::size_t nb_cols = values.cols();

  if (myfile.is_open())
  {
    while(row < nb_rows) {
      while(col < nb_cols) {
        myfile << values(row,col) << " ";
        col++;
      }
      col = 0;
      row++;
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
