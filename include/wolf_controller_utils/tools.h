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

#ifndef WOLF_CONTROLLER_UTILS_TOOLS_H
#define WOLF_CONTROLLER_UTILS_TOOLS_H

#include <string>
#include <vector>
#include <memory>
#include <assert.h>

namespace wolf_controller_utils
{

class Ramp
{
public:

  typedef std::shared_ptr<Ramp> Ptr;

  enum type_t {DOWN=0,UP};

  Ramp(const double& T, type_t type = DOWN)
  {
    assert(T>0.0);
    T_ = T;
    type_ = type;
    reset();
  }

  void reset()
  {
    t_ = 0.0;
  }

  double update(const double& dt)
  {
    double out;
    if(type_ == DOWN)
    {
      out = 1.0 - t_/T_;
    }
    else
    {
      out = t_/T_;
    }

    if(t_>=T_)
      t_ = T_;
    else
      t_ += dt;

    return out;
  }

private:
  double t_;
  double T_;
  type_t type_;

};

class Counter
{
public:

  typedef std::shared_ptr<Counter> Ptr;

  Counter(unsigned int upper_limit)
  {
    cnt_ = 0;
    upper_limit_ = upper_limit;
  }

  void increase()
  {
    cnt_++;
  }

  void decrease()
  {
    if(cnt_>0)
      cnt_--;
  }

  void reset()
  {
    cnt_ = 0;
  }

  bool upperLimitReached()
  {
    return (cnt_ >= upper_limit_ ? true : false);
  }

private:
  long long cnt_;
  unsigned int upper_limit_;
};

class Trigger
{
public:

  Trigger()
  {
    old_value = false;
  }
  bool update(const bool& value)
  {
    bool res = (value && old_value != value ? true : false);
    old_value = value;
    return res;
  }
private:
  bool old_value;
};

class AxisToTrigger
{

public:
  enum status_t {UP=0,DOWN,STEADY};

  AxisToTrigger()
  {
    axis_old_value_ = 0.0;
  }

  void update(const double axis)
  {
    status_ = STEADY;
    if(std::abs(axis)>0.0 && axis_old_value_!=axis)
    {
      if(axis>=1.0)
        status_ = UP;
      else if (axis<=-1.0)
        status_ = DOWN;
    }

    axis_old_value_ = axis;
  }

  status_t getStatus()
  {
    return status_;
  }

private:
  double axis_old_value_;
  status_t status_;

};

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}; // namespace

#endif // WOLF_CONTROLLER_UTILS_UTILS_H
