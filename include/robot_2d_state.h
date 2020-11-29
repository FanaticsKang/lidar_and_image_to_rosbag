#pragma once

#include <iomanip>
#include <iostream>

struct Robot2DState {
  double timestamp;
  float x;
  float y;
  float yaw;

  friend std::ostream& operator<<(std::ostream& os, const Robot2DState& state) {
    os << "time: " << state.timestamp;
    os << ", pose[x, y, yaw]:" << std::setprecision(4) << state.x << ", "
       << state.y << ", " << state.yaw;
    return os;
  }
};
