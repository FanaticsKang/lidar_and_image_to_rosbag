#pragma once
#include <iostream>

class SpeedPulseRaw {
 protected:
  unsigned int id;
  unsigned int size;
  unsigned long long timestamp;
  char data[0];

  unsigned short left_front;
  unsigned short right_front;
  unsigned short left_rear;
  unsigned short right_rear;
};

class SpeedPulse : public SpeedPulseRaw {
 public:
  // TODO erase
  // SpeedPulse() {}
  SpeedPulse(const SpeedPulseRaw& data)
      : SpeedPulseRaw(data), timestamp_sec(timestamp * 1e-6) {}

  unsigned short LeftFront() const { return left_front; }
  unsigned short RightFront() const { return right_front; }
  unsigned short LeftRear() const { return left_rear; }
  unsigned short RightRear() const { return right_rear; }

  friend std::ostream& operator<<(std::ostream& os, const SpeedPulse& sp) {
    os << "Timestamp: " << sp.timestamp << ", [" << sp.left_front << ", "
       << sp.right_front << ", " << sp.right_front << ", " << sp.right_rear
       << "]";
    return os;
  }
  double TimestampSec() const { return timestamp_sec; }

 private:
  double timestamp_sec;
};

struct Header{
  unsigned int id;
  unsigned int size;
  unsigned long long timestamp;
  char data[0];
};