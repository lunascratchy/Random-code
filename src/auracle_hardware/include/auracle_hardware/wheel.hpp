#ifndef AURACLE_HARDWARE_WHEEL_HPP
#define AURACLE_HARDWARE_WHEEL_HPP

#include <string>
#include <cmath>

namespace auracle_hardware
{
class Wheel
{
public:
  std::string name;
  double position = 0.0;         // rad
  double velocity = 0.0;         // rad/s
  double command = 0.0;          // rad/s, commanded velocity from the diff_drive_controller
  long enc_ticks = 0;            // last raw encoder tick count applied (rear joints only)
  double ticks_per_rev = 0.0;
  double direction_sign = 1.0;   // set to -1.0 in the xacro if this side reads/drives backwards

  Wheel() = default;

  void setup(const std::string & wheel_name, double counts_per_rev, double sign = 1.0)
  {
    name = wheel_name;
    ticks_per_rev = counts_per_rev;
    direction_sign = sign;
  }

  double ticksToRadians(long ticks) const
  {
    if (ticks_per_rev == 0.0) {
      return 0.0;
    }
    return direction_sign * (static_cast<double>(ticks) / ticks_per_rev) * 2.0 * M_PI;
  }
};

}

#endif 
