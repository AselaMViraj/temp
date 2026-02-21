#ifndef DIFFDRIVE_ZLAC8015D_CONFIG_H
#define DIFFDRIVE_ZLAC8015D_CONFIG_H

#include <string>

struct Config
{
  std::string left_wheel_name = "left_wheel_joint";
  std::string right_wheel_name = "right_wheel_joint";
  float loop_rate = 30;
  std::string device = "/dev/ttyACM0";
  int baud_rate = 115200;
  int timeout = 1000;
  int slave_id = 1;
  int enc_counts_per_rev = 16384;  // ZLAC8015D encoder resolution (verify with your motor spec)
};

#endif // DIFFDRIVE_ZLAC8015D_CONFIG_H
