#ifndef NONO_JETSON__MD25_H
#define NONO_JETSON__MD25_H

#include <ros/ros.h>
#include <cinttypes>

class NonoJetsonMd25
{
public:
  NonoJetsonMd25();
  bool open();
  bool read(const ros::Time time);
  void write(int8_t vl, int8_t vr);
  void reset();
  uint8_t _batt,  _cur_l, _cur_r;
  int32_t _enc_l, _enc_r;
 private:
  int _fd;
};

#endif // NONO_JETSON__MD25_H
