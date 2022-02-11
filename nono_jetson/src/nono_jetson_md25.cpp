#include <nono_jetson/nono_jetson_md25.h>

#define I2C_DEV "/dev/i2c-1"

#define MD25_ADDR   0x58
#define MD25_R_SL   0x00
#define MD25_R_SR   0x01
#define MD25_R_EL   0x02
#define MD25_R_ER   0x06
#define MD25_R_BATT 0x0A
#define MD25_R_CMD  0x10

#define MD25_C_RST_ENC 0x20
#define MD25_C_DISABLE_SPEED_REG 0x30

extern "C" {
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>

static int _c_open(void) {
  int fd;
  if((fd = open(I2C_DEV, O_RDWR)) < 0)  {
    fprintf(stderr, "%s : %s Open Error!\n", __func__, I2C_DEV);
    return  -1;
  }
  if(ioctl(fd, I2C_SLAVE, MD25_ADDR) < 0)  {
    fprintf(stderr, "%s : ioctl I2C_SLAVE Setup Error!\n", __func__);
    return -2;
  }
  return  fd;
 }

static int _c_write(int fd, uint8_t* buf, int len) {return write(fd, buf, len);}

} // extern "C"



NonoJetsonMd25::NonoJetsonMd25() {

}

bool NonoJetsonMd25::open() {
  _fd = _c_open();
  return _fd > 0;
}

bool NonoJetsonMd25::read(const ros::Time time) {
  //std::cerr << "read " << 1./period.toSec() << " Hz" <<std::endl;
  uint8_t tx = MD25_R_EL;
  uint8_t rx[16];
  struct i2c_msg msgs[2] = {
    { .addr = MD25_ADDR,
      .flags = 0,
      .len = 1,
      .buf = &tx, }, 
    { .addr = MD25_ADDR,
      .flags = I2C_M_RD,
      .len = 11,
      .buf = rx, },};
  struct i2c_rdwr_ioctl_data ioctl_data = { .msgs=msgs, 2};
  ioctl(_fd, I2C_RDWR, &ioctl_data);
  _enc_l = __builtin_bswap32(*((int32_t*)rx));
  _enc_r = __builtin_bswap32(*((int32_t*)(rx+4)));
  _batt = uint8_t(rx[8]);
  _cur_l = uint8_t(rx[9]);
  _cur_r = uint8_t(rx[10]);
  return true;
}

void NonoJetsonMd25::write(int8_t vl, int8_t vr) {
  uint8_t tx[3] = {MD25_R_SL, uint8_t(vl+0x80), uint8_t(vr+0x80)};
  _c_write(_fd, tx, 3);
}

void NonoJetsonMd25::reset() {
  uint8_t tx[2] = {MD25_R_CMD,  MD25_C_RST_ENC};
  _c_write(_fd, tx, 2);
  uint8_t tx2[2] = {MD25_R_CMD, MD25_C_DISABLE_SPEED_REG+1};
  _c_write(_fd, tx2, 2);
}
