#pragma once
#include <LSM6DSL.h>
#include <LIS3MDL.h>

#include <cstring>         // For strerror
#include <fcntl.h>         // For open()
extern "C"
{
    #include<linux/i2c-dev.h>
    #include <linux/i2c.h>
    #include <i2c/smbus.h>
}
// #include <linux/i2c-dev.h> // For I2C_SLAVE
#include <sys/ioctl.h>     // For ioctl()
#include <unistd.h>        // For close()
// c++ stl includes
#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <unistd.h>

class I2CDriver {
public:
  explicit I2CDriver(const std::string &device)
      : _device_file(-1), _device_path(device) {}
  ~I2CDriver() {
    if (_device_file >= 0) {
      close(_device_file);
    }
  }

  struct xyz_vec
  {
    float x;
    float y;
    float z;
  };

  struct imu_data
  {
    xyz_vec gyr_data;
    xyz_vec accel_data; 
  };
  
  imu_data sample_data();

private:
  void _read_block(uint8_t command, uint8_t size, uint8_t *data);
  void _select_device(int dev_file, int addr);
  bool _open_device(const std::string &device_path);


  std::array<int, 3> _read_vector(int dev_file, uint8_t command);
  void _write_reg(uint8_t reg, uint8_t value, int dev_file, int dev_addr);
  void _enable(int dev_file);
private:
  int _device_file;
  std::string _device_path;
};