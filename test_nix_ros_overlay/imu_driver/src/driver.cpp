#include <driver.hpp>
#include <iostream>


#include <array>

bool I2CDriver::_open_device(const std::string &device_path) {
  _device_file = open(device_path.c_str(), O_RDWR);
  if (_device_file < 0) {
    std::cerr << "Failed to open I2C device: " << strerror(errno) << std::endl;
    return false;
  }

  return true;
}

void I2CDriver::_select_device(int dev_file, int addr) {
  if (ioctl(_device_file, I2C_SLAVE, addr) < 0) {
    std::cerr << "Failed to set I2C address: " << strerror(errno) << std::endl;
  }
}

void I2CDriver::_read_block(uint8_t command, uint8_t size, uint8_t *data) {
  int result = i2c_smbus_read_i2c_block_data(_device_file, command, size, data);
  if (result != size) {
    printf("Failed to read block from I2C.");
    exit(1);
  }
}

std::array<int, 3> I2CDriver::_read_vector(int dev_file, uint8_t command) {
  std::array<int, 3> arr;
  uint8_t block[6];

  _read_block(command, sizeof(block), block);

  arr[0] = (int16_t)(block[0] | block[1] << 8);
  arr[1] = (int16_t)(block[2] | block[3] << 8);
  arr[2] = (int16_t)(block[4] | block[5] << 8);
  return arr;
}

void I2CDriver::_write_reg(uint8_t reg, uint8_t value, int dev_file,
                           int dev_addr) {

  _select_device(dev_file, dev_addr);
  int result = i2c_smbus_write_byte_data(dev_file, reg, value);
  if (result == -1) {
    std::cerr << "Failed to write byte to I2C Mag." << std::endl;
  }
}

void I2CDriver::_enable(int dev_file)
{
    	//Enable  gyroscope
		_write_reg(LSM6DSL_CTRL2_G,0b10011100, dev_file, LSM6DSL_ADDRESS);        // ODR 3.3 kHz, 2000 dps

		// Enable the accelerometer
		_write_reg(LSM6DSL_CTRL1_XL,0b10011111, dev_file, LSM6DSL_ADDRESS);       // ODR 3.33 kHz, +/- 8g , BW = 400hz
		_write_reg(LSM6DSL_CTRL8_XL,0b11001000, dev_file, LSM6DSL_ADDRESS);       // Low pass filter enabled, BW9, composite filter
		_write_reg(LSM6DSL_CTRL3_C,0b01000100, dev_file, LSM6DSL_ADDRESS);        // Enable Block Data update, increment during multi byte read

		//Enable  magnetometer
		_write_reg(LIS3MDL_CTRL_REG1, 0b11011100, dev_file, LIS3MDL_ADDRESS);     // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
		_write_reg(LIS3MDL_CTRL_REG2, 0b00100000, dev_file, LIS3MDL_ADDRESS);     // +/- 8 gauss
		_write_reg(LIS3MDL_CTRL_REG3, 0b00000000, dev_file, LIS3MDL_ADDRESS);     // Continuous-conversion mode
}

I2CDriver::imu_data I2CDriver::sample_data() {

    I2CDriver::imu_data data;
    
    auto gyr_data = _read_vector(_device_file, LSM6DSL_OUTX_L_G); 

    // from https://ozzmaker.com/berryimu/
    
    // For the gyro, we need to work out how fast it is rotating 
    /// in degrees per second(dps). This is easily done by multiplying 
    // the raw value by the sensitivity level that was used when enabling the gyro. 
    // G_GAIN = 0.07, which is based off a sensitivity level of 2000 dps.  
    // Looking at the table on page 3 of the datasheet, we can see that we need to multiple
    //  it by 0.07 to get degrees per second. The table shows it as 70 mdps (milli degrees 
    /// per second). Another example: Looking at the table, if we chose a sensitivity level of 
    // 250dps when enabling the gyro, then we would have to multiply the raw values by 0.00875.

    data.gyr_data.x = static_cast<float>(gyr_data[0] * 0.07);
    data.gyr_data.y = static_cast<float>(gyr_data[1] * 0.07);
    data.gyr_data.z = static_cast<float>(gyr_data[2] * 0.07);

 
    auto accel_data = _read_vector(_device_file, LSM6DSL_OUTX_L_XL);
    // 0.244 : from page 21 table 3. g = gravity (multiply by 9.807 to get m_ss)
    data.accel_data.x = ((accel_data[0] *0.244)/ 1000.0f * 9.807);
    data.accel_data.y = ((accel_data[0] *0.244)/ 1000.0f * 9.807);
    data.accel_data.z = ((accel_data[0] *0.244)/ 1000.0f * 9.807);

    auto mag_data = _read_vector(_device_file, LIS3MDL_ADDRESS);
    data.mag_data.x = mag_data[0];
    data.mag_data.y = mag_data[1];
    data.mag_data.z = mag_data[2];
    return data;
}