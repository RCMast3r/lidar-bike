## information about the gps / imu combo 

### i2c connection
addresses: 

| address   | device |
| -------- | ------- |
| 0x6A     | for the gyroscope and accelerometer |
| 0x1C     | for the magnetometer |
| 0x77     | for the pressure sensor |
| 0x42     | GPS |

- gps: uBlox CAM-M8
    - https://ozzmaker.com/accessing-gps-via-i2c-on-a-berrygps-imu/
        - TODO: jump the jp11 and jp10 jumpers

- gyro / accel: LSM6DSL
    - for the imu, RT
- Magnetometer – LIS3MDL
- pressure sensor: BM388
