# LiDAR-Bike project
## overall description

This project contains both device driver implementations as well as ROS drivers, nodes and launch files for data recording and processing.

The project also contains the integration description through the NixOS description and the integration of a `devshell` for local development usage while testing components
## local usage

0. clone this repo then set your ethernet's address to be something like this (has to be on the same subnet)
    ![alt text](image.png)
1. install nix: 
- `curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install --extra-conf "trusted-users = root $USER" --extra-conf "builders-use-substitutes = true"`

2. switch to the `test_nix_ros_overlay` directory
3. enter a nix dev shell: `nix develop` for each of the following commands
4. run the launch commands (in separate nix dev shells):
- `ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.87.155 viz:=false lidar_mode:=512x20`
    - launches the lidar driver
- `ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2" -p "pixel_format:=mjpeg2rgb" -p frame_rate:=60`
    - launches the usb camera driver
- `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
    - launches the foxglove live view driver

5. start recording all topics:
- `ros2 bag record -a`

## NixOs description

[understand what NixOs is here](https://www.youtube.com/watch?v=bjTxiFLSNFA)

nix tools being used:
- [nix-ros-overlay](https://github.com/lopsided98/nix-ros-overlay)
    - can be understood through [this video](https://vimeo.com/767139940)
- [raspberry-pi-nix](https://github.com/nix-community/raspberry-pi-nix)


### nixos building commands

```nix build .#nixosConfigurations.test-pi.config.system.build.toplevel --builders "ssh://ubuntu@100.125.71.41" --system aarch64-linux --max-jobs 0 -L```

```nix build .#nixosConfigurations.test-pi.config.system.build.toplevel --system aarch64-linux```

```nix build .#nixosConfigurations.test-pi.config.system.build.sdImage --system aarch64-linux```

```sudo zstd -d result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img.zst```

```sudo dd bs=4M if=result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img of=/dev/sda conv=fsync oflag=direct status=progress```


## nixos usage

1. ssh in (password is `nixos`):
```ssh nixos@192.168.1.4``` 

2. increase network buffer size:
```sudo sysctl -w net.core.rmem_max=2147483647```

3. launch everything:

- `ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.87.155 viz:=false lidar_mode:=512x20`
    - launches the lidar driver
- `ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2" -p "pixel_format:=mjpeg2rgb" -p frame_rate:=60`
    - launches the usb camera driver
- `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
    - launches the foxglove live view driver
- `ros2 run imu_driver imu_node`
    - launches the imu driver
- `ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyUSB0 -p baud:=9600`
    - launches the gps driver


## errata and details

### information about the berry gps / imu combo 

#### i2c connection
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
