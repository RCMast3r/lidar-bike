```nix build .#nixosConfigurations.test-pi.config.system.build.toplevel --builders "ssh://ubuntu@100.125.71.41" --system aarch64-linux --max-jobs 0 -L```


```nix build .#nixosConfigurations.test-pi.config.system.build.toplevel --system aarch64-linux```

```nix build .#nixosConfigurations.test-pi.config.system.build.sdImage --system aarch64-linux```

```sudo zstd -d result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img.zst```

```sudo dd bs=4M if=result/sd-image/nixos-sd-image-<version-tag-here>-aarch64-linux.img of=/dev/sda conv=fsync oflag=direct status=progress```


## usage

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