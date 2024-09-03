# LiDAR-Bike

within `test_nix_ros_overlay`:

- `nix develop`

- `ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2" -p "pixel_format:=mjpeg2rgb" -p frame_rate:=60`

- `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`

- `ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=169.254.87.155 viz:=false lidar_mode:=512x20`