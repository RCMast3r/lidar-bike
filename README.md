# LiDAR-Bike

- `nix develop`

- `ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:="/dev/video2" -p "pixel_format:=mjpeg2rgb" -p frame_rate:=60`

- `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`