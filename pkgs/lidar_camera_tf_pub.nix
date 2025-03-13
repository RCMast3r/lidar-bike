{ buildRosPackage, ament-cmake-ros, ament-cmake, geometry-msgs, sensor-msgs, rclpy, launch-ros, python311Packages }:
buildRosPackage {
  pname = "ros-jazzy-lidar-camera-tf-pub";
  version = "0.0.1a";
  src = ./lidar_camera_tf_pub;

  buildType = "ament_python";
  buildInputs = [ geometry-msgs sensor-msgs python311Packages.opencv4 python311Packages.tkinter ];
  propagatedBuildInputs = [ ];
  nativeBuildInputs = [ rclpy ];

  meta = {
    description = "launch meta";
  };
}

