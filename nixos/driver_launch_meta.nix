{ buildRosPackage, ament-cmake-ros, ament-cmake, imu-gps-driver, ublox, rclpy, launch-ros }:
buildRosPackage {
  pname = "ros-jazzy-meta-launch";
  version = "0.0.1a";
  src = ./driver_launch;

  buildType = "ament_python";
  propagatedBuildInputs = [ imu-gps-driver ublox launch-ros ];
  nativeBuildInputs = [ rclpy ];

  meta = {
    description = "launch meta";
  };
}
