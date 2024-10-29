{ buildRosPackage, ament-cmake-ros, ament-cmake, ament-python, imu-gps-drive, ublox, rclpy }:
buildRosPackage {
  pname = "simple-berry-driver";
  version = "0.0.1a";
  src = ./driver_launch;

  buildType = "ament_python";
  propagatedBuildInputs = [ ament-cmake-ros ament-python imu-gps-driver ublox rclpy];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake ament-python ];

  meta = {
    description = "launch meta";
  };
}
