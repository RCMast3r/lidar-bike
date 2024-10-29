{ buildRosPackage, ament-cmake, rclcpp, rclcpp-components, sensor-msgs, ament-cmake-ros, ament-lint-auto, ament-lint-common, ros-environment, i2c-tools, tf2 }:
buildRosPackage {
  pname = "simple-berry-driver";
  version = "0.0.1a";
  src = ./imu_driver;

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake-ros rclcpp sensor-msgs rclcpp-components ament-cmake ros-environment i2c-tools tf2 ];
  checkInputs = [ ament-lint-auto ament-lint-common ];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake ];

  meta = {
    description = "simple berry imu gps v3 driver";
  };
}
