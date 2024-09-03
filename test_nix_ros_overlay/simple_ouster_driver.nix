{ src, lib, eigen, jsoncpp, curl, libtins, libGLU, glew, glfw, buildRosPackage, ament-cmake, rclcpp, rclcpp-components, sensor-msgs, ament-cmake-ros, ament-lint-auto, ament-lint-common, git, ros-environment }:
buildRosPackage {
  pname = "simple-ouster-driver";
  version = "0.0.1a";
  inherit src;

  buildType = "ament_cmake";
  buildInputs = [ jsoncpp curl libGLU glew glfw libtins eigen ament-cmake-ros rclcpp sensor-msgs rclcpp-components ament-cmake git ros-environment ];
  checkInputs = [ ament-lint-auto ament-lint-common ];
  nativeBuildInputs = [ ament-cmake-ros ament-cmake git ];

  meta = {
    description = "simple ouster driver";
    # license = with lib.licenses; [ asl20 ];
  };
}
