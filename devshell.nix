{nix-ros-pkgs, ... }:
{
  shellHook = ''
    sudo sysctl -w net.core.rmem_max=2147483647
  '';
  name = "lidar-bike-env";
  RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
  ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST";
  RMW_CONNEXT_PUBLICATION_MODE = "ASYNCHRONOUS";
  CYCLONEDDS_URI = "file://config/ddsconfig.xml";
  packages = [
    nix-ros-pkgs.colcon
    # ... other non-ROS packages
    (with nix-ros-pkgs.rosPackages.jazzy; buildEnv {
      paths = [
        ros-core
        ros-base
        foxglove-bridge
        rosbag2-storage-mcap
        ouster-ros
        rmw-cyclonedds-cpp
        ublox
        imu-gps-driver
        ublox-dgnss
        nmea-navsat-driver
        meta-launch
      ];
    })
  ];
}
