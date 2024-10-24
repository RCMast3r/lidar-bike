{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ];
        };
        my_overlay = final: prev: {
          imu-gps-driver = final.callPackage ./imu_gps_driver.nix { };
        };

        my-ros-overlay = final: prev: {
          rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
        };
      in {
        devShells.default = pkgs.mkShell {
          shellHook = ''
            sudo sysctl -w net.core.rmem_max=2147483647
          '';
          name = "lidar-bike-env";
          RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file://config/ddsconfig.xml";
          packages = [
            pkgs.colcon
            
            # ... other non-ROS packages
            (with pkgs.rosPackages.jazzy; buildEnv {
                paths = [
                    ros-core
                    ros-base
                    foxglove-bridge
                    rosbag2-storage-mcap
                    ouster-ros
                    rmw-cyclonedds-cpp
                    ublox
                    imu-gps-driver
                    (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
                      propagatedBuildInputs = with pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
                      nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ pkgs.pkg-config ];
                    }))
                ];
            })
          ];
        };

        packages = pkgs;
      });
  
}
