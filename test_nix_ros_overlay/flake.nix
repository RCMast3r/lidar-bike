{
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let

        
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "Example project";
          packages = [
            pkgs.colcon
            # ... other non-ROS packages
            (with pkgs.rosPackages.humble; buildEnv {
                paths = [
                    ros-core
                    ros-base
                    foxglove-bridge
                    rosbag2-storage-mcap
                    (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
                      propagatedBuildInputs = with pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
                      nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ pkgs.pkg-config ];
                    }))
                ];
            })
          ];
        };

        legacyPackages = pkgs;
      });
  
}