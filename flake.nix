{
  description = "raspberry-pi-nix example";
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" "https://raspberry-pi-nix.cachix.org" "https://nix-community.cachix.org" "https://rcmast3r.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs=" "raspberry-pi-nix.cachix.org-1:WmV2rdSangxW0rZjY/tBvBDSaNFQ3DyEQsVw8EvHn9o=" "rcmast3r.cachix.org-1:dH22dF877RZ1j7uvAgqnQWNChxdQDeqgBRWpXzoi84c=" ];
  };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nix-ros-nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!

    raspberry-pi-nix.url = "github:nix-community/raspberry-pi-nix";
    nixpkgs.url = "github:NixOS/nixpkgs";
    nixpkgs.follows = "raspberry-pi-nix/nixpkgs";
    lb-ros-components.url = "github:RCMast3r/lb_ros_components";
    lb-ros-components.inputs.nixpkgs.follows = "nix-ros-nixpkgs";
    lb-ros-components.inputs.nix-ros-overlay.follows = "nix-ros-overlay";
    ros2-v4l2-camera-src.url = "gitlab:rcmast3r1/ros2_v4l2_camera/compressed_formats";
    ros2-v4l2-camera-src.flake = false;
  };

  outputs = { self, nixpkgs, raspberry-pi-nix, nix-ros-overlay, nix-ros-nixpkgs, lb-ros-components, ros2-v4l2-camera-src }:
    let
      nixpkg_overlays =
        {
          nixpkgs.overlays =
            [
              (self: super: {
                linux-router = super.linux-router.override {
                  useQrencode = false;
                };
              })
            ];
        };
      linux-network-module = ./modules/linux-network.nix;
      
      test-nix-ros-pkgs = import nix-ros-nixpkgs {
        system = "x86_64-linux";
        overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ];
      };
      nix-ros-pkgs = import nix-ros-nixpkgs {
        system = "aarch64-linux";
        overlays = [ nix-ros-overlay.overlays.default my-ros-overlay lb-ros-components.overlays.aarch64-linux ];
      };

      jazzy_ros_packages = with nix-ros-pkgs.rosPackages.jazzy; [ ament-cmake geometry-msgs launch launch-ros ouster-sensor-msgs pcl-conversions pcl-ros rclcpp rclcpp-components rclcpp-lifecycle rosidl-default-runtime sensor-msgs std-msgs std-srvs tf2-ros tf2-eigen ];
      my_overlay = final: prev: {
        imu-gps-driver = final.callPackage ./pkgs/imu_gps_driver.nix { };
        meta-launch = final.callPackage ./pkgs/driver_launch_meta.nix { };
        v4l2-camera = prev.v4l2-camera.overrideAttrs (prev: {
          src = ros2-v4l2-camera-src;
          propagatedBuildInputs = prev.propagatedBuildInputs ++ [ nix-ros-pkgs.rosPackages.jazzy.cv-bridge ];
        });
        ouster-ros = prev.ouster-ros.overrideAttrs (finalAttrs: previousAttrs: {
          buildType = "ament_cmake";
          buildInputs = with nix-ros-pkgs; [ libtins spdlog rosPackages.jazzy.ament-cmake eigen pcl rosPackages.jazzy.rosidl-default-generators rosPackages.jazzy.tf2-eigen ] ++ jazzy_ros_packages;
          checkInputs = [ nix-ros-pkgs.gtest ];
          propagatedBuildInputs = with nix-ros-pkgs; [ curl jsoncpp ] ++ jazzy_ros_packages;
          nativeBuildInputs = with nix-ros-pkgs.rosPackages.jazzy; [ ament-cmake rosidl-default-generators ];
          src = nix-ros-pkgs.fetchurl {
            url = "https://github.com/ros2-gbp/ouster-ros-release/archive/release/jazzy/ouster_ros/0.13.2.tar.gz";
            name = "0.13.2.tar.gz";
            sha256 = "sha256-TEO7xqCYxkDCcXejx0qV/sSL1VQccntUI5+q2KtjOJA=";
          };
        });
      };

      my-ros-overlay = final: prev: {
        rosPackages = prev.rosPackages // { jazzy = prev.rosPackages.jazzy.overrideScope my_overlay; };
      };
      devshell_def = import ./devshell.nix;
    in
    rec {

      devShells.x86_64-linux.default = test-nix-ros-pkgs.mkShell (devshell_def {nix-ros-pkgs=test-nix-ros-pkgs;});

      legacyPackages.x86_64-linux = test-nix-ros-pkgs;
      legacyPackages.aarch64-linux = nix-ros-pkgs;
      nixosConfigurations.test-pi = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";

        modules = [
          linux-network-module
          raspberry-pi-nix.nixosModules.raspberry-pi
          raspberry-pi-nix.nixosModules.sd-image
          basic-config
          {
            _module.args = { inherit nix-ros-pkgs; };
          }
        ];
      };
    };
}
