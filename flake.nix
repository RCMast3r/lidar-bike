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
      linux-network-module = ./linux-network.nix;
      basic-config = { pkgs, lib, nix-ros-pkgs, ... }:
        let
          asdf = (with nix-ros-pkgs.rosPackages.jazzy; buildEnv {
            paths = [
              v4l2-camera
              ros-core
              ros-base
              foxglove-bridge
              rosbag2-storage-mcap
              rmw-cyclonedds-cpp
              ublox
              imu-gps-driver
              ublox-dgnss
              nmea-navsat-driver
              meta-launch
              lidar-bike-components
            ];
          });
        in
        {
          fileSystems."/media/data" =
            {
              device = "/dev/disk/by-uuid/f87fa14e-0519-4377-9988-4a037fcd967f";
              fsType = "ext4";
            };
          systemd.tmpfiles.rules = [
            "d /media/data 0755 nixos nixos -"
          ];

          nix.settings.experimental-features = [ "nix-command" "flakes" ];
          nix.settings.require-sigs = false;
          security.sudo.enable = true;
          hardware.i2c.enable = true;

          system.activationScripts.copyFile = lib.mkAfter ''
            mkdir -p /home/nixos
            cp ${./config/ddsconfig.xml} /home/nixos/ddsconfig.xml
            chown nixos:nixos /home/nixos/ddsconfig.xml
          '';

          environment.variables = {
            RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
            ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST";
            RMW_CONNEXT_PUBLICATION_MODE = "ASYNCHRONOUS";
            CYCLONEDDS_URI = "file:///home/nixos/ddsconfig.xml";
          };
          environment.systemPackages = [
            nix-ros-pkgs.colcon
            pkgs.tmux

            pkgs.ser2net
            # ... other non-ROS packages

            pkgs.i2c-tools
            (pkgs.python3.withPackages (ps: with ps; [ numpy pandas smbus2 i2c-tools ]))
            asdf
          ];

          systemd.services.init_network = {
            script = ''
              /run/current-system/sw/bin/sysctl -w net.core.rmem_max=2147483647
            '';
            wantedBy = [ "multi-user.target" ];
          };

          systemd.services.meta-launch-service = {
            description = "driver ros2 launch Service";
            wantedBy = [ "multi-user.target" ];
            after = [ "network.target" ];
            environment = {
              ROS_LOG_DIR = "/home/nixos/ros_log_dir";
            };
            serviceConfig = {
              path = [ asdf pkgs.bash pkgs.bashInteractive ];
              After = [ "network.target" ];

              ExecStart = "alias bash=\"/run/current-system/sw/bin/bash\" && /run/current-system/sw/bin/ros2 launch meta_launch multi_node_launch.py";
              ExecStop = "/bin/kill -9 $MAINPID";
              Restart = "on-failure";
            };
          };

          systemd.services.sshd.wantedBy = lib.mkOverride 40 [ "multi-user.target" ];
          services.openssh = { enable = true; };
          services.openssh.listenAddresses = [
            {
              addr = "0.0.0.0";
              port = 22;
            }
            {
              addr = ":";
              port = 22;
            }
          ];
          time.timeZone = "America/New_York";
          users.users.root.initialPassword = "root";
          users.users.nixos.group = "nixos";

          users.users.nixos.password = "nixos";
          users.groups.nixos = { };
          users.users.nixos.extraGroups = [ "wheel" "dialout" "i2c" "video" ];

          users.users.nixos.isNormalUser = true;
          networking.firewall.enable = false;

          networking.interfaces.eth0.ipv4 = {
            addresses = [
              {
                address = "169.254.3.5"; # Your static IP address
                prefixLength = 16; # Netmask, 24 for 255.255.255.0
              }
            ];
            routes = [
              {
                address = "0.0.0.0";
                prefixLength = 0;
                via = "169.254.3.1"; # Your gateway IP address
              }
            ];
          };
          raspberry-pi-nix.board = "bcm2712";
          hardware = {
            raspberry-pi = {
              config = {
                all = {

                  options = {
                    i2c_arm_baudrate =
                      {
                        enable = true;
                        value = 400000;
                      };
                  };
                  base-dt-params = {
                    uart0 = {
                      enable = true;
                      value = "on";
                    };
                    i2c_arm = {
                      enable = true;
                      value = "on";
                    };
                    spi = {
                      enable = true;
                      value = "on";
                    };
                  };
                };
              };
            };
          };
          security.rtkit.enable = true;
          services.pipewire = {
            enable = true;
            alsa.enable = true;
            alsa.support32Bit = true;
            pulse.enable = true;
          };
        };
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
        imu-gps-driver = final.callPackage ./imu_gps_driver.nix { };
        meta-launch = final.callPackage ./driver_launch_meta.nix { };
        v4l2-camera = prev.v4l2-camera.overrideAttrs (prev: {
          src = ros2-v4l2-camera-src;
          propagatedBuildInputs = prev.propagatedBuildInputs ++ [ nix-ros-pkgs.rosPackages.jazzy.cv-bridge ];
          # buildInputs = prev.buildInputs ++ [ pkgs.rosPackages.jazzy.cv-bridge ];
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
    in
    rec {

      devShells.x86_64-linux.default = test-nix-ros-pkgs.mkShell {
        shellHook = ''
          sudo sysctl -w net.core.rmem_max=2147483647
        '';
        name = "lidar-bike-env";
        RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
        ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST";
        RMW_CONNEXT_PUBLICATION_MODE = "ASYNCHRONOUS";
        CYCLONEDDS_URI = "file://config/ddsconfig.xml";
        packages = [
          test-nix-ros-pkgs.colcon
          # ... other non-ROS packages
          (with test-nix-ros-pkgs.rosPackages.jazzy; buildEnv {
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
              # (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
              #   propagatedBuildInputs = with test-nix-ros-pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
              #   nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ test-nix-ros-pkgs.pkg-config ];
              # }))
            ];
          })
        ];
      };

      legacyPackages.x86_64-linux = test-nix-ros-pkgs;
      legacyPackages.aarch64-linux = nix-ros-pkgs;
      nixosConfigurations.test-pi = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";

        modules = [
          # (nixpkg_overlays)
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
