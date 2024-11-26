{
  description = "raspberry-pi-nix example";
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };

  # nixConfig = {
  #   extra-substituters = [
  #     # "https://raspberry-pi-nix.cachix.org"
  #     # "https://nix-community.cachix.org"
  #   ];
  #   extra-trusted-public-keys = [
  #     # "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
  #     # "raspberry-pi-nix.cachix.org-1:WmV2rdSangxW0rZjY/tBvBDSaNFQ3DyEQsVw8EvHn9o="
  #   ];
  # };

  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/develop";
    nix-ros-nixpkgs.follows = "nix-ros-overlay/nixpkgs"; # IMPORTANT!!!

    raspberry-pi-nix.url = "github:nix-community/raspberry-pi-nix";
    nixpkgs.url = "github:NixOS/nixpkgs";
    nixpkgs.follows = "raspberry-pi-nix/nixpkgs";
  };

  outputs = { self, nixpkgs, raspberry-pi-nix, nix-ros-overlay, nix-ros-nixpkgs }:
    let
      basic-config = { pkgs, lib, nix-ros-pkgs, ... }: let
        asdf = (with nix-ros-pkgs.rosPackages.jazzy; buildEnv {
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
                    (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
                      propagatedBuildInputs = with nix-ros-pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
                      nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ nix-ros-pkgs.pkg-config ];
                    }))
                ];
            });
      in {
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
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file:///home/nixos/ddsconfig.xml";
        };
        environment.systemPackages = [
          nix-ros-pkgs.colcon
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

          serviceConfig = {
            path = [asdf];
            After = [ "network.target" ];
            ExecStart = "/run/current-system/sw/bin/ros2 launch meta_launch multi_node_launch.py";
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
        overlays = [ nix-ros-overlay.overlays.default my-ros-overlay ];
      };
      my_overlay = final: prev: {
        imu-gps-driver = final.callPackage ./imu_gps_driver.nix { };
        meta-launch = final.callPackage ./driver_launch_meta.nix { };
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
          ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST";
          RMW_CONNEXT_PUBLICATION_MODE="ASYNCHRONOUS";
          CYCLONEDDS_URI="file://config/ddsconfig.xml";
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
                    (usb-cam.overrideAttrs (finalAttrs: previousAttrs: {
                      propagatedBuildInputs = with test-nix-ros-pkgs; [ builtin-interfaces camera-info-manager cv-bridge ffmpeg_4 image-transport image-transport-plugins rclcpp rclcpp-components rosidl-default-runtime sensor-msgs std-msgs std-srvs v4l-utils ];
                      nativeBuildInputs = previousAttrs.nativeBuildInputs ++ [ test-nix-ros-pkgs.pkg-config ];
                    }))
                ];
            })
          ];
        };
        
      legacyPackages.x86_64-linux = test-nix-ros-pkgs;
      legacyPackages.aarch64-linux = nix-ros-pkgs;
      nixosConfigurations.test-pi = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";

        modules = [
          raspberry-pi-nix.nixosModules.raspberry-pi
          basic-config
          {
            _module.args = { inherit nix-ros-pkgs; };
          }
        ];
      };
    };
}
