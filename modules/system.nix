{ pkgs, lib, nix-ros-pkgs, ... }:
let
  ros-env = (with nix-ros-pkgs.rosPackages.jazzy; buildEnv {
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
    ros-env
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
      path = [ ros-env pkgs.bash pkgs.bashInteractive ];
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
}