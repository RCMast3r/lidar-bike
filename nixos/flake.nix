{
  description = "raspberry-pi-nix example";
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
    raspberry-pi-nix.url = "github:nix-community/raspberry-pi-nix";
    nixpkgs.url = "github:NixOS/nixpkgs";
    nixpkgs.follows = "raspberry-pi-nix/nixpkgs";
  };

  outputs = { self, nixpkgs, raspberry-pi-nix }:
    let
      basic-config = { pkgs, lib, ... }: {
        nix.settings.experimental-features = [ "nix-command" "flakes" ];
        nix.settings.require-sigs = false;
        security.sudo.enable = true;
        hardware.i2c.enable = true;
        environment.systemPackages = [
          pkgs.i2c-tools
          (pkgs.python3.withPackages (ps: with ps; [ numpy pandas smbus2 i2c-tools ]))
          # pkgs.python311Packages.i2c-tools
          # pkgs.python311Packages.smbus2

        ];
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
        users.users.nixos.extraGroups = [ "wheel" "dialout" "i2c" ];

        users.users.nixos.isNormalUser = true;
        networking.firewall.enable = false;

        networking.interfaces.eth0.ipv4 = {
          addresses = [
            {
              address = "192.168.1.69"; # Your static IP address
              prefixLength = 24; # Netmask, 24 for 255.255.255.0
            }
          ];
          routes = [
            {
              address = "0.0.0.0";
              prefixLength = 0;
              via = "192.168.1.1"; # Your gateway IP address
            }
          ];
        };
        raspberry-pi-nix.board = "bcm2712";
        hardware = {
          raspberry-pi = {
            config = {
              all = {
                # base-dt-params = {
                # BOOT_UART = {
                #   value = 1;
                #   enable = true;
                # };
                # uart_2ndstage = {
                #   value = 1;
                #   enable = true;
                # };
                options = {
                  i2c_arm_baudrate =
                    {
                      enable = true;
                      value = 400000;
                    };
                };
                base-dt-params = {
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


    in
    rec {

      nixosConfigurations.test-pi = nixpkgs.lib.nixosSystem {
        system = "aarch64-linux";

        modules = [
          raspberry-pi-nix.nixosModules.raspberry-pi
          basic-config
        ];
      };
    };
}
