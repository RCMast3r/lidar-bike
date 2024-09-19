{
  description = "raspberry-pi-nix example";
  nixConfig = {
    extra-substituters = [ "https://raspberry-pi-nix.cachix.org" ];
    extra-trusted-public-keys = [
      "raspberry-pi-nix.cachix.org-1:WmV2rdSangxW0rZjY/tBvBDSaNFQ3DyEQsVw8EvHn9o="
    ];
  };

  inputs = {
    raspberry-pi-nix.url = "github:nix-community/raspberry-pi-nix";
    nixpkgs.url = "github:NixOS/nixpkgs/205fd4226592cc83fd4c0885a3e4c9c400efabb5";
    nixpkgs.follows = "raspberry-pi-nix/nixpkgs";
  };

  outputs = { self, nixpkgs, raspberry-pi-nix }:
    let
      inherit (nixpkgs.lib) nixosSystem;
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      basic-config = { pkgs, lib, ... }: {
        # bcm2711 for rpi 3, 3+, 4, zero 2 w
        # bcm2712 for rpi 5
        # See the docs at:
        # https://www.raspberrypi.com/documentation/computers/linux_kernel.html#native-build-configuration
        raspberry-pi-nix.board = "bcm2712";
        time.timeZone = "America/New_York";
        users.users.root.initialPassword = "root";
        networking = {
          hostName = "basic-example";
          useDHCP = false;
          interfaces = {
            wlan0.useDHCP = true;
            eth0.useDHCP = true;
          };
        };
        hardware = {
          bluetooth.enable = true;
          raspberry-pi = {
            config = {
              all = {
                base-dt-params = {
                  # enable autoprobing of bluetooth driver
                  # https://github.com/raspberrypi/linux/blob/c8c99191e1419062ac8b668956d19e788865912a/arch/arm/boot/dts/overlays/README#L222-L224
                  krnbt = {
                    enable = true;
                    value = "on";
                  };
                  i2c_arm = {
                    enable = true;
                    value = "on";
                  };
                };
              };
            };
          };
        };
      };

    in
    rec {
      legacyPackages.x86_64-linux = import nixpkgs {
        system = "x86_64-linux";
      };
      nixosConfigurations.test-pi = nixpkgs.lib.nixosSystem {
        system = "x86_64-linux";

        modules = [
          {
            nixpkgs.pkgs = pkgs.pkgsCross.aarch64-multiplatform;
          }
          raspberry-pi-nix.nixosModules.raspberry-pi
          basic-config
        ];
      };
    };
}
