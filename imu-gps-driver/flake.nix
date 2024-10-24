{
  description = "drivebrain flake";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.05";

    flake-parts.url = "github:hercules-ci/flake-parts";
    flake-parts.inputs.nixpkgs-lib.follows = "nixpkgs";

  };
  outputs = { self, nixpkgs, flake-parts, ... }@inputs:
    flake-parts.lib.mkFlake { inherit inputs; }

      {
        systems = [
          "x86_64-linux"
          "aarch64-linux"
        ];

        flake.overlays = {
          driver-overlay = final: prev: {
            imu-gps-driver = final.callPackage ./default.nix { };
          };

        };

        perSystem = { config, pkgs, system, ... }:

          {
            _module.args.pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [
                self.overlays.driver-overlay
              ];
              config = { };
            };
            packages.default = pkgs.imu-gps-driver;
            packages.imu-gps-driver = pkgs.imu-gps-driver;

            devShells.default = pkgs.mkShell rec {
              name = "nix-devshell";
              shellHook =
                let icon = "f121";
                in ''
                  dbc_path=${pkgs.ht_can_pkg}
                  export DBC_PATH=$dbc_path
                  export PS1="$(echo -e '\u${icon}') {\[$(tput sgr0)\]\[\033[38;5;228m\]\w\[$(tput sgr0)\]\[\033[38;5;15m\]} (${name}) \\$ \[$(tput sgr0)\]"
                  alias build="rm -rf build && mkdir build && cd build && cmake .. && make -j && cd .."
                '';
              inputsFrom = [
                pkgs.imu-gps-driver
              ];
            };
            legacyPackages =
              import nixpkgs {
                inherit system;
                overlays = [
                  self.overlays.driver-overlay
                ];
              };
          };
      };

}