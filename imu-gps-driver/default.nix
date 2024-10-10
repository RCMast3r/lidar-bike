{ pkgs, stdenv, cmake, i2c-tools, ... }:

stdenv.mkDerivation {
  name = "drivebrain_software";
  src = ./.;
  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ i2c-tools ];
}