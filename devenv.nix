{ pkgs, ... }:

{
  # Not a trusted Nix user, so let devenv skip auto-managing its Cachix cache.
  cachix.enable = false;

  # https://devenv.sh/packages/
  packages = [
    pkgs.gcc
    pkgs.gnumake
    pkgs.gdb
    pkgs.clang-tools
    pkgs.rustc
  ];

  # https://devenv.sh/scripts/
  scripts.build.exec = "make all";
  scripts.clean.exec = "make clean";
  scripts.run.exec = "make all && ./bin/main \"$@\"";

  enterShell = ''
    echo "vmplib C++ environment ready — g++, make, gdb, clangd and rustc are on PATH"
  '';

  # See full reference at https://devenv.sh/reference/options/
}
