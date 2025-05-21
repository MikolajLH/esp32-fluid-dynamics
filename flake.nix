{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flakeUtils.url = "github:numtide/flake-utils";
    environments.url = "gitlab:Rellikeht/environments";
  };

  outputs = {
    self,
    nixpkgs,
    flakeUtils,
    environments,
  }:
    flakeUtils.lib.eachDefaultSystem (
      system: let
        pkgs = nixpkgs.legacyPackages.${system};
        envs = environments.devShells.${system};

        packages = with pkgs; [
          arduino-ide

          uv
          python313Full
        ];

        libs = with pkgs; [
          zlib
          stdenv.cc.cc.lib
          glib

          # for matplotlib to fucking work
          fontconfig
          xorg.libX11
          libxkbcommon
          freetype
          dbus
        ];
        ld_path = "${pkgs.lib.makeLibraryPath libs}";
      in {
        devShells = {
          default = envs.arduino.overrideAttrs (
            oldAttrs: {
              buildInputs = oldAttrs.buildInputs ++ packages ++ libs;
              shellHook = ''
                export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${ld_path}"
              '';
            }
          );
        };
      }
    );
}
