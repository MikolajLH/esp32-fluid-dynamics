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
        ];
      in {
        devShells = {
          default = envs.arduino.overrideAttrs (
            oldAttrs: {buildInputs = oldAttrs.buildInputs ++ packages;}
          );
        };
      }
    );
}
