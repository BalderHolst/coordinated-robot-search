{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      system = "aarch64-linux";
      pkgs = import nixpkgs { inherit system; };
      opencvGtk = pkgs.opencv.override (old : { enableGtk2 = true; });
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          cmake
          opencvGtk
          rustc
          cargo
        ];
        nativeBuildInputs = with pkgs; [
          pkg-config
        ];
        shellHook = ''
          echo "Entering devShell"

          export LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib"

          export C_INCLUDE_PATH="${pkgs.llvmPackages.libclang.lib}/lib/clang/19/include/"
          export CPLUS_INCLUDE_PATH="${pkgs.llvmPackages.libclang.lib}/lib/clang/19/include/"
          export CPATH="${pkgs.llvmPackages.libclang.lib}/lib/clang/19/include/"

          export C="${pkgs.clang}/bin/clang"
          export CXX="${pkgs.clang}/bin/clang++"

          echo "Nix development environment loaded!"
        '';
      };
    };
}
