{
    inputs = {
        flake-utils.url = "github:numtide/flake-utils";
        nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
        nixpkgs.follows = "nix-ros-overlay/nixpkgs";
    };
    outputs = { flake-utils, nixpkgs, ... }@inputs:
        flake-utils.lib.eachDefaultSystem (system:
            let
                pkgs = import nixpkgs {
                    inherit system;
                    overlays = [ inputs.nix-ros-overlay.overlays.default ];
                };
            in {
        devShells.default = pkgs.mkShell {
            packages = with pkgs; [
                git
                docker
                colcon
                (with rosPackages.humble; buildEnv {
                    paths = [
                        ros-core
                        rviz2
                    ];
                })
            ];

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
                wayland
                libGL
                libxkbcommon
            ]);

            shellHool = ''
                # Setup git hooks
                git config core.hooksPath .hooks
            '';

        };
    });
    nixConfig = {
        extra-substituters = [ "https://ros.cachix.org" ];
        extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
    };
}
