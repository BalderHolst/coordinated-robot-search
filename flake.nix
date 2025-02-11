{
    inputs = {
        flake-utils.url = "github:numtide/flake-utils";
    };
    outputs = { flake-utils, nixpkgs, ... }:
        flake-utils.lib.eachDefaultSystem (system:
        let
            pkgs = import nixpkgs { inherit system; };
        in {
        devShells.default = pkgs.mkShell {
            packages = with pkgs; [
                git
                docker
            ];

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
                wayland
                libGL
                libxkbcommon
            ]);

            shellHool = /* bash */ ''

                # Set up git hooks
                git config core.hooksPath .hooks

            '';

        };
    });
}
