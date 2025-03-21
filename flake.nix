{
    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
        flake-utils.url = "github:numtide/flake-utils";
    };
    outputs = { flake-utils, nixpkgs, ... }:
        flake-utils.lib.eachDefaultSystem (system:
        let
            pkgs = import nixpkgs { inherit system; };
        in {
        packages = rec {
            site = pkgs.stdenv.mkDerivation {
                name = "site";
                unpackPhase = "true";
                installPhase = ''
                    mkdir -p $out
                    cp -r ${report} $out/report.pdf
                    cp -r ${botbrain-docs} $out/docs
                '';
            };
            report = pkgs.stdenv.mkDerivation {
                name = "report";
                src = ./report;
                buildInputs = with pkgs; [ texliveFull ];
                shellHook = ''
                    export SIMULATOR="$(pwd)/simple_sim/target/release/simple_sim"
                    export DATA_DIR="$(pwd)/data"
                    export PLOT_DIR="$(pwd)/report/figures/plot"
                '';
                buildPhase = ''
                    latexmk -pdf -bibtex-cond -shell-escape -interaction=nonstopmode main.tex || true
                '';
                installPhase = "cp main.pdf $out";
            };
            botbrain-docs = pkgs.stdenv.mkDerivation rec {
                name = "docs";
                src = ./botbrain;
                cargoDeps = pkgs.rustPlatform.importCargoLock {
                    lockFile = src + "/Cargo.lock";
                };
                buildInputs = with pkgs; [ cargo rustPlatform.cargoSetupHook ];
                buildPhase = ''
                    cargo doc --no-deps --frozen
                '';
                installPhase = ''
                    mkdir -p $out
                    cp -r target/doc/* $out
                '';
            };
        };
        devShells.default = pkgs.mkShell {
            packages = with pkgs; [
                git
                (python3.withPackages (ps: with ps; [
                    polars
                    matplotlib
                ]))
            ];

            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
                wayland
                libGL
                libxkbcommon
            ]);

            shellHook = /* bash */ ''

                # Set up git hooks
                git config core.hooksPath .hooks

                export SIMULATOR="$(pwd)/simple_sim/target/release/simple_sim"
                export DATA_DIR="$(pwd)/data"
                export PLOT_DIR="$(pwd)/report/figures/plot"


                export PYTHONPATH="$PYTHONPATH:$(pwd)/botplot/src/"
            '';

        };
    });
}
