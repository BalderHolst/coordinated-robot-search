{
    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
        flake-utils.url = "github:numtide/flake-utils";
        rust-overlay = {
            url = "github:oxalica/rust-overlay";
            inputs.nixpkgs.follows = "nixpkgs";
        };
    };
    outputs = { flake-utils, nixpkgs, rust-overlay, ... }:
        flake-utils.lib.eachDefaultSystem (system:
        let
            overlays = [ (import rust-overlay) ];
            pkgs = import nixpkgs { inherit system overlays; };
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
                rust-bin.stable.latest.default
                (python3.withPackages (ps: with ps; [
                    polars
                    matplotlib
                ]))
                cargo-hack    # Check feature combinations
                cargo-machete # Find unused dependencies
                (pkgs.writeShellScriptBin "cargo-fmt-all" ''
                    ROOT=$(git rev-parse --show-toplevel)
                    CRATES=$(find $ROOT -type f -name Cargo.toml)
                    for abs_path in $CRATES; do
                        rel_path=$(realpath -s --relative-to="$ROOT" "$abs_path")
                        echo "Formatting $(dirname $rel_path)"
                        cargo fmt --all --manifest-path $ROOT/$rel_path
                    done
                '')
            ];

            env = {
                LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
                    wayland
                    libGL
                    libxkbcommon
                    vulkan-loader
                ]);
            };

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
