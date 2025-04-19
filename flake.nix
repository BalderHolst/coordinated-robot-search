{
    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
        flake-utils.url = "github:numtide/flake-utils";
        rust-overlay = {
            url = "github:oxalica/rust-overlay";
            inputs.nixpkgs.follows = "nixpkgs";
        };
    };
    outputs = 
    { flake-utils, nixpkgs, rust-overlay, ... }:
        flake-utils.lib.eachDefaultSystem (system:
        let
            overlays = [ (import rust-overlay) ];
            pkgs = import nixpkgs { inherit system overlays; };
            all-crates = pkgs: name: cmd: ignore:
                (pkgs.writeShellScriptBin "${name}" ''
                    ROOT=$(git rev-parse --show-toplevel)
                    MANIFESTS=$(find $ROOT -type f -name Cargo.toml)
                    for MANIFEST in $MANIFESTS; do
                        MANIFEST=$(realpath -s --relative-to="." "$MANIFEST")
                        CRATE=$(dirname $MANIFEST)
                        CRATE_NAME=$(basename $CRATE)

                        ${builtins.concatStringsSep "\n" (map (ignored: /*bash*/ ''
                            [[ "$CRATE_NAME" == "${ignored}" ]] && continue
                        '') ignore)}

                        echo "Running '${cmd}'"
                        ${cmd} || exit 1
                    done
                '');
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
                (all-crates pkgs 
                    "cargo-fmt-all"
                    "cargo fmt --all --manifest-path $MANIFEST"
                    []
                )
                (all-crates pkgs
                    "cargo-hack-all"
                    "cargo hack --manifest-path $MANIFEST --feature-powerset check"
                    ["multi_robot_control"]
                )
                (all-crates pkgs
                    "cargo-clippy-all"
                    "cargo clippy --all-features --manifest-path $MANIFEST -- -D warnings"
                    ["multi_robot_control"]
                )
                (all-crates pkgs
                    "cargo-machete-all"
                    "cargo machete --manifest-path $MANIFEST"
                    ["multi_robot_control"]
                )
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
