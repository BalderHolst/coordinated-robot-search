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
            default = pkgs.stdenv.mkDerivation {
                name = "coordinated-robot-search";
                unpackPhase = "true"; # No `src`
                installPhase = ''
                    mkdir -p $out/bin
                    mkdir -p $out/docs

                    cp ${simple_sim}/bin/simple_sim $out/bin
                    cp ${trainer}/bin/trainer $out/bin

                    cp -r ${botbrain-docs} $out/docs/botbrain
                    cp -r ${simple_sim-docs} $out/docs/simple_sim

                    cp -r ${report} $out/report.pdf
                '';
            };
            trainer = pkgs.stdenv.mkDerivation rec {
                name = "trainer";
                src = ./.;
                cargoRoot = name;
                cargoDeps = pkgs.rustPlatform.importCargoLock {
                    lockFile = src + "/${cargoRoot}/Cargo.lock";
                };
                buildInputs = with pkgs; [ cargo rustPlatform.cargoSetupHook ];
                buildPhase = ''
                    cargo build --release --frozen --manifest-path ${cargoRoot}/Cargo.toml
                '';
                installPhase = ''
                    mkdir -p $out/bin
                    cp -r ./${cargoRoot}/target/release/trainer $out/bin
                '';
            };
            simple_sim = pkgs.stdenv.mkDerivation rec {
                name = "simple_sim";
                src = ./.;
                cargoRoot = name;
                cargoDeps = pkgs.rustPlatform.importCargoLock {
                    lockFile = src + "/${cargoRoot}/Cargo.lock";
                };
                buildInputs = with pkgs; [ cargo rustPlatform.cargoSetupHook ];
                buildPhase = ''
                    cargo build --release --frozen --manifest-path ${cargoRoot}/Cargo.toml
                '';
                installPhase = ''
                    mkdir -p $out/bin
                    cp -r ./${cargoRoot}/target/release/simple_sim $out/bin
                '';
            };
            site = pkgs.stdenv.mkDerivation {
                name = "site";
                unpackPhase = "true";
                installPhase = ''
                    mkdir -p $out/docs
                    cp -r ${report} $out/report.pdf
                    cp -r ${botbrain-docs} $out/docs/botbrain
                    cp -r ${simple_sim-docs} $out/docs/simple_sim
                '';
            };
            report = pkgs.stdenv.mkDerivation {
                name = "report";
                src = ./report;
                buildInputs = with pkgs; [ texliveFull ];
                buildPhase = ''
                    latexmk -pdf -bibtex-cond -shell-escape -interaction=nonstopmode main.tex || true
                    latexmk -pdf -bibtex-cond -shell-escape -interaction=nonstopmode main.tex || true
                '';
                installPhase = "cp main.pdf $out";
            };
            botbrain-docs = pkgs.stdenv.mkDerivation rec {
                name = "botbrain";
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
            simple_sim-docs = pkgs.stdenv.mkDerivation rec {
                name = "simple_sim";
                src = ./.;
                cargoRoot = name;
                cargoDeps = pkgs.rustPlatform.importCargoLock {
                    lockFile = src + "/${cargoRoot}/Cargo.lock";
                };
                buildInputs = with pkgs; [ cargo rustPlatform.cargoSetupHook ];
                buildPhase = ''
                    cargo doc --no-deps --frozen --manifest-path ${cargoRoot}/Cargo.toml
                '';
                installPhase = ''
                    mkdir -p $out
                    cp -r ./${cargoRoot}/target/doc/* $out
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
                samply        # Profiling tool
                pandoc        # Markdown to PDF conversion
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
                    "cargo hack --manifest-path $MANIFEST --feature-powerset clippy -- -D warnings"
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

                export PYTHONPATH="$PYTHONPATH:$(pwd)/botplot/src/"
            '';

        };
    });
}
