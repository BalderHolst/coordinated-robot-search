#!/usr/bin/env bash

ROOT=$(git rev-parse --show-toplevel)
FIGDIR="$ROOT/report/figures/generated-worlds"

# Create figure directory
mkdir -p "$FIGDIR"

# Generate worlds and images
cargo run --manifest-path "$ROOT/trainer/Cargo.toml" -- world-gen --force -n 6 --output "$FIGDIR" --render "$FIGDIR"

# Remove worlds
rm -v "$FIGDIR"/*.ron
