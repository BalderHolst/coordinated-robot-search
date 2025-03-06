# Parameters from Rust

[Params](https://github.com/sequenceplanner/r2r/blob/master/r2r/examples/parameters.rs)
[Params trait (derive)](https://github.com/sequenceplanner/r2r/blob/master/r2r/examples/parameters_derive.rs)

## Call from ros2 run

```bash
ros2 run {package} {node_executable} --ros-args -p par1:=5.1 -p nested.par4:=42 -r __ns:=/demo -r __node:=my_node
```

## Call directly from cargo

```rust
cargo run {node_executable} -- --ros-args -p par1:=5.1 -p nested.par4:=42 -r __ns:=/demo -r __node:=my_node
```
