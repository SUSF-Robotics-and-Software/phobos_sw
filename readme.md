# Phobos Software

Software for the University of Southampton Spaceflight Socieity's Phobos Rover.

Written in **Rust**.

## Architecture

There are two executables and one library:

* `rov_exec`: Rover executable - runs on the Rover's Raspberry Pi 4.
* `gnd_exec`: Ground station executbale - runs at the groundstation and commands the Rover.
* `comms_if`: Communications interface library providing for coherent Telemetry and Telecommand (TmTc) between the ground station and rover.
* `util`: Utility library including logging, archiving, and any other concept which is used in both executbales but does not fit into the reams of communications.

## Development and Executing

For testing use `cargo run --bin <BINARY_NAME>` to execute a specific binary.

Use `cargo build` to just build and not run.

When running real events (like competition missions) use `cargo run --release --bin <BINARY_NAME>` to use release mode, which optimises the binaries and makes execution far faster.

I recommend using `rust-analyser` for VSCode linting/development.
