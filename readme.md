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

`rov_exec` uses the `eyre` crate for error handling which produces nice
backtraces. To enable them prepend `RUST_BACKTRACE=1` to your run command. Will
not work properly under release mode due to a lack of symbols.

I recommend using `rust-analyser` for VSCode linting/development.

## Scripts

`rov_exec` is capable of running scripts (which are stored in the `scripts`
directory). To run a specific script append it's path to the end of the run
command, such as:

```shell
RUST_BACKTRACE=1 cargo run --bin rov_exec scripts/demo_01.prs
```

## Tools

Two tools (shell scripts) are provided for ease of use:

* `clearsessions`: Clear all session directories. Make sure to backup the ones
    you want!
* `synctopi`: Send the current directory to a raspberry pi (or anyting else,
    make sure you change the username, ip address, or path if needed)
