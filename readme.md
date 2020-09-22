# Phobos Software

Software for the University of Southampton Spaceflight Socieity's Phobos Rover.

Written in **Rust**.

## Architecture

> :warning: **Note**: The architecture is currently undergoing a rework, which
> is being tracked in [this issue](https://github.com/SUSF-Robotics-and-Software/phobos_sw/issues/8).

There are two executables and two libraries:

* `rov_exec`: Rover executable - runs on the Rover's Raspberry Pi 4.
* `gnd_exec`: Ground station executbale - runs at the groundstation and commands the Rover.
* `comms_if`: Communications interface library providing for coherent Telemetry and Telecommand (TmTc) between the ground station and rover.
* `util`: Utility library including logging, archiving, and any other concept which is used in both executbales but does not fit into the reams of communications.


## Scripts

`rov_exec` is capable of running scripts (which are stored in the `scripts`
directory). To run a specific script append it's path to the end of the run
command, such as:

```shell
RUST_BACKTRACE=1 cargo run --bin rov_exec scripts/demo_01.prs
```
## Requirements

The following are required to be able to build and run the software:

- `rust` - Follow the guide [on the rust website](https://www.rust-lang.org/learn/get-started) for your OS
  > **Note**: For Windows you should be using the `x86_64-pc-windows-msvc` default
  > build target, *not* `i686-pc-windows-msvc`.
- `python` - Currently the electronic driver uses a python interface, though this
  should be changed in the future. For now this means that python 3.7+ and the 
  associated development libs are required.

  **Linux**: `sudo apt install python3-dev` should do the trick.

  **Windows**: Either use anaconda, which has the dll, or place the correct 
  version of the dll in the same directory as `python37.exe`.

## Development and Executing

Before running the software setup an environment variable to point to the 
root of the `phobos_sw` directory. This is used to easily find the sessions and
params directories. For example, in your `.bashrc` file:

```bash
export SUSF_PHOBOS_SW_ROOT="/c/Users/{USR}/Development/SUSF-Robotics-and-Software/phobos_sw"
```

For testing use `cargo run --bin <BINARY_NAME>` to execute a specific binary.

Use `cargo build` to just build and not run.

When running real events (like competition missions) use `cargo run --release --bin <BINARY_NAME>` to use release mode, which optimises the binaries and makes execution far faster.

`rov_exec` uses the `eyre` crate for error handling which produces nice
backtraces. To enable them prepend `RUST_BACKTRACE=1` to your run command. Will
not work properly under release mode due to a lack of symbols.

I recommend using `rust-analyser` for VSCode linting/development.

## Tools

Two tools (shell scripts) are provided for ease of use:

* `clearsessions`: Clear all session directories. Make sure to backup the ones
    you want!
* `synctopi`: Send the current directory to a raspberry pi (or anyting else,
    make sure you change the username, ip address, or path if needed)
