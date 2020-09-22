//! Simple subscriber network test

use comms_if::net::{MonitoredSocket, SocketOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {

    // Create context
    let ctx = zmq::Context::new();

    // Create socket options
    let socket_options = SocketOptions {
        ..Default::default()
    };

    // Create socket
    let socket = MonitoredSocket::new(
        &ctx,
        zmq::SUB,
        socket_options,
        "tcp://localhost:5001"
    )?;

    // Subscribe to the "timestamp" messages, so we won't get the "random" messages.
    socket.set_subscribe(b"timestamp")?;

    // Recieve messages from publisher
    loop {

        let msg = socket.recv_msg(0)?;

        println!("Got message: {:?}", msg.as_str());
    }
}