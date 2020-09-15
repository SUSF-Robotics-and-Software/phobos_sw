//! Simple network server test

use comms_if::net::{MonitoredSocket, SocketOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create the context for zmq
    let ctx = zmq::Context::new();

    // Set the socket options
    let socket_options = SocketOptions {
        bind: true,
        block_on_first_connect: false,
        ..Default::default()
    };

    // Create the socket
    let socket = MonitoredSocket::new(
        &ctx,
        zmq::REP,
        socket_options,
        "tcp://*:5000"
    )?;

    println!("Server running on port 5000");

    // Respond to client requests
    loop {
        // Wait for the client to send us a message
        let msg = socket.recv_msg(0)?;

        match msg.as_str() {
            // If the client sent a valid message
            Some(r) => {
                println!("Recieved \"{}\", sending response", r);

                // Echo the message back to the client
                socket.send(r, 0)?;
            },
            None => println!("Received no data")
        }
    }
}