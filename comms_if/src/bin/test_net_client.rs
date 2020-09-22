//! Simple network client test

use comms_if::net::{MonitoredSocket, SocketOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {

    // Create the context for zmq
    let ctx = zmq::Context::new();

    // Set the socket options
    let socket_options = SocketOptions {
        connect_timeout: 1000,
        heartbeat_ivl: 500,
        heartbeat_ttl: 1000,
        heartbeat_timeout: 1000,
        linger: 1,
        recv_timeout: 10,
        send_timeout: 10,
        req_correlate: true,
        req_relaxed: true,
        ..Default::default()
    };

    // Create the socket
    let socket = match MonitoredSocket::new(
        &ctx,
        zmq::REQ,
        socket_options,
        "tcp://localhost:5000"
    ) {
        Ok(s) => s,
        Err(e) => {
            println!("Could not connect to the server");
            return Err(e.into())
        }
    };

    // Loop over sending data to the server
    loop {
        // If the socket isn't connected wait a bit. We do this so that we don't build up a big 
        // backlog of messages to be sent to the server, as zmq will buffer all the messages we send
        // until the server is back up.
        if !socket.connected() {
            println!("Waiting for connection");
            std::thread::sleep(std::time::Duration::from_millis(1000));
            continue;
        }

        // Send the data to the server
        print!("Sending data... ");
        match socket.send("HELLO", 0) {
            Ok(_) => (),
            // If the operation wasn't completed wait a bit
            Err(e) => {
                println!("could not send: {}", e);
                std::thread::sleep(std::time::Duration::from_millis(1000));
                continue;
            }
        }

        // Recieve the response from the server
        let msg = match socket.recv_msg(0) {
            Ok(m) => m,
            // If we didn't get a response wait a bit
            Err(e) => {
                println!("could not read from server: {}", e);
                std::thread::sleep(std::time::Duration::from_millis(1000));
                continue;
            }
        };

        // Print some info about the response
        match msg.as_str() {
            Some(r) => println!("response: {}", r),
            None => println!("no response")
        }

        // Wait a bit
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}