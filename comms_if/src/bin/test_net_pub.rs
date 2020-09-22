//! Simple network publisher test

use comms_if::net::{MonitoredSocket, SocketOptions};
use chrono::Utc;

fn main() -> Result<(), Box<dyn std::error::Error>> {

    // Create zmq context
    let ctx = zmq::Context::new();

    // Create socket options
    let socket_options = SocketOptions {
        bind: true,
        block_on_first_connect: false,
        ..Default::default()
    };

    // Create the socket
    let socket = MonitoredSocket::new(
        &ctx,
        zmq::PUB,
        socket_options,
        "tcp://*:5001"
    )?;

    println!("Publisher server open on port 5001");

    // Send data to subscribers
    loop {
        // We'll just send the current time in seconds, as "timestamp {}"
        //
        // The first part here, separated by a space, is the topic. Subscribers can filter data by
        // this topic using the `.set_subscribe()` function.
        let timestamp_str = format!("timestamp {}", Utc::now().timestamp());

        // Send the timestamp
        match socket.send(&timestamp_str, 0) {
            Ok(_) => (),
            Err(e) => println!("Failed to send timestsamp: {}", e)
        }
        
        // Also send a random message, to demonstrate subscription
        socket.send("random", 0).ok();

        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
}
