use rustyline::error::ReadlineError;
use rustyline::Editor;
use structopt::StructOpt;
use comms_if::{
    tc::{Tc, TcResponse},
    net::{zmq, MonitoredSocket, SocketOptions}, 
};
use color_eyre::{Result, eyre::WrapErr};

// const str ascii_art = """
//  ____  _   _  ___  ____   ___  ____
// |  _ \| | | |/ _ \| __ ) / _ \/ ___|
// | |_) | |_| | | | |  _ \| | | \___ \
// |  __/|  _  | |_| | |_) | |_| |___) |
// |_|   |_| |_|\___/|____/ \___/|____/
// """

const PROMPT: &str = "[Phobos] $ ";
const HISTORY_PATH: &str = "clr_history.txt";


fn main() -> Result<()> {
    // Rustline input
    let mut rl = Editor::<()>::new();

    // Load history if some exists
    if rl.load_history(HISTORY_PATH).is_err() {
        println!("No history detected");
    }

    // Create the zmq context
    let ctx = zmq::Context::new();

    // Create the socket options
    let socket_options = SocketOptions {
        bind: true,
        block_on_first_connect: false,
        recv_timeout: 200,
        send_timeout: 10,
        ..Default::default()
    };

    // Bind the server
    let socket = MonitoredSocket::new(
        &ctx,
        zmq::REQ,
        socket_options,
        "tcp://*:5020"
    ).wrap_err("Failed to create the TcServer")?;

    println!("TcServer started");

    // Main loop
    loop {

        // Get line of input from the user
        let readline = rl.readline(PROMPT);

        // Handle the input
        match readline {
            // Valid line
            Ok(line) => {
                // Add it to the history so we can select with arrow keys
                rl.add_history_entry(line.as_str());

                // Strip any spaces off the line
                let line = line.trim();

                // If empty string just continue
                if line.is_empty() {
                    continue
                }
                
                // Split on spaces to parse with structopt
                let cmd: Vec<&str> = line.split(' ').collect();

                // Get the clap matches for this TC
                let tc = match Tc::from_iter_safe(cmd) {
                    Ok(m) => m,
                    Err(e) => {
                        println!("\n{:#}\n", e.message);
                        continue;
                    }
                };

                // Serialize the TC
                let tc_str = serde_json::to_string(&tc)
                    .wrap_err("Failed to serialize the TC")?;

                // Send the TC
                match socket.send(&tc_str, 0) {
                    Ok(_) => (),
                    Err(zmq::Error::EAGAIN) => {
                        println!("Client not connected, TC not sent");
                        continue;
                    },
                    Err(e) => return Err(e).wrap_err("Could not send TC")
                }
                

                // Recieve response from client
                let response = serde_json::from_str(match socket.recv_string(0){
                    Ok(Ok(ref s)) => s,
                    Ok(Err(_)) => {
                        println!("Client responed with invalid UTF-8 message");
                        continue;
                    }
                    Err(e) => {
                        return Err(e).wrap_err("Could not deserialise client's response")
                    }
                }).wrap_err("Could not deserialise response from client")?;

                // Print response message
                match response {
                    TcResponse::Ok => (),
                    TcResponse::Invalid => 
                        println!("Client responded that the send TC was invalid"),
                    TcResponse::CannotExecute => 
                        println!("Client responded that the sent TC could not be executed")
                }
            }
            Err(ReadlineError::Interrupted) => {
                
                break
            }
            Err(err) => {
                println!("Unhandled Error: {:?}", err);
                break
            }
        }
    }

    rl.save_history(HISTORY_PATH).unwrap();

    Ok(())
}