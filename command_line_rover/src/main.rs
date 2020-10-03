use rustyline::error::ReadlineError;
use rustyline::Editor;
use clap::{App, Arg, AppSettings};


// const str ascii_art = """
//  ____  _   _  ___  ____   ___  ____
// |  _ \| | | |/ _ \| __ ) / _ \/ ___|
// | |_) | |_| | | | |  _ \| | | \___ \
// |  __/|  _  | |_| | |_) | |_| |___) |
// |_|   |_| |_|\___/|____/ \___/|____/
// """

const PROMPT: &str = "Phobos $ ";
const HISTORY_PATH: &str = "data/history.txt";


fn main() {
    let mut rl = Editor::<()>::new();
    if rl.load_history(HISTORY_PATH).is_err() {
        println!("No history detected");
    }



    loop {
        let readline = rl.readline(PROMPT);
        match readline {
            Ok(line) => {
                rl.add_history_entry(line.as_str());
                // println!("Line: {}", line);
                parse(&line);
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
}


fn parse(line: &str) {
    let command_parser = App::new("Phobos")
        // .version("0.1")
        // .author("The Great Richard")
        // .about("Issues commands directly to the rover exec, how cool is that?")
        .subcommand(App::new("mnvr")
            .setting(AppSettings::AllowExternalSubcommands)
            .about("controls maneuvering")
            .subcommand(App::new("ack")
                .setting(AppSettings::AllowExternalSubcommands)
                .arg(Arg::new("velocity")
                    .short('v')
                    // .index(1)
                    .required(true)
                    .takes_value(true))
                .arg(Arg::new("radius")
                    .short('r')
                    // .index(2)
                    .required(true)
                    .takes_value(true))
                .arg(Arg::new("crab")
                    .short('c')
                    // .index(3)
                    .required(true)
                    .takes_value(true))
                    )
            )
        .subcommand(App::new("ping")
            .setting(AppSettings::AllowExternalSubcommands)
    );

    let split:Vec<&str> = line.split(" ").collect();
    println!("{:?}", split);
    let matches = command_parser.get_matches_from(split);
    match matches.subcommand() {
        Some(("mnvr", sub_m)) => {
            println!("got command mnvr");
        },
        Some(("ping", sub_m)) => {
            println!("pong");
        }
        _ => {
            println!("NOTHING");
        }
    }
}


fn shutdown() {
    println!("Exiting...");
}