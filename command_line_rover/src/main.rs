use clap::{Arg, App};


// const str ascii_art = """
//  ____  _   _  ___  ____   ___  ____
// |  _ \| | | |/ _ \| __ ) / _ \/ ___|
// | |_) | |_| | | | |  _ \| | | \___ \
// |  __/|  _  | |_| | |_) | |_| |___) |
// |_|   |_| |_|\___/|____/ \___/|____/
// """


fn main() {
    let matches = App::new("Phobos")
        .version("0.1")
        .author("The Great Richard")
        .about("Issues commands directly to the rover exec, how cool is that?")
        .subcommand(App::new("mnvr")
            .about("controls maneuvering")
            .subcommand(App::new("ack")
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
                ))
        .get_matches();


    if let Some(subman) = matches.subcommand_matches("mnvr") {
        if let Some(submnvrack) = subman.subcommand_matches("ack") {
            if let Some(v) = submnvrack.value_of("velocity") {
                println!("v = {}", v);
            }
        }
    }

}
