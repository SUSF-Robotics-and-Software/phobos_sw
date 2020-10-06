use rustyline::error::ReadlineError;
use rustyline::Editor;
use structopt::StructOpt;
use comms_if::tc::Tc;


// const str ascii_art = """
//  ____  _   _  ___  ____   ___  ____
// |  _ \| | | |/ _ \| __ ) / _ \/ ___|
// | |_) | |_| | | | |  _ \| | | \___ \
// |  __/|  _  | |_| | |_) | |_| |___) |
// |_|   |_| |_|\___/|____/ \___/|____/
// """

const PROMPT: &str = "[Phobos] $ ";
const HISTORY_PATH: &str = "clr_history.txt";


fn main() {
    // Rustline input
    let mut rl = Editor::<()>::new();

    // Load history if some exists
    if rl.load_history(HISTORY_PATH).is_err() {
        println!("No history detected");
    }

    // TcParser form clap
    let tc_parser_app = Tc::clap();

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
                let matches = match tc_parser_app.clone().get_matches_from_safe(cmd) {
                    Ok(m) => m,
                    Err(e) => {
                        println!("\n{:#}\n", e.message);
                        continue;
                    }
                };

                // Parse the tc
                let tc = Tc::from_clap(&matches);

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