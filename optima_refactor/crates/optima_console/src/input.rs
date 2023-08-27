use std::io;
use std::io::BufRead;
use crate::output::{oprint_full, PrintColor, PrintMode};

pub fn get_console_input_string(prompt: &str, print_color: PrintColor) -> String {
    return if cfg!(target_arch = "wasm32") {
        panic!("wasm32 does not support console input.");
    } else {
        oprint_full(prompt, PrintMode::Println, print_color, true, 0, None, vec![]);
        let stdin = io::stdin();
        let line = stdin.lock().lines().next().unwrap().unwrap();
        line
    }
}