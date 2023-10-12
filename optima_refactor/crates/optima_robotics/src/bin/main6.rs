

fn test(a: f64, b: f64) -> Option<f64> {
    if b != 0.0 { Some(a/b) }
    else { None }
}

fn test2(a: f64, b: Option<f64>) -> f64 {
    match b {
        None => { a }
        Some(b) => { a + b }
    }
}

fn test3(a: f64, b: f64) -> Result<f64, String> {
    if b != 0.0 { Ok(a/b) }
    else { Err("cannot divide by zero".to_string()) }
}

fn main() {
    let res = test3(1.0, 0.0);

    match res {
        Ok(res) => {  }
        Err(res) => {  }
    }
}