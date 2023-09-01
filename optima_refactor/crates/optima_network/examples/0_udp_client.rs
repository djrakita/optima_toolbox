mod udp_client;

use std::net::{UdpSocket, SocketAddr};
use std::thread;
use socket_utils::{UdpClient, Message};

fn udp_server() -> Result<(), Box<dyn std::error::Error>> {
    let server_address: SocketAddr = "127.0.0.1:8080".parse()?;
    let udp_socket = UdpSocket::bind(server_address)?;

    println!("UDP server is listening...");

    let mut buffer = [0u8; 1024];
    loop {
        match udp_socket.recv_from(&mut buffer) {
            Ok((size, client_address)) => {
                let received_message = &buffer[..size];
                let message_str = String::from_utf8_lossy(received_message);
                println!("Received message from {}: {}", client_address, message_str);
            }
            Err(e) => {
                eprintln!("Error receiving data: {}", e);
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let udp_client_thread = thread::spawn(|| {
        let server_address: SocketAddr = "127.0.0.1:8080".parse()?;
        // Create a new UdpClient instance
        let udp_client = UdpClient::new()?;
        println!("UdpClient initialized");

        // Ensure the connection to the server
        udp_client.ensure_connection(server_address)?;
        println!("Connected to the server");

        let message = Message {
            content: "Hello, UDP Server!".to_string(),
        };

        udp_client.send_message(&message)?;

        Ok(())
    });

    let udp_server_thread = thread::spawn(|| {
        udp_server()
    });

    udp_client_thread.join().unwrap()?;
    udp_server_thread.join().unwrap()?;

    Ok(())
}
