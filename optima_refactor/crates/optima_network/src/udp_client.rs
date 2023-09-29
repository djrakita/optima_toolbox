use std::net::{UdpSocket};
use serde_derive;
use serde_json;
use serde::{Serialize};

#[derive(serde_derive::Serialize, serde_derive::Deserialize, Debug)]
pub struct Message {
    pub content: String,
}

/// A UDP client that provides methods for sending and receiving messages.
pub struct UdpClient {
    socket: UdpSocket,
}

impl UdpClient {
    /// Creates a new instance of the UDP client.
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        Ok(UdpClient { socket })
    }

    /// Ensures a connection to the specified server address.
    pub fn ensure_connection(&self, server_address: &str) -> Result<(), Box<dyn std::error::Error>> {
        // Check if the socket is already connected to the specified address
        self.socket.connect(server_address)?;
        Ok(())
    }

    /// Sends a message to the specified server address.
    pub fn send_message<T>(&self, _server_address: &str, message: &T) -> Result<(), Box<dyn std::error::Error>>
    where
        T: Serialize,
        T: std::fmt::Debug,
    {
        let message_data = serde_json::to_vec(&message)?;

        self.socket.send(&message_data)?;

        println!("Message sent: {:?}", message);

        Ok(())
    }

    /// Receives a message from the UDP socket.
    pub fn receive_message<T>(&self) -> Result<T, Box<dyn std::error::Error>>
    where
        T: serde::de::DeserializeOwned,
    {
        let mut buffer = [0u8; 1024];
        let (size, _) = self.socket.recv_from(&mut buffer)?;

        if size == 0 {
            return Err("No data received".into());
        }

        let received_data = &buffer[..size];
        let received_message: T = serde_json::from_slice(received_data)?;

        Ok(received_message)
    }
}
