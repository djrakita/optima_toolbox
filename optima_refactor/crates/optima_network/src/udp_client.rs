use std::net::{UdpSocket};
use serde_derive;
use serde_json;
use std::vec;
use serde::{Serialize, Deserialize};

pub struct UdpClient {
    socket: UdpSocket,
}

impl UdpClient {
    pub fn new() -> Result<Self, Box<dyn std::error::Error>> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        Ok(UdpClient { socket })
    }

    pub fn ensure_connection(&self, server_address: &str) -> Result<(), Box<dyn std::error::Error>> {
        self.socket.connect(server_address)?;
        Ok(())
    }

    pub fn send_message<T>(&self, server_address: &str, message: &T) -> Result<(), Box<dyn std::error::Error>>
    where
        T: Serialize,
        T: std::fmt::Debug,
    {
        let message_data = serde_json::to_vec(&message)?;

        self.socket.send(&message_data)?;

        println!("Message sent: {:?}", message);

        Ok(())
    }

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
