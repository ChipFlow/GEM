// SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

//! TCP transport layer for the CXXRTL debug protocol.
//!
//! Messages are null-terminated (U+0000) JSON strings over a TCP stream.
//! Only one client connection is supported at a time.

use std::io::{self, BufRead, BufReader, BufWriter, Write};
use std::net::{TcpListener, TcpStream};

/// A framed connection that reads and writes null-terminated JSON messages.
pub struct Connection {
    reader: BufReader<TcpStream>,
    writer: BufWriter<TcpStream>,
}

impl Connection {
    /// Wrap an accepted TCP stream into a framed connection.
    pub fn new(stream: TcpStream) -> io::Result<Self> {
        let reader = BufReader::new(stream.try_clone()?);
        let writer = BufWriter::new(stream);
        Ok(Connection { reader, writer })
    }

    /// Read the next null-terminated JSON message.
    ///
    /// Returns `None` on EOF (client disconnected).
    pub fn read_message(&mut self) -> io::Result<Option<serde_json::Value>> {
        let mut buf = Vec::new();
        let n = self.reader.read_until(0x00, &mut buf)?;
        if n == 0 {
            return Ok(None); // EOF
        }
        // Strip trailing null byte
        if buf.last() == Some(&0x00) {
            buf.pop();
        }
        if buf.is_empty() {
            return Ok(None);
        }
        let text = String::from_utf8(buf)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        let value: serde_json::Value = serde_json::from_str(&text)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        Ok(Some(value))
    }

    /// Send a JSON message with null terminator.
    pub fn send_message(&mut self, value: &serde_json::Value) -> io::Result<()> {
        let text = serde_json::to_string(value)
            .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
        self.writer.write_all(text.as_bytes())?;
        self.writer.write_all(&[0x00])?;
        self.writer.flush()?;
        Ok(())
    }
}

/// A TCP server that listens for CXXRTL client connections.
pub struct Server {
    listener: TcpListener,
}

impl Server {
    /// Bind to the given address (e.g. `"127.0.0.1:9000"`).
    pub fn bind(addr: &str) -> io::Result<Self> {
        let listener = TcpListener::bind(addr)?;
        clilog::info!("CXXRTL server listening on {}", addr);
        Ok(Server { listener })
    }

    /// Accept one client connection.
    ///
    /// This blocks until a client connects. Only one client is supported
    /// at a time (the CXXRTL protocol is single-client).
    pub fn accept(&self) -> io::Result<Connection> {
        let (stream, addr) = self.listener.accept()?;
        clilog::info!("CXXRTL client connected from {}", addr);
        Connection::new(stream)
    }

    /// Get the local address the server is bound to.
    pub fn local_addr(&self) -> io::Result<std::net::SocketAddr> {
        self.listener.local_addr()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_roundtrip_message() {
        let server = Server::bind("127.0.0.1:0").unwrap();
        let addr = server.local_addr().unwrap();

        let client_thread = thread::spawn(move || {
            let stream = TcpStream::connect(addr).unwrap();
            let mut conn = Connection::new(stream).unwrap();
            // Send a message
            let msg = serde_json::json!({"type": "greeting", "version": 0});
            conn.send_message(&msg).unwrap();
            // Read a response
            let resp = conn.read_message().unwrap().unwrap();
            assert_eq!(resp["type"], "greeting");
        });

        let mut conn = server.accept().unwrap();
        let msg = conn.read_message().unwrap().unwrap();
        assert_eq!(msg["type"], "greeting");
        assert_eq!(msg["version"], 0);
        // Send response
        let resp = serde_json::json!({"type": "greeting", "version": 0, "commands": []});
        conn.send_message(&resp).unwrap();

        client_thread.join().unwrap();
    }
}
