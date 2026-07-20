use std::io::{Read, Write};

pub fn decimal_to_2byte_hex(value: u16) -> [u8; 2] {
    [(value >> 8) as u8, (value & 0xFF) as u8]
}

/// Converts a 2-byte [MSB, LSB] array to a decimal u16.
pub fn hex_to_decimal(bytes: [u8; 2]) -> u16 {
    ((bytes[0] as u16) << 8) | (bytes[1] as u16)
}
pub fn calculate_checksum(frame: &[u8]) -> [u8; 2] {
    let sum: u16 = frame.iter().map(|&b| b as u16).sum();
    [(sum & 0xFF) as u8, (sum >> 8) as u8]
}

pub fn find_serial_port_by_id(id: &str) -> Result<String, String> {
    let by_id_dir = std::path::Path::new("/dev/serial/by-id");
    let entries = std::fs::read_dir(by_id_dir)
        .map_err(|e| format!("Failed to read {}: {}", by_id_dir.display(), e))?;

    for entry in entries {
        let entry = entry.map_err(|e| format!("Failed to read directory entry: {}", e))?;
        if entry.file_name().to_string_lossy().contains(id) {
            return Ok(entry.path().to_string_lossy().into_owned());
        }
    }

    Err(format!("No serial port found matching id '{}'", id))
}

/// Sends a command over UART (RS485).
pub fn send_command(port_name: &str, baud_rate: u32, command: &[u8]) -> Result<u16, String> {
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(std::time::Duration::from_millis(200))
        .open()
        .map_err(|e| format!("Error opening port '{}': {}", port_name, e))?;

    let checksum = calculate_checksum(command); // returns [u8; 2]
    let mut frame = Vec::from(command); // original command
    frame.extend_from_slice(&checksum); // append 2 bytes only

    //println!("Sending: {:02X?}", frame);

    port.write_all(&frame)
        .map_err(|e| format!("Write error: {}", e))?;

    let mut buffer = [0u8; 8]; // exactly 8 bytes
    port.read_exact(&mut buffer)
        .map_err(|e| format!("Read error (no/short response from device): {}", e))?;
    // Print the received buffer in hexadecimal format
    //println!("Received: {:02X?}", buffer);

    Ok(hex_to_decimal([buffer[4], buffer[3]])) // Assuming response is in bytes 3 and 4
}
