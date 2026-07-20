use crate::utils::*;

pub struct SelectorController {
    port_name: String,
    baudrate: u32,
    slave_address: u8,
    max_pos: u16,
}

impl SelectorController {
    pub fn new(
        port_name: String,
        baud_rate: u32,
        slave_address: u8,
        max_pos: u16,
    ) -> Result<Self, String> {
        if port_name.is_empty() {
            return Err("Port name cannot be empty".to_string());
        }
        if baud_rate == 0 {
            return Err("Baud rate must be greater than zero".to_string());
        }
        Ok(Self {
            port_name,
            baudrate: baud_rate,
            slave_address,
            max_pos,
        })
    }

    pub fn max_pos(&self) -> u16 {
        self.max_pos
    }

    pub fn set_home_pos(&self) -> Result<(), String> {
        let command = vec![0xCC, self.slave_address, 0x45, 0x00, 0x00, 0xDD]; // Previously 0x45
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn set_valve_pos(&self, pos: u16) -> Result<(), String> {
        if !(0..=self.max_pos).contains(&pos) {
            return Err(format!("Position must be between 0 and {}", self.max_pos));
        }
        let command_args = decimal_to_2byte_hex(pos);
        let command = [
            0xCC,
            self.slave_address,
            0x44,
            command_args[1],
            command_args[0],
            0xDD,
        ];
        send_command(&self.port_name, self.baudrate, &command)?;
        std::thread::sleep(std::time::Duration::from_millis(750));
        Ok(())
    }
}
