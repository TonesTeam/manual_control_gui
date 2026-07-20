use std::thread;
use std::time::Duration;

use crate::utils::*;

pub struct PumpController {
    port_name: String,
    baudrate: u32,
    slave_address: u8,
    has_solenoids: bool,
}

const SYR_SWAP_DELAY: u64 = 750; // Delay in milliseconds for syringe swap

impl PumpController {
    pub fn new(port_name: String, baud_rate: u32, slave_address: u8) -> Result<Self, String> {
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
            has_solenoids: false,
        })
    }

    /// Marks this pump as having a T-shaped solenoid for flow direction selection.
    pub fn with_solenoids(mut self, has_solenoids: bool) -> Self {
        self.has_solenoids = has_solenoids;
        self
    }

    pub fn has_solenoids(&self) -> bool {
        self.has_solenoids
    }

    pub fn get_pos(&self) -> Result<u16, String> {
        let command = vec![0xCC, self.slave_address, 0x66, 0x00, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)
    }

    pub fn force_stop(&self) -> Result<u16, String> {
        let command = vec![0xCC, self.slave_address, 0x49, 0x00, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)
    }

    pub fn get_motor_status(&self) -> Result<u16, String> {
        let command = vec![0xCC, self.slave_address, 0x4A, 0x00, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)
    }

    pub fn set_max_pos(&self) -> Result<(), String> {
        let command = vec![0xCC, self.slave_address, 0x4D, 0xE2, 0x0E, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn set_solenoids_input(&self) -> Result<(), String> {
        if !self.has_solenoids {
            return Err("This pump does not have solenoids.".to_string());
        }
        let command = vec![0xCC, self.slave_address, 0x61, 0x01, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)?;
        thread::sleep(Duration::from_millis(SYR_SWAP_DELAY));
        Ok(())
    }

    pub fn set_solenoids_output(&self) -> Result<(), String> {
        if !self.has_solenoids {
            return Err("This pump does not have solenoids.".to_string());
        }
        let command = vec![0xCC, self.slave_address, 0x60, 0x01, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)?;
        thread::sleep(Duration::from_millis(SYR_SWAP_DELAY));
        Ok(())
    }

    pub fn set_home_pos(&self) -> Result<(), String> {
        let command = vec![0xCC, self.slave_address, 0x45, 0x00, 0x00, 0xDD];
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn set_pos(&self, pos: u16) -> Result<(), String> {
        if !(0..=3810).contains(&pos) {
            return Err("Position must be between 0 and 3810".to_string());
        }
        let current = self.get_pos()?;
        let adjustment = pos as i16 - current as i16;
        if adjustment > 0 {
            self.increase_pos_by(adjustment as u16)
        } else if adjustment < 0 {
            self.decrease_pos_by(adjustment.unsigned_abs())
        } else {
            Ok(())
        }
    }

    pub fn increase_pos_by(&self, pos: u16) -> Result<(), String> {
        if !(0..=3810).contains(&pos) {
            return Err("Position must be between 0 and 3810".to_string());
        }
        let command_args = decimal_to_2byte_hex(pos);
        let command = [
            0xCC,
            self.slave_address,
            0x4D,
            command_args[1],
            command_args[0],
            0xDD,
        ];
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn decrease_pos_by(&self, pos: u16) -> Result<(), String> {
        if !(0..=3810).contains(&pos) {
            return Err("Position must be between 0 and 3810".to_string());
        }
        let command_args = decimal_to_2byte_hex(pos);
        let command = [
            0xCC,
            self.slave_address,
            0x42,
            command_args[1],
            command_args[0],
            0xDD,
        ];
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn set_speed(&self, speed: u16) -> Result<(), String> {
        if !(0..=500).contains(&speed) {
            return Err("Speed must be between 0 and 500".to_string());
        }
        let command_args = decimal_to_2byte_hex(speed);
        let command = [
            0xCC,
            self.slave_address,
            0x4B,
            command_args[1],
            command_args[0],
            0xDD,
        ];
        send_command(&self.port_name, self.baudrate, &command)?;
        Ok(())
    }

    pub fn wait_for_pos(&self, target_pos: u16) -> Result<(), String> {
        let mut last_pos;
        let start_pos = self.get_pos()?;
        let mut current_pos;
        loop {
            last_pos = self.get_pos()?;
            thread::sleep(Duration::from_millis(300));
            if last_pos == self.get_pos()? {
                thread::sleep(Duration::from_millis(300));
                if last_pos == self.get_pos()? {
                    eprintln!("Pump seems to be stuck at position {}", last_pos);
                    break;
                }
            }
            current_pos = self.get_pos()?;
            if current_pos == target_pos {
                break;
            }
            if (start_pos < target_pos && current_pos > target_pos)
                || (start_pos > target_pos && current_pos < target_pos)
            {
                break;
            }
            // Check if ever moved in opposite direction
            if (start_pos < target_pos && current_pos < start_pos)
                || (start_pos > target_pos && current_pos > start_pos)
            {
                println!(
                    "Pump moved in opposite direction. Start: {}, Target: {}, Current: {}",
                    start_pos, target_pos, current_pos
                );
                break;
            }
        }
        println!("Exiting wait for pos.");
        Ok(())
    }

    pub fn set_pos_autostop(&self, pos: u16) -> Result<(), String> {
        if !(0..=3810).contains(&pos) {
            return Err("Position must be between 0 and 3810".to_string());
        }
        self.set_pos(pos)?;
        let mut last_pos;
        while self.get_pos()? != pos {
            last_pos = self.get_pos()?;
            thread::sleep(Duration::from_millis(300));
            if last_pos == self.get_pos()? {
                eprintln!("Pump seems to be stuck at position {}", last_pos);
                break;
            }
        }
        thread::sleep(Duration::from_millis(500));
        let current = self.get_pos()?;
        if current == pos {
            return Ok(());
        }
        println!("Warning: Pump did not reach the desired position {}", pos);
        if (pos as i16 - current as i16).abs() < 2 {
            println!("Pump is close to the desired position. Ending command.");
            thread::sleep(Duration::from_millis(750));
            self.force_stop()?;
            return Ok(());
        }
        thread::sleep(Duration::from_millis(2500));
        self.force_stop()?;
        Ok(())
    }
}
