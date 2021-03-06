//! IngressManager
//! 
//! All communicated date is run through here, parsed, then executed. 

use crate::ingress::buffer::{Buffer, Type};
use heapless::consts::*;
use heapless::spsc::Queue;
use simple_hex::hex_byte_to_byte;
use crate::system::system::System;
use crate::system::syscall::Syscall;
use core::str::FromStr;

#[derive(Copy, Clone, PartialEq, Debug)]
enum State {
    /// Waiting for a STX byte, or just received an ETX, or entered an invalid state
    Wait,
    /// Init state, just after receiving and STX
    Init,
    /// Write into an internal buffer for parsing
    Payload,

    /// Parse the application checksum
    ApplicationChecksum,
    /// Store the application in ram
    ApplicationStore,

    /// Notification Source - what generated the push notification
    NotificationSource,
    /// Notification title
    NotificationTitle,
    /// Notification body
    NotificationBody,
}

const STX: u8 = 2;
const ETX: u8 = 3;
const PAYLOAD: u8 = 31; // Unit Separator

pub struct IngressManager {
    buffer: Buffer,
    rb: Queue<u8, U512>,
    state: State,

    hex_chars: [u8; 2],
    hex_idx: usize,

    nsi: [usize; 3],
    nsi_idx: usize,
}

impl IngressManager {

    /// Constructs a new IngressManager
    pub fn new() -> Self {
        IngressManager {
            buffer: Buffer::default(),
            rb: Queue::new(),
            state: State::Init,
            hex_chars: [0u8; 2],
            hex_idx: 0,
            nsi: [0usize; 3], // notification section pointers
            nsi_idx: 0,
        }
    }

    /// Write data into the internal ring buffer
    /// raw bytes being the core type allows the ingress manager to 
    /// be abstracted over the communication medium,
    /// in theory if we setup usb serial, we could have two ingress managers
    /// working in harmony 
    pub fn write(&mut self, data: &[u8]) {
        for byte in data {
            match self.rb.enqueue(*byte) {
                Ok(_) => {},
                Err(e) => panic!("Ring buffer overflow by {:?} bytes", e)
            }
        }
    }

    /// Processs the internal ringbuffer's bytes and execute if the payload is complete
    pub fn process(&mut self, system: &mut System) {
        match self.match_rb(system) {
            Some(buffer_type) => {
                match buffer_type {
                    Type::Unknown => self.state = State::Wait, // if the type cannot be determined abort, and wait until next STX
                    Type::Application => {
                        match system.am().verify() {
                            Ok(_) => {}
                            Err(e) => panic!("{:?} || AMNG: {:?}", e, system.am().status()),
                        }
                    }
                    Type::Notification => {
                        self.nsi[2] = self.nsi_idx;
                        info!("Adding notification from: {:?}, with section indexes {:?}", self.buffer, self.nsi);
                        system.nm().add(&self.buffer, &self.nsi).unwrap_or_else(|err|{
                            error!("Failed to add notification {:?}", err);
                        });
                    },
                    Type::Syscall => {
                        info!("Parsing syscall from: {:?}", self.buffer);
                        match Syscall::from_str(self.buffer.as_str()) {
                            Ok(syscall) => syscall.execute(system),
                            Err(e) => error!("Failed to parse syscall {:?}", e)
                        }
                    }
                }
            },
            None => {}
        }
    }

    /// The internal state machine that handles the incoming bytes
    fn run_state_machine(&mut self, byte: u8, system: &mut System) {
        match self.state {
            State::Init => {
                self.buffer.btype = self.determine_type(byte);
                info!("New buffer of type {:?}", self.buffer.btype);
                if let Type::Unknown = self.buffer.btype {
                    error!("Buffer type is unknown. Going back to wait state.");
                    self.state = State::Wait 
                }
            }
            State::Payload => {
                self.buffer.write(byte);
            }
            State::ApplicationChecksum | State::ApplicationStore => {
                self.hex_chars[self.hex_idx] = byte;
                self.hex_idx += 1;
                if self.hex_idx > 1 {
                    match self.state {
                        State::ApplicationChecksum => {
                            match hex_byte_to_byte(self.hex_chars[0], self.hex_chars[1]) {
                                Ok(byte) => {
                                    system.am().write_checksum_byte(byte).unwrap_or_else(|err|{
                                        error!("Failed to write checksum byte {:?}", err);
                                        self.state = State::Wait;
                                    });
                                }
                                Err(err) => {
                                    error!("Failed to parse hex bytes to byte {:?}", err);
                                    self.state = State::Wait; // abort
                                }
                            }
                        }
                        State::ApplicationStore => {
                            match hex_byte_to_byte(self.hex_chars[0], self.hex_chars[1]) {
                                Ok(byte) => {
                                    system.am().write_ram_byte(byte).unwrap_or_else(|err|{
                                        error!("Failed to write ram byte {:?}", err);
                                        self.state = State::Wait;
                                    });
                                }
                                Err(err) => {
                                    error!("Failed to parse hex bytes to byte {:?}", err);
                                    self.state = State::Wait; // abort
                                }
                            }
                        }
                        _ => unreachable!()
                    }
                    self.hex_idx = 0;
                }
            }
            State::NotificationBody | State::NotificationTitle | State::NotificationSource => {
                self.nsi_idx += 1;
                self.buffer.write(byte);
            }
            State::Wait => {
                // do nothing, useless bytes
            }
        }
    }

    /// Run the internal state machine to parse payloads over a byte stream in the ring buffer
    fn match_rb(&mut self, system: &mut System) -> Option<Type> {
        if !self.rb.is_empty() {
            while let Some(byte) = self.rb.dequeue() {
                match byte {
                    STX => {
                        if self.state != State::Wait {
                            warn!("Partial buffer detected: {:?}", self.buffer);
                        }
                        /* Start of packet */
                        self.hex_idx = 0;
                        self.nsi_idx = 0;
                        self.buffer.clear();
                        self.state = State::Init; // activate processing
                    }
                    ETX => {
                        /* End of packet */
                        /* Finalize messge then reset state machine ready for next msg*/
                        self.state = State::Wait;
                        return Some(self.buffer.btype);
                    }
                    PAYLOAD => {
                        match self.buffer.btype {
                            Type::Unknown => {
                                warn!("Dropping buffer of unknown type {:?}", self.buffer.btype);
                                self.state = State::Wait
                            }
                            Type::Application => {
                                if self.state == State::ApplicationChecksum {
                                    // We've parsed the checksum, now we write the data into ram
                                    self.state = State::ApplicationStore
                                } else {
                                    // reset before we load the new application
                                    system.am().kill().unwrap_or_else(|err| {
                                        warn!("Failed to kill application, writing over live data! {:?}", err);
                                    });
                                    // parse the checksum
                                    self.state = State::ApplicationChecksum;
                                }
                            }
                            Type::Notification => {
                                if self.state == State::NotificationSource { // we've parsed the app source
                                    self.nsi[0] = self.nsi_idx;
                                    self.state = State::NotificationTitle;
                                } else if self.state == State::NotificationTitle { // weve parsed the title
                                    self.nsi[1] = self.nsi_idx;
                                    self.state = State::NotificationBody;
                                } else {
                                    self.state = State::NotificationSource; // new parse
                                }
                            }
                            _ => self.state = State::Payload,
                        }
                    }
                    _ => {
                        /* Run through byte state machine */
                        self.run_state_machine(byte, system);
                    }
                }
            }
        }
        None
    }

    /// Based on the type byte, determine the type of the incoming payload
    fn determine_type(&mut self, type_byte: u8) -> Type {
        self.buffer.btype = match type_byte {
            b'N' => Type::Notification, /* NOTIFICATION i.e FB Msg */
            b'S' => Type::Syscall,
            b'A' => Type::Application,  /* Load Application */
            _ => Type::Unknown,
        };
        self.buffer.btype
    }

}


// #[cfg(test)]
// mod test {
//     use super::*;
//     use heapless::consts::*;
//     use heapless::spsc::Queue;
//     #[test]
//     fn ingress_syscall() {
//         let system = {
//             System::new(rtc, bms, nmgr, amgr)
//         };
//         let mut imgr = IngressManager::new();
//         let mut data = vec![STX, b'S', PAYLOAD];
//         for byte in "T00:00:00".bytes() {
//             data.push(byte);
//         }
//         data.push(ETX);
//         imgr.write(&data);
//         imgr.process();

//         assert_eq!(imgr.state, State::Wait);
//     }
// }
