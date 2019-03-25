//! Input
//! 
//! Here we multiplex all the hardware inputs (3) to create a series of
//! unique output combinations (7)

use crate::types::InputEvent;
use crate::types::{LeftButton, MiddleButton, RightButton, TouchSenseController};
use crate::types::hal::tsc::Event as TscEvent;

pub const LEFT: u8 = 1;
pub const MIDDLE: u8 = 2;
pub const RIGHT: u8 = 4;
pub const LEFT_MIDDLE: u8 = LEFT | MIDDLE;
pub const RIGHT_MIDDLE: u8 = RIGHT | MIDDLE;
pub const LEFT_RIGHT: u8 = LEFT | RIGHT;
pub const ALL: u8 = LEFT | MIDDLE | RIGHT;
pub const NONE: u8 = 0;

pub const TSC_SAMPLES: u16 = 10;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Error {
    NoInput,
    InvalidInputVector(u8),
    InvalidInputPin,
    AcquisitionInProgress,
    Incomplete
}

/// Input manager, assumes control over the tsc peripheral and handles the raw inputs
pub struct InputManager
{
    raw_vector: u8,
    last_vector: u8,
    pin_idx: u8,
    tsc_threshold: u16,
    tsc: TouchSenseController,
    left: LeftButton,
    middle: MiddleButton,
    right: RightButton,
}

impl InputManager {
    /// Creates a new instance of the InputManager
    pub fn new(tsc: TouchSenseController, threshold: u16, left: LeftButton, middle: MiddleButton, right: RightButton) -> Self {
        let mut tsc = tsc;
        tsc.listen(TscEvent::EndOfAcquisition);
        // tsc.listen(TscEvent::MaxCountError); // TODO

        Self {
            tsc,
            tsc_threshold: threshold,
            raw_vector: 0,
            last_vector: 0,
            pin_idx: 0,
            left,
            middle,
            right,
        }
    }

    /// Update thes the internal state of the manager with the raw hardware input
    pub fn update_input(&mut self, active: bool) {
        if active {
            self.raw_vector |= match self.pin_idx {
                0 => 1 ,
                1 => 1 << 1,
                2 => 1 << 2,
                _ => panic!("Invalid pin index")
            };
        } else {
            self.raw_vector &= match self.pin_idx {
                0 => !1,
                1 => !(1 << 1),
                2 => !(1 << 2),
                _ => panic!("Invalid pin index")
            };
        }
        
        // update the index once the input has been set
        self.pin_idx += 1;
        if self.pin_idx > 2 {
            self.pin_idx = 0;
        }
    }

    /// Based on the current state of the inputmanager's internal vector, produce an output
    pub fn output(&mut self) -> Result<InputEvent, Error> {
        if self.raw_vector != self.last_vector {
            let result = match self.raw_vector {
                ALL => Ok(InputEvent::Multi),
                LEFT_RIGHT => Ok(InputEvent::Dual),
                LEFT_MIDDLE => Ok(InputEvent::LeftMiddle),
                RIGHT_MIDDLE => Ok(InputEvent::RightMiddle),
                LEFT => Ok(InputEvent::Left),
                MIDDLE => Ok(InputEvent::Middle),
                RIGHT => Ok(InputEvent::Right),
                NONE => Err(Error::NoInput), // no input
                _ => Err(Error::InvalidInputVector(self.raw_vector)),
            };
            self.last_vector = self.raw_vector;
            result
        } else {
            Err(Error::NoInput)
        }
    }

    /// Begin a new hardware (tsc) acquisition
    pub fn start_new(&mut self) -> Result<(), Error> {
        if self.tsc.in_progress() {
            return Err(Error::AcquisitionInProgress);
        }
        match self.pin_idx {
            0 => self.tsc.start(&mut self.left),
            1 => self.tsc.start(&mut self.middle),
            2 => self.tsc.start(&mut self.right),
            _ => panic!("Invalid pin index")
        }
        Ok(())
    }

    /// Call when the aquisition is complete, this function read
    /// the registers and update the interal state
    pub fn process_result(&mut self) -> Result<(), Error> {
        let value = match self.pin_idx {
            0 => self.tsc.read(&mut self.left).expect("Expected TSC pin 0"),
            1 => self.tsc.read(&mut self.middle).expect("Expected TSC pin 1"),
            2 => self.tsc.read(&mut self.right).expect("Expected TSC pin 2"),
            _ => panic!("Invalid pin index")
        };
        trace!("tsc[{}] {} < {}?", self.pin_idx, value, self.tsc_threshold);
        self.update_input(value < self.tsc_threshold);
        self.tsc.clear(TscEvent::EndOfAcquisition);
        if self.pin_idx == 2 { // we've read all the pins now process the output
            Ok(())
        } else {
            Err(Error::Incomplete)
        }
    }

    /// returns the threshold value required to identify a touch
    pub fn threshold(&self) -> u16 {
        self.tsc_threshold
    }
}