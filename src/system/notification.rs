//! Notification
//! 
//! Push notification parsing

use crate::ingress::buffer::Buffer;

pub const BUFF_SIZE: usize = 512;
pub const BUFF_COUNT: usize = 24;

#[derive(Copy, Clone)]
pub struct Notification {
    section_indexes: [usize; 3],
    inner: Buffer,
}

impl Notification {
    pub const fn default() -> Notification {
        Notification {
            section_indexes: [0usize; 3],
            inner: Buffer {
                btype: crate::ingress::buffer::Type::Unknown,
                payload: [0u8; BUFF_SIZE],
                payload_idx: 0,
            },
        }
    }

    pub fn buffer(&self) -> &[u8] {
        &self.inner.payload[..self.inner.payload_idx]
    }

    pub fn from_buffer(buffer: &Buffer, idxs: &[usize; 3]) -> Result<Notification, NotificationError> {
        Ok(Notification {
            section_indexes: idxs.clone(),
            inner: buffer.clone()
        })
    }

    pub fn source(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.inner.payload[0..self.section_indexes[1]]) }
    }

    pub fn title(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.inner.payload[self.section_indexes[0]..self.section_indexes[1]]) }
    }

    pub fn body(&self) -> &str {
        unsafe { core::str::from_utf8_unchecked(&self.inner.payload[self.section_indexes[1]..self.section_indexes[2]]) }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum NotificationError {
    Parsing,
}

pub struct NotificationManager {
    pool: &'static mut [Notification],
    idx: usize,
}

impl NotificationManager {
    pub fn new(buffers: &'static mut [Notification]) -> NotificationManager {
        NotificationManager {
            pool: buffers,
            idx: 0,
        }
    }

    /// takes a closure to execute on the buffer
    pub fn peek_notification<F>(&mut self, index: usize, f: F)
    where
        F: FnOnce(&Notification),
    {
        let notification = &self.pool[index];
        f(&notification);
    }

    pub fn idx(&self) -> usize {
        self.idx
    }

    // Parses a buffer for notification info, copying into the pool
    pub fn add(&mut self, buffer: &Buffer, idxs: &[usize; 3]) -> Result<(), NotificationError> {
        self.pool[self.idx] = Notification::from_buffer(buffer, idxs)?;

        self.idx += 1;
        if self.idx + 1 > self.pool.len() {
            // TODO impl a cirucular buffer that track head and tail
            /* buffer is full, wrap around */

            self.idx = 0;
        }
        Ok(())
    }
}

// TODO testing of parsing
