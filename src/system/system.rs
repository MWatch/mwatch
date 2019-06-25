//! System
//! 
//! Performs housekeeping of system hardware and provides a nice sofware abstraction to read / manipulate it

use stm32l4xx_hal::rtc::Rtc;
use crate::system::bms::BatteryManagement;
use crate::system::notification::NotificationManager;
use crate::application::application_manager::ApplicationManager;


pub const DMA_HALF_BYTES: usize = 64;

pub const CPU_USAGE_POLL_HZ: u32 = 1; // hz
pub const SYSTICK_HZ: u32 = 3; // hz
pub const TSC_HZ: u32 = (8 * 3); // 8 polls per second (for 3 inputs)
pub const TSC_IDLE_HZ: u32 = (1 * 3); // 1 polls per second (for 3 inputs) when idle to save power

pub const SYS_CLK_HZ: u32 = 16_000_000;
pub const SPI_MHZ: u32 = SYS_CLK_HZ / 2_000_000; // spi is always half of sysclock
pub const I2C_KHZ: u32 = 100;

pub const IDLE_TIMEOUT_SECONDS: u32 = 15;

/// A grouping of core sysem peripherals
pub struct System {
    rtc: Rtc,
    bms: BatteryManagement,
    nm: NotificationManager,
    am: ApplicationManager,
    stats: Stats,
}

impl System {
    pub fn new(rtc: Rtc, bms: BatteryManagement, nm: NotificationManager, am: ApplicationManager) -> Self {
        Self {
            rtc,
            bms,
            nm,
            am,
            stats: Stats::default(),
        }
    }

    /// Real time clock
    pub fn rtc(&mut self) -> &mut Rtc {
        &mut self.rtc
    }

    /// Battery management
    pub fn bms(&mut self) -> &mut BatteryManagement {
        &mut self.bms
    }

    /// Application manager
    pub fn am(&mut self) -> &mut ApplicationManager {
        &mut self.am
    }

    /// Notification Manager
    pub fn nm(&mut self) -> &mut NotificationManager {
        &mut self.nm
    }

    /// System stats
    pub fn ss(&mut self) -> &mut Stats {
        &mut self.stats
    }

    pub fn is_idle(&mut self) -> bool {
        (self.ss().idle_count / SYSTICK_HZ) > IDLE_TIMEOUT_SECONDS
    }

    pub fn get_free_stack() -> usize {
        unsafe {
            extern "C" {
                static __ebss: u32;
                static __sdata: u32;
            }
            let ebss = &__ebss as *const u32 as usize;
            let start = &__sdata as *const u32 as usize;
            let total = ebss - start;
            (16 * 1024) - total // ram for stack in linker script
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub cpu_usage: f32,
    pub tsc_events: u32,
    pub idle_count: u32,
    pub tsc_threshold: u16,
}
    

impl Default for Stats {
    fn default() -> Self {
        Self {
            cpu_usage: 0.0,
            tsc_events: 0,
            idle_count: 0,
            tsc_threshold: 0,
        }
    }
}

