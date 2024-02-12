/********************************************************************************
 *
 *      ____                     ____          __           __       _
 *     / __ \__  __________     /  _/___  ____/ /_  _______/ /______(_)__  _____
 *    / / / / / / / ___/ _ \    / // __ \/ __  / / / / ___/ __/ ___/ / _ \/ ___/
 *   / /_/ / /_/ (__  )  __/  _/ // / / / /_/ / /_/ (__  ) /_/ /  / /  __(__  )
 *  /_____/\__, /____/\___/  /___/_/ /_/\__,_/\__,_/____/\__/_/  /_/\___/____/
 *        /____/
 *
 *
 *
 ********************************************************************************/
//!
//!
//!
//! # Building
//!
//!
//! # Examples
//!
//!
//! # Configuration
//!
//!

#![no_std]
// #![warn(missing_docs)]

pub mod rid {}
pub mod rt_tasks {}

pub const PTP_DRIFT_GAIN: f32 = 1_000.0;

pub fn led<const N: u8, P: teensy4_bsp::hal::iomuxc::gpio::Pin<N>>(
    gpio: &mut teensy4_bsp::hal::gpio::Port<N>,
    pin: P,
) -> teensy4_bsp::hal::gpio::Output<P> {
    gpio.output(pin)
}

pub fn blink_status<const N: u8, P: teensy4_bsp::hal::iomuxc::gpio::Pin<N>>(
    led: &mut teensy4_bsp::hal::gpio::Output<P>,
    state: SystemState,
) {
    match state {
        SystemState::Dead => led.clear(),
        SystemState::Alive => led.set(),
        _ => led.toggle(),
    };
}

#[derive(Copy, Clone)]
pub enum SystemState {
    Dead = 0,
    Alive = 1,
    Panic = 2,
    Blink = 3,
    Toggle = 4,
}
