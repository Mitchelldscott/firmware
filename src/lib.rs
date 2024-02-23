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
//! # Dyse Industries RTIC firmware
//! Utilizes the RID Communication crate to manage tasks
//! and peripherals on the Teensy 4.1

#![no_std]
#![warn(missing_docs)]

pub const N_DIGITAL_OUTPUTS: usize = 3;

/// Output Pin Factory
pub fn digital_output<const N: u8, P: teensy4_bsp::hal::iomuxc::gpio::Pin<N>>(
    gpio: &mut teensy4_bsp::hal::gpio::Port<N>,
    pin: P,
) -> teensy4_bsp::hal::gpio::Output<P> {
    gpio.output(pin)
}

/// Set a digital pins output
pub fn set_output<const N: u8, P: teensy4_bsp::hal::iomuxc::gpio::Pin<N>>(
    led: &mut teensy4_bsp::hal::gpio::Output<P>,
    state: SystemState,
) {
    match state {
        SystemState::Dead => led.clear(),
        SystemState::Alive => led.set(),
        SystemState::Standby => {},
        _ => led.toggle(),
    };
}

#[derive(Copy, Clone)]
/// Type for LED status
pub enum SystemState {
    /// off
    Dead = 0,
    /// on
    Alive = 1,
    /// toggle
    Panic = 2,
    /// toggle
    Blink = 3,
    /// toggle
    Toggle = 4,
    /// standmby mode
    Standby = 5,
}

pub struct DigitalPins {
    builtin_led: teensy4_bsp::hal::gpio::Output<teensy4_bsp::pins::common::P13>,
    system_led: teensy4_bsp::hal::gpio::Output<teensy4_bsp::pins::common::P2>,
    rt_task_led: teensy4_bsp::hal::gpio::Output<teensy4_bsp::pins::common::P3>,
}

impl DigitalPins {
    pub fn new(
        gpio2: &mut teensy4_bsp::hal::gpio::Port<2>,
        gpio4: &mut teensy4_bsp::hal::gpio::Port<4>,
        pins: teensy4_bsp::pins::t41::Pins,
    ) -> DigitalPins {
        DigitalPins {
            builtin_led: gpio2.output(pins.p13),
            system_led: gpio4.output(pins.p2),
            rt_task_led: gpio4.output(pins.p3),
        }
    }

    pub fn write(&mut self, state: [SystemState; N_DIGITAL_OUTPUTS]) {

        set_output(&mut self.builtin_led, state[0]);
        set_output(&mut self.system_led, state[1]);
        set_output(&mut self.rt_task_led, state[2]);

    }
}

// impl DigitalPins {
//     pub fn new<const N: u8>(pins: teensy4_bsp::Pins, gpio2: &mut teensy4_bsp::hal::gpio::Port<N>, gpio4: &mut teensy4_bsp::hal::gpio::Port<N>) -> DigitalPins {
//         DigitalPins {
//             builtin_led: digital_output(gpio2, pins.p13),
//             system_led: digital_output(gpio4, pins.p2),
//             rt_task_led: digital_output(gpio4, pins.p3),
//         }
//     }
// }
