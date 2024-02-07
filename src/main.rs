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
 *  RTIC firmware & USB interface 
 * Manages peripheral IO and the task graph
 * (ethernet coming soon?)
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
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true)]
mod app {

    use bsp::board;
    use bsp::hal::usbd::{BusAdapter, EndpointMemory, EndpointState, Speed};

    use teensy4_bsp as bsp;

    use usb_device::{
        bus::UsbBusAllocator,
        device::{UsbDevice, UsbDeviceBuilder, UsbDeviceState, UsbVidPid},
    };

    use usbd_hid::{
        descriptor::{generator_prelude::*},
        hid_class::HIDClass,
    };

    use firmware::{SystemState};
    use rid::{PTPStamp, Duration, RID_PACKET_SIZE, RIDReport};

    #[gen_hid_descriptor(
        (collection = 0x01, usage = 0x01, usage_page = 0xff00) = {
            data=input;
        }
    )]
    #[allow(dead_code)]
    struct RticHidReport {
        data: [u8; 64],
    }

    /// Change me if you want to play with a full-speed USB device.
    const SPEED: Speed = Speed::High;
    const VID_PID: UsbVidPid = UsbVidPid(0x1331, 0x0001);
    const PRODUCT: &str="Dyse Industries RTIC Firmware";

    /// How frequently we push updates to usb host
    const GPT_UPDATE_INTERVAL_US: u32 = 1_000;
    const GPT_INSTANCE: imxrt_usbd::gpt::Instance = imxrt_usbd::gpt::Instance::Gpt0;

    const TICK_TO_MS: f32 = 1_000.0 / board::PERCLK_FREQUENCY as f32;
    const TICK_TO_US: f32 = 1_000.0 * TICK_TO_MS;
    const PIT_DELAY_MS: u32 = (0.5 / TICK_TO_MS) as u32;


    /// This allocation is shared across all USB endpoints. It needs to be large
    /// enough to hold the maximum packet size for *all* endpoints. If you start
    /// noticing panics, check to make sure that this is large enough for all endpoints.
    static EP_MEMORY: EndpointMemory<1024> = EndpointMemory::new();
    /// This manages the endpoints. It's large enough to hold the maximum number
    /// of endpoints; we're not using all the endpoints in this example.
    static EP_STATE: EndpointState = EndpointState::max_endpoints();

    // use core::{iter::Cycle, slice::Iter};

    type Bus = BusAdapter;


    #[shared]
    struct Shared {
        system_time: Duration,
        usb_state: SystemState,
        rt_task_state: SystemState,
        system_state: SystemState,
        system_status_report: Option<RIDReport>,
        _rt_task_report: Option<RIDReport>,
        system_config_report: Option<RIDReport>,
    }

    #[local]
    struct Local {
        builtin_led: bsp::hal::gpio::Output<bsp::pins::common::P13>,
        system_led: bsp::hal::gpio::Output<bsp::pins::common::P2>,
        rt_task_led: bsp::hal::gpio::Output<bsp::pins::common::P3>,
        usb_hid: HIDClass<'static, Bus>,
        usb_device: UsbDevice<'static, Bus>,
        pit: bsp::hal::pit::Pit<2>,
        ptp_stamp: PTPStamp,
    }

    #[init(local = [bus: Option<UsbBusAllocator<Bus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let board::Resources {
            pins,
            mut gpio2,
            mut gpio4,
            pit: (_, _, mut pit, _),
            usb,
            ..
        } = board::t41(ctx.device);

        let builtin_led = firmware::led(&mut gpio2, pins.p13);
        let system_led = firmware::led(&mut gpio4, pins.p2);
        let rt_task_led = firmware::led(&mut gpio4, pins.p3);

        pit.set_interrupt_enable(true);
        pit.set_load_timer_value(PIT_DELAY_MS);
        pit.enable();

        let bus = BusAdapter::with_speed(usb, &EP_MEMORY, &EP_STATE, SPEED);
        bus.set_interrupts(true);
        bus.gpt_mut(GPT_INSTANCE, |gpt| {
            gpt.stop();
            gpt.clear_elapsed();
            gpt.set_interrupt_enabled(true);
            gpt.set_mode(imxrt_usbd::gpt::Mode::Repeat);
            gpt.set_load(GPT_UPDATE_INTERVAL_US);
            gpt.reset();
            gpt.run();
        });

        let bus = ctx.local.bus.insert(UsbBusAllocator::new(bus));
        // Note that "4" correlates to a 1ms polling interval. Since this is a high speed
        // device, bInterval is computed differently.
        let usb_hid = HIDClass::new(bus, RticHidReport::desc(), 4);
        let usb_device = UsbDeviceBuilder::new(bus, VID_PID)
            .product(PRODUCT)
            .device_class(2)
            .max_packet_size_0(64)
            .build();

        let ptp_stamp = PTPStamp::new(0,0,0,0);

        (
            Shared {
                system_time: Duration::default(),
                usb_state: SystemState::Dead,
                rt_task_state: SystemState::Dead,
                system_state: SystemState::Alive,
                system_status_report: None,
                _rt_task_report: None,
                system_config_report: None,
            },
            Local {
                builtin_led,
                system_led,
                rt_task_led,
                usb_hid,
                usb_device,
                pit,
                ptp_stamp,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }

    #[task(binds = PIT, local = [builtin_led, system_led, rt_task_led, pit, loop_timer: u32 = 0], shared = [system_time, usb_state, rt_task_state, system_state, system_status_report], priority = 1)]
    fn system_control(mut ctx: system_control::Context) {

        let mut ticks = 0;

        while ctx.local.pit.is_elapsed() { // this while feels unnecessary but since everyone else is doing it I'll leave it for now
            ticks = ctx.local.pit.current_timer_value();
            ctx.local.pit.clear_elapsed();
        }

        let system_millis = ctx.shared.system_time.lock(|timer| timer.add_micros((ticks as f32 * TICK_TO_US) as i32));

        if system_millis - *ctx.local.loop_timer > 250 {

            *ctx.local.loop_timer = system_millis;

            let usb_state = ctx.shared.usb_state.lock(|s| *s);
            let system_state = ctx.shared.system_state.lock(|s| *s);
            let rt_task_state = ctx.shared.rt_task_state.lock(|s| *s);

            firmware::blink_status(ctx.local.builtin_led, usb_state);
            firmware::blink_status(ctx.local.system_led, system_state);
            firmware::blink_status(ctx.local.rt_task_led, rt_task_state);

            let mut buffer = [0; RID_PACKET_SIZE];
            buffer[0] = 0x13;
            buffer[1] = usb_state as u8;
            buffer[2] = system_state as u8;
            buffer[3] = rt_task_state as u8;

            ctx.shared
                .system_status_report
                .lock(|report| match *report {
                    None => {
                        *report = Some(buffer);
                    },
                    Some(_) => {},
                });
        }
        
    }

    #[task(binds = USB_OTG1, local = [usb_hid, usb_device, ptp_stamp], shared = [system_time, usb_state, system_status_report, system_config_report], priority = 1)]
    fn usb1(mut ctx: usb1::Context) {
        let usb1::LocalResources {
            usb_hid,
            usb_device,
            ptp_stamp,
        } = ctx.local;

        let mut usb_state = ctx.shared.usb_state.lock(|state| *state );

        // USB dev poll only in the interrupt handler
        usb_device.poll(&mut [usb_hid]);

        // Check if device is configured
        match usb_device.state() {
            
            UsbDeviceState::Configured => {
            
                match usb_state {
                    // USB needs configuration
                    SystemState::Dead => { 
                        usb_device.bus().configure();
                        usb_state = SystemState::Alive;
                    }
                    // USB is configured, try reading and writing
                    _ => { 
                        // get bus timer elapsed
                        let elapsed = usb_device.bus().gpt_mut(GPT_INSTANCE, |gpt| {

                            let elapsed = gpt.is_elapsed();
                            
                            while gpt.is_elapsed() {
                                gpt.clear_elapsed();
                            }
                            
                            elapsed
                        });

                        if elapsed {

                            // Try reading a report
                            let mut buffer = [0; RID_PACKET_SIZE];

                            usb_state = match usb_hid.pull_raw_output(&mut buffer).ok() {
                            
                                Some(64) => {

                                    ctx.shared.system_config_report.lock(|report| { 
                                        match *report {
                                            Some(_) => {},
                                            None => { *report = Some(buffer); }
                                        }
                                    });

                                    let ptp_gain = ptp_stamp.client_read(&buffer, ctx.shared.system_time.lock(|timer| timer.millis()));
                                    ctx.shared.system_time.lock(|timer| timer.add_micros(ptp_gain));

                                    SystemState::Toggle
                                }
                                _ => SystemState::Alive,
                            };

                            // write available system report
                            let mut buffer = match ctx.shared.system_status_report.lock(|report| *report ) {
                                Some(buffer) => buffer,
                                None => [0u8; RID_PACKET_SIZE], // add match to check for other reports
                            };

                            ptp_stamp.client_stamp(&mut buffer, ctx.shared.system_time.lock(|timer| timer.millis()));
                            usb_hid.push_raw_input(&buffer).ok();
                        }
                    }
                }
            }
            _ => usb_state = SystemState::Dead,
        };

        ctx.shared.usb_state.lock(|state| { *state = usb_state; } );
    }
}
