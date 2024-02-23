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

    use usbd_hid::{descriptor::generator_prelude::*, hid_class::HIDClass};

    use firmware::{
        DigitalPins,
        SystemState,
        N_DIGITAL_OUTPUTS,
    };
    use rid::{
        ptp::{Duration, TimeStamp},
        RID_DEFAULT_VID, RID_DEFAULT_PID,
        RIDReport, RID_PACKET_SIZE, RID_CYCLE_TIME_US,
        RID_MODE_INDEX, RID_TOGL_INDEX,
    };

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
    const PRODUCT: &str = "Dyse Industries RTIC Firmware";
    const VID_PID: UsbVidPid = UsbVidPid(RID_DEFAULT_VID, RID_DEFAULT_PID);

    /// How frequently we push updates to usb host
    const GPT_UPDATE_INTERVAL_US: u32 = RID_CYCLE_TIME_US as u32;
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

    type Bus = BusAdapter;

    #[shared]
    struct Shared {
        system_time: Duration,
        system_status_report: Option<RIDReport>,
        _rt_task_report: Option<RIDReport>,
        system_config_report: Option<RIDReport>,
    }

    #[local]
    struct Local {
        digital_pin_states: [SystemState; N_DIGITAL_OUTPUTS],
        digital_pins: DigitalPins,
        usb_hid: HIDClass<'static, Bus>,
        usb_device: UsbDevice<'static, Bus>,
        pit: bsp::hal::pit::Pit<2>,
        ptp_stamp: TimeStamp,
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

        let digital_pins = DigitalPins::new(&mut gpio2, &mut gpio4, pins);

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
            .device_class(0)
            .max_packet_size_0(RID_PACKET_SIZE as u8)
            .build();

        let ptp_stamp = TimeStamp::new(0, 0, 0, 0);

        (
            Shared {
                system_time: Duration::new(0),
                system_status_report: None,
                _rt_task_report: None,
                system_config_report: None,
            },
            Local {
                digital_pin_states: [SystemState::Standby; N_DIGITAL_OUTPUTS],
                digital_pins,
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

    #[task(binds = PIT, local = [digital_pins, digital_pin_states, pit, loop_timer1: u32 = 0, loop_timer2: u32 = 0], shared = [system_time, system_status_report, system_config_report], priority = 1)]
    fn system_control(mut ctx: system_control::Context) {
        let mut ticks = 0;

        while ctx.local.pit.is_elapsed() {
            // this while feels unnecessary but since everyone else is doing it I'll leave it for now
            ticks = ctx.local.pit.current_timer_value();
            ctx.local.pit.clear_elapsed();
        }

        let system_micros = ctx
            .shared
            .system_time
            .lock(|timer| timer.add_micros((ticks as f32 * TICK_TO_US) as u32));

        if system_micros - *ctx.local.loop_timer2 > 250_000 {

            let mut buffer = [0u8; RID_PACKET_SIZE];
            buffer[RID_MODE_INDEX] = 0x13;
            buffer[RID_TOGL_INDEX] = 0x13;

            *ctx.local.loop_timer2 = system_micros;

            ctx.local.digital_pin_states[1] = SystemState::Alive;


            if ctx.shared
                .system_status_report
                .lock(|report| match *report {
                    None => { *report = Some(buffer); true },
                    Some(_) => false,
                }) {
                    ctx.local.digital_pin_states[0] = SystemState::Toggle;
                }
        }

        if system_micros - *ctx.local.loop_timer1 > 1_000 {

            *ctx.local.loop_timer1 = system_micros;

            if let Some(buffer) = ctx.shared.system_config_report.lock(|report| *report) {

                ctx.local.digital_pin_states[2] = SystemState::Alive;
            
            }
        
            ctx.local.digital_pins.write(*ctx.local.digital_pin_states);

            *ctx.local.digital_pin_states = [SystemState::Standby; N_DIGITAL_OUTPUTS];

        }
    }

    #[task(binds = USB_OTG1, local = [usb_hid, usb_device, ptp_stamp, configured: bool = false], shared = [system_time, system_status_report, system_config_report], priority = 1)]
    fn usb1(mut ctx: usb1::Context) {
        let usb1::LocalResources {
            usb_hid,
            usb_device,
            ptp_stamp,
            configured,
        } = ctx.local;

        // USB dev poll only in the interrupt handler
        usb_device.poll(&mut [usb_hid]);

        // Check if device is configured
        match usb_device.state() {
            UsbDeviceState::Configured => {
                match configured {
                    // USB needs configuration
                    false => {
                        usb_device.bus().configure();
                        *configured = true;
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
                            let micros = ctx.shared.system_time.lock(|timer| timer.micros());

                            // Try reading a report
                            let mut buffer = [0; RID_PACKET_SIZE];

                            match usb_hid.pull_raw_output(&mut buffer).ok() {
                                Some(RID_PACKET_SIZE) => {
                                    ctx.shared
                                        .system_config_report
                                        .lock(|report| match *report {
                                            Some(_) => {}
                                            None => {
                                                *report = Some(buffer);
                                            }
                                        });

                                    ptp_stamp.client_read(&buffer, micros);

                                    let report = ctx.shared.system_status_report.lock(|report| {
                                        match *report {
                                            Some(buffer) => {
                                                *report = None; // clear this when we read the packet... kinda beat
                                                Some(buffer)
                                            }
                                            None => Some([0u8; RID_PACKET_SIZE]),
                                        }
                                    });

                                    if let Some(mut buffer) = report {
                                        let ticks =
                                            usb_device.bus().gpt_mut(GPT_INSTANCE, |gpt| gpt.load());
                                        ptp_stamp.client_stamp(
                                            &mut buffer,
                                            micros + (ticks as f32 * TICK_TO_US) as u32,
                                        );
                                        usb_hid.push_raw_input(&buffer).ok();
                                    }

                                }
                                _ => {},
                            }
                        }
                    }
                }
            }
            _ => {},
        };
    }
}
