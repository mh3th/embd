#![no_std]
#![no_main]

use bsp::entry;
use bsp::hal::{
    adc::{Adc, AdcPin},
    clocks::init_clocks_and_plls,
    pac,
    sio::Sio,
    usb::UsbBus,
    watchdog::Watchdog,
    Timer,
};
use core::fmt::Write;
use defmt_rtt as _;
use embedded_hal::digital::StatefulOutputPin;
use embedded_hal_0_2::adc::OneShot;
use heapless::String;
use panic_probe as _;
use rp_pico as bsp;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.gpio8.into_push_pull_output();
    let mut adc_pin_0 = AdcPin::new(pins.gpio26).unwrap();
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")])
        .unwrap()
        .device_class(USB_CLASS_CDC)
        .build();
    let mut last_toggle = timer.get_counter_low();
    let mut last_print = timer.get_counter_low();
    let toggle_interval = 5_000_000;
    let print_interval = 500_000;
    loop {
        let now = timer.get_counter_low();

        if now - last_toggle >= toggle_interval {
            led_pin.toggle().unwrap();
            last_toggle = now;
        }

        if now - last_print >= print_interval {
            let value: u16 = adc.read(&mut adc_pin_0).unwrap();

            let mut text: String<64> = String::new();
            write!(text, "Value: {}\r\n", value).unwrap();
            _ = serial.write(text.as_bytes());
            last_print = now;
        }

        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(_e) => {}
                Ok(0) => {}
                Ok(count) => {
                    buf.iter_mut().take(count).for_each(|b| {
                        b.make_ascii_uppercase();
                    });
                    let mut wr_ptr = &buf[..count];
                    while !wr_ptr.is_empty() {
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}
