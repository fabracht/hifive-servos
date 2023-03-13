#![no_std]
#![no_main]
extern crate panic_halt;

use core::f64::EPSILON;

use hifive1::hal::i2c::Speed;
use hifive1::hal::prelude::*;

use hifive1::hal::{delay::Delay, i2c::I2c, DeviceResources};
use hifive1::{pin, sprintln};

use pwm_pca9685::{Address, Channel, Pca9685};
use riscv_rt::entry;

const MIN_DUTY: u16 = 125;
const MID_DUTY: u16 = 350;
const MAX_DUTY: u16 = 599;
const MIN_CLAW_DUTY: u16 = 132;
const MID_CLAW_DUTY: u16 = 225;
const MAX_CLAW_DUTY: u16 = 325;
const POS_FACTOR: i16 = 4;
const NEG_FACTOR: i16 = -4;
// pick_channel!(i, value) -> pwm.set_channel_off(Channel::C#i, #value)

#[entry]
fn main() -> ! {
    let mut wait_for = 3000.0;
    let dr = DeviceResources::take().unwrap();
    let dp = dr.peripherals;
    let pins = dr.pins;
    // setup_wifi(dp, pins);

    let clocks = hifive1::clock::configure(dp.PRCI, dp.AONCLK, 100.mhz().into());
    let mut delay = Delay::new();
    // Configure UART for stdout
    let mut led = pin!(pins, dig13).into_output();
    let button_r = pin!(pins, dig15).into_pull_up_input();
    let button_m = pin!(pins, dig7).into_pull_up_input();
    let button_l = pin!(pins, dig17).into_pull_up_input();
    let sda = pin!(pins, i2c0_sda).into_iof0();
    let scl = pin!(pins, i2c0_scl).into_iof0();
    let i2c = I2c::new(dp.I2C0, sda, scl, Speed::Normal, clocks);
    hifive1::stdout::configure(
        dp.UART0,
        pin!(pins, uart0_tx),
        pin!(pins, uart0_rx),
        115_200.bps(),
        clocks,
    );
    let mut pwm = Pca9685::new(i2c, Address::default()).unwrap();
    // This results in about 60 Hz, which is the frequency at which servos operate.
    pwm.set_prescale(100).unwrap();
    pwm.enable().unwrap();
    // Turn all channels on at time "0".
    pwm.set_channel_on(Channel::All, 0).unwrap();
    let servo_min = MIN_DUTY;
    let servo_max = MAX_DUTY;
    let claw_min = MIN_CLAW_DUTY;
    let claw_max = MAX_CLAW_DUTY;
    led.set_high().unwrap();
    delay.delay_ms(499);
    pwm.set_channel_off(Channel::C0, 275).unwrap();

    led.toggle().unwrap();
    delay.delay_ms(999);

    led.toggle().unwrap();
    let mut servo_duties = [
        MID_CLAW_DUTY,
        MID_DUTY,
        MID_DUTY - 125,
        MID_DUTY + 175,
        MID_DUTY,
        MID_DUTY,
    ];

    for (channel, duty) in servo_duties.iter().enumerate() {
        match channel {
            0 => pwm.set_channel_off(Channel::C0, *duty).unwrap(),
            1 => pwm.set_channel_off(Channel::C1, *duty).unwrap(),
            2 => pwm.set_channel_off(Channel::C2, *duty).unwrap(),
            3 => pwm.set_channel_off(Channel::C3, *duty).unwrap(),
            4 => pwm.set_channel_off(Channel::C4, *duty).unwrap(),
            5 => pwm.set_channel_off(Channel::C5, *duty).unwrap(),
            _ => (),
        }
    }

    let mut servo_index = 0;
    let mut factor: i16 = 0;
    loop {
        sprintln!("Wait for : {}", wait_for);
        if button_m.is_low().unwrap() {
            servo_index = (servo_index + 1) % 6;
            delay.delay_ms(250);
        }

        if button_r.is_high().unwrap() && button_l.is_high().unwrap() {
            if wait_for < 3000.0 {
                wait_for = sigmoidal_delay(wait_for, 3000.0, 100000.0);
            } else {
                factor = 0;
                led.toggle().unwrap();
            }
        }
        if button_r.is_low().unwrap() {
            factor = NEG_FACTOR;

            led.set_high().unwrap();
            wait_for = sigmoidal_delay(wait_for, 0.001, 100000.0);
        }
        if button_l.is_low().unwrap() {
            factor = POS_FACTOR;
            led.set_high().unwrap();

            wait_for = sigmoidal_delay(wait_for, 0.001, 100000.0);
        }
        delay.delay_us(wait_for as u32);
        if servo_index == 0 {
            if servo_duties[servo_index] >= claw_max {
                led.toggle().unwrap();
                servo_duties[servo_index] -= 1;
            } else if servo_duties[servo_index] <= claw_min {
                led.toggle().unwrap();
                servo_duties[servo_index] += 1;
            }
        } else {
            if servo_duties[servo_index] > servo_max {
                sprintln!("MAX over capacity: {}", servo_duties[servo_index]);
                servo_duties[servo_index] = servo_max;

                factor = 0;
            } else if servo_duties[servo_index] < servo_min {
                servo_duties[servo_index] = servo_min;

                factor = 0;
            }
        }

        servo_duties[servo_index] = (servo_duties[servo_index] as i16 + factor) as u16;
        match servo_index {
            i if i == 0 => pwm.set_channel_off(Channel::C0, servo_duties[i]).unwrap(),
            i if i == 1 => pwm.set_channel_off(Channel::C1, servo_duties[i]).unwrap(),
            i if i == 2 => pwm.set_channel_off(Channel::C2, servo_duties[i]).unwrap(),
            i if i == 3 => pwm.set_channel_off(Channel::C3, servo_duties[i]).unwrap(),
            i if i == 4 => pwm.set_channel_off(Channel::C4, servo_duties[i]).unwrap(),
            i if i == 5 => pwm.set_channel_off(Channel::C5, servo_duties[i]).unwrap(),
            _ => (),
        }
    }
    // loop {}
}

fn sigmoid(x: f64) -> f64 {
    let mut y = x;
    let mut i = 1.0;
    let mut s = x;
    if s < 0.0 {
        s = -s;
    }
    while y * y > EPSILON * EPSILON {
        i += 1.0;
        y *= -(x * x) / (2.0 * i - 1.0) / (2.0 * i - 2.0);
        s += y;
        if s < 0.0 {
            s = -s;
        }
    }
    s / (1.0 + s)
}

fn sigmoidal_delay(current_delay: f64, target_delay: f64, step_size: f64) -> f64 {
    let error = target_delay - current_delay;
    let sigmoid_input = error / 3000.0 * 10.0;
    let mut delta_delay = step_size * sigmoid(sigmoid_input) * (1.0 - sigmoid(sigmoid_input));
    if error < 0.0 {
        delta_delay = -delta_delay;
    }
    let next_delay = current_delay + delta_delay;
    if next_delay < 0.0 {
        0.0
    } else if next_delay > 3000.0 {
        3000.0
    } else {
        next_delay
    }
}
