#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::{
    stm32f4_gpio::{GPIOPort, GPIO_Mode, GPIO_OutputType, GPIO_Pin, GPIO_PuPd, GPIO_Speed, GPIO_Handle, GPIO},
    gpio_init, gpio_toggle_pin, gpio_write_pin
};

#[cortex_m_rt::pre_init]
unsafe fn pre_init() {

}

#[entry]
fn main() -> ! {
    let gpio_handle: GPIO_Handle<'_> = gpio_init!(
        GPIOPort::GPIOD,
        GPIO_Pin::GPIO_PIN_12,
        GPIO_Mode::GPIO_MODE_OUTPUT,
        GPIO_Speed::GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_PuPd::GPIO_NOPULL,
        GPIO_OutputType::GPIO_OUTPUT_PP
    );

    let _gpio_handle2: GPIO_Handle<'_> = gpio_init!(
        GPIOPort::GPIOD,
        GPIO_Pin::GPIO_PIN_13,
        GPIO_Mode::GPIO_MODE_OUTPUT,
        GPIO_Speed::GPIO_SPEED_FREQ_VERY_HIGH,
        GPIO_PuPd::GPIO_NOPULL,
        GPIO_OutputType::GPIO_OUTPUT_PP
    );

    unsafe {
        gpio_write_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_12, 1);
        gpio_write_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_13, 1);
        gpio_write_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_14, 1);
        gpio_write_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_15, 1);
    }
    loop {
        for _ in 0..500_000 {
            cortex_m::asm::nop();
        }

        unsafe {
            gpio_toggle_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_12);
            gpio_toggle_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_13);
            gpio_toggle_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_14);
            gpio_toggle_pin!(gpio_handle.pgpiox, GPIO_Pin::GPIO_PIN_15);
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}