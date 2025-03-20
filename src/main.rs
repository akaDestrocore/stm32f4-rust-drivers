#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::{
    stm32f4xx::RegValue,
    stm32f4_rcc::init_max_performance_clocks,
    stm32f4_gpio::{
        GpioHandle, GpioPort, GpioPin, GpioMode, GpioSpeed, 
        GpioPullUpDown, GpioOutputType, init_gpio_pin
    },
    stm32f4_systick::{init_systick, SysTick},
};

#[cortex_m_rt::pre_init]
unsafe fn __pre_init() {

}

fn delay_ms(ms: u32) {
    let start: u32 = SysTick::get_tick();
    while (SysTick::get_tick() - start) < ms {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    // SysTick init 
    let rcc_handle = init_max_performance_clocks().unwrap();
    let sysclk: u32 = rcc_handle.get_sys_clock_freq();
    let _systick_handle = init_systick(sysclk).unwrap();
    
    // Init LED pins
    let green_led: GpioHandle<'_> = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin12,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let orange_led: GpioHandle<'_> = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin13,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let red_led: GpioHandle<'_> = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin14,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let blue_led: GpioHandle<'_> = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin15,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    // infinite loop
    loop {
        green_led.toggle();
        orange_led.toggle();
        red_led.toggle();
        blue_led.toggle();

        delay_ms(2000);
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}