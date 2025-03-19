#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::{
    stm32f4xx::{TIM6},
    stm32f4_rcc::{RccRegister, init_max_performance_clocks},
    stm32f4_gpio::{
        GpioHandle, GpioPort, GpioPin, GpioMode, GpioSpeed, 
        GpioPullUpDown, GpioOutputType, init_gpio_pin
    },
};

#[cortex_m_rt::pre_init]
unsafe fn __pre_init() {

}

// TIM6 init
fn init_delay_timer() {
    let rcc_reg: RccRegister = match RccRegister::new() {
        Ok(reg) => reg,
        Err(_) => return,
    };
    
    // Enable TIM6
    let _ = rcc_reg.modify_apb1enr(|mut reg: stm32f4_rust_drivers::stm32f4xx::RegValue| {
        reg.set_bits(1 << 4);
        reg
    });
    
    unsafe {
        (*TIM6).PSC = 42000 - 1; // 42 MHz / 42000 = 1 kHz (1 ms period)
    }
}

fn delay_ms(ms: u32) {
    unsafe {
        (*TIM6).ARR = ms - 1;
        (*TIM6).CNT = 0;
        (*TIM6).SR = 0;
        (*TIM6).CR1 |= 1;
        while 0 == ((*TIM6).SR & 0x1) {
            cortex_m::asm::nop();
        }
        (*TIM6).CR1 &= !1;
    }
}

#[entry]
fn main() -> ! {
    // Initialize system clock
    let _ = init_max_performance_clocks();
    
    // Initialize delay timer
    init_delay_timer();
    
    // Initialize GPIO pins for LEDs
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
        // turn on all LEDs
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