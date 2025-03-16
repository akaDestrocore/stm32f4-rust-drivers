#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::{
    stm32f4xx::{TIM6, RCC},
    stm32f4_rcc::init_max_performance_clocks,
};

#[cortex_m_rt::pre_init]
unsafe fn __pre_init() {

}

// TIM6 init
fn init_delay_timer() {
    unsafe {
        (*RCC).APB1ENR |= 1 << 4;
        
        (*TIM6).PSC = 42000 - 1; // 42 MHz / 42000 = 1 kHz (1 ms period)
    }
}

fn delay_ms(ms: u32) {
    unsafe {
        (*TIM6).ARR = ms - 1;
        (*TIM6).CNT = 0;
        (*TIM6).SR = 0;
        (*TIM6).CR1 |= 1;
        while ((*TIM6).SR & 0x1) == 0 {
            cortex_m::asm::nop();
        }
        (*TIM6).CR1 &= !1;
    }
}

#[entry]
fn main() -> ! {
    let _ = init_max_performance_clocks();
    init_delay_timer();
    unsafe {
        (*RCC).AHB1ENR |= 1 << 3;
    }
    
    unsafe {
        let gpiod: *mut stm32f4_rust_drivers::stm32f4xx::GPIORegDef = stm32f4_rust_drivers::stm32f4xx::GPIOD;
        
        // green
        (*gpiod).MODER &= !(0x3 << (12 * 2)); // reset bits
        (*gpiod).MODER |= 0x1 << (12 * 2);    // output
        
        // orange
        (*gpiod).MODER &= !(0x3 << (13 * 2));
        (*gpiod).MODER |= 0x1 << (13 * 2);
        
        // Pred
        (*gpiod).MODER &= !(0x3 << (14 * 2));
        (*gpiod).MODER |= 0x1 << (14 * 2);
        
        // blue
        (*gpiod).MODER &= !(0x3 << (15 * 2));
        (*gpiod).MODER |= 0x1 << (15 * 2);
        
        // high speed
        (*gpiod).OSPEEDR |= 0x3 << (12 * 2);
        (*gpiod).OSPEEDR |= 0x3 << (13 * 2);
        (*gpiod).OSPEEDR |= 0x3 << (14 * 2);
        (*gpiod).OSPEEDR |= 0x3 << (15 * 2);
        
        (*gpiod).OTYPER &= !(0x1 << 12);
        (*gpiod).OTYPER &= !(0x1 << 13);
        (*gpiod).OTYPER &= !(0x1 << 14);
        (*gpiod).OTYPER &= !(0x1 << 15);
        
        (*gpiod).PUPDR &= !(0x3 << (12 * 2));
        (*gpiod).PUPDR &= !(0x3 << (13 * 2));
        (*gpiod).PUPDR &= !(0x3 << (14 * 2));
        (*gpiod).PUPDR &= !(0x3 << (15 * 2));
    }
    
    // infinite loop
    loop {
        unsafe {
            let gpiod: *mut stm32f4_rust_drivers::stm32f4xx::GPIORegDef = stm32f4_rust_drivers::stm32f4xx::GPIOD;
            
            // enable all LEDs
            (*gpiod).BSRR = (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);
            
            delay_ms(2000);
            
            // disable all LEDs
            (*gpiod).BSRR = (1 << (12 + 16)) | (1 << (13 + 16)) | (1 << (14 + 16)) | (1 << (15 + 16));
            
            delay_ms(2000);
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}