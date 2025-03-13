#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::stm32f4xx::{RCC, GPIOD};

#[cortex_m_rt::pre_init]
unsafe fn pre_init() {

}

#[entry]
fn main() -> ! {
    unsafe {
        (*RCC).AHB1ENR |= (1 << 3);
        
        let mode_mask: u32 = (0b11 << (12*2)) | (0b11 << (13*2)) | (0b11 << (14*2)) | (0b11 << (15*2));
        (*GPIOD).MODER &= !mode_mask;
        
        let output_mask: u32 = (0b01 << (12*2)) | (0b01 << (13*2)) | (0b01 << (14*2)) | (0b01 << (15*2));
        (*GPIOD).MODER |= output_mask;
        
        (*GPIOD).ODR |= (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15);
    }

    loop {
        for _ in 0..1000000 {
            cortex_m::asm::nop();
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}