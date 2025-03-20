#![no_std]
#![no_main]

use core::panic::PanicInfo;
use cortex_m_rt::entry;
use stm32f4_rust_drivers::{
    stm32f4xx::{USART2, PWR, PWRRegDef},
    stm32f4_rcc::{
        RccHandle, OscConfig, OscillatorType, HsiState, PllConfig, PllState, 
        PllSource, PllP, ClockConfig, ClockType, SysClkSource, 
        AhbClkDivider, ApbClkDivider
    },
    stm32f4_gpio::{
        GpioHandle, GpioPort, GpioPin, GpioMode, GpioSpeed, 
        GpioPullUpDown, GpioOutputType, init_gpio_pin, init_gpio_pin_with_af
    },
    stm32f4_usart::{
        UsartMode, UsartBaud, UsartStopBits, UsartWordLength, 
        UsartParity, UsartHwFlowControl, init_usart
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

fn system_clock_config() -> Result<RccHandle<'static>, stm32f4_rust_drivers::stm32f4_rcc::RccError> {
    let rcc_handle = RccHandle::new()?;
    
    // Enable PWR clock
    rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 28, true)?;
    
    // PWR_REGULATOR_VOLTAGE_SCALE1
    unsafe {
        let pwr: &mut PWRRegDef = &mut *(PWR as *mut stm32f4_rust_drivers::stm32f4xx::PWRRegDef);
        pwr.CR |= 3 << 14; // PWR_CR_VOS
    }
    
    let pll_config: PllConfig = PllConfig {
        state: PllState::On,
        source: PllSource::Hsi,
        m: 8,
        n: 168,
        p: PllP::Div2,
        q: 4,
    };
    
    let osc_config: OscConfig = OscConfig {
        oscillator_type: OscillatorType::HSI,
        hsi_state: HsiState::On,
        hsi_calibration: 16,
        pll: pll_config,
        ..Default::default()
    };
    
    let clock_config: ClockConfig = ClockConfig {
        clock_type: ClockType::SystemClock as u32 | ClockType::HClock as u32 | 
                   ClockType::PClock1 as u32 | ClockType::PClock2 as u32,
        sys_clk_source: SysClkSource::PllClk,
        ahb_clk_divider: AhbClkDivider::Div1,
        apb1_clk_divider: ApbClkDivider::Div4,
        apb2_clk_divider: ApbClkDivider::Div2,
    };
    
    rcc_handle.osc_config(&osc_config)?;
    rcc_handle.clock_config(&clock_config)?;
    
    Ok(rcc_handle)
}

#[entry]
fn main() -> ! {

    let rcc_handle = system_clock_config().unwrap();
    let sysclk: u32 = rcc_handle.get_sys_clock_freq();
    
    // for delay
    let _systick_handle = init_systick(sysclk).unwrap();
    
    // LED init
    let green_led = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin12,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let orange_led = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin13,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let red_led = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin14,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let blue_led = init_gpio_pin(
        GpioPort::GpioD,
        GpioPin::Pin15,
        GpioMode::Output,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::NoPull,
        GpioOutputType::PushPull
    ).unwrap();
    
    let _usart2_tx = init_gpio_pin_with_af(
        GpioPort::GpioA,
        GpioPin::Pin2,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::PullUp,
        GpioOutputType::PushPull,
        7
    ).unwrap();
    
    let _usart2_rx = init_gpio_pin_with_af(
        GpioPort::GpioA,
        GpioPin::Pin3,
        GpioSpeed::VeryHigh,
        GpioPullUpDown::PullUp,
        GpioOutputType::PushPull,
        7
    ).unwrap();

    let usart_handle = init_usart(
        USART2,
        UsartMode::TxRx,
        UsartBaud::Baud115200,
        UsartStopBits::StopBits1,
        UsartWordLength::WordLengthBits8,
        UsartParity::Disable,
        UsartHwFlowControl::HwFlowControlNone
    ).unwrap();
    
    let test_message: &[u8; 29] = b"Hello from Rust on STM32F4!\r\n";
    
    loop {
        usart_handle.send_data(test_message).unwrap();
        
        green_led.toggle().unwrap();
        delay_ms(300);
        
        orange_led.toggle().unwrap();
        delay_ms(300);
        
        red_led.toggle().unwrap();
        delay_ms(300);
        
        blue_led.toggle().unwrap();
        delay_ms(1000);
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}