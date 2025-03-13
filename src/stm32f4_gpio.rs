#![allow(non_camel_case_types)]

use core::marker::PhantomData;
use crate::stm32f4xx::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIO_RegDef, RCC};

#[macro_export]
macro_rules! gpio_init {
    ($port:expr, $pin:expr, $mode:expr, $speed:expr, $pupd:expr, $output_type:expr) => {
        {
            let mut gpio_handle = $crate::stm32f4_gpio::GPIO_Handle::new($port);
            gpio_handle.config_pin($pin, $mode, $speed, $pupd, $output_type, 0);
            gpio_handle.init();
            gpio_handle
        }
    };
}

#[macro_export]
macro_rules! gpio_write_pin {
    ($port:expr, $pin:expr, $value:expr) => {
        $crate::stm32f4_gpio::GPIO::write_pin($port, $pin, $value)
    };
}

#[macro_export]
macro_rules! gpio_read_pin {
    ($port:expr, $pin:expr) => {
        $crate::stm32f4_gpio::GPIO::read_pin($port, $pin)
    };
}

#[macro_export]
macro_rules! gpio_toggle_pin {
    ($port:expr, $pin:expr) => {
        $crate::stm32f4_gpio::GPIO::toggle_pin($port, $pin)
    };
}

#[derive(Debug, Clone, Copy)]
pub enum GPIOPort {
    GPIOA,
    GPIOB,
    GPIOC,
    GPIOD,
    GPIOE,
    GPIOF,
    GPIOG,
    GPIOH,
    GPIOI,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    DISABLE = 0,
    ENABLE = 1,
}

#[derive(Debug, Clone, Copy)]
pub enum GPIO_Mode {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT = 1,
    GPIO_MODE_AF = 2,
    GPIO_MODE_ANALOG = 3,
    GPIO_MODE_IT_FALLING = 4,  // Falling edge interrupt
    GPIO_MODE_IT_RISING = 5,  // Rising edge interrupt
    GPIO_MODE_IT_RISING_FALLING = 6, // Rising-Falling edge interrupt
}

#[derive(Debug, Clone, Copy)]
pub enum GPIO_OutputType {
    GPIO_OUTPUT_PP = 0,
    GPIO_OUTPUT_OD = 1,
}

#[derive(Debug, Clone, Copy)]
pub enum GPIO_Speed {
    GPIO_SPEED_FREQ_LOW = 0,
    GPIO_SPEED_FREQ_MEDIUM = 1,
    GPIO_SPEED_FREQ_HIGH = 2,
    GPIO_SPEED_FREQ_VERY_HIGH = 3,
}

#[derive(Debug, Clone, Copy)]
pub enum GPIO_PuPd {
    GPIO_NOPULL = 0,
    GPIO_PULLUP = 1,
    GPIO_PULLDOWN = 2,
}

#[derive(Debug, Clone, Copy)]
pub enum GPIO_Pin {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1 = 1,
    GPIO_PIN_2 = 2,
    GPIO_PIN_3 = 3,
    GPIO_PIN_4 = 4,
    GPIO_PIN_5 = 5,
    GPIO_PIN_6 = 6,
    GPIO_PIN_7 = 7,
    GPIO_PIN_8 = 8,
    GPIO_PIN_9 = 9,
    GPIO_PIN_10 = 10,
    GPIO_PIN_11 = 11,
    GPIO_PIN_12 = 12,
    GPIO_PIN_13 = 13,
    GPIO_PIN_14 = 14,
    GPIO_PIN_15 = 15,
}

pub struct GPIO_PinConfig {
    pub pin_number: GPIO_Pin,
    pub mode: GPIO_Mode,
    pub speed: GPIO_Speed,
    pub pupd: GPIO_PuPd,
    pub output_type: GPIO_OutputType,
    pub alt_func: u8,
}

impl GPIO_PinConfig {
    pub fn new() -> Self {
        GPIO_PinConfig {
            pin_number: GPIO_Pin::GPIO_PIN_0,
            mode: GPIO_Mode::GPIO_MODE_INPUT,
            speed: GPIO_Speed::GPIO_SPEED_FREQ_LOW,
            pupd: GPIO_PuPd::GPIO_NOPULL,
            output_type: GPIO_OutputType::GPIO_OUTPUT_PP,
            alt_func: 0,
        }
    }
}

pub struct GPIO_Handle<'a> {
    pub pgpiox: *mut GPIO_RegDef,
    pub config: GPIO_PinConfig,
    _marker: PhantomData<&'a ()>,
}

impl<'a> GPIO_Handle<'a> {
    pub fn new(port: GPIOPort) -> Self {
        let pgpiox: *mut GPIO_RegDef = match port {
            GPIOPort::GPIOA => GPIOA,
            GPIOPort::GPIOB => GPIOB,
            GPIOPort::GPIOC => GPIOC,
            GPIOPort::GPIOD => GPIOD,
            GPIOPort::GPIOE => GPIOE,
            GPIOPort::GPIOF => GPIOF,
            GPIOPort::GPIOG => GPIOG,
            GPIOPort::GPIOH => GPIOH,
            GPIOPort::GPIOI => GPIOI,
        };

        GPIO_Handle {
            pgpiox,
            config: GPIO_PinConfig::new(),
            _marker: PhantomData,
        }
    }

    pub fn config_pin(
        &mut self,
        pin: GPIO_Pin,
        mode: GPIO_Mode,
        speed: GPIO_Speed,
        pupd: GPIO_PuPd,
        output_type: GPIO_OutputType,
        alt_func: u8,
    ) {
        self.config.pin_number = pin;
        self.config.mode = mode;
        self.config.speed = speed;
        self.config.pupd = pupd;
        self.config.output_type = output_type;
        self.config.alt_func = alt_func;
    }

    pub fn init(&self) {
        unsafe {
            GPIO::periph_clock_control(self.pgpiox, State::ENABLE);

            let pin: u8 = self.config.pin_number as u8;

            if let GPIO_Mode::GPIO_MODE_INPUT | GPIO_Mode::GPIO_MODE_OUTPUT | GPIO_Mode::GPIO_MODE_AF | GPIO_Mode::GPIO_MODE_ANALOG = self.config.mode {
                let mode_value: u32 = self.config.mode as u32;
                let mode_shift: u8 = 2 * pin;
                let mode_mask: u32 = 0x3 << mode_shift;
                (*self.pgpiox).MODER &= !(mode_mask);
                (*self.pgpiox).MODER |= (mode_value << mode_shift);
            } else {

            }

            let speed_value: u32 = self.config.speed as u32;
            let speed_shift: u8 = 2 * pin;
            let speed_mask: u32 = 0x3 << speed_shift;
            (*self.pgpiox).OSPEEDR &= !(speed_mask);
            (*self.pgpiox).OSPEEDR |= (speed_value << speed_shift);

            let pupd_value: u32 = self.config.pupd as u32;
            let pupd_shift: u8 = 2 * pin;
            let pupd_mask: u32 = 0x3 << pupd_shift;
            (*self.pgpiox).PUPDR &= !(pupd_mask);
            (*self.pgpiox).PUPDR |= (pupd_value << pupd_shift);

            let otype_value: u32 = self.config.output_type as u32;
            let otype_shift: u8 = pin;
            let otype_mask: u32 = 0x1 << otype_shift;
            (*self.pgpiox).OTYPER &= !(otype_mask);
            (*self.pgpiox).OTYPER |= (otype_value << otype_shift);

            if let GPIO_Mode::GPIO_MODE_AF = self.config.mode {
                let afr_index: usize = (pin / 8) as usize;
                let afr_shift: u8 = (pin % 8) * 4;
                let afr_mask: u32 = 0xF << afr_shift;
                
                (*self.pgpiox).AFR[afr_index] &= !(afr_mask);
                (*self.pgpiox).AFR[afr_index] |= ((self.config.alt_func as u32) << afr_shift);
            }
        }
    }
}

pub struct GPIO;

impl GPIO {
    pub unsafe fn periph_clock_control(pgpiox: *mut GPIO_RegDef, state: State) {
        let state_bit: u32 = state as u32;
        
        if pgpiox == GPIOA {
            (*RCC).AHB1ENR |= (state_bit << 0);
        } else if pgpiox == GPIOB {
            (*RCC).AHB1ENR |= (state_bit << 1);
        } else if pgpiox == GPIOC {
            (*RCC).AHB1ENR |= (state_bit << 2);
        } else if pgpiox == GPIOD {
            (*RCC).AHB1ENR |= (state_bit << 3);
        } else if pgpiox == GPIOE {
            (*RCC).AHB1ENR |= (state_bit << 4);
        } else if pgpiox == GPIOF {
            (*RCC).AHB1ENR |= (state_bit << 5);
        } else if pgpiox == GPIOG {
            (*RCC).AHB1ENR |= (state_bit << 6);
        } else if pgpiox == GPIOH {
            (*RCC).AHB1ENR |= (state_bit << 7);
        } else if pgpiox == GPIOI {
            (*RCC).AHB1ENR |= (state_bit << 8);
        }
    }

    pub unsafe fn deinit(pgpiox: *mut GPIO_RegDef) {
        if pgpiox == GPIOA {
            (*RCC).AHB1RSTR |= (1 << 0);
            (*RCC).AHB1RSTR &= !(1 << 0);
        } else if pgpiox == GPIOB {
            (*RCC).AHB1RSTR |= (1 << 1);
            (*RCC).AHB1RSTR &= !(1 << 1);
        } else if pgpiox == GPIOC {
            (*RCC).AHB1RSTR |= (1 << 2);
            (*RCC).AHB1RSTR &= !(1 << 2);
        } else if pgpiox == GPIOD {
            (*RCC).AHB1RSTR |= (1 << 3);
            (*RCC).AHB1RSTR &= !(1 << 3);
        } else if pgpiox == GPIOE {
            (*RCC).AHB1RSTR |= (1 << 4);
            (*RCC).AHB1RSTR &= !(1 << 4);
        } else if pgpiox == GPIOF {
            (*RCC).AHB1RSTR |= (1 << 5);
            (*RCC).AHB1RSTR &= !(1 << 5);
        } else if pgpiox == GPIOG {
            (*RCC).AHB1RSTR |= (1 << 6);
            (*RCC).AHB1RSTR &= !(1 << 6);
        } else if pgpiox == GPIOH {
            (*RCC).AHB1RSTR |= (1 << 7);
            (*RCC).AHB1RSTR &= !(1 << 7);
        } else if pgpiox == GPIOI {
            (*RCC).AHB1RSTR |= (1 << 8);
            (*RCC).AHB1RSTR &= !(1 << 8);
        }
    }

    pub unsafe fn read_pin(pgpiox: *mut GPIO_RegDef, pin: GPIO_Pin) -> u8 {
        let pin_number: u8 = pin as u8;
        let value: u32 = ((*pgpiox).IDR >> pin_number) & 0x1;
        value as u8
    }

    pub unsafe fn read_port(pgpiox: *mut GPIO_RegDef) -> u16 {
        (*pgpiox).IDR as u16
    }

    pub unsafe fn write_pin(pgpiox: *mut GPIO_RegDef, pin: GPIO_Pin, value: u8) {
        let pin_number: u8 = pin as u8;
        
        if 1 == value {
            (*pgpiox).ODR |= (1 << pin_number);
        } else {
            (*pgpiox).ODR &= !(1 << pin_number);
        }
    }

    pub unsafe fn write_port(pgpiox: *mut GPIO_RegDef, value: u16) {
        (*pgpiox).ODR = value as u32;
    }

    pub unsafe fn toggle_pin(pgpiox: *mut GPIO_RegDef, pin: GPIO_Pin) {
        let pin_number: u8 = pin as u8;
        (*pgpiox).ODR ^= (1 << pin_number);
    }
}