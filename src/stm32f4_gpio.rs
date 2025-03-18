use core::marker::PhantomData;
use crate::stm32f4xx::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIORegDef, RCC, RegValue};
// Error type for GPIO operations
#[derive(Debug, Clone, Copy)]
pub enum GpioError {
    InvalidPort,
    InvalidPin,
    InvalidConfiguration,
    HardwareFault,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioPort {
    GpioA,
    GpioB,
    GpioC,
    GpioD,
    GpioE,
    GpioF,
    GpioG,
    GpioH,
    GpioI,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Disable = 0,
    Enable  = 1,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioMode {
    Input                   = 0,
    Output                  = 1,
    AlternateFunction       = 2,
    Analog                  = 3,
    InterruptFalling        = 4,
    InterruptRising         = 5,
    InterruptRisingFalling  = 6,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioOutputType {
    PushPull    = 0,
    OpenDrain   = 1,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioSpeed {
    Low         = 0,
    Medium      = 1,
    High        = 2,
    VeryHigh    = 3,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioPullUpDown {
    NoPull      = 0,
    PullUp      = 1,
    PullDown    = 2,
}

#[derive(Debug, Clone, Copy)]
pub enum GpioPin {
    Pin0 = 0,
    Pin1 = 1,
    Pin2 = 2,
    Pin3 = 3,
    Pin4 = 4,
    Pin5 = 5,
    Pin6 = 6,
    Pin7 = 7,
    Pin8 = 8,
    Pin9 = 9,
    Pin10 = 10,
    Pin11 = 11,
    Pin12 = 12,
    Pin13 = 13,
    Pin14 = 14,
    Pin15 = 15,
}

pub struct GpioPinConfig {
    pub pin_number: GpioPin,
    pub mode: GpioMode,
    pub speed: GpioSpeed,
    pub pull_type: GpioPullUpDown,
    pub output_type: GpioOutputType,
    pub alt_func: u8,
}

impl GpioPinConfig {
    pub fn new() -> Self {
        GpioPinConfig {
            pin_number: GpioPin::Pin0,
            mode: GpioMode::Input,
            speed: GpioSpeed::Low,
            pull_type: GpioPullUpDown::NoPull,
            output_type: GpioOutputType::PushPull,
            alt_func: 0,
        }
    }
}

// Container for GPIO register access
struct GpioRegister {
    register: *mut GPIORegDef,
}

impl GpioRegister {
    fn new(port: GpioPort) -> Result<Self, GpioError> {
        let register: *mut GPIORegDef = match port {
            GpioPort::GpioA => GPIOA,
            GpioPort::GpioB => GPIOB,
            GpioPort::GpioC => GPIOC,
            GpioPort::GpioD => GPIOD,
            GpioPort::GpioE => GPIOE,
            GpioPort::GpioF => GPIOF,
            GpioPort::GpioG => GPIOG,
            GpioPort::GpioH => GPIOH,
            GpioPort::GpioI => GPIOI,
        };
        
        Ok(GpioRegister { register })
    }
    
    fn read_register(&self, offset: usize) -> Result<RegValue, GpioError> {
        if self.register.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value: u32 = unsafe {
            match offset {
                0 => (*self.register).MODER,
                1 => (*self.register).OTYPER,
                2 => (*self.register).OSPEEDR,
                3 => (*self.register).PUPDR,
                4 => (*self.register).IDR,
                5 => (*self.register).ODR,
                6 => (*self.register).BSRR,
                7 => (*self.register).LCKR,
                _ => return Err(GpioError::InvalidConfiguration),
            }
        };
        
        Ok(RegValue::new(value))
    }
    
    fn write_register(&self, offset: usize, value: RegValue) -> Result<(), GpioError> {
        if self.register.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            match offset {
                0 => (*self.register).MODER = value.get(),
                1 => (*self.register).OTYPER = value.get(),
                2 => (*self.register).OSPEEDR = value.get(),
                3 => (*self.register).PUPDR = value.get(),
                5 => (*self.register).ODR = value.get(),
                6 => (*self.register).BSRR = value.get(),
                7 => (*self.register).LCKR = value.get(),
                _ => return Err(GpioError::InvalidConfiguration),
            }
        }
        
        Ok(())
    }
    
    fn modify_register<F>(&self, offset: usize, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_register(offset)?;
        let new_value: RegValue = f(value);
        self.write_register(offset, new_value)
    }
    
    // Read AFR
    fn read_afr(&self, index: usize) -> Result<RegValue, GpioError> {
        if self.register.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        if index > 1 {
            return Err(GpioError::InvalidConfiguration);
        }
        
        let value: u32 = unsafe { (*self.register).AFR[index] };
        
        Ok(RegValue::new(value))
    }
    
    // Write to AFR
    fn write_afr(&self, index: usize, value: RegValue) -> Result<(), GpioError> {
        if self.register.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        if index > 1 {
            return Err(GpioError::InvalidConfiguration);
        }
        
        unsafe {
            (*self.register).AFR[index] = value.get();
        }
        
        Ok(())
    }
    
    // Safe modification of AFR register values
    fn modify_afr<F>(&self, index: usize, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_afr(index)?;
        let new_value: RegValue = f(value);
        self.write_afr(index, new_value)
    }
    
    // Get raw register pointer for operations that need direct access
    fn get_raw_ptr(&self) -> *mut GPIORegDef {
        self.register
    }
}

pub struct GpioHandle<'a> {
    pub pgpiox: *mut GPIORegDef,
    pub config: GpioPinConfig,
    register: GpioRegister,
    _marker: PhantomData<&'a ()>,
}

impl<'a> GpioHandle<'a> {
    pub fn new(port: GpioPort) -> Self {
        let register: GpioRegister = GpioRegister::new(port).unwrap_or_else(|_| {
            GpioRegister { register: core::ptr::null_mut() }
        });
        
        GpioHandle {
            pgpiox: register.get_raw_ptr(),
            config: GpioPinConfig::new(),
            register,
            _marker: PhantomData,
        }
    }

    pub fn config_pin(
        &mut self,
        pin: GpioPin,
        mode: GpioMode,
        speed: GpioSpeed,
        pull_type: GpioPullUpDown,
        output_type: GpioOutputType,
        alt_func: u8,
    ) -> Result<(), GpioError> {
        self.config.pin_number = pin;
        self.config.mode = mode;
        self.config.speed = speed;
        self.config.pull_type = pull_type;
        self.config.output_type = output_type;
        self.config.alt_func = alt_func;
        Ok(())
    }

    pub fn init(&self) -> Result<(), GpioError> {
        // Enable clock for the GPIO port
        Gpio::periph_clock_control(self.pgpiox, State::Enable)?;

        let pin: u8 = self.config.pin_number as u8;

        // Configure MODER
        if let GpioMode::Input | GpioMode::Output | GpioMode::AlternateFunction | GpioMode::Analog = self.config.mode {
            let mode_value: u32 = self.config.mode as u32;
            let mode_shift: u8 = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
            let mode_mask: u32 = u32::checked_shl(0x3, mode_shift.into()).ok_or(GpioError::InvalidPin)?;
            
            self.register.modify_register(0, |mut reg: RegValue| {
                reg.clear_bits(mode_mask);
                reg.set_bits(u32::checked_shl(mode_value, mode_shift.into()).unwrap_or(0));
                reg
            })?;
        } else {

        }

        // Configure SPEEDR
        let speed_value: u32 = self.config.speed as u32;
        let speed_shift: u8 = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
        let speed_mask: u32 = u32::checked_shl(0x3, speed_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_register(2, |mut reg| {
            reg.clear_bits(speed_mask);
            reg.set_bits(u32::checked_shl(speed_value, speed_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure pull-up/pull-down register
        let pupd_value: u32 = self.config.pull_type as u32;
        let pupd_shift: u8 = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
        let pupd_mask: u32 = u32::checked_shl(0x3, pupd_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_register(3, |mut reg: RegValue| {
            reg.clear_bits(pupd_mask);
            reg.set_bits(u32::checked_shl(pupd_value, pupd_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure OTYPER
        let otype_value: u32 = self.config.output_type as u32;
        let otype_shift: u8 = pin;
        let otype_mask: u32 = u32::checked_shl(0x1, otype_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_register(1, |mut reg| {
            reg.clear_bits(otype_mask);
            reg.set_bits(u32::checked_shl(otype_value, otype_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure alternate function register if needed
        if let GpioMode::AlternateFunction = self.config.mode {
            let afr_index: usize = (pin / 8) as usize;
            let afr_shift: u8 = (pin % 8) * 4;
            let afr_mask: u32 = 0xF << afr_shift;
            
            self.register.modify_afr(afr_index, |mut reg: RegValue| {
                reg.clear_bits(afr_mask);
                reg.set_bits((self.config.alt_func as u32) << afr_shift);
                reg
            })?;
        }

        Ok(())
    }
    
    pub fn write(&self, state: u8) -> Result<(), GpioError> {
        Gpio::write_pin(self.pgpiox, self.config.pin_number, state)
    }
    
    pub fn read(&self) -> Result<u8, GpioError> {
        Gpio::read_pin(self.pgpiox, self.config.pin_number)
    }
    
    pub fn toggle(&self) -> Result<(), GpioError> {
        Gpio::toggle_pin(self.pgpiox, self.config.pin_number)
    }
}

pub fn init_gpio_pin(
    port: GpioPort,
    pin: GpioPin,
    mode: GpioMode,
    speed: GpioSpeed,
    pull_type: GpioPullUpDown,
    output_type: GpioOutputType,
) -> Result<GpioHandle<'static>, GpioError> {
    let mut handle: GpioHandle<'_> = GpioHandle::new(port);
    handle.config_pin(pin, mode, speed, pull_type, output_type, 0)?;
    handle.init()?;
    Ok(handle)
}

pub struct Gpio;

impl Gpio {
    pub fn periph_clock_control(pgpiox: *mut GPIORegDef, state: State) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let state_bit: u32 = state as u32;
        
        // temp wrapper for RCC
        struct RccRegister;
        
        impl RccRegister {
            fn modify_ahb1enr<F>(f: F) -> Result<(), GpioError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(GpioError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).AHB1ENR;
                    (*RCC).AHB1ENR = f(current);
                }
                Ok(())
            }
        }
        
        // Determine the bit position based on which GPIO port we're using
        let bit_position: i32 = if pgpiox == GPIOA {
            0
        } else if pgpiox == GPIOB {
            1
        } else if pgpiox == GPIOC {
            2
        } else if pgpiox == GPIOD {
            3
        } else if pgpiox == GPIOE {
            4
        } else if pgpiox == GPIOF {
            5
        } else if pgpiox == GPIOG {
            6
        } else if pgpiox == GPIOH {
            7
        } else if pgpiox == GPIOI {
            8
        } else {
            return Err(GpioError::InvalidPort);
        };
        
        // Modify the RCC AHB1ENR register to enable/disable the GPIO clock
        RccRegister::modify_ahb1enr(|reg: u32| {
            if state == State::Enable {
                reg | (state_bit << bit_position)
            } else {
                reg & !(1 << bit_position)
            }
        })?;
        
        Ok(())
    }

    pub fn deinit(pgpiox: *mut GPIORegDef) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        // Create temp RCC wrapper
        struct RccRegister;
        
        impl RccRegister {
            fn modify_ahb1rstr<F>(f: F) -> Result<(), GpioError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(GpioError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).AHB1RSTR;
                    (*RCC).AHB1RSTR = f(current);
                }
                Ok(())
            }
        }
        
        // Determine the bit position based on which GPIO port we're using
        let bit_position: i32 = if pgpiox == GPIOA {
            0
        } else if pgpiox == GPIOB {
            1
        } else if pgpiox == GPIOC {
            2
        } else if pgpiox == GPIOD {
            3
        } else if pgpiox == GPIOE {
            4
        } else if pgpiox == GPIOF {
            5
        } else if pgpiox == GPIOG {
            6
        } else if pgpiox == GPIOH {
            7
        } else if pgpiox == GPIOI {
            8
        } else {
            return Err(GpioError::InvalidPort);
        };
        
        // First set the reset bit
        RccRegister::modify_ahb1rstr(|reg: u32| reg | (1 << bit_position))?;
        
        // Then clear it
        RccRegister::modify_ahb1rstr(|reg: u32| reg & !(1 << bit_position))?;
        
        Ok(())
    }

    pub fn read_pin(pgpiox: *mut GPIORegDef, pin: GpioPin) -> Result<u8, GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number: u8 = pin as u8;
        
        // Read the IDR
        let idr_value: u32 = unsafe { (*pgpiox).IDR };
        
        // Extract the pin value using safe bit operations
        let pin_value: u32 = (idr_value >> pin_number) & 0x1;
        
        Ok(pin_value as u8)
    }

    pub fn read_port(pgpiox: *mut GPIORegDef) -> Result<u16, GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        // Read the IDR
        let idr_value: u32 = unsafe { (*pgpiox).IDR };
        
        // The port value is the lower 16 bits of IDR
        Ok((idr_value & 0xFFFF) as u16)
    }

    pub fn write_pin(pgpiox: *mut GPIORegDef, pin: GpioPin, value: u8) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number: u8 = pin as u8;
        
        // Get current ODR value
        let odr_value: u32 = unsafe { (*pgpiox).ODR };
        
        // Modify the ODR value based on the requested pin value
        let new_odr_value: u32 = if value == 1 {
            odr_value | (1 << pin_number)
        } else {
            odr_value & !(1 << pin_number)
        };
        
        // Write the new ODR value
        unsafe {
            (*pgpiox).ODR = new_odr_value;
        }
        
        Ok(())
    }

    pub fn write_port(pgpiox: *mut GPIORegDef, value: u16) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        // Write the value to the ODR
        unsafe {
            (*pgpiox).ODR = value as u32;
        }
        
        Ok(())
    }

    pub fn toggle_pin(pgpiox: *mut GPIORegDef, pin: GpioPin) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number: u8 = pin as u8;
        
        // Get current ODR value
        let odr_value: u32 = unsafe { (*pgpiox).ODR };
        
        // Toggle the pin by XORing with 1 at the pin position
        let new_odr_value: u32 = odr_value ^ (1 << pin_number);
        
        // Write the new ODR value
        unsafe {
            (*pgpiox).ODR = new_odr_value;
        }
        
        Ok(())
    }
}