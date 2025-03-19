use core::marker::PhantomData;
use crate::stm32f4xx::{
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI, GPIORegDef, RegValue
};
use crate::stm32f4_rcc::{RccHandle, RccRegister, RccError};

// Error type for GPIO operations
#[derive(Debug, Clone, Copy)]
pub enum GpioError {
    InvalidPort,
    InvalidPin,
    InvalidConfiguration,
    HardwareFault,
    RccError(RccError),
}

// Convert RccError to GpioError
impl From<RccError> for GpioError {
    fn from(error: RccError) -> Self {
        GpioError::RccError(error)
    }
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

#[derive(Debug, Clone, Copy)]
pub struct GpioPinConfig {
    pub pinnumber: GpioPin,
    pub mode: GpioMode,
    pub speed: GpioSpeed,
    pub pulltype: GpioPullUpDown,
    pub outputtype: GpioOutputType,
    pub altfunc: u8,
}

impl Default for GpioPinConfig {
    fn default() -> Self {
        GpioPinConfig {
            pinnumber: GpioPin::Pin0,
            mode: GpioMode::Input,
            speed: GpioSpeed::Low,
            pulltype: GpioPullUpDown::NoPull,
            outputtype: GpioOutputType::PushPull,
            altfunc: 0,
        }
    }
}

pub struct GpioRegister {
    reg: *mut GPIORegDef,
}

impl GpioRegister {
    pub fn new(port: GpioPort) -> Result<Self, GpioError> {
        let reg = match port {
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
        
        if reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        Ok(GpioRegister { reg })
    }
    
    // MODER register
    pub fn read_moder(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).MODER };
        Ok(RegValue::new(value))
    }
    
    pub fn write_moder(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).MODER = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_moder<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_moder()?;
        let new_value = f(value);
        self.write_moder(new_value)
    }
    
    // OTYPER register
    pub fn read_otyper(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).OTYPER };
        Ok(RegValue::new(value))
    }
    
    pub fn write_otyper(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).OTYPER = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_otyper<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_otyper()?;
        let new_value = f(value);
        self.write_otyper(new_value)
    }
    
    // OSPEEDR register
    pub fn read_ospeedr(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).OSPEEDR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_ospeedr(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).OSPEEDR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_ospeedr<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_ospeedr()?;
        let new_value = f(value);
        self.write_ospeedr(new_value)
    }
    
    // PUPDR register
    pub fn read_pupdr(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).PUPDR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_pupdr(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).PUPDR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_pupdr<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_pupdr()?;
        let new_value = f(value);
        self.write_pupdr(new_value)
    }
    
    // IDR register
    pub fn read_idr(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).IDR };
        Ok(RegValue::new(value))
    }
    
    // ODR register
    pub fn read_odr(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).ODR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_odr(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).ODR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_odr<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_odr()?;
        let new_value = f(value);
        self.write_odr(new_value)
    }
    
    // BSRR register
    pub fn write_bsrr(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).BSRR = value.get();
        }
        
        Ok(())
    }
    
    // LCKR register
    pub fn read_lckr(&self) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let value = unsafe { (*self.reg).LCKR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_lckr(&self, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        unsafe {
            (*self.reg).LCKR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_lckr<F>(&self, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_lckr()?;
        let new_value = f(value);
        self.write_lckr(new_value)
    }
    
    // AFR registers
    pub fn read_afr(&self, index: usize) -> Result<RegValue, GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        if index > 1 {
            return Err(GpioError::InvalidConfiguration);
        }
        
        let value = unsafe { (*self.reg).AFR[index] };
        Ok(RegValue::new(value))
    }
    
    pub fn write_afr(&self, index: usize, value: RegValue) -> Result<(), GpioError> {
        if self.reg.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        if index > 1 {
            return Err(GpioError::InvalidConfiguration);
        }
        
        unsafe {
            (*self.reg).AFR[index] = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_afr<F>(&self, index: usize, f: F) -> Result<(), GpioError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_afr(index)?;
        let new_value = f(value);
        self.write_afr(index, new_value)
    }
    
    // for operations that need direct access
    pub fn get_raw_ptr(&self) -> *mut GPIORegDef {
        self.reg
    }
}

pub struct GpioHandle<'a> {
    pub pgpiox: *mut GPIORegDef,
    pub config: GpioPinConfig,
    register: GpioRegister,
    _marker: PhantomData<&'a ()>,
}

impl<'a> GpioHandle<'a> {
    pub fn new(port: GpioPort) -> Result<Self, GpioError> {
        let register = GpioRegister::new(port)?;
        
        Ok(GpioHandle {
            pgpiox: register.get_raw_ptr(),
            config: GpioPinConfig::default(),
            register,
            _marker: PhantomData,
        })
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
        self.config.pinnumber = pin;
        self.config.mode = mode;
        self.config.speed = speed;
        self.config.pulltype = pull_type;
        self.config.outputtype = output_type;
        self.config.altfunc = alt_func;
        Ok(())
    }

    pub fn init(&self) -> Result<(), GpioError> {
        // Enable GPIO clock
        Gpio::periph_clock_control(self.pgpiox, true)?;

        let pin = self.config.pinnumber as u8;

        // Configure MODER
        if let GpioMode::Input | GpioMode::Output | GpioMode::AlternateFunction | GpioMode::Analog = self.config.mode {
            let mode_value = self.config.mode as u32;
            let mode_shift = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
            let mode_mask = u32::checked_shl(0x3, mode_shift.into()).ok_or(GpioError::InvalidPin)?;
            
            self.register.modify_moder(|mut reg| {
                reg.clear_bits(mode_mask);
                reg.set_bits(u32::checked_shl(mode_value, mode_shift.into()).unwrap_or(0));
                reg
            })?;
        }

        // Configure OSPEEDR
        let speed_value = self.config.speed as u32;
        let speed_shift = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
        let speed_mask = u32::checked_shl(0x3, speed_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_ospeedr(|mut reg| {
            reg.clear_bits(speed_mask);
            reg.set_bits(u32::checked_shl(speed_value, speed_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure PUPDR
        let pupd_value = self.config.pulltype as u32;
        let pupd_shift = u8::checked_mul(pin, 2).ok_or(GpioError::InvalidPin)?;
        let pupd_mask = u32::checked_shl(0x3, pupd_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_pupdr(|mut reg| {
            reg.clear_bits(pupd_mask);
            reg.set_bits(u32::checked_shl(pupd_value, pupd_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure OTYPER
        let otype_value = self.config.outputtype as u32;
        let otype_shift = pin;
        let otype_mask = u32::checked_shl(0x1, otype_shift.into()).ok_or(GpioError::InvalidPin)?;
        
        self.register.modify_otyper(|mut reg| {
            reg.clear_bits(otype_mask);
            reg.set_bits(u32::checked_shl(otype_value, otype_shift.into()).unwrap_or(0));
            reg
        })?;

        // Configure AFR if needed
        if let GpioMode::AlternateFunction = self.config.mode {
            let afr_index = (pin / 8) as usize;
            let afr_shift = (pin % 8) * 4;
            let afr_mask = 0xF << afr_shift;
            
            self.register.modify_afr(afr_index, |mut reg| {
                reg.clear_bits(afr_mask);
                reg.set_bits((self.config.altfunc as u32) << afr_shift);
                reg
            })?;
        }

        Ok(())
    }
    
    pub fn write(&self, state: u8) -> Result<(), GpioError> {
        Gpio::write_pin(self.pgpiox, self.config.pinnumber, state)
    }
    
    pub fn read(&self) -> Result<u8, GpioError> {
        Gpio::read_pin(self.pgpiox, self.config.pinnumber)
    }
    
    pub fn toggle(&self) -> Result<(), GpioError> {
        Gpio::toggle_pin(self.pgpiox, self.config.pinnumber)
    }
    
    pub fn set_pin(&self) -> Result<(), GpioError> {
        Gpio::write_pin(self.pgpiox, self.config.pinnumber, 1)
    }
    
    pub fn reset_pin(&self) -> Result<(), GpioError> {
        Gpio::write_pin(self.pgpiox, self.config.pinnumber, 0)
    }
    
    pub fn get_port_value(&self) -> Result<u16, GpioError> {
        Gpio::read_port(self.pgpiox)
    }
    
    pub fn write_port(&self, value: u16) -> Result<(), GpioError> {
        Gpio::write_port(self.pgpiox, value)
    }
}

pub struct Gpio;

impl Gpio {
    pub fn periph_clock_control(pgpiox: *mut GPIORegDef, enable: bool) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let rcc_handle = RccHandle::new()?;
        rcc_handle.gpio_clock_control(pgpiox as u32, enable)?;
        Ok(())
    }

    pub fn deinit(pgpiox: *mut GPIORegDef) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        // Get GPIO bit position for reset
        let bit_pos = match () {
            _ if pgpiox == GPIOA => 0,
            _ if pgpiox == GPIOB => 1,
            _ if pgpiox == GPIOC => 2,
            _ if pgpiox == GPIOD => 3,
            _ if pgpiox == GPIOE => 4,
            _ if pgpiox == GPIOF => 5,
            _ if pgpiox == GPIOG => 6,
            _ if pgpiox == GPIOH => 7,
            _ if pgpiox == GPIOI => 8,
            _ => return Err(GpioError::InvalidPort),
        };
        
        // Get RCC handle and reset the peripheral
        let rcc_handle = RccHandle::new()?;
        let rcc_reg = &rcc_handle.rcc_reg;
        rcc_reg.reset_peripheral("AHB1", bit_pos)?;
        
        Ok(())
    }

    pub fn read_pin(pgpiox: *mut GPIORegDef, pin: GpioPin) -> Result<u8, GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number = pin as u8;
        let reg = GpioRegister { reg: pgpiox };
        
        // Read the IDR
        let idr_value = reg.read_idr()?.get();
        
        // Extract the pin value
        let pin_value = (idr_value >> pin_number) & 0x1;
        
        Ok(pin_value as u8)
    }

    pub fn read_port(pgpiox: *mut GPIORegDef) -> Result<u16, GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let reg = GpioRegister { reg: pgpiox };
        
        // Read the IDR
        let idr_value = reg.read_idr()?.get();
        
        // The port value is the lower 16 bits of IDR
        Ok((idr_value & 0xFFFF) as u16)
    }

    pub fn write_pin(pgpiox: *mut GPIORegDef, pin: GpioPin, value: u8) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number = pin as u8;
        let reg = GpioRegister { reg: pgpiox };
        
        // Use BSRR for atomic bit set/reset
        if 1 == value {
            // Set the pin
            reg.write_bsrr(RegValue::new(1 << pin_number))?;
        } else {
            // Reset the pin
            reg.write_bsrr(RegValue::new(1 << (pin_number + 16)))?;
        }
        
        Ok(())
    }

    pub fn write_port(pgpiox: *mut GPIORegDef, value: u16) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let reg = GpioRegister { reg: pgpiox };
        
        // Write the value to the ODR
        reg.write_odr(RegValue::new(value as u32))?;
        
        Ok(())
    }

    pub fn toggle_pin(pgpiox: *mut GPIORegDef, pin: GpioPin) -> Result<(), GpioError> {
        if pgpiox.is_null() {
            return Err(GpioError::InvalidPort);
        }
        
        let pin_number = pin as u8;
        let reg = GpioRegister { reg: pgpiox };
        
        reg.modify_odr(|mut reg_val| {
            reg_val.modify(|val| val ^ (1 << pin_number));
            reg_val
        })?;
        
        Ok(())
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
    let mut handle = GpioHandle::new(port)?;
    handle.config_pin(pin, mode, speed, pull_type, output_type, 0)?;
    handle.init()?;
    Ok(handle)
}

pub fn init_gpio_pin_with_af(
    port: GpioPort,
    pin: GpioPin,
    speed: GpioSpeed,
    pull_type: GpioPullUpDown,
    output_type: GpioOutputType,
    alt_func: u8,
) -> Result<GpioHandle<'static>, GpioError> 
{
    let mut handle = GpioHandle::new(port)?;
    handle.config_pin(pin, GpioMode::AlternateFunction, speed, pull_type, output_type, alt_func)?;
    handle.init()?;
    Ok(handle)
}