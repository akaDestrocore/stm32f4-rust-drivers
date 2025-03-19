use core::marker::PhantomData;
use crate::stm32f4xx::{RCC, FLASH_R, RCCRegDef, FLASHRegDef, RegValue};

// Error type for RCC
#[derive(Debug, Clone, Copy)]
pub enum RccError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClockType {
    SystemClock = 1,
    HClock      = 2,
    PClock1     = 4,
    PClock2     = 8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OscillatorType {
    HSE = 1,
    HSI = 2,
    LSE = 4,
    LSI = 8,
}

impl core::ops::BitOr for OscillatorType {
    type Output = u32;

    fn bitor(self, rhs: Self) -> Self::Output {
        self as u32 | rhs as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HseState {
    Off     = 0,
    On      = 1,
    Bypass  = 2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HsiState {
    Off = 0,
    On  = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LseState {
    Off     = 0,
    On      = 1,
    Bypass  = 2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LsiState {
    Off = 0,
    On  = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PllState {
    None = 0,
    Off  = 1,
    On   = 2,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SysClkSource {
    HSI     = 0,
    HSE     = 1,
    PllClk  = 2,
    PllRClk = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AhbClkDivider {
    Div1   = 0x00,
    Div2   = 0x08,
    Div4   = 0x09,
    Div8   = 0x0A,
    Div16  = 0x0B,
    Div64  = 0x0C,
    Div128 = 0x0D,
    Div256 = 0x0E,
    Div512 = 0x0F,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApbClkDivider {
    Div1  = 0,
    Div2  = 4,
    Div4  = 5,
    Div8  = 6,
    Div16 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum McoIndex {
    Mco1 = 0,
    Mco2 = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mco1Source {
    Hsi    = 0,
    Lse    = 1,
    Hse    = 2,
    PllClk = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mco2Source {
    SysClk  = 0,
    PllI2s  = 1,
    Hse     = 2,
    PllClk  = 3,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum McoDiv {
    Div1 = 0,
    Div2 = 4,
    Div3 = 5,
    Div4 = 6,
    Div5 = 7,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PllSource {
    Hsi = 0,
    Hse = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PllP {
    Div2 = 0,
    Div4 = 1,
    Div6 = 2,
    Div8 = 3,
}

#[derive(Debug, Clone, Copy)]
pub struct PllConfig {
    pub state: PllState,
    pub source: PllSource,
    pub m: u8,
    pub n: u16,
    pub p: PllP,
    pub q: u8,
}

impl Default for PllConfig {
    fn default() -> Self {
        PllConfig {
            state: PllState::None,
            source: PllSource::Hsi,
            m: 4,
            n: 168,
            p: PllP::Div2,
            q: 4,
        }
    }
}

#[derive(Debug, Clone)]
pub struct OscConfig {
    pub oscillator_type: OscillatorType,
    pub hse_state: HseState,
    pub lse_state: LseState,
    pub hsi_state: HsiState,
    pub hsi_calibration: u8,
    pub lsi_state: LsiState,
    pub pll: PllConfig,
}

impl Default for OscConfig {
    fn default() -> Self {
        OscConfig {
            oscillator_type: OscillatorType::HSI,
            hse_state: HseState::Off,
            lse_state: LseState::Off,
            hsi_state: HsiState::On,
            hsi_calibration: 16,
            lsi_state: LsiState::Off,
            pll: PllConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ClockConfig {
    pub clock_type: u32,
    pub sys_clk_source: SysClkSource,
    pub ahb_clk_divider: AhbClkDivider,
    pub apb1_clk_divider: ApbClkDivider,
    pub apb2_clk_divider: ApbClkDivider,
}

impl Default for ClockConfig {
    fn default() -> Self {
        ClockConfig {
            clock_type: ClockType::SystemClock as u32,
            sys_clk_source: SysClkSource::HSI,
            ahb_clk_divider: AhbClkDivider::Div1,
            apb1_clk_divider: ApbClkDivider::Div1,
            apb2_clk_divider: ApbClkDivider::Div1,
        }
    }
}

pub static mut SYSTEM_CORE_CLOCK: u32 = 16_000_000;

const HSI_VALUE: u32 = 16_000_000;
const HSE_VALUE: u32 = 8_000_000;

const AHB_PRESCALER: [u16; 8] = [2, 4, 8, 16, 64, 128, 256, 512];
const APB_PRESCALER: [u16; 4] = [2, 4, 8, 16];

// RCC register access
pub struct RccRegister {
    reg: *mut RCCRegDef,
}

impl RccRegister {
    pub fn new() -> Result<Self, RccError> {
        if RCC.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        Ok(RccRegister { reg: RCC })
    }
    
    // Helper method to read any register
    fn read_register(&self, offset: usize) -> Result<RegValue, RccError> {
        if self.reg.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value = unsafe {
            match offset {
                0x00 => (*self.reg).CR,
                0x04 => (*self.reg).PLLCFGR,
                0x08 => (*self.reg).CFGR,
                0x0C => (*self.reg).CIR,
                0x10 => (*self.reg).AHB1RSTR,
                0x14 => (*self.reg).AHB2RSTR,
                0x18 => (*self.reg).AHB3RSTR,
                0x20 => (*self.reg).APB1RSTR,
                0x24 => (*self.reg).APB2RSTR,
                0x30 => (*self.reg).AHB1ENR,
                0x34 => (*self.reg).AHB2ENR,
                0x38 => (*self.reg).AHB3ENR,
                0x40 => (*self.reg).APB1ENR,
                0x44 => (*self.reg).APB2ENR,
                0x50 => (*self.reg).AHB1LPENR,
                0x54 => (*self.reg).AHB2LPENR,
                0x58 => (*self.reg).AHB3LPENR,
                0x60 => (*self.reg).APB1LPENR,
                0x64 => (*self.reg).APB2LPENR,
                0x70 => (*self.reg).BDCR,
                0x74 => (*self.reg).CSR,
                0x80 => (*self.reg).SSCGR,
                0x84 => (*self.reg).PLLI2SCFGR,
                _ => return Err(RccError::InvalidConfiguration),
            }
        };
        
        Ok(RegValue::new(value))
    }
    
    // Helper method to write any register
    fn write_register(&self, offset: usize, value: RegValue) -> Result<(), RccError> {
        if self.reg.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            match offset {
                0x00 => (*self.reg).CR = value.get(),
                0x04 => (*self.reg).PLLCFGR = value.get(),
                0x08 => (*self.reg).CFGR = value.get(),
                0x0C => (*self.reg).CIR = value.get(),
                0x10 => (*self.reg).AHB1RSTR = value.get(),
                0x14 => (*self.reg).AHB2RSTR = value.get(),
                0x18 => (*self.reg).AHB3RSTR = value.get(),
                0x20 => (*self.reg).APB1RSTR = value.get(),
                0x24 => (*self.reg).APB2RSTR = value.get(),
                0x30 => (*self.reg).AHB1ENR = value.get(),
                0x34 => (*self.reg).AHB2ENR = value.get(),
                0x38 => (*self.reg).AHB3ENR = value.get(),
                0x40 => (*self.reg).APB1ENR = value.get(),
                0x44 => (*self.reg).APB2ENR = value.get(),
                0x50 => (*self.reg).AHB1LPENR = value.get(),
                0x54 => (*self.reg).AHB2LPENR = value.get(),
                0x58 => (*self.reg).AHB3LPENR = value.get(),
                0x60 => (*self.reg).APB1LPENR = value.get(),
                0x64 => (*self.reg).APB2LPENR = value.get(),
                0x70 => (*self.reg).BDCR = value.get(),
                0x74 => (*self.reg).CSR = value.get(),
                0x80 => (*self.reg).SSCGR = value.get(),
                0x84 => (*self.reg).PLLI2SCFGR = value.get(),
                _ => return Err(RccError::InvalidConfiguration),
            }
        };
        
        Ok(())
    }
    
    // Helper method to modify any register
    fn modify_register<F>(&self, offset: usize, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_register(offset)?;
        let new_value = f(value);
        self.write_register(offset, new_value)
    }
    
    // CR
    pub fn read_cr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x00)
    }
    
    pub fn write_cr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x00, value)
    }
    
    pub fn modify_cr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x00, f)
    }
    
    // PLLCFGR
    pub fn read_pllcfgr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x04)
    }
    
    pub fn write_pllcfgr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x04, value)
    }
    
    pub fn modify_pllcfgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x04, f)
    }
    
    // CFGR
    pub fn read_cfgr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x08)
    }
    
    pub fn write_cfgr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x08, value)
    }
    
    pub fn modify_cfgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x08, f)
    }
    
    // CIR
    pub fn read_cir(&self) -> Result<RegValue, RccError> {
        self.read_register(0x0C)
    }
    
    pub fn write_cir(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x0C, value)
    }
    
    pub fn modify_cir<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x0C, f)
    }
    
    // AHB1RSTR
    pub fn read_ahb1rstr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x10)
    }
    
    pub fn write_ahb1rstr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x10, value)
    }
    
    pub fn modify_ahb1rstr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x10, f)
    }
    
    // AHB2RSTR
    pub fn read_ahb2rstr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x14)
    }
    
    pub fn write_ahb2rstr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x14, value)
    }
    
    pub fn modify_ahb2rstr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x14, f)
    }
    
    // AHB3RSTR
    pub fn read_ahb3rstr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x18)
    }
    
    pub fn write_ahb3rstr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x18, value)
    }
    
    pub fn modify_ahb3rstr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x18, f)
    }
    
    // APB1RSTR
    pub fn read_apb1rstr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x20)
    }
    
    pub fn write_apb1rstr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x20, value)
    }
    
    pub fn modify_apb1rstr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x20, f)
    }
    
    // APB2RSTR
    pub fn read_apb2rstr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x24)
    }
    
    pub fn write_apb2rstr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x24, value)
    }
    
    pub fn modify_apb2rstr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x24, f)
    }
    
    // AHB1ENR
    pub fn read_ahb1enr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x30)
    }
    
    pub fn write_ahb1enr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x30, value)
    }
    
    pub fn modify_ahb1enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x30, f)
    }
    
    // AHB2ENR
    pub fn read_ahb2enr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x34)
    }
    
    pub fn write_ahb2enr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x34, value)
    }
    
    pub fn modify_ahb2enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x34, f)
    }
    
    // AHB3ENR
    pub fn read_ahb3enr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x38)
    }
    
    pub fn write_ahb3enr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x38, value)
    }
    
    pub fn modify_ahb3enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x38, f)
    }
    
    // APB1ENR
    pub fn read_apb1enr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x40)
    }
    
    pub fn write_apb1enr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x40, value)
    }
    
    pub fn modify_apb1enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x40, f)
    }
    
    // APB2ENR
    pub fn read_apb2enr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x44)
    }
    
    pub fn write_apb2enr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x44, value)
    }
    
    pub fn modify_apb2enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x44, f)
    }
    
    // AHB1LPENR
    pub fn read_ahb1lpenr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x50)
    }
    
    pub fn write_ahb1lpenr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x50, value)
    }
    
    pub fn modify_ahb1lpenr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x50, f)
    }
    
    // AHB2LPENR
    pub fn read_ahb2lpenr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x54)
    }
    
    pub fn write_ahb2lpenr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x54, value)
    }
    
    pub fn modify_ahb2lpenr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x54, f)
    }
    
    // AHB3LPENR
    pub fn read_ahb3lpenr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x58)
    }
    
    pub fn write_ahb3lpenr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x58, value)
    }
    
    pub fn modify_ahb3lpenr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x58, f)
    }
    
    // APB1LPENR
    pub fn read_apb1lpenr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x60)
    }
    
    pub fn write_apb1lpenr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x60, value)
    }
    
    pub fn modify_apb1lpenr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x60, f)
    }
    
    // APB2LPENR
    pub fn read_apb2lpenr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x64)
    }
    
    pub fn write_apb2lpenr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x64, value)
    }
    
    pub fn modify_apb2lpenr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x64, f)
    }
    
    // BDCR
    pub fn read_bdcr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x70)
    }
    
    pub fn write_bdcr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x70, value)
    }
    
    pub fn modify_bdcr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x70, f)
    }
    
    // CSR
    pub fn read_csr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x74)
    }
    
    pub fn write_csr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x74, value)
    }
    
    pub fn modify_csr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x74, f)
    }
    
    // SSCGR
    pub fn read_sscgr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x80)
    }
    
    pub fn write_sscgr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x80, value)
    }
    
    pub fn modify_sscgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x80, f)
    }
    
    // PLLI2SCFGR
    pub fn read_plli2scfgr(&self) -> Result<RegValue, RccError> {
        self.read_register(0x84)
    }
    
    pub fn write_plli2scfgr(&self, value: RegValue) -> Result<(), RccError> {
        self.write_register(0x84, value)
    }
    
    pub fn modify_plli2scfgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        self.modify_register(0x84, f)
    }
    
    // Helper method to enable/disable a specific peripheral clock
    pub fn enable_peripheral_clock(&self, bus_type: &str, peripheral_bit: u32, enable: bool) -> Result<(), RccError> {
        match bus_type {
            "AHB1" => self.modify_ahb1enr(|mut reg| {
                if enable {
                    reg.set_bits(1 << peripheral_bit);
                } else {
                    reg.clear_bits(1 << peripheral_bit);
                }
                reg
            }),
            "AHB2" => self.modify_ahb2enr(|mut reg| {
                if enable {
                    reg.set_bits(1 << peripheral_bit);
                } else {
                    reg.clear_bits(1 << peripheral_bit);
                }
                reg
            }),
            "AHB3" => self.modify_ahb3enr(|mut reg| {
                if enable {
                    reg.set_bits(1 << peripheral_bit);
                } else {
                    reg.clear_bits(1 << peripheral_bit);
                }
                reg
            }),
            "APB1" => self.modify_apb1enr(|mut reg| {
                if enable {
                    reg.set_bits(1 << peripheral_bit);
                } else {
                    reg.clear_bits(1 << peripheral_bit);
                }
                reg
            }),
            "APB2" => self.modify_apb2enr(|mut reg| {
                if enable {
                    reg.set_bits(1 << peripheral_bit);
                } else {
                    reg.clear_bits(1 << peripheral_bit);
                }
                reg
            }),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
    
    // Helper method to reset a specific peripheral
    pub fn reset_peripheral(&self, bus_type: &str, peripheral_bit: u32) -> Result<(), RccError> {
        match bus_type {
            "AHB1" => {
                self.modify_ahb1rstr(|mut reg| {
                    reg.set_bits(1 << peripheral_bit);
                    reg
                })?;
                self.modify_ahb1rstr(|mut reg| {
                    reg.clear_bits(1 << peripheral_bit);
                    reg
                })
            },
            "AHB2" => {
                self.modify_ahb2rstr(|mut reg| {
                    reg.set_bits(1 << peripheral_bit);
                    reg
                })?;
                self.modify_ahb2rstr(|mut reg| {
                    reg.clear_bits(1 << peripheral_bit);
                    reg
                })
            },
            "AHB3" => {
                self.modify_ahb3rstr(|mut reg| {
                    reg.set_bits(1 << peripheral_bit);
                    reg
                })?;
                self.modify_ahb3rstr(|mut reg| {
                    reg.clear_bits(1 << peripheral_bit);
                    reg
                })
            },
            "APB1" => {
                self.modify_apb1rstr(|mut reg| {
                    reg.set_bits(1 << peripheral_bit);
                    reg
                })?;
                self.modify_apb1rstr(|mut reg| {
                    reg.clear_bits(1 << peripheral_bit);
                    reg
                })
            },
            "APB2" => {
                self.modify_apb2rstr(|mut reg| {
                    reg.set_bits(1 << peripheral_bit);
                    reg
                })?;
                self.modify_apb2rstr(|mut reg| {
                    reg.clear_bits(1 << peripheral_bit);
                    reg
                })
            },
            _ => Err(RccError::InvalidConfiguration),
        }
    }
}

// FLASH register access
struct FlashRegister {
    register: *mut FLASHRegDef,
}

impl FlashRegister {
    pub fn new() -> Result<Self, RccError> {
        if FLASH_R.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        Ok(FlashRegister { register: FLASH_R })
    }
    
    // Read ACR register
    pub fn read_acr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).ACR };
        Ok(RegValue::new(value))
    }
    
    // Write ACR register
    pub fn write_acr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).ACR = value.get();
        }
        
        Ok(())
    }
    
    // Modify ACR register
    pub fn modify_acr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_acr()?;
        let new_value: RegValue = f(value);
        self.write_acr(new_value)
    }
}

pub struct RccHandle<'a> {
    pub rcc_reg: RccRegister,
    flash_reg: FlashRegister,
    _marker: PhantomData<&'a ()>,
}

impl<'a> RccHandle<'a> {
    pub fn new() -> Result<Self, RccError> {
        let rcc_reg: RccRegister = RccRegister::new()?;
        let flash_reg: FlashRegister = FlashRegister::new()?;
        
        Ok(RccHandle {
            rcc_reg,
            flash_reg,
            _marker: PhantomData,
        })
    }

    pub fn osc_config(&self, config: &OscConfig) -> Result<(), RccError> {
        // Configure HSE
        if (config.oscillator_type as u32 & OscillatorType::HSE as u32) != 0 {
            match config.hse_state {
                HseState::Bypass => {
                    self.rcc_reg.modify_cr(|mut reg: RegValue| {
                        reg.clear_bits(1 << 16); // Turn off HSE first
                        reg.set_bits(1 << 18);   // Set bypass mode
                        reg.set_bits(1 << 16);   // Turn on HSE
                        reg
                    })?;
                },
                HseState::On => {
                    self.rcc_reg.modify_cr(|mut reg: RegValue| {
                        reg.set_bits(1 << 16);   // Turn on HSE
                        reg
                    })?;
                },
                HseState::Off => {
                    self.rcc_reg.modify_cr(|mut reg: RegValue| {
                        reg.clear_bits(1 << 16); // Turn off HSE
                        reg
                    })?;
                },
            }
        }

        // Configure HSI
        if (config.oscillator_type as u32 & OscillatorType::HSI as u32) != 0 {
            match config.hsi_state {
                HsiState::On => {
                    self.rcc_reg.modify_cr(|mut reg: RegValue| {
                        reg.set_bits(1 << 0);    // Turn on HSI
                        reg
                    })?;
                },
                HsiState::Off => {
                    self.rcc_reg.modify_cr(|mut reg: RegValue| {
                        reg.clear_bits(1 << 0);  // Turn off HSI
                        reg
                    })?;
                },
            }
        }

        // Configure LSE
        if (config.oscillator_type as u32 & OscillatorType::LSE as u32) != 0 {
            match config.lse_state {
                LseState::On => {
                    self.rcc_reg.modify_bdcr(|mut reg: RegValue| {
                        reg.set_bits(1 << 0);    // Turn on LSE
                        reg
                    })?;
                },
                LseState::Off => {
                    self.rcc_reg.modify_bdcr(|mut reg: RegValue| {
                        reg.clear_bits(1 << 0);  // Turn off LSE
                        reg
                    })?;
                },
                LseState::Bypass => {
                    self.rcc_reg.modify_bdcr(|mut reg: RegValue| {
                        reg.clear_bits(1 << 0);  // Turn off LSE first
                        reg.set_bits(1 << 1);    // Set bypass mode
                        reg.set_bits(1 << 0);    // Turn on LSE
                        reg
                    })?;
                },
            }
        }

        // Configure LSI
        if (config.oscillator_type as u32 & OscillatorType::LSI as u32) != 0 {
            match config.lsi_state {
                LsiState::On => {
                    self.rcc_reg.modify_csr(|mut reg: RegValue| {
                        reg.set_bits(1 << 0);    // Turn on LSI
                        reg
                    })?;
                },
                LsiState::Off => {
                    self.rcc_reg.modify_csr(|mut reg| {
                        reg.clear_bits(1 << 0);  // Turn off LSI
                        reg
                    })?;
                },
            }
        }

        // Configure PLL
        if config.pll.state != PllState::None {
            // Turn off PLL first
            self.rcc_reg.modify_cr(|mut reg: RegValue| {
                reg.clear_bits(1 << 24);  // Turn off PLL
                reg
            })?;

            if config.pll.state == PllState::On {
                // Configure PLLCFGR register
                self.rcc_reg.modify_pllcfgr(|mut reg: RegValue| {
                    // Clear all configurable bits
                    reg.clear_bits(0x3F);           // Clear M bits
                    reg.clear_bits(0x1FF << 6);     // Clear N bits
                    reg.clear_bits(0x3 << 16);      // Clear P bits
                    reg.clear_bits(0xF << 24);      // Clear Q bits
                    reg.clear_bits(1 << 22);        // Clear source bit
                    
                    // Set new values
                    reg.set_bits((config.pll.m as u32) & 0x3F);
                    reg.set_bits(((config.pll.n as u32) & 0x1FF) << 6);
                    reg.set_bits(((config.pll.p as u32) & 0x3) << 16);
                    reg.set_bits(((config.pll.q as u32) & 0xF) << 24);
                    reg.set_bits(((config.pll.source as u32) & 0x1) << 22);
                    
                    reg
                })?;
                
                // Turn on PLL
                self.rcc_reg.modify_cr(|mut reg: RegValue| {
                    reg.set_bits(1 << 24);
                    reg
                })?;
                
                // Wait for PLL to stabilize
                let mut timeout: i32 = 2000;
                while timeout > 0 {
                    let cr_val = self.rcc_reg.read_cr()?;
                    
                    if (cr_val.get() & (1 << 25)) != 0 {
                        break;
                    }
                    
                    timeout -= 1;
                }
                
                if 0 == timeout {
                    return Err(RccError::Timeout);
                }
            }
        }

        Ok(())
    }

    pub fn clock_config(&self, config: &ClockConfig) -> Result<(), RccError> {
        // Configure flash latency
        self.flash_reg.modify_acr(|mut reg: RegValue| {
            reg.clear_bits(0xF);    // Clear latency bits
            reg.set_bits(0x5);      // Set 5 wait states (for high speed)
            reg
        })?;
        
        // Wait for flash latency to update
        let mut timeout: i32 = 2000;
        while timeout > 0 {
            let acr_val: RegValue = self.flash_reg.read_acr()?;
            
            if 0x5 == (acr_val.get() & 0xF) {
                break;
            }
            
            timeout -= 1;
        }
        
        if 0 == timeout {
            return Err(RccError::Timeout);
        }

        // Configure AHB, APB1, and APB2 prescalers
        if (config.clock_type & (ClockType::HClock as u32)) != 0 {
            self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                // Clear the APB1 and APB2 prescaler bits first
                reg.clear_bits(0x7 << 10);  // Clear APB1 prescaler bits
                reg.clear_bits(0x7 << 13);  // Clear APB2 prescaler bits
                
                // Then set them to default values (1:1)
                reg.set_bits(0x7 << 10);
                reg.set_bits(0x7 << 13);
                
                // Clear and set AHB prescaler
                reg.clear_bits(0xF << 4);   // Clear AHB prescaler bits
                reg.set_bits((config.ahb_clk_divider as u32) << 4);
                
                reg
            })?;
        }

        // Configure system clock source
        if 0 != (config.clock_type & (ClockType::SystemClock as u32)) {
            // Check clock source readiness
            match config.sys_clk_source {
                SysClkSource::HSE => {
                    let mut timeout: i32 = 2000;
                    while timeout > 0 {
                        let cr_val: RegValue = self.rcc_reg.read_cr()?;
                        
                        if (cr_val.get() & (1 << 17)) != 0 {
                            break;
                        }
                        
                        timeout -= 1;
                    }
                    
                    if 0 == timeout {
                        return Err(RccError::Timeout);
                    }
                },
                SysClkSource::PllClk | SysClkSource::PllRClk => {
                    let mut timeout: i32 = 2000;
                    while timeout > 0 {
                        let cr_val: RegValue = self.rcc_reg.read_cr()?;
                        
                        if (cr_val.get() & (1 << 25)) != 0 {
                            break;
                        }
                        
                        timeout -= 1;
                    }
                    
                    if 0 == timeout {
                        return Err(RccError::Timeout);
                    }
                },
                SysClkSource::HSI => {
                    let mut timeout: i32 = 2000;
                    while timeout > 0 {
                        let cr_val: RegValue = self.rcc_reg.read_cr()?;
                        
                        if (cr_val.get() & (1 << 1)) != 0 {
                            break;
                        }
                        
                        timeout -= 1;
                    }
                    
                    if 0 == timeout {
                        return Err(RccError::Timeout);
                    }
                },
            }

            // Set system clock source
            self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                reg.clear_bits(0x3);  // Clear system clock source bits
                reg.set_bits(config.sys_clk_source as u32);
                reg
            })?;
            
            // Wait for system clock switch
            let sws_mask: u32 = config.sys_clk_source as u32;
            let mut timeout: i32 = 2000;
            while timeout > 0 {
                let cfgr_val: RegValue = self.rcc_reg.read_cfgr()?;
                
                if ((cfgr_val.get() >> 2) & 0x3) == sws_mask {
                    break;
                }
                
                timeout -= 1;
            }
            
            if 0 == timeout {
                return Err(RccError::Timeout);
            }
        }

        // Configure APB1 prescaler
        if (config.clock_type & (ClockType::PClock1 as u32)) != 0 {
            self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                reg.clear_bits(0x7 << 10);  // Clear APB1 prescaler bits
                reg.set_bits((config.apb1_clk_divider as u32) << 10);
                reg
            })?;
        }

        // Configure APB2 prescaler
        if (config.clock_type & (ClockType::PClock2 as u32)) != 0 {
            self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                reg.clear_bits(0x7 << 13);  // Clear APB2 prescaler bits
                reg.set_bits((config.apb2_clk_divider as u32) << 13);
                reg
            })?;
        }

        // Update system core clock
        unsafe {
            SYSTEM_CORE_CLOCK = self.get_sys_clock_freq();
        }

        Ok(())
    }

    pub fn get_sys_clock_freq(&self) -> u32 {
        let cfgr_val: u32 = match self.rcc_reg.read_cfgr() {
            Ok(val) => val.get(),
            Err(_) => return HSI_VALUE,
        };
        
        let clk_src: u32 = (cfgr_val >> 2) & 0x3;
        
        match clk_src {
            0 => HSI_VALUE,
            1 => HSE_VALUE,
            2 => self.get_pll_output_clock(),
            _ => HSI_VALUE,
        }
    }

    pub fn get_pll_output_clock(&self) -> u32 {
        let pllcfgr_val: u32 = match self.rcc_reg.read_pllcfgr() {
            Ok(val) => val.get(),
            Err(_) => return HSI_VALUE,
        };
        
        let pll_source: u32 = (pllcfgr_val >> 22) & 0x1;
        let pll_input_freq: u32 = if pll_source == 0 { HSI_VALUE } else { HSE_VALUE };
        
        let pllm: u32 = pllcfgr_val & 0x3F;
        let plln: u32 = (pllcfgr_val >> 6) & 0x1FF;
        let pllp_val: u32 = (((pllcfgr_val >> 16) & 0x3) * 2) + 2;
        
        (pll_input_freq / pllm) * plln / pllp_val
    }

    pub fn get_hclk_freq(&self) -> u32 {
        unsafe {
            SYSTEM_CORE_CLOCK
        }
    }

    pub fn get_pclk1_freq(&self) -> u32 {
        let cfgr_val: u32 = match self.rcc_reg.read_cfgr() {
            Ok(val) => val.get(),
            Err(_) => return 0,
        };
        
        let mut ahb_prescaler: u32 = 1;
        let mut apb1_prescaler: u32 = 1;
        
        let temp: u32 = (cfgr_val >> 4) & 0xF;
        if temp >= 8 {
            ahb_prescaler = AHB_PRESCALER[(temp - 8) as usize] as u32;
        }
        
        let temp: u32 = (cfgr_val >> 10) & 0x7;
        if temp >= 4 {
            apb1_prescaler = APB_PRESCALER[(temp - 4) as usize] as u32;
        }
        
        self.get_hclk_freq() / ahb_prescaler / apb1_prescaler
    }

    pub fn get_pclk2_freq(&self) -> u32 {
        let cfgr_val: u32 = match self.rcc_reg.read_cfgr() {
            Ok(val) => val.get(),
            Err(_) => return 0,
        };
        
        let mut ahb_prescaler: u32 = 1;
        let mut apb2_prescaler: u32 = 1;
        
        let temp: u32 = (cfgr_val >> 4) & 0xF;
        if temp >= 8 {
            ahb_prescaler = AHB_PRESCALER[(temp - 8) as usize] as u32;
        }
        
        let temp: u32 = (cfgr_val >> 13) & 0x7;
        if temp >= 4 {
            apb2_prescaler = APB_PRESCALER[(temp - 4) as usize] as u32;
        }
        
        self.get_hclk_freq() / ahb_prescaler / apb2_prescaler
    }

    pub fn enable_css(&self) -> Result<(), RccError> {
        self.rcc_reg.modify_cr(|mut reg: RegValue| {
            reg.set_bits(1 << 19);
            reg
        })
    }

    pub fn disable_css(&self) -> Result<(), RccError> {
        self.rcc_reg.modify_cr(|mut reg: RegValue| {
            reg.clear_bits(1 << 19);
            reg
        })
    }

    pub fn mco_config(&self, mco: McoIndex, source: u32, div: McoDiv) -> Result<(), RccError> {
        match mco {
            McoIndex::Mco1 => {
                self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                    reg.clear_bits(0x3 << 21);
                    reg.set_bits((source & 0x3) << 21);
                    
                    reg.clear_bits(0x7 << 24);
                    reg.set_bits((div as u32) << 24);
                    
                    reg
                })?;
            },
            McoIndex::Mco2 => {
                self.rcc_reg.modify_cfgr(|mut reg: RegValue| {
                    reg.clear_bits(0x3 << 30);
                    reg.set_bits((source & 0x3) << 30);
                    
                    reg.clear_bits(0x7 << 27);
                    reg.set_bits((div as u32) << 27);
                    
                    reg
                })?;
            },
        }
        
        Ok(())
    }
    
    // GPIO clock control
    pub fn gpio_clock_control(&self, gpio_base: u32, enable: bool) -> Result<(), RccError> {
        use crate::stm32f4xx::{GPIOA_BASE, GPIOB_BASE, GPIOC_BASE, GPIOD_BASE, 
                              GPIOE_BASE, GPIOF_BASE, GPIOG_BASE, GPIOH_BASE, GPIOI_BASE};
        
        let bit_position = match gpio_base {
            GPIOA_BASE => 0,
            GPIOB_BASE => 1,
            GPIOC_BASE => 2,
            GPIOD_BASE => 3,
            GPIOE_BASE => 4,
            GPIOF_BASE => 5,
            GPIOG_BASE => 6,
            GPIOH_BASE => 7,
            GPIOI_BASE => 8,
            _ => return Err(RccError::InvalidConfiguration),
        };
        
        self.rcc_reg.enable_peripheral_clock("AHB1", bit_position, enable)
    }
    
    // USART clock control
    pub fn usart_clock_control(&self, usart_base: u32, enable: bool) -> Result<(), RccError> {
        use crate::stm32f4xx::{USART1_BASE, USART2_BASE, USART3_BASE, UART4_BASE, UART5_BASE, USART6_BASE};
        
        match usart_base {
            USART1_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 4, enable),
            USART2_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 17, enable),
            USART3_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 18, enable),
            UART4_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 19, enable),
            UART5_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 20, enable),
            USART6_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 5, enable),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
    
    // USART reset control
    pub fn usart_reset(&self, usart_base: u32) -> Result<(), RccError> {
        use crate::stm32f4xx::{USART1_BASE, USART2_BASE, USART3_BASE, UART4_BASE, UART5_BASE, USART6_BASE};
        
        match usart_base {
            USART1_BASE => self.rcc_reg.reset_peripheral("APB2", 4),
            USART2_BASE => self.rcc_reg.reset_peripheral("APB1", 17),
            USART3_BASE => self.rcc_reg.reset_peripheral("APB1", 18),
            UART4_BASE => self.rcc_reg.reset_peripheral("APB1", 19),
            UART5_BASE => self.rcc_reg.reset_peripheral("APB1", 20),
            USART6_BASE => self.rcc_reg.reset_peripheral("APB2", 5),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
    
    // SPI clock control
    pub fn spi_clock_control(&self, spi_base: u32, enable: bool) -> Result<(), RccError> {
        use crate::stm32f4xx::{SPI1_BASE, SPI2_BASE, SPI3_BASE};
        
        match spi_base {
            SPI1_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 12, enable),
            SPI2_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 14, enable),
            SPI3_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 15, enable),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
    
    // I2C clock control
    pub fn i2c_clock_control(&self, i2c_base: u32, enable: bool) -> Result<(), RccError> {
        use crate::stm32f4xx::{I2C1_BASE, I2C2_BASE, I2C3_BASE};
        
        match i2c_base {
            I2C1_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 21, enable),
            I2C2_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 22, enable),
            I2C3_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 23, enable),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
    
    // Timer clock control
    pub fn timer_clock_control(&self, timer_base: u32, enable: bool) -> Result<(), RccError> {
        use crate::stm32f4xx::{TIM1_BASE, TIM2_BASE, TIM3_BASE, TIM4_BASE, TIM5_BASE, 
                              TIM6_BASE, TIM7_BASE, TIM8_BASE, TIM9_BASE, TIM10_BASE,
                              TIM11_BASE, TIM12_BASE, TIM13_BASE, TIM14_BASE};
        
        match timer_base {
            TIM1_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 0, enable),
            TIM2_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 0, enable),
            TIM3_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 1, enable),
            TIM4_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 2, enable),
            TIM5_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 3, enable),
            TIM6_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 4, enable),
            TIM7_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 5, enable),
            TIM8_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 1, enable),
            TIM9_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 16, enable),
            TIM10_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 17, enable),
            TIM11_BASE => self.rcc_reg.enable_peripheral_clock("APB2", 18, enable),
            TIM12_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 6, enable),
            TIM13_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 7, enable),
            TIM14_BASE => self.rcc_reg.enable_peripheral_clock("APB1", 8, enable),
            _ => Err(RccError::InvalidConfiguration),
        }
    }
}

pub fn init_default_clocks() -> Result<RccHandle<'static>, RccError> {
    let rcc: RccHandle<'_> = RccHandle::new()?;
    
    let osc_config: OscConfig = OscConfig {
        oscillator_type: OscillatorType::HSI,
        hsi_state: HsiState::On,
        ..OscConfig::default()
    };
    
    let clock_config: ClockConfig = ClockConfig::default();
    
    rcc.osc_config(&osc_config)?;
    rcc.clock_config(&clock_config)?;
    
    Ok(rcc)
}

pub fn init_max_performance_clocks() -> Result<RccHandle<'static>, RccError> {
    let rcc: RccHandle<'_> = RccHandle::new()?;
    
    let pll_config: PllConfig = PllConfig {
        state: PllState::On,
        source: PllSource::Hse,
        m: 4,
        n: 168,
        p: PllP::Div2,
        q: 4,
    };
    
    let osc_config: OscConfig = OscConfig {
        oscillator_type: OscillatorType::HSE,
        hse_state: HseState::On,
        hsi_state: HsiState::On,
        pll: pll_config,
        ..OscConfig::default()
    };
    
    let clock_config: ClockConfig = ClockConfig {
        clock_type: ClockType::SystemClock as u32 | ClockType::HClock as u32 | 
                   ClockType::PClock1 as u32 | ClockType::PClock2 as u32,
        sys_clk_source: SysClkSource::PllClk,
        ahb_clk_divider: AhbClkDivider::Div1,
        apb1_clk_divider: ApbClkDivider::Div4,
        apb2_clk_divider: ApbClkDivider::Div2,
    };
    
    rcc.osc_config(&osc_config)?;
    rcc.clock_config(&clock_config)?;
    
    Ok(rcc)
}