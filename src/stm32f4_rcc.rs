use core::marker::PhantomData;
use crate::stm32f4xx::{RCC, FLASH_R, RCCRegDef, FLASHRegDef};

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

// Helper struct for register value operations
#[derive(Copy, Clone)]
pub struct RegValue(u32);

impl RegValue {
    pub fn new(value: u32) -> Self {
        RegValue(value)
    }
    
    pub fn get(&self) -> u32 {
        self.0
    }
    
    pub fn set_bits(&mut self, mask: u32) {
        self.0 |= mask;
    }
    
    pub fn clear_bits(&mut self, mask: u32) {
        self.0 &= !mask;
    }
    
    pub fn modify<F>(&mut self, f: F) where F: FnOnce(u32) -> u32 {
        self.0 = f(self.0);
    }
}

// RCC register access
pub struct RccRegister {
    register: *mut RCCRegDef,
}

impl RccRegister {
    pub fn new() -> Result<Self, RccError> {
        if RCC.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        Ok(RccRegister { register: RCC })
    }
    
    // Read CR register
    pub fn read_cr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CR };
        Ok(RegValue::new(value))
    }
    
    // Write CR register
    pub fn write_cr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CR = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR register
    pub fn modify_cr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr()?;
        let new_value: RegValue = f(value);
        self.write_cr(new_value)
    }
    
    // Read CFGR register
    pub fn read_cfgr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CFGR };
        Ok(RegValue::new(value))
    }
    
    // Write CFGR register
    pub fn write_cfgr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CFGR = value.get();
        }
        
        Ok(())
    }
    
    // Modify CFGR register
    pub fn modify_cfgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cfgr()?;
        let new_value: RegValue = f(value);
        self.write_cfgr(new_value)
    }
    
    // Read PLLCFGR register
    pub fn read_pllcfgr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).PLLCFGR };
        Ok(RegValue::new(value))
    }
    
    // Write PLLCFGR register
    pub fn write_pllcfgr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).PLLCFGR = value.get();
        }
        
        Ok(())
    }
    
    // Modify PLLCFGR register
    pub fn modify_pllcfgr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_pllcfgr()?;
        let new_value: RegValue = f(value);
        self.write_pllcfgr(new_value)
    }
    
    // Read BDCR register
    pub fn read_bdcr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).BDCR };
        Ok(RegValue::new(value))
    }
    
    // Write BDCR register
    pub fn write_bdcr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).BDCR = value.get();
        }
        
        Ok(())
    }
    
    // Modify BDCR register
    pub fn modify_bdcr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_bdcr()?;
        let new_value: RegValue = f(value);
        self.write_bdcr(new_value)
    }
    
    // Read CSR register
    pub fn read_csr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CSR };
        Ok(RegValue::new(value))
    }
    
    // Write CSR register
    pub fn write_csr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CSR = value.get();
        }
        
        Ok(())
    }
    
    // Modify CSR register
    pub fn modify_csr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_csr()?;
        let new_value: RegValue = f(value);
        self.write_csr(new_value)
    }
    
    // Read AHB1ENR register
    pub fn read_ahb1enr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).AHB1ENR };
        Ok(RegValue::new(value))
    }
    
    // Write AHB1ENR register
    pub fn write_ahb1enr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).AHB1ENR = value.get();
        }
        
        Ok(())
    }
    
    // Modify AHB1ENR register
    pub fn modify_ahb1enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_ahb1enr()?;
        let new_value: RegValue = f(value);
        self.write_ahb1enr(new_value)
    }
    
    // Read APB1ENR register
    pub fn read_apb1enr(&self) -> Result<RegValue, RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).APB1ENR };
        Ok(RegValue::new(value))
    }
    
    // Write APB1ENR register
    pub fn write_apb1enr(&self, value: RegValue) -> Result<(), RccError> {
        if self.register.is_null() {
            return Err(RccError::HardwareFault);
        }
        
        unsafe {
            (*self.register).APB1ENR = value.get();
        }
        
        Ok(())
    }
    
    // Modify APB1ENR register
    pub fn modify_apb1enr<F>(&self, f: F) -> Result<(), RccError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_apb1enr()?;
        let new_value: RegValue = f(value);
        self.write_apb1enr(new_value)
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
                    reg.set_bits(1 << 24);  // Turn on PLL
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
                
                if timeout == 0 {
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
            
            if (acr_val.get() & 0xF) == 0x5 {
                break;
            }
            
            timeout -= 1;
        }
        
        if timeout == 0 {
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
        if (config.clock_type & (ClockType::SystemClock as u32)) != 0 {
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
                    
                    if timeout == 0 {
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
                    
                    if timeout == 0 {
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
                    
                    if timeout == 0 {
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
            
            if timeout == 0 {
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
    
    // Added helper methods
    pub fn enable_tim6_clock(&self) -> Result<(), RccError> {
        self.rcc_reg.modify_apb1enr(|mut reg: RegValue| {
            reg.set_bits(1 << 4);
            reg
        })
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