use core::marker::PhantomData;
use crate::stm32f4xx::{RCC, FLASH_R, RCCRegDef};

// Error type for GPIO operations
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
            m: 16,
            n: 336,
            p: PllP::Div4,
            q: 7,
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

pub struct RccHandle<'a> {
    _prcc: *mut RCCRegDef,
    _marker: PhantomData<&'a ()>,
}

impl<'a> RccHandle<'a> {
    pub fn new() -> Self {
        RccHandle {
            _prcc: RCC,
            _marker: PhantomData,
        }
    }

    pub fn osc_config(&self, config: &OscConfig) -> Result<(), RccError> {
        if 0 != (config.oscillator_type as u32 & OscillatorType::HSE as u32) {
            match config.hse_state {
                HseState::Bypass => unsafe {
                    (*RCC).CR &= !(1 << 16);
                    (*RCC).CR |= (1 << 18);
                    (*RCC).CR |= (1 << 16);
                },
                HseState::On => unsafe {
                    (*RCC).CR |= 1 << 16;
                },
                HseState::Off => unsafe {
                    (*RCC).CR &= !(1 << 16);
                },
            }
        }

        if (config.oscillator_type as u32 & OscillatorType::HSI as u32) != 0 {
            match config.hsi_state {
                HsiState::On => unsafe {
                    (*RCC).CR |= 1 << 0;
                },
                HsiState::Off => unsafe {
                    (*RCC).CR &= !(1 << 0);
                },
            }
        }

        if (config.oscillator_type as u32 & OscillatorType::LSE as u32) != 0 {
            match config.lse_state {
                LseState::On => unsafe {
                    (*RCC).BDCR |= 1 << 0;
                },
                LseState::Off => unsafe {
                    (*RCC).BDCR &= !(1 << 0);
                },
                LseState::Bypass => unsafe {
                    (*RCC).BDCR &= !(1 << 0);
                    (*RCC).BDCR |= 1 << 1;
                    (*RCC).BDCR |= 1 << 0;
                },
            }
        }

        if 0 != (config.oscillator_type as u32 & OscillatorType::LSI as u32) {
            match config.lsi_state {
                LsiState::On => unsafe {
                    (*RCC).CSR |= 1 << 0;
                },
                LsiState::Off => unsafe {
                    (*RCC).CSR &= !(1 << 0);
                },
            }
        }

        if config.pll.state != PllState::None {
            unsafe {
                (*RCC).CR &= !(1 << 24);
            }

            if config.pll.state == PllState::On {
                let pllcfgr_value: u32 = unsafe {
                    let mut val: u32 = (*RCC).PLLCFGR;

                    val &= !(0x3F << 0);
                    val &= !(0x1FF << 6);
                    val &= !(0x3 << 16);
                    val &= !(0xF << 24);
                    val &= !(1 << 22);

                    val |= (config.pll.m as u32) & 0x3F;
                    val |= ((config.pll.n as u32) & 0x1FF) << 6;
                    val |= ((config.pll.p as u32) & 0x3) << 16;
                    val |= ((config.pll.q as u32) & 0xF) << 24;
                    val |= ((config.pll.source as u32) & 0x1) << 22;
                    
                    val
                };
                
                unsafe {
                    (*RCC).PLLCFGR = pllcfgr_value;
                    (*RCC).CR |= 1 << 24;
                    
                    let mut timeout: i32 = 2000;
                    while (0 == ((*RCC).CR & (1 << 25))) && (timeout > 0) {
                        timeout -= 1;
                    }
                    
                    if timeout == 0 {
                        return Err(RccError::Timeout);
                    }
                }
            }
        }

        Ok(())
    }

    pub fn clock_config(&self, config: &ClockConfig) -> Result<(), RccError> {
        unsafe {
            (*FLASH_R).ACR = ((*FLASH_R).ACR & !0xF) | 0x5;
            
            let mut timeout: i32 = 2000;
            while (0x5 != ((*FLASH_R).ACR & 0xF)) && (timeout > 0) {
                timeout -= 1;
            }
            
            if 0 == timeout {
                return Err(RccError::Timeout);
            }
        }

        if 0 != (config.clock_type & (ClockType::HClock as u32)) {
            unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 10)) | (0x7 << 10);
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 13)) | (0x7 << 13);
                
                (*RCC).CFGR = ((*RCC).CFGR & !(0xF << 4)) | ((config.ahb_clk_divider as u32) << 4);
            }
        }

        if 0 != (config.clock_type & (ClockType::SystemClock as u32)) {
            let ready_bit: bool = match config.sys_clk_source {
                SysClkSource::HSE => {
                    unsafe {
                        let mut timeout: i32 = 2000;
                        while (((*RCC).CR & (1 << 17)) == 0) && (timeout > 0) {
                            timeout -= 1;
                        }
                        
                        if 0 == timeout {
                            return Err(RccError::Timeout);
                        }
                    }
                    true
                },
                SysClkSource::PllClk | SysClkSource::PllRClk => {
                    unsafe {
                        let mut timeout: i32 = 2000;
                        while (((*RCC).CR & (1 << 25)) == 0) && (timeout > 0) {
                            timeout -= 1;
                        }
                        
                        if timeout == 0 {
                            return Err(RccError::Timeout);
                        }
                    }
                    true
                },
                SysClkSource::HSI => {
                    unsafe {
                        let mut timeout: i32 = 2000;
                        while (((*RCC).CR & (1 << 1)) == 0) && (timeout > 0) {
                            timeout -= 1;
                        }
                        
                        if 0 == timeout {
                            return Err(RccError::Timeout);
                        }
                    }
                    true
                },
            };

            if !ready_bit {
                return Err(RccError::Timeout);
            }

            unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x3 << 0)) | ((config.sys_clk_source as u32) << 0);
                
                let sws_mask: u32 = config.sys_clk_source as u32;
                let mut timeout: i32 = 2000;
                while ((((*RCC).CFGR >> 2) & 0x3) != sws_mask) && (timeout > 0) {
                    timeout -= 1;
                }
                
                if timeout == 0 {
                    return Err(RccError::Timeout);
                }
            }
        }

        if (config.clock_type & (ClockType::PClock1 as u32)) != 0 {
            unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 10)) | ((config.apb1_clk_divider as u32) << 10);
            }
        }

        if 0 != (config.clock_type & (ClockType::PClock2 as u32)) {
            unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 13)) | ((config.apb2_clk_divider as u32) << 13);
            }
        }

        unsafe {
            SYSTEM_CORE_CLOCK = self.get_sys_clock_freq();
        }

        Ok(())
    }

    pub fn get_sys_clock_freq(&self) -> u32 {
        let sys_clk_freq: u32;
        let clk_src: u32;
        
        unsafe {
            clk_src = ((*RCC).CFGR >> 2) & 0x3;
            
            match clk_src {
                0 => sys_clk_freq = HSI_VALUE,
                1 => sys_clk_freq = HSE_VALUE,
                2 => sys_clk_freq = self.get_pll_output_clock(),
                _ => sys_clk_freq = HSI_VALUE,
            }
        }
        
        sys_clk_freq
    }

    pub fn get_pll_output_clock(&self) -> u32 {
        let mut pll_source: u32;
        let mut pll_input_freq: u32;
        let mut pll_output_freq: u32;
        
        unsafe {
            pll_source = ((*RCC).PLLCFGR >> 22) & 0x1;
            
            if 0 == pll_source {
                pll_input_freq = HSI_VALUE;
            } else {
                pll_input_freq = HSE_VALUE;
            }
            
            let pllm: u32 = ((*RCC).PLLCFGR & 0x3F) as u32;
            let plln: u32 = (((*RCC).PLLCFGR >> 6) & 0x1FF) as u32;
            let pllp_val: u32 = ((((*RCC).PLLCFGR >> 16) & 0x3) as u32) * 2 + 2;
            
            pll_output_freq = (pll_input_freq / pllm) * plln / pllp_val;
        }
        
        pll_output_freq
    }

    pub fn get_hclk_freq(&self) -> u32 {
        unsafe {
            SYSTEM_CORE_CLOCK
        }
    }

    pub fn get_pclk1_freq(&self) -> u32 {
        let mut pclk1: u32;
        let mut ahb_prescaler: u32 = 1;
        let mut apb1_prescaler: u32 = 1;
        
        unsafe {
            let temp: u32 = ((*RCC).CFGR >> 4) & 0xF;
            if temp >= 8 {
                ahb_prescaler = AHB_PRESCALER[(temp - 8) as usize] as u32;
            }
            
            let temp: u32 = ((*RCC).CFGR >> 10) & 0x7;
            if temp >= 4 {
                apb1_prescaler = APB_PRESCALER[(temp - 4) as usize] as u32;
            }
            
            pclk1 = SYSTEM_CORE_CLOCK / ahb_prescaler / apb1_prescaler;
        }
        
        pclk1
    }

    pub fn get_pclk2_freq(&self) -> u32 {
        let mut pclk2: u32;
        let mut ahb_prescaler: u32 = 1;
        let mut apb2_prescaler: u32 = 1;
        
        unsafe {
            let temp: u32 = ((*RCC).CFGR >> 4) & 0xF;
            if temp >= 8 {
                ahb_prescaler = AHB_PRESCALER[(temp - 8) as usize] as u32;
            }
            
            let temp: u32 = ((*RCC).CFGR >> 13) & 0x7;
            if temp >= 4 {
                apb2_prescaler = APB_PRESCALER[(temp - 4) as usize] as u32;
            }
            
            pclk2 = SYSTEM_CORE_CLOCK / ahb_prescaler / apb2_prescaler;
        }
        
        pclk2
    }

    pub fn enable_css(&self) {
        unsafe {
            (*RCC).CR |= 1 << 19;
        }
    }

    pub fn disable_css(&self) {
        unsafe {
            (*RCC).CR &= !(1 << 19);
        }
    }

    pub fn mco_config(&self, mco: McoIndex, source: u32, div: McoDiv) -> Result<(), RccError> {
        match mco {
            McoIndex::Mco1 => unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x3 << 21)) | ((source & 0x3) << 21);
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 24)) | ((div as u32) << 24);
            },
            McoIndex::Mco2 => unsafe {
                (*RCC).CFGR = ((*RCC).CFGR & !(0x3 << 30)) | ((source & 0x3) << 30);
                (*RCC).CFGR = ((*RCC).CFGR & !(0x7 << 27)) | ((div as u32) << 27);
            },
        }
        
        Ok(())
    }
}

pub fn init_default_clocks() -> Result<RccHandle<'static>, RccError> {
    let rcc: RccHandle<'_> = RccHandle::new();
    
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
    let rcc: RccHandle<'_> = RccHandle::new();
    
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