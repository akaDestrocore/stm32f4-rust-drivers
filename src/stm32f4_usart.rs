use core::marker::PhantomData;
use crate::stm32f4xx::{
    USART1, USART2, USART3, UART4, UART5, USART6, 
    USARTRegDef, RCC
};
use crate::stm32f4_rcc::{RegValue};

// Error type for USART operations
#[derive(Debug, Clone, Copy)]
pub enum UsartError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
    InvalidPeripheral,
}

// USART Mode enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartMode {
    TxOnly = 0,
    RxOnly = 1,
    TxRx   = 2,
}

// USART Baud Rate enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartBaud {
    Baud1200   = 1200,
    Baud2400   = 2400,
    Baud9600   = 9600,
    Baud19200  = 19200,
    Baud38400  = 38400,
    Baud57600  = 57600,
    Baud115200 = 115200,
    Baud230400 = 230400,
    Baud460800 = 460800,
    Baud921600 = 921600,
    Baud2M     = 2000000,
    Baud3M     = 3000000,
}

// USART Parity enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartParity {
    Disable  = 0,
    EvenParity = 1,
    OddParity  = 2,
}

// USART Word Length enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartWordLength {
    Bits8 = 0,
    Bits9 = 1,
}

// USART Stop Bits enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartStopBits {
    Bits1   = 0,
    Bits0_5 = 1,
    Bits2   = 2,
    Bits1_5 = 3,
}

// USART Hardware Flow Control enum
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartHwFlowControl {
    None   = 0,
    CTS    = 1,
    RTS    = 2,
    CtsRts = 3,
}

// USART Status Register Flags
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartStatusFlag {
    PE   = 0,  // Parity Error
    FE   = 1,  // Framing Error
    NF   = 2,  // Noise Flag
    ORE  = 3,  // Overrun Error
    IDLE = 4,  // IDLE Line Detected
    RXNE = 5,  // Read Data Register Not Empty
    TC   = 6,  // Transmission Complete
    TXE  = 7,  // Transmit Data Register Empty
    LBD  = 8,  // LIN Break Detection Flag
    CTS  = 9,  // CTS Flag
}

// USART Application State
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartState {
    Ready    = 0,
    BusyInRx = 1,
    BusyInTx = 2,
}

// USART Application Events
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartEvent {
    TxComplete  = 0,
    RxComplete  = 1,
    Idle        = 2,
    Cts         = 3,
    ParityError = 4,
    FramingError = 5,
    NoiseError   = 6,
    OverrunError = 7,
}

// USART Configuration
#[derive(Debug, Clone, Copy)]
pub struct UsartConfig {
    pub mode: UsartMode,
    pub baud: UsartBaud,
    pub stop_bits: UsartStopBits,
    pub word_length: UsartWordLength,
    pub parity: UsartParity,
    pub hw_flow_control: UsartHwFlowControl,
}

impl Default for UsartConfig {
    fn default() -> Self {
        UsartConfig {
            mode: UsartMode::TxRx,
            baud: UsartBaud::Baud115200,
            stop_bits: UsartStopBits::Bits1,
            word_length: UsartWordLength::Bits8,
            parity: UsartParity::Disable,
            hw_flow_control: UsartHwFlowControl::None,
        }
    }
}

// USART Register wrapper
struct UsartRegister {
    register: *mut USARTRegDef,
}

impl UsartRegister {
    fn new(pusartx: *mut USARTRegDef) -> Result<Self, UsartError> {
        if pusartx.is_null() {
            return Err(UsartError::InvalidPeripheral);
        }
        
        Ok(UsartRegister { register: pusartx })
    }
    
    // Read SR register
    fn read_sr(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).SR };
        Ok(RegValue::new(value))
    }
    
    // Write SR register
    fn write_sr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).SR = value.get();
        }
        
        Ok(())
    }
    
    // Modify SR register
    fn modify_sr<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_sr()?;
        let new_value: RegValue = f(value);
        self.write_sr(new_value)
    }
    
    // Read CR1 register
    fn read_cr1(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CR1 };
        Ok(RegValue::new(value))
    }
    
    // Write CR1 register
    fn write_cr1(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CR1 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR1 register
    fn modify_cr1<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr1()?;
        let new_value: RegValue = f(value);
        self.write_cr1(new_value)
    }
    
    // Read CR2 register
    fn read_cr2(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CR2 };
        Ok(RegValue::new(value))
    }
    
    // Write CR2 register
    fn write_cr2(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CR2 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR2 register
    fn modify_cr2<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr2()?;
        let new_value: RegValue = f(value);
        self.write_cr2(new_value)
    }
    
    // Read CR3 register
    fn read_cr3(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).CR3 };
        Ok(RegValue::new(value))
    }
    
    // Write CR3 register
    fn write_cr3(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).CR3 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR3 register
    fn modify_cr3<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr3()?;
        let new_value: RegValue = f(value);
        self.write_cr3(new_value)
    }
    
    // Read BRR register
    fn read_brr(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).BRR };
        Ok(RegValue::new(value))
    }
    
    // Write BRR register
    fn write_brr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).BRR = value.get();
        }
        
        Ok(())
    }
    
    // Modify BRR register
    fn modify_brr<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_brr()?;
        let new_value: RegValue = f(value);
        self.write_brr(new_value)
    }
    
    // Read DR register
    fn read_dr(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.register).DR };
        Ok(RegValue::new(value))
    }
    
    // Write DR register
    fn write_dr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.register).DR = value.get();
        }
        
        Ok(())
    }
    
    // Get raw register pointer for operations that need direct access
    fn get_raw_ptr(&self) -> *mut USARTRegDef {
        self.register
    }
}

// USART Handle struct
pub struct UsartHandle<'a> {
    pub pusartx: *mut USARTRegDef,
    pub config: UsartConfig,
    register: UsartRegister,
    tx_buffer: Option<&'a [u8]>,
    rx_buffer: Option<&'a mut [u8]>,
    tx_len: usize,
    rx_len: usize,
    tx_state: UsartState,
    rx_state: UsartState,
    _marker: PhantomData<&'a ()>,
}

impl<'a> UsartHandle<'a> {
    pub fn new(pusartx: *mut USARTRegDef) -> Result<Self, UsartError> {
        let register: UsartRegister = UsartRegister::new(pusartx)?;
        
        Ok(UsartHandle {
            pusartx,
            config: UsartConfig::default(),
            register,
            tx_buffer: None,
            rx_buffer: None,
            tx_len: 0,
            rx_len: 0,
            tx_state: UsartState::Ready,
            rx_state: UsartState::Ready,
            _marker: PhantomData,
        })
    }
    
    pub fn config_usart(
        &mut self,
        mode: UsartMode,
        baud: UsartBaud,
        stop_bits: UsartStopBits,
        word_length: UsartWordLength,
        parity: UsartParity,
        hw_flow_control: UsartHwFlowControl,
    ) -> Result<(), UsartError> {
        self.config.mode = mode;
        self.config.baud = baud;
        self.config.stop_bits = stop_bits;
        self.config.word_length = word_length;
        self.config.parity = parity;
        self.config.hw_flow_control = hw_flow_control;
        
        Ok(())
    }
    
    pub fn init(&self) -> Result<(), UsartError> {
        // Enable USART clock
        Usart::periph_clock_control(self.pusartx, true)?;
        
        // Configure CR1 register
        self.register.modify_cr1(|mut reg: RegValue| {
            // Configure USART mode (TX, RX, or both)
            match self.config.mode {
                UsartMode::RxOnly => {
                    reg.set_bits(1 << 2);     // RE bit
                },
                UsartMode::TxOnly => {
                    reg.set_bits(1 << 3);     // TE bit
                },
                UsartMode::TxRx => {
                    reg.set_bits(1 << 2);     // RE bit
                    reg.set_bits(1 << 3);     // TE bit
                },
            }
            
            // Configure word length
            if self.config.word_length == UsartWordLength::Bits9 {
                reg.set_bits(1 << 12);     // M bit
            } else {
                reg.clear_bits(1 << 12);   // 8 bits
            }
            
            // Configure parity
            match self.config.parity {
                UsartParity::EvenParity => {
                    reg.set_bits(1 << 10);    // PCE bit
                    reg.clear_bits(1 << 9);   // PS bit (even)
                },
                UsartParity::OddParity => {
                    reg.set_bits(1 << 10);    // PCE bit
                    reg.set_bits(1 << 9);     // PS bit (odd)
                },
                UsartParity::Disable => {
                    reg.clear_bits(1 << 10);  // PCE bit
                },
            }
            
            reg
        })?;
        
        // Configure CR2 register for stop bits
        self.register.modify_cr2(|mut reg: RegValue| {
            reg.clear_bits(0x3 << 12);        // Clear STOP bits
            reg.set_bits((self.config.stop_bits as u32) << 12);  // Set new STOP bits
            reg
        })?;
        
        // Configure CR3 register for hardware flow control
        self.register.modify_cr3(|mut reg: RegValue| {
            match self.config.hw_flow_control {
                UsartHwFlowControl::CTS => {
                    reg.set_bits(1 << 9);     // CTSE bit
                },
                UsartHwFlowControl::RTS => {
                    reg.set_bits(1 << 8);     // RTSE bit
                },
                UsartHwFlowControl::CtsRts => {
                    reg.set_bits(1 << 9);     // CTSE bit
                    reg.set_bits(1 << 8);     // RTSE bit
                },
                UsartHwFlowControl::None => {
                    reg.clear_bits(1 << 9);   // CTSE bit
                    reg.clear_bits(1 << 8);   // RTSE bit
                },
            }
            reg
        })?;
        
        // Set baud rate
        self.set_baud_rate(self.config.baud)?;
        
        // Enable USART
        self.register.modify_cr1(|mut reg: RegValue| {
            reg.set_bits(1 << 13);    // UE bit
            reg
        })?;
        
        Ok(())
    }
    
    pub fn deinit(&self) -> Result<(), UsartError> {
        // Disable USART
        self.register.modify_cr1(|mut reg: RegValue| {
            reg.clear_bits(1 << 13);    // UE bit
            reg
        })?;
        
        // Reset USART peripheral using RCC
        struct RccRegister;
        
        impl RccRegister {
            fn modify_apb1rstr<F>(f: F) -> Result<(), UsartError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(UsartError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).APB1RSTR;
                    (*RCC).APB1RSTR = f(current);
                }
                Ok(())
            }
            
            fn modify_apb2rstr<F>(f: F) -> Result<(), UsartError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(UsartError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).APB2RSTR;
                    (*RCC).APB2RSTR = f(current);
                }
                Ok(())
            }
        }
        
        if self.pusartx == USART1 {
            RccRegister::modify_apb2rstr(|reg: u32| reg | (1 << 4))?;
            RccRegister::modify_apb2rstr(|reg: u32| reg & !(1 << 4))?;
        } else if self.pusartx == USART2 {
            RccRegister::modify_apb1rstr(|reg: u32| reg | (1 << 17))?;
            RccRegister::modify_apb1rstr(|reg: u32| reg & !(1 << 17))?;
        } else if self.pusartx == USART3 {
            RccRegister::modify_apb1rstr(|reg: u32| reg | (1 << 18))?;
            RccRegister::modify_apb1rstr(|reg: u32| reg & !(1 << 18))?;
        } else if self.pusartx == UART4 {
            RccRegister::modify_apb1rstr(|reg: u32| reg | (1 << 19))?;
            RccRegister::modify_apb1rstr(|reg: u32| reg & !(1 << 19))?;
        } else if self.pusartx == UART5 {
            RccRegister::modify_apb1rstr(|reg: u32| reg | (1 << 20))?;
            RccRegister::modify_apb1rstr(|reg: u32| reg & !(1 << 20))?;
        } else if self.pusartx == USART6 {
            RccRegister::modify_apb2rstr(|reg: u32| reg | (1 << 5))?;
            RccRegister::modify_apb2rstr(|reg: u32| reg & !(1 << 5))?;
        } else {
            return Err(UsartError::InvalidPeripheral);
        }
        
        Ok(())
    }
    
    pub fn set_baud_rate(&self, baud: UsartBaud) -> Result<(), UsartError> {
        use crate::stm32f4_rcc::{RccHandle};
        
        let rcc_handle = RccHandle::new()?;
        
        // Get the appropriate peripheral clock
        let pclk: u32 = if self.pusartx == USART1 || self.pusartx == USART6 {
            rcc_handle.get_pclk2_freq()  // USART1 and USART6 are on APB2
        } else {
            rcc_handle.get_pclk1_freq()  // Other USARTs are on APB1
        };
        
        // Calculate USARTDIV
        let mut usartdiv: u32 = 0;
        let over8: bool = unsafe { ((*self.pusartx).CR1 & (1 << 15)) != 0 };
        
        if over8 {
            // Oversampling by 8
            usartdiv = ((25 * pclk) / (2 * baud as u32));
        } else {
            // Oversampling by 16
            usartdiv = ((25 * pclk) / (4 * baud as u32));
        }
        
        // Calculate mantissa and fraction parts
        let mantissa: u32 = usartdiv / 100;
        let fraction: u32 = usartdiv - (mantissa * 100);
        let final_fraction: u32 = if over8 {
            ((fraction * 8) + 50) / 100 & 0x07
        } else {
            ((fraction * 16) + 50) / 100 & 0x0F
        };
        
        // Write to BRR register
        let brr_value: u32 = (mantissa << 4) | final_fraction;
        self.register.write_brr(RegValue::new(brr_value))?;
        
        Ok(())
    }
    
    pub fn get_flag_status(&self, flag: UsartStatusFlag) -> Result<bool, UsartError> {
        let sr_value: RegValue = self.register.read_sr()?;
        
        let bit_position: u32 = match flag {
            UsartStatusFlag::PE   => 0,
            UsartStatusFlag::FE   => 1,
            UsartStatusFlag::NF   => 2,
            UsartStatusFlag::ORE  => 3,
            UsartStatusFlag::IDLE => 4,
            UsartStatusFlag::RXNE => 5,
            UsartStatusFlag::TC   => 6,
            UsartStatusFlag::TXE  => 7,
            UsartStatusFlag::LBD  => 8,
            UsartStatusFlag::CTS  => 9,
        };
        
        Ok((sr_value.get() & (1 << bit_position)) != 0)
    }
    
    pub fn clear_flag(&self, flag: UsartStatusFlag) -> Result<(), UsartError> {
        let bit_position: u32 = match flag {
            UsartStatusFlag::PE   => 0,
            UsartStatusFlag::FE   => 1,
            UsartStatusFlag::NF   => 2,
            UsartStatusFlag::ORE  => 3,
            UsartStatusFlag::IDLE => 4,
            UsartStatusFlag::RXNE => 5,
            UsartStatusFlag::TC   => 6,
            UsartStatusFlag::TXE  => 7,
            UsartStatusFlag::LBD  => 8,
            UsartStatusFlag::CTS  => 9,
        };
        
        self.register.modify_sr(|mut reg: RegValue| {
            reg.clear_bits(1 << bit_position);
            reg
        })?;
        
        Ok(())
    }
    
    pub fn send_data(&self, tx_buffer: &[u8]) -> Result<(), UsartError> {
        for &byte in tx_buffer {
            // Wait until TXE flag is set
            while !self.get_flag_status(UsartStatusFlag::TXE)? {
                // Wait
            }
            
            // Handle 9-bit data if needed
            if self.config.word_length == UsartWordLength::Bits9 {
                if self.config.parity == UsartParity::Disable {
                    // 9 bits of data, no parity
                    // Need to handle 9-bit data here, but in Rust we're dealing with bytes
                    // This would need a more complex implementation for 9-bit data
                    self.register.write_dr(RegValue::new(byte as u32))?;
                } else {
                    // 8 bits of data + 1 parity bit (handled by hardware)
                    self.register.write_dr(RegValue::new(byte as u32))?;
                }
            } else {
                // 8-bit data
                self.register.write_dr(RegValue::new(byte as u32))?;
            }
        }
        
        // Wait until TC flag is set before returning
        while !self.get_flag_status(UsartStatusFlag::TC)? {
            // Wait
        }
        
        Ok(())
    }
    
    pub fn receive_data(&self, rx_buffer: &mut [u8], len: usize) -> Result<(), UsartError> {
        for i in 0..len.min(rx_buffer.len()) {
            // Wait until RXNE flag is set
            while !self.get_flag_status(UsartStatusFlag::RXNE)? {
                // Wait
            }
            
            if self.config.word_length == UsartWordLength::Bits9 {
                if self.config.parity == UsartParity::Disable {
                    // 9 bits of data, no parity
                    // This is a simplification - 9-bit data would need more complex handling
                    let dr_value = self.register.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                } else {
                    // 8 bits of data + 1 parity bit (handled by hardware)
                    let dr_value = self.register.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                }
            } else {
                // 8-bit data
                if self.config.parity == UsartParity::Disable {
                    // 8 bits of data
                    let dr_value = self.register.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                } else {
                    // 7 bits of data + 1 parity bit
                    let dr_value = self.register.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0x7F) as u8;
                }
            }
        }
        
        Ok(())
    }
    
    pub fn send_data_it(&mut self, tx_buffer: &'a [u8]) -> Result<UsartState, UsartError> {
        let tx_state = self.tx_state;
        
        if tx_state != UsartState::BusyInTx {
            self.tx_buffer = Some(tx_buffer);
            self.tx_len = tx_buffer.len();
            self.tx_state = UsartState::BusyInTx;
            
            // Enable TXE interrupt
            self.register.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 7);  // TXEIE bit
                reg
            })?;
            
            // Enable TC interrupt
            self.register.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 6);  // TCIE bit
                reg
            })?;
        }
        
        Ok(tx_state)
    }
    
    pub fn receive_data_it(&mut self, rx_buffer: &'a mut [u8], len: usize) -> Result<UsartState, UsartError> {
        let rx_state = self.rx_state;
        
        if rx_state != UsartState::BusyInRx {
            self.rx_buffer = Some(rx_buffer);
            self.rx_len = len.min(rx_buffer.len());
            self.rx_state = UsartState::BusyInRx;
            
            // Dummy read of DR to clear any pending RXNE flag
            let _ = self.register.read_dr()?;
            
            // Enable RXNE interrupt
            self.register.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 5);  // RXNEIE bit
                reg
            })?;
        }
        
        Ok(rx_state)
    }
    
    pub fn irq_handler(&mut self) -> Result<(), UsartError> {
        // Check for TC flag
        let tc_flag: bool = self.get_flag_status(UsartStatusFlag::TC)?;
        let tcie_flag: bool = unsafe { ((*self.pusartx).CR1 & (1 << 6)) != 0 };
        
        if tc_flag && tcie_flag {
            // TC interrupt handling
            if self.tx_state == UsartState::BusyInTx && self.tx_len == 0 {
                // Clear TC flag
                self.clear_flag(UsartStatusFlag::TC)?;
                
                // Disable TCIE
                self.register.modify_cr1(|mut reg: RegValue| {
                    reg.clear_bits(1 << 6);  // TCIE bit
                    reg
                })?;
                
                // Reset TX state
                self.tx_state = UsartState::Ready;
                self.tx_buffer = None;
                
                // Call application callback if needed
                Usart::application_event_callback(self.pusartx, UsartEvent::TxComplete);
            }
        }
        
        // Check for TXE flag
        let txe_flag: bool = self.get_flag_status(UsartStatusFlag::TXE)?;
        let txeie_flag: bool = unsafe { ((*self.pusartx).CR1 & (1 << 7)) != 0 };
        
        if txe_flag && txeie_flag {
            // TXE interrupt handling
            if self.tx_state == UsartState::BusyInTx && self.tx_len > 0 {
                if let Some(buffer) = self.tx_buffer {
                    if self.tx_len > 0 && !buffer.is_empty() {
                        let index = buffer.len() - self.tx_len;
                        
                        // Send data byte
                        self.register.write_dr(RegValue::new(buffer[index] as u32))?;
                        
                        // Decrement TX length
                        self.tx_len -= 1;
                    }
                }
                
                if self.tx_len == 0 {
                    // Disable TXEIE
                    self.register.modify_cr1(|mut reg: RegValue| {
                        reg.clear_bits(1 << 7);  // TXEIE bit
                        reg
                    })?;
                }
            }
        }
        
        // Check for RXNE flag
        let rxne_flag: bool = self.get_flag_status(UsartStatusFlag::RXNE)?;
        let rxneie_flag: bool = unsafe { ((*self.pusartx).CR1 & (1 << 5)) != 0 };
        
        if rxne_flag && rxneie_flag {
            // RXNE interrupt handling
            if self.rx_state == UsartState::BusyInRx && self.rx_len > 0 {
                if let Some(buffer) = &mut self.rx_buffer {
                    let index = buffer.len() - self.rx_len;
                    
                    if self.config.word_length == UsartWordLength::Bits9 {
                        if self.config.parity == UsartParity::Disable {
                            // 9 bits, no parity
                            let dr_value = self.register.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        } else {
                            // 8 bits + parity
                            let dr_value = self.register.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        }
                    } else {
                        // 8-bit word length
                        if self.config.parity == UsartParity::Disable {
                            // 8 bits, no parity
                            let dr_value = self.register.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        } else {
                            // 7 bits + parity
                            let dr_value = self.register.read_dr()?.get();
                            buffer[index] = (dr_value & 0x7F) as u8;
                        }
                    }
                    
                    // Decrement RX length
                    self.rx_len -= 1;
                    
                    if self.rx_len == 0 {
                        // Disable RXNEIE
                        self.register.modify_cr1(|mut reg: RegValue| {
                            reg.clear_bits(1 << 5);  // RXNEIE bit
                            reg
                        })?;
                        
                        // Reset RX state
                        self.rx_state = UsartState::Ready;
                        
                        // Call application callback if needed
                        Usart::application_event_callback(self.pusartx, UsartEvent::RxComplete);
                    }
                }
            }
        }
        
        // Check for CTS flag (if HW flow control is enabled)
        let cts_flag: bool = self.get_flag_status(UsartStatusFlag::CTS)?;
        let ctse_flag: bool = unsafe { ((*self.pusartx).CR3 & (1 << 9)) != 0 };
        let ctsie_flag: bool = unsafe { ((*self.pusartx).CR3 & (1 << 10)) != 0 };
        
        if cts_flag && ctse_flag && ctsie_flag {
            // Clear CTS flag
            self.clear_flag(UsartStatusFlag::CTS)?;
            
            // Call application callback if needed
            Usart::application_event_callback(self.pusartx, UsartEvent::Cts);
        }
        
        // Check for IDLE flag
        let idle_flag: bool = self.get_flag_status(UsartStatusFlag::IDLE)?;
        let idleie_flag: bool = unsafe { ((*self.pusartx).CR1 & (1 << 4)) != 0 };
        
        if idle_flag && idleie_flag {
            // Clear IDLE flag - requires reading SR then DR
            let _ = self.register.read_sr()?;
            let _ = self.register.read_dr()?;
            
            // Call application callback if needed
            Usart::application_event_callback(self.pusartx, UsartEvent::Idle);
        }
        
        // Check for ORE flag
        let ore_flag: bool = self.get_flag_status(UsartStatusFlag::ORE)?;
        
        if ore_flag && rxneie_flag {
            // Overrun error handling
            // Call application callback if needed
            Usart::application_event_callback(self.pusartx, UsartEvent::OverrunError);
        }
        
        Ok(())
    }
}

pub struct Usart;

impl Usart {
    pub fn periph_clock_control(pusartx: *mut USARTRegDef, state: bool) -> Result<(), UsartError> {
        struct RccRegister;
        
        impl RccRegister {
            fn modify_apb1enr<F>(f: F) -> Result<(), UsartError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(UsartError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).APB1ENR;
                    (*RCC).APB1ENR = f(current);
                }
                Ok(())
            }
            
            fn modify_apb2enr<F>(f: F) -> Result<(), UsartError> 
            where F: FnOnce(u32) -> u32 {
                unsafe {
                    if RCC.is_null() {
                        return Err(UsartError::HardwareFault);
                    }
                    
                    let current: u32 = (*RCC).APB2ENR;
                    (*RCC).APB2ENR = f(current);
                }
                Ok(())
            }
        }
        
        let state_bit: u32 = if state { 1 } else { 0 };
        
        if pusartx == USART1 {
            RccRegister::modify_apb2enr(|reg: u32| {
                if state {
                    reg | (1 << 4)
                } else {
                    reg & !(1 << 4)
                }
            })?;
        } else if pusartx == USART2 {
            RccRegister::modify_apb1enr(|reg: u32| {
                if state {
                    reg | (1 << 17)
                } else {
                    reg & !(1 << 17)
                }
            })?;
        } else if pusartx == USART3 {
            RccRegister::modify_apb1enr(|reg: u32| {
                if state {
                    reg | (1 << 18)
                } else {
                    reg & !(1 << 18)
                }
            })?;
        } else if pusartx == UART4 {
            RccRegister::modify_apb1enr(|reg: u32| {
                if state {
                    reg | (1 << 19)
                } else {
                    reg & !(1 << 19)
                }
            })?;
        } else if pusartx == UART5 {
            RccRegister::modify_apb1enr(|reg: u32| {
                if state {
                    reg | (1 << 20)
                } else {
                    reg & !(1 << 20)
                }
            })?;
        } else if pusartx == USART6 {
            RccRegister::modify_apb2enr(|reg: u32| {
                if state {
                    reg | (1 << 5)
                } else {
                    reg & !(1 << 5)
                }
            })?;
        } else {
            return Err(UsartError::InvalidPeripheral);
        }
        
        Ok(())
    }

    pub fn application_event_callback(_pusartx: *mut USARTRegDef, _event: UsartEvent) {
        // This is a weak implementation that should be overridden by the application
    }
}

// Helper function to initialize USART
pub fn init_usart(
    pusartx: *mut USARTRegDef,
    mode: UsartMode,
    baud: UsartBaud,
    stop_bits: UsartStopBits,
    word_length: UsartWordLength,
    parity: UsartParity,
    hw_flow_control: UsartHwFlowControl,
) -> Result<UsartHandle<'static>, UsartError> {
    let mut handle: UsartHandle<'_> = UsartHandle::new(pusartx)?;
    handle.config_usart(mode, baud, stop_bits, word_length, parity, hw_flow_control)?;
    handle.init()?;
    Ok(handle)
}