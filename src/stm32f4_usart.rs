use core::marker::PhantomData;
use crate::stm32f4xx::{
    USART1, USART2, USART3, UART4, UART5, USART6, 
    USARTRegDef, RegValue
};
use crate::stm32f4_rcc::{RccHandle, RccRegister, RccError};

// Error type for USART operations
#[derive(Debug, Clone, Copy)]
pub enum UsartError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
    InvalidPeripheral,
    RccError(RccError),
}

// Convert RccError to UsartError
impl From<RccError> for UsartError {
    fn from(error: RccError) -> Self {
        UsartError::RccError(error)
    }
}

// USART Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartMode {
    TxOnly = 0,
    RxOnly = 1,
    TxRx   = 2,
}

// USART Baudrate
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

// USART Parity
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartParity {
    Disable     = 0,
    EvenParity  = 1,
    OddParity   = 2,
}

// USART Word Length
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartWordLength {
    WordLengthBits8 = 0,
    WordLengthBits9 = 1,
}

// USART Stop Bits
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartStopBits {
    StopBits1   = 0,
    StopBits0_5 = 1,
    StopBits2   = 2,
    StopBits1_5 = 3,
}

// USART Hardware Flow Control
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartHwFlowControl {
    HwFlowControlNone   = 0,
    HwFlowControlCTS    = 1,
    HwFlowControlRTS    = 2,
    HwFlowControlCtsRts = 3,
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
    pub stopbits: UsartStopBits,
    pub wordlength: UsartWordLength,
    pub parity: UsartParity,
    pub hwflowcontrol: UsartHwFlowControl,
}

impl Default for UsartConfig {
    fn default() -> Self {
        UsartConfig {
            mode: UsartMode::TxRx,
            baud: UsartBaud::Baud115200,
            stopbits: UsartStopBits::StopBits1,
            wordlength: UsartWordLength::WordLengthBits8,
            parity: UsartParity::Disable,
            hwflowcontrol: UsartHwFlowControl::HwFlowControlNone,
        }
    }
}


struct UsartRegister {
    reg: *mut USARTRegDef,
}

impl UsartRegister {
    fn new(pusartx: *mut USARTRegDef) -> Result<Self, UsartError> {
        if pusartx.is_null() {
            return Err(UsartError::InvalidPeripheral);
        }
        
        Ok(UsartRegister { reg: pusartx })
    }
    
    // Read SR
    fn read_sr(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).SR };
        Ok(RegValue::new(value))
    }
    
    // Write SR
    fn write_sr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).SR = value.get();
        }
        
        Ok(())
    }
    
    // Modify SR
    fn modify_sr<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_sr()?;
        let new_value: RegValue = f(value);
        self.write_sr(new_value)
    }
    
    // Read CR1
    fn read_cr1(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR1 };
        Ok(RegValue::new(value))
    }
    
    // Write CR1
    fn write_cr1(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR1 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR1
    fn modify_cr1<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr1()?;
        let new_value: RegValue = f(value);
        self.write_cr1(new_value)
    }
    
    // Read CR2
    fn read_cr2(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR2 };
        Ok(RegValue::new(value))
    }
    
    // Write CR2
    fn write_cr2(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR2 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR2
    fn modify_cr2<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr2()?;
        let new_value: RegValue = f(value);
        self.write_cr2(new_value)
    }
    
    // Read CR3
    fn read_cr3(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR3 };
        Ok(RegValue::new(value))
    }
    
    // Write CR3
    fn write_cr3(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR3 = value.get();
        }
        
        Ok(())
    }
    
    // Modify CR3
    fn modify_cr3<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr3()?;
        let new_value: RegValue = f(value);
        self.write_cr3(new_value)
    }
    
    // Read BRR
    fn read_brr(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).BRR };
        Ok(RegValue::new(value))
    }
    
    // Write BRR
    fn write_brr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).BRR = value.get();
        }
        
        Ok(())
    }
    
    // Modify BRR
    fn modify_brr<F>(&self, f: F) -> Result<(), UsartError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_brr()?;
        let new_value: RegValue = f(value);
        self.write_brr(new_value)
    }
    
    // Read DR
    fn read_dr(&self) -> Result<RegValue, UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).DR };
        Ok(RegValue::new(value))
    }
    
    // Write DR
    fn write_dr(&self, value: RegValue) -> Result<(), UsartError> {
        if self.reg.is_null() {
            return Err(UsartError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).DR = value.get();
        }
        
        Ok(())
    }
    
    // for operations that need direct access
    fn get_raw_ptr(&self) -> *mut USARTRegDef {
        self.reg
    }
}

// USART Handle struct
pub struct UsartHandle<'a> {
    pub pusartx: *mut USARTRegDef,
    pub config: UsartConfig,
    reg: UsartRegister,
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
        let reg: UsartRegister = UsartRegister::new(pusartx)?;
        
        Ok(UsartHandle {
            pusartx,
            config: UsartConfig::default(),
            reg,
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
        self.config.stopbits = stop_bits;
        self.config.wordlength = word_length;
        self.config.parity = parity;
        self.config.hwflowcontrol = hw_flow_control;
        
        Ok(())
    }
    
    pub fn init(&self) -> Result<(), UsartError> {
        // Get RCC handle
        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        
        // Enable USART clock
        rcc_handle.usart_clock_control(self.pusartx as u32, true)?;
        
        // Configure CR1
        self.reg.modify_cr1(|mut reg: RegValue| {
            // Configure USART
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
            if self.config.wordlength == UsartWordLength::WordLengthBits9 {
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
        
        // Configure CR2
        self.reg.modify_cr2(|mut reg: RegValue| {
            reg.clear_bits(0x3 << 12);
            reg.set_bits((self.config.stopbits as u32) << 12);  // Set new STOP bits
            reg
        })?;
        
        // Configure CR3
        self.reg.modify_cr3(|mut reg: RegValue| {
            match self.config.hwflowcontrol {
                UsartHwFlowControl::HwFlowControlCTS => {
                    reg.set_bits(1 << 9);     // CTSE bit
                },
                UsartHwFlowControl::HwFlowControlRTS => {
                    reg.set_bits(1 << 8);     // RTSE bit
                },
                UsartHwFlowControl::HwFlowControlCtsRts => {
                    reg.set_bits(1 << 9);     // CTSE bit
                    reg.set_bits(1 << 8);     // RTSE bit
                },
                UsartHwFlowControl::HwFlowControlNone => {
                    reg.clear_bits(1 << 9);   // CTSE bit
                    reg.clear_bits(1 << 8);   // RTSE bit
                },
            }
            reg
        })?;
        
        // Set baud rate
        self.set_baudrate(self.config.baud)?;
        
        // Enable USART
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.set_bits(1 << 13);    // UE bit
            reg
        })?;
        
        Ok(())
    }
    
    pub fn deinit(&self) -> Result<(), UsartError> {
        // Disable USART
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.clear_bits(1 << 13);    // UE bit
            reg
        })?;
        
        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        rcc_handle.usart_reset(self.pusartx as u32)?;
        
        Ok(())
    }

    pub fn set_baudrate(&self, baud: UsartBaud) -> Result<(), UsartError> {
        let rcc_handle: RccHandle<'_> = RccHandle::new()?;

        let pclk: u32 = if self.pusartx == USART1 || self.pusartx == USART6 {
            rcc_handle.get_pclk2_freq() // USART1 and USART6 are on APB2
        } else {
            rcc_handle.get_pclk1_freq() // Other USART are on APB1
        };

        // Calculate USARTDIV
        let mut usartdiv: u32 = 0;
        let over8: bool = unsafe { ((*self.pusartx).CR1 & (1 << 15)) != 0 };

        if over8 {
            // Oversampling by 8
            usartdiv = ((pclk * 25) / (2 * baud as u32));
        } else {
            // Oversampling by 16
            usartdiv = ((pclk * 25) / (4 * baud as u32));
        }

        // Calculate mantissa and fraction parts
        let mantissa: u32 = usartdiv / 100;
        let fraction: u32 = usartdiv - (mantissa * 100);
        let final_fraction: u32 = if over8 {
            ((fraction * 8) + 50) / 100 & 0x07
        } else {
            ((fraction * 16) + 50) / 100 & 0x0F
        };
        
        // Write to BRR
        let brr_value: u32 = (mantissa << 4) | final_fraction;
        self.reg.write_brr(RegValue::new(brr_value))?;

        Ok(())
    }

    pub fn get_flag_status(&self, flag: UsartStatusFlag) -> Result<bool, UsartError> {
        let sr_value: RegValue = self.reg.read_sr()?;
    
        let bit_pos: u32 = match flag {
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

        Ok((sr_value.get() & (1 << bit_pos)) != 0)
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
        
        self.reg.modify_sr(|mut reg: RegValue| {
            reg.clear_bits(1 << bit_position);
            reg
        })?;
        
        Ok(())
    }
    
    pub fn send_data(&self, tx_buffer: &[u8]) -> Result<(), UsartError> {
        for &byte in tx_buffer {
            // Wait until TXE
            while !self.get_flag_status(UsartStatusFlag::TXE)? {
                // TODO: Timeout may be implemented
            }
            
            // Handle 9-bit data if needed
            if self.config.wordlength == UsartWordLength::WordLengthBits8 {
                if self.config.parity == UsartParity::Disable {
                    // TODO: 9bit data handling may need a more complex approach
                    self.reg.write_dr(RegValue::new(byte as u32))?;
                } else {
                    self.reg.write_dr(RegValue::new(byte as u32))?;
                }
            } else {
                // 8-bit data
                self.reg.write_dr(RegValue::new(byte as u32))?;
            }
        }
        
        // Wait until TC
        while !self.get_flag_status(UsartStatusFlag::TC)? {
            // TODO: Timeout may be implemented
        }
        
        Ok(())
    }
    
    pub fn receive_data(&self, rx_buffer: &mut [u8], len: usize) -> Result<(), UsartError> {
        for i in 0..len.min(rx_buffer.len()) {
            // Wait until RXNE
            while !self.get_flag_status(UsartStatusFlag::RXNE)? {
                // TODO: Timeout may be implemented
            }
            
            if self.config.wordlength == UsartWordLength::WordLengthBits8 {
                if self.config.parity == UsartParity::Disable {
                    // TODO: 9-bit data would need more complex handling
                    let dr_value: u32 = self.reg.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                } else {
                    let dr_value: u32 = self.reg.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                }
            } else {
                // 8-bit word length
                if self.config.parity == UsartParity::Disable {
                    // 8 bits of data
                    let dr_value: u32 = self.reg.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0xFF) as u8;
                } else {
                    // 7 bits of data + 1 parity bit
                    let dr_value: u32 = self.reg.read_dr()?.get();
                    rx_buffer[i] = (dr_value & 0x7F) as u8;
                }
            }
        }
        
        Ok(())
    }
    
    pub fn send_data_it(&mut self, tx_buffer: &'a [u8]) -> Result<UsartState, UsartError> {
        let tx_state: UsartState = self.tx_state;
        
        if tx_state != UsartState::BusyInTx {
            self.tx_buffer = Some(tx_buffer);
            self.tx_len = tx_buffer.len();
            self.tx_state = UsartState::BusyInTx;
            
            // TXEIE bit
            self.reg.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 7);
                reg
            })?;
            
            // TCIE bit
            self.reg.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 6);
                reg
            })?;
        }
        
        Ok(tx_state)
    }
    
    pub fn receive_data_it(&mut self, rx_buffer: &'a mut [u8], len: usize) -> Result<UsartState, UsartError> {
        let rx_state: UsartState = self.rx_state;
        
        if rx_state != UsartState::BusyInRx {
            // Get the buffer length
            let buffer_len: usize = rx_buffer.len();
            // Calculate rx_len
            self.rx_len = len.min(buffer_len);
            // Now store the buffer
            self.rx_buffer = Some(rx_buffer);
            self.rx_state = UsartState::BusyInRx;
            
            // Dummy read DR
            let _ = self.reg.read_dr()?;
            
            // RXNEIE bit
            self.reg.modify_cr1(|mut reg: RegValue| {
                reg.set_bits(1 << 5);
                reg
            })?;
        }
        
        Ok(rx_state)
    }
    
    pub fn irq_handler(&mut self) -> Result<(), UsartError> {
        // Check for TC
        let tc_flag: bool = self.get_flag_status(UsartStatusFlag::TC)?;
        let tcie_flag: bool = unsafe { 0 != ((*self.pusartx).CR1 & (1 << 6)) };
        
        if tc_flag && tcie_flag {
            if self.tx_state == UsartState::BusyInTx && 0 == self.tx_len {
                // Clear TC flag
                self.clear_flag(UsartStatusFlag::TC)?;
                
                // Disable TCIE
                self.reg.modify_cr1(|mut reg: RegValue| {
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
                        let index: usize = buffer.len() - self.tx_len;
                        
                        // Send data byte
                        self.reg.write_dr(RegValue::new(buffer[index] as u32))?;
                        
                        // Decrement TX length
                        self.tx_len -= 1;
                    }
                }
                
                if self.tx_len == 0 {
                    // TXEIE bit
                    self.reg.modify_cr1(|mut reg: RegValue| {
                        reg.clear_bits(1 << 7);
                        reg
                    })?;
                }
            }
        }
        
        // Check for RXNE flag
        let rxne_flag: bool = self.get_flag_status(UsartStatusFlag::RXNE)?;
        let rxneie_flag: bool = unsafe { 0 != ((*self.pusartx).CR1 & (1 << 5)) };
        
        if rxne_flag && rxneie_flag {
            // RXNE interrupt handling
            if self.rx_state == UsartState::BusyInRx && self.rx_len > 0 {
                if let Some(buffer) = &mut self.rx_buffer {
                    let index: usize = buffer.len() - self.rx_len;
                    
                    if self.config.wordlength == UsartWordLength::WordLengthBits9 {
                        if self.config.parity == UsartParity::Disable {
                            // 9 bits, no parity
                            let dr_value: u32 = self.reg.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        } else {
                            // 8 bits + parity
                            let dr_value: u32 = self.reg.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        }
                    } else {
                        // 8-bit word length
                        if self.config.parity == UsartParity::Disable {
                            // 8 bits, no parity
                            let dr_value: u32 = self.reg.read_dr()?.get();
                            buffer[index] = (dr_value & 0xFF) as u8;
                        } else {
                            // 7 bits + parity
                            let dr_value: u32 = self.reg.read_dr()?.get();
                            buffer[index] = (dr_value & 0x7F) as u8;
                        }
                    }
                    
                    // Decrement RX length
                    self.rx_len -= 1;
                    
                    if self.rx_len == 0 {
                        // Disable RXNEIE
                        self.reg.modify_cr1(|mut reg: RegValue| {
                            reg.clear_bits(1 << 5);
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
        
        // Check for CTS flag
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
            let _ = self.reg.read_sr()?;
            let _ = self.reg.read_dr()?;
            
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
        if pusartx.is_null() {
            return Err(UsartError::InvalidPeripheral);
        }

        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        rcc_handle.usart_clock_control(pusartx as u32, state)?;
        Ok(())
    }

    pub fn application_event_callback(_pusartx: *mut USARTRegDef, _event: UsartEvent) {
        //TODO: this is weak
    }
}

pub fn init_usart(
    pusartx: *mut USARTRegDef,
    mode: UsartMode,
    baud: UsartBaud,
    stop_bits: UsartStopBits,
    word_length: UsartWordLength,
    parity: UsartParity,
    hw_flow_control: UsartHwFlowControl,
) -> Result<UsartHandle<'static>, UsartError> 
{
    let mut handle: UsartHandle<'_> = UsartHandle::new(pusartx)?;
    handle.config_usart(mode, baud, stop_bits, word_length, parity, hw_flow_control)?;
    handle.init()?;
    Ok(handle)
}