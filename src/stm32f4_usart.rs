use core::marker::PhantomData;
use crate::stm32f4xx::{USART1, USART2, USART3, UART4, UART5, USART6, USARTRegDef, RCC, RegValue};

// Error type for USART operations
#[derive(Debug, Clone, Copy)]
pub enum UsartError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
    InvalidPeripheral,
}

// USART Mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartMode {
    TxOnly = 0,
    RxOnly = 1,
    TxRx   = 2,
}

// USART Baud Rate
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
    WordLength8Bits = 0,
    WordLength9Bits = 1,
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
    HardwareFlowControlNone     = 0,
    HardwareFlowControlCTS      = 1,
    HardwareFlowControlRTS      = 2,
    HardwareFlowControlCTSRTS   = 3,

}

// USART Status Register Flags
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartStatusFlag {
    ParityError             = 0,  // Parity Error
    FramingError            = 1,  // Framing Error
    NoiseDetectedFlag       = 2,  // Noise Flag
    OverrunError            = 3,  // Overrun Error
    IdleLineDetected        = 4,  // IDLE Line Detected
    RxNotEmpty              = 5,  // Read Data Register Not Empty
    TranmissionComplete     = 6,  // Transmission Complete
    TxNotEmpty              = 7,  // Transmit Data Register Empty
    LinBreakDetectionFlag   = 8,  // LIN Break Detection Flag
    CtsFlag                 = 9,  // CTS Flag
}

// USART Application State
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AppState {
    Ready      = 0,
    BusyInRx   = 1,
    BusyInTx   = 2,
}

// USART Application Events
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UsartEvent {
    TxComplete      = 0,
    RxComplete      = 1,
    Idle            = 2,
    Cts             = 3,
    ParityError     = 4,
    FramingError    = 5,
    NoiseError      = 6,
    OverrunError    = 7,
}

pub struct UsartConfig {
    pub usart_mode : UsartMode,
    pub usart_baud : UsartBaud,
    pub usart_stopbits : UsartStopBits,
    pub usart_wordlength: UsartWordLength,
    pub usart_parity: UsartParity,
    pub usart_hwflowcontrol: UsartHwFlowControl,
}

impl Default for UsartConfig {
    fn default() -> Self {
        UsartConfig {
            usart_mode: UsartMode::TxRx,
            usart_baud: UsartBaud::Baud115200,
            usart_stopbits: UsartStopBits::StopBits1,
            usart_wordlength: UsartWordLength::WordLength8Bits,
            usart_parity: UsartParity::Disable,
            usart_hwflowcontrol: UsartHwFlowControl::HardwareFlowControlNone,
        }
    }
}

// Container for USART register access
struct UsartRegister {
    register: *mut USARTRegDef,
}

impl UsartRegister {
    fn new(pusartx: *mut USARTRegDef) -> Result<Self, UsartError> {
        if pusartx.is_null() {
            return Err(UsartError::InvalidPeripheral);
        }
        
        Ok(UsartRegister {
            register: pusartx,
        })
    }

    // Read SR
    fn read_sr(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }

        let value: u32 = unsafe {
            (*self.register).SR
        };

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

    // Read CR1
    fn read_cr1(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }

        let value: u32 = unsafe {
            (*self.register).CR1
        };

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

    // Read CR2
    fn read_cr2(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }

        let value: u32 = unsafe {
            (*self.register).CR2
        };

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

    // Read CR3
    fn read_cr3(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }

        let value: u32 = unsafe {
            (*self.register).CR3
        };

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

    // Read BRR
    fn read_brr(&self) -> Result<RegValue, UsartError> {
        if self.register.is_null() {
            return Err(UsartError::HardwareFault);
        }

        let value: u32 = unsafe {
            (*self.register).BRR
        };

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
}