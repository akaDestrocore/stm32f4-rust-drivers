use core::marker::PhantomData;
use crate::stm32f4xx::{
    I2C1, I2C2, I2C3, I2C1_BASE, I2C2_BASE, I2C3_BASE, I2CRegDef, RegValue
};

use crate::stm32f4_rcc::{
    RccHandle, RccRegister, RccError
};

// Error type for I2C
#[derive(Debug, Clone, Copy)]
pub enum I2cError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
    InvalidPeripheral,
    ArbitrationLoss,
    AckFailure,
    BusError,
    Overrun,
    RccError(RccError),
}

// Convert RccError to I2cError
impl From<RccError> for I2cError {
    fn from(err: RccError) -> Self {
        I2cError::RccError(err)
    }
}

// I2C Speed
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum I2cSpeed {
    Standard = 100_000,
    Fast200K = 200_000,
    Fast400K = 400_000,
}

// I2C Ack Control
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cAckControl {
    Disable = 0,
    Enable  = 1,
}

// I2C Fast Mode Duty Cycle
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cFmDutyCycle {
    Duty2       = 0,
    Duty16_9    = 1,
}

// I2C Application State
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cState {
    Ready   = 0,
    BusyInRx = 1,
    BusyInTx = 2,
}

// I2C Application Events
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cEvent {
    TxComplete      = 0,
    RxComplete      = 1,
    Stop            = 2,
    DataRequest     = 3,
    DataReceive     = 4,
    BusError        = 5,
    ArbitrationLoss = 6,
    AckFailure      = 7,
    Overrun         = 8,
    Timeout         = 9,
}

// I2C Status Register 1 Flags
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cSr1Flag {
    StartBit                = 0,    // SB
    AddressSent             = 1,    // ADDR
    ByteTransferFinished    = 2,    // BTF
    AddressSent10bit        = 3,    // ADD10
    StopDetection           = 4,    // STOPF
    RxNotEmpty              = 6,    // RXNE
    TxEmpty                 = 7,    // TXE
    BusError                = 8,    // BERR
    ArbitrationLoss         = 9,    // ARLO
    AckFailure              = 10,   // AF
    Overrun                 = 11,   // OVR
    PecError                = 12,   // PECERR
    Timeout                 = 14,   // TIMEOUT
    SmbAlert                = 15,   // SMBALERT
}

// I2C Memory Address Size
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum I2cMemAddSize {
    Size8bit = 1,
    Size16bit = 0,
}

// I2C Configuration
#[derive(Debug, Clone, Copy)]
pub struct I2cConfig {
    pub sclspeed: I2cSpeed,
    pub deviceaddress: u8,
    pub ackcontrol: I2cAckControl,
    pub fmdutycycle: I2cFmDutyCycle,
}

impl Default for I2cConfig {
    fn default() -> Self {
        I2cConfig {
            sclspeed: I2cSpeed::Standard,
            deviceaddress: 0x00,
            ackcontrol: I2cAckControl::Enable,
            fmdutycycle: I2cFmDutyCycle::Duty2,
        }
    }
}

// I2C Register access struct
struct I2cRegister {
    reg: *mut I2CRegDef,
}

impl I2cRegister {
    fn new(pi2cx: *mut I2CRegDef) -> Result<Self, I2cError> {
        if pi2cx.is_null() {
            return Err(I2cError::InvalidPeripheral);
        }
        
        Ok(I2cRegister { reg: pi2cx })
    }

    // CR1 register
    fn read_cr1(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR1 };
        Ok(RegValue::new(value))
    }

    fn write_cr1(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR1 = value.get();
        }
        
        Ok(())
    }

    fn modify_cr1<F>(&self, f: F) -> Result<(), I2cError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr1()?;
        let new_value: RegValue = f(value);
        self.write_cr1(new_value)
    }

    // CR2 register
    fn read_cr2(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR2 };
        Ok(RegValue::new(value))
    }

    fn write_cr2(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR2 = value.get();
        }
        
        Ok(())
    }

    fn modify_cr2<F>(&self, f: F) -> Result<(), I2cError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr2()?;
        let new_value: RegValue = f(value);
        self.write_cr2(new_value)
    }

    // OAR1 register
    fn read_oar1(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).OAR1 };
        Ok(RegValue::new(value))
    }

    fn write_oar1(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).OAR1 = value.get();
        }
        
        Ok(())
    }

    // OAR2 register
    fn read_oar2(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).OAR2 };
        Ok(RegValue::new(value))
    }

    // DR register
    fn read_dr(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).DR };
        Ok(RegValue::new(value))
    }

    fn write_dr(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).DR = value.get();
        }
        
        Ok(())
    }

    // SR1 register
    fn read_sr1(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).SR1 };
        Ok(RegValue::new(value))
    }

    fn write_sr1(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).SR1 = value.get();
        }
        
        Ok(())
    }

    fn modify_sr1<F>(&self, f: F) -> Result<(), I2cError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_sr1()?;
        let new_value: RegValue = f(value);
        self.write_sr1(new_value)
    }

    // SR2 register
    fn read_sr2(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).SR2 };
        Ok(RegValue::new(value))
    }

    // CCR register
    fn read_ccr(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CCR };
        Ok(RegValue::new(value))
    }

    fn write_ccr(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CCR = value.get();
        }
        
        Ok(())
    }

    // TRISE register
    fn read_trise(&self) -> Result<RegValue, I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).TRISE };
        Ok(RegValue::new(value))
    }

    fn write_trise(&self, value: RegValue) -> Result<(), I2cError> {
        if self.reg.is_null() {
            return Err(I2cError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).TRISE = value.get();
        }
        
        Ok(())
    }

    // Helper to get raw ptr
    fn get_raw_ptr(&self) -> *mut I2CRegDef {
        self.reg
    }

}

// I2C Handle struct
pub struct I2cHandle<'a> {
    pub pi2cx: *mut I2CRegDef,
    pub config: I2cConfig,
    reg: I2cRegister,
    tx_buffer: Option<&'a [u8]>,
    rx_buffer: Option<&'a mut [u8]>,
    tx_len: usize,
    rx_len: usize,
    rx_size: usize,
    tx_state: I2cState,
    rx_state: I2cState,
    device_addr: u8,
    repeated_start: bool,
    _marker: PhantomData<&'a ()>,
}

impl<'a> I2cHandle<'a> {
    pub fn new(pi2cx: *mut I2CRegDef) -> Result<Self, I2cError> {
        let reg: I2cRegister = I2cRegister::new(pi2cx)?;
        
        Ok(I2cHandle {
            pi2cx,
            config: I2cConfig::default(),
            reg,
            tx_buffer: None,
            rx_buffer: None,
            tx_len: 0,
            rx_len: 0,
            rx_size: 0,
            tx_state: I2cState::Ready,
            rx_state: I2cState::Ready,
            device_addr: 0,
            repeated_start: false,
            _marker: PhantomData,
        })
    }

    pub fn config_i2c(
        &mut self,
        scl_speed: I2cSpeed,
        device_address: u8,
        ack_control: I2cAckControl,
        fm_duty_cycle: I2cFmDutyCycle,
    ) -> Result<(), I2cError> {
        self.config.sclspeed = scl_speed;
        self.config.deviceaddress = device_address;
        self.config.ackcontrol = ack_control;
        self.config.fmdutycycle = fm_duty_cycle;
        
        Ok(())
    }

    pub fn init(&self) -> Result<(), I2cError> {
        // Get RCC handle
        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        
        // Enable I2C clock
        match self.pi2cx as u32 {
            I2C1_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 21, true)?,
            I2C2_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 22, true)?,
            I2C3_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 23, true)?,
            _ => return Err(I2cError::InvalidPeripheral),
        }
        
        // Configure CR1
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.clear_bits(1 << 0); // PE bit
            reg
        })?;
        
        // Configure the frequency field in CR2
        let pclk1_freq: u32 = rcc_handle.get_pclk1_freq() / 1_000_000;
        self.reg.modify_cr2(|mut reg: RegValue| {
            reg.clear_bits(0x3F); // Clear FREQ[5:0]
            reg.set_bits(pclk1_freq & 0x3F); // Set freq value
            reg
        })?;
        
        // Program device own address
        let temp: u32 = ((self.config.deviceaddress as u32) << 1) | (1 << 14);
        self.reg.write_oar1(RegValue::new(temp))?;
        
        // CCR calculations
        let mut ccr_value: u32;
        if self.config.sclspeed <= I2cSpeed::Standard {
            // Standard mode calculation
            ccr_value = rcc_handle.get_pclk1_freq() / (2 * self.config.sclspeed as u32);
        } else {
            // Fast mode calculation
            if self.config.fmdutycycle == I2cFmDutyCycle::Duty2 {
                ccr_value = rcc_handle.get_pclk1_freq() / (3 * self.config.sclspeed as u32);
            } else {
                ccr_value = rcc_handle.get_pclk1_freq() / (25 * self.config.sclspeed as u32);
            }
            // Set fast mode (1 << 15) and duty cycle bit if needed
            ccr_value |= (1 << 15); // Set FM mode
            if self.config.fmdutycycle == I2cFmDutyCycle::Duty16_9 {
                ccr_value |= (1 << 14); // Set duty cycle
            }
        }
        self.reg.write_ccr(RegValue::new(ccr_value))?;
        
        // TRISE Configuration
        let trise_value: u32;
        if self.config.sclspeed <= I2cSpeed::Standard {
            // For standard mode
            trise_value = (pclk1_freq + 1) & 0x3F;
        } else {
            // For fast mode
            trise_value = (((pclk1_freq * 300) / 1000) + 1) & 0x3F;
        }
        self.reg.write_trise(RegValue::new(trise_value))?;
        
        // Enable the peripheral
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.set_bits(1 << 0); // PE bit
            reg
        })?;
        
        // Configure ACK control
        self.manage_acking(self.config.ackcontrol)?;
        
        Ok(())
    }

    pub fn deinit(&self) -> Result<(), I2cError> {
        // Disable the I2C peripheral
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.clear_bits(1 << 0); // PE bit
            reg
        })?;
        
        // Reset the I2C peripheral
        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        use crate::stm32f4xx::{I2C1_BASE, I2C2_BASE, I2C3_BASE};
        match self.pi2cx as u32 {
            I2C1_BASE => rcc_handle.rcc_reg.reset_peripheral("APB1", 21)?,
            I2C2_BASE => rcc_handle.rcc_reg.reset_peripheral("APB1", 22)?,
            I2C3_BASE => rcc_handle.rcc_reg.reset_peripheral("APB1", 23)?,
            _ => return Err(I2cError::InvalidPeripheral),
        }
        
        Ok(())
    }

    // Helper methods
    fn generate_start_condition(&self) -> Result<(), I2cError> {
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.set_bits(1 << 8); // START bit
            reg
        })
    }

    fn generate_stop_condition(&self) -> Result<(), I2cError> {
        self.reg.modify_cr1(|mut reg: RegValue| {
            reg.set_bits(1 << 9); // STOP bit
            reg
        })
    }

    fn execute_address_phase_write(&self, slave_addr: u8) -> Result<(), I2cError> {
        let addr: u8 = (slave_addr << 1) & 0xFE; // Clear bit 0 for write
        self.reg.write_dr(RegValue::new(addr as u32))
    }
    
    fn execute_address_phase_read(&self, slave_addr: u8) -> Result<(), I2cError> {
        let addr: u8 = (slave_addr << 1) | 0x01; // Set bit 0 for read
        self.reg.write_dr(RegValue::new(addr as u32))
    }
    
    fn clear_addr_flag(&self) -> Result<(), I2cError> {
        // Need to read both SR1 and SR2 to clear the ADDR flag
        let _ = self.reg.read_sr1()?;
        let _ = self.reg.read_sr2()?;
        Ok(())
    }
    
    pub fn manage_acking(&self, state: I2cAckControl) -> Result<(), I2cError> {
        self.reg.modify_cr1(|mut reg: RegValue| {
            if state == I2cAckControl::Enable {
                reg.set_bits(1 << 10); // ACK bit
            } else {
                reg.clear_bits(1 << 10); // ACK bit
            }
            reg
        })
    }
    
    pub fn get_flag_status(&self, flag: I2cSr1Flag) -> Result<bool, I2cError> {
        let sr1_value: u32 = self.reg.read_sr1()?.get();
        
        let bit_pos: i32 = match flag {
            I2cSr1Flag::StartBit => 0,
            I2cSr1Flag::AddressSent => 1,
            I2cSr1Flag::ByteTransferFinished => 2,
            I2cSr1Flag::AddressSent10bit => 3,
            I2cSr1Flag::StopDetection => 4,
            I2cSr1Flag::RxNotEmpty => 6,
            I2cSr1Flag::TxEmpty => 7,
            I2cSr1Flag::BusError => 8,
            I2cSr1Flag::ArbitrationLoss => 9,
            I2cSr1Flag::AckFailure => 10,
            I2cSr1Flag::Overrun => 11,
            I2cSr1Flag::PecError => 12,
            I2cSr1Flag::Timeout => 14,
            I2cSr1Flag::SmbAlert => 15,
        };
        
        Ok((sr1_value & (1 << bit_pos)) != 0)
    }

    pub fn master_send_data(&self, tx_buffer: &[u8], len: usize, slave_addr: u8, sr: bool) -> Result<(), I2cError> {
        // Generate START condition
        self.generate_start_condition()?;
        
        // Wait until START condition is generated
        let mut timeout: i32 = 1000;
        while !self.get_flag_status(I2cSr1Flag::StartBit)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        // Send slave address with write bit
        self.execute_address_phase_write(slave_addr)?;
        
        // Wait for address to be acknowledged
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::AddressSent)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        // Clear ADDR flag
        self.clear_addr_flag()?;
        
        // Send data byte by byte
        for i in 0..len.min(tx_buffer.len()) {
            // Wait for TXE (Transmit buffer empty)
            timeout = 1000;
            while !self.get_flag_status(I2cSr1Flag::TxEmpty)? {
                timeout -= 1;
                if 0 == timeout {
                    return Err(I2cError::Timeout);
                }
            }
            
            // Send the byte
            self.reg.write_dr(RegValue::new(tx_buffer[i] as u32))?;
        }
        
        // Wait for TXE and BTF (Byte transfer finished)
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::TxEmpty)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::ByteTransferFinished)? {
            timeout -= 1;
            if timeout == 0 {
                return Err(I2cError::Timeout);
            }
        }
        
        // Generate STOP condition if repeated start is not requested
        if !sr {
            self.generate_stop_condition()?;
        }
        
        Ok(())
    }

    pub fn master_receive_data(&self, rx_buffer: &mut [u8], len: usize, slave_addr: u8, sr: bool) -> Result<(), I2cError> {
        if len == 0 || rx_buffer.is_empty() {
            return Ok(());
        }
        
        // Generate START condition
        self.generate_start_condition()?;
        
        // Wait until START condition is generated
        let mut timeout: i32 = 1000;
        while !self.get_flag_status(I2cSr1Flag::StartBit)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        // Send slave address with read bit
        self.execute_address_phase_read(slave_addr)?;
        
        // Wait for address to be acknowledged
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::AddressSent)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        // Special case for receiving only 1 byte
        if 1 == len {
            // Disable Acknowledging before clearing ADDR
            self.manage_acking(I2cAckControl::Disable)?;
            
            // Clear ADDR flag
            self.clear_addr_flag()?;
            
            // Wait for RXNE
            timeout = 1000;
            while !self.get_flag_status(I2cSr1Flag::RxNotEmpty)? {
                timeout -= 1;
                if 0 == timeout {
                    return Err(I2cError::Timeout);
                }
            }
            
            // Generate STOP condition if not repeated start
            if !sr {
                self.generate_stop_condition()?;
            }
            
            // Read the data
            let data: u32 = self.reg.read_dr()?.get();
            rx_buffer[0] = data as u8;
            
            // Re-enable ACK if it was enabled in config
            if self.config.ackcontrol == I2cAckControl::Enable {
                self.manage_acking(I2cAckControl::Enable)?;
            }
            
            return Ok(());
        }
        
        // Clear ADDR flag
        self.clear_addr_flag()?;
        
        // Read bytes until length becomes 0
        for i in 0..len.min(rx_buffer.len()) {
            // Wait for RXNE
            timeout = 1000;
            while !self.get_flag_status(I2cSr1Flag::RxNotEmpty)? {
                timeout -= 1;
                if 0 == timeout {
                    return Err(I2cError::Timeout);
                }
            }
            
            // For last two bytes
            if i == len - 2 {
                // Disable acknowledging
                self.manage_acking(I2cAckControl::Disable)?;
                
                // Generate STOP condition if not repeated start
                if !sr {
                    self.generate_stop_condition()?;
                }
            }
            
            // Read the data
            let data: u32 = self.reg.read_dr()?.get();
            rx_buffer[i] = data as u8;
        }
        
        // Re-enable ACK if it was enabled in config
        if self.config.ackcontrol == I2cAckControl::Enable {
            self.manage_acking(I2cAckControl::Enable)?;
        }
        
        Ok(())
    }

    pub fn mem_write(&self, dev_address: u16, mem_address: u16, mem_add_size: I2cMemAddSize, data: &[u8], size: usize) -> Result<(), I2cError> {
        // Generate START condition
        self.generate_start_condition()?;
        
        // Wait until START condition is generated
        let mut timeout: i32 = 1000;
        while !self.get_flag_status(I2cSr1Flag::StartBit)? {
            timeout -= 1;
            if timeout == 0 {
                return Err(I2cError::Timeout);
            }
        }
        
        // Send device address with write bit
        let dev_addr: u8 = (dev_address as u8) & 0xFE; // Clear bit 0 for write
        self.reg.write_dr(RegValue::new(dev_addr as u32))?;
        
        // Wait for address to be acknowledged
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::AddressSent)? {
            timeout -= 1;
            if timeout == 0 {
                return Err(I2cError::Timeout);
            }
        }
        
        // Clear ADDR flag
        self.clear_addr_flag()?;
        
        // Send memory address
        if mem_add_size == I2cMemAddSize::Size16bit {
            // Send MSB of memory address
            let mem_addr_msb: u8 = ((mem_address >> 8) & 0xFF) as u8;
            timeout = 1000;
            while !self.get_flag_status(I2cSr1Flag::TxEmpty)? {
                timeout -= 1;
                if 0 == timeout {
                    return Err(I2cError::Timeout);
                }
            }
            self.reg.write_dr(RegValue::new(mem_addr_msb as u32))?;
        }
        
        // Send LSB of memory address
        let mem_addr_lsb: u8 = (mem_address & 0xFF) as u8;
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::TxEmpty)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        self.reg.write_dr(RegValue::new(mem_addr_lsb as u32))?;
        
        // Send data byte by byte
        for i in 0..size.min(data.len()) {
            // Wait for TXE
            timeout = 1000;
            while !self.get_flag_status(I2cSr1Flag::TxEmpty)? {
                timeout -= 1;
                if 0 == timeout {
                    return Err(I2cError::Timeout);
                }
            }
            
            // Send the byte
            self.reg.write_dr(RegValue::new(data[i] as u32))?;
        }
        
        // Wait for TXE and BTF
        timeout = 1000;
        while !self.get_flag_status(I2cSr1Flag::TxEmpty)? || !self.get_flag_status(I2cSr1Flag::ByteTransferFinished)? {
            timeout -= 1;
            if 0 == timeout {
                return Err(I2cError::Timeout);
            }
        }
        
        // Generate STOP condition
        self.generate_stop_condition()?;
        
        Ok(())
    }

    pub fn master_send_data_it(&mut self, tx_buffer: &'a [u8], len: usize, slave_addr: u8, sr: bool) -> Result<I2cState, I2cError> {
        let current_state: I2cState = self.tx_state;
        
        if current_state != I2cState::BusyInTx && current_state != I2cState::BusyInRx {
            self.tx_buffer = Some(tx_buffer);
            self.tx_len = len;
            self.tx_state = I2cState::BusyInTx;
            self.device_addr = slave_addr;
            self.repeated_start = sr;
            
            // Generate START condition
            self.generate_start_condition()?;
            
            // Enable interrupts
            self.reg.modify_cr2(|mut reg: RegValue| {
                reg.set_bits(1 << 9); // ITBUFEN - Buffer interrupt enable
                reg.set_bits(1 << 10); // ITEVTEN - Event interrupt enable
                reg.set_bits(1 << 8);  // ITERREN - Error interrupt enable
                reg
            })?;
        }
        
        Ok(current_state)
    }

    pub fn master_receive_data_it(&mut self, rx_buffer: &'a mut [u8], len: usize, slave_addr: u8, sr: bool) -> Result<I2cState, I2cError> {
        let current_state: I2cState = self.rx_state;
        
        if current_state != I2cState::BusyInTx && current_state != I2cState::BusyInRx {
            self.rx_buffer = Some(rx_buffer);
            self.rx_len = len;
            self.rx_size = len;
            self.rx_state = I2cState::BusyInRx;
            self.device_addr = slave_addr;
            self.repeated_start = sr;
            
            // Generate START condition
            self.generate_start_condition()?;
            
            // Enable interrupts
            self.reg.modify_cr2(|mut reg: RegValue| {
                reg.set_bits(1 << 9); // ITBUFEN - Buffer interrupt enable
                reg.set_bits(1 << 10); // ITEVTEN - Event interrupt enable
                reg.set_bits(1 << 8);  // ITERREN - Error interrupt enable
                reg
            })?;
        }
        
        Ok(current_state)
    }

    pub fn close_receive_data(&mut self) -> Result<(), I2cError> {
        // Disable interrupts
        self.reg.modify_cr2(|mut reg: RegValue| {
            reg.clear_bits(1 << 9); // ITBUFEN
            reg.clear_bits(1 << 10); // ITEVTEN
            reg
        })?;
        
        // Reset state
        self.rx_state = I2cState::Ready;
        self.rx_buffer = None;
        self.rx_len = 0;
        self.rx_size = 0;
        
        // Re-enable ACK if needed
        if self.config.ackcontrol == I2cAckControl::Enable {
            self.manage_acking(I2cAckControl::Enable)?;
        }
        
        Ok(())
    }

    pub fn close_send_data(&mut self) -> Result<(), I2cError> {
        // Disable interrupts
        self.reg.modify_cr2(|mut reg: RegValue| {
            reg.clear_bits(1 << 9); // ITBUFEN
            reg.clear_bits(1 << 10); // ITEVTEN
            reg
        })?;
        
        // Reset state
        self.tx_state = I2cState::Ready;
        self.tx_buffer = None;
        self.tx_len = 0;
        
        Ok(())
    }

    // Handle TXE interrupt
    fn handle_txe_interrupt(&mut self) -> Result<(), I2cError> {
        if self.tx_state == I2cState::BusyInTx && self.tx_len > 0 {
            if let Some(buffer) = self.tx_buffer {
                let index: usize = buffer.len() - self.tx_len;
                if index < buffer.len() {
                    self.reg.write_dr(RegValue::new(buffer[index] as u32))?;
                    self.tx_len -= 1;
                }
            }
        }
        Ok(())
    }

    // Handle RXNE interrupt
    fn handle_rxne_interrupt(&mut self) -> Result<(), I2cError> {
        if self.rx_state != I2cState::BusyInRx {
            return Ok(());
        }
        
        if self.rx_len == 2 {
            // Disable ACKing before reading next byte
            self.manage_acking(I2cAckControl::Disable)?;
        }
        
        let data: u8 = self.reg.read_dr()?.get() as u8;
        
        if let Some(buffer) = &mut self.rx_buffer {
            let index: usize = buffer.len() - self.rx_len;
            if index < buffer.len() {
                buffer[index] = data;
                self.rx_len -= 1;
            }
        }
        
        if self.rx_len == 0 {
            // Generate STOP if needed
            if !self.repeated_start {
                self.generate_stop_condition()?;
            }
            
            // Close receive operation
            self.close_receive_data()?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::RxComplete);
        }
        
        Ok(())
    }

    // IRQ Handler for events
    pub fn event_irq_handler(&mut self) -> Result<(), I2cError> {
        // Check for events
        let sr1: u32 = self.reg.read_sr1()?.get();
        let cr2: u32 = self.reg.read_cr2()?.get();
        
        // Check for SB (Start Bit) event - only in master mode
        if ((cr2 & (1 << 10)) != 0) && ((sr1 & (1 << 0)) != 0) {
            // Start bit is set
            if self.tx_state == I2cState::BusyInTx {
                self.execute_address_phase_write(self.device_addr)?;
            } else if self.rx_state == I2cState::BusyInRx {
                self.execute_address_phase_read(self.device_addr)?;
            }
        }
        
        // Check for ADDR event
        if ((cr2 & (1 << 10)) != 0) && ((sr1 & (1 << 1)) != 0) {
            // Address sent/matched
            self.clear_addr_flag()?;
            
            // Special case for single byte reception
            if self.rx_state == I2cState::BusyInRx && self.rx_size == 1 {
                // Disable ACK before clearing ADDR for single byte reception
                self.manage_acking(I2cAckControl::Disable)?;
            }
        }
        
        // Check for BTF (Byte Transfer Finished) event
        if ((cr2 & (1 << 10)) != 0) && ((sr1 & (1 << 2)) != 0) {
            // BTF is set
            if self.tx_state == I2cState::BusyInTx {
                // If BTF and TXE are set and no more data to send
                if ((sr1 & (1 << 7)) != 0) && self.tx_len == 0 {
                    // Generate STOP if not repeated start
                    if !self.repeated_start {
                        self.generate_stop_condition()?;
                    }
                    
                    // Reset transmission state
                    self.close_send_data()?;
                    
                    // Notify application
                    I2c::application_event_callback(self.pi2cx, I2cEvent::TxComplete);
                }
            }
        }
        
        // Check for RXNE (Receive buffer not empty)
        if ((cr2 & (1 << 10)) != 0) && ((cr2 & (1 << 9)) != 0) && ((sr1 & (1 << 6)) != 0) {
            if self.rx_state == I2cState::BusyInRx {
                self.handle_rxne_interrupt()?;
            }
        }
        
        // Check for TXE (Transmit buffer empty)
        if ((cr2 & (1 << 10)) != 0) && ((cr2 & (1 << 9)) != 0) && ((sr1 & (1 << 7)) != 0) {
            if self.tx_state == I2cState::BusyInTx {
                self.handle_txe_interrupt()?;
            }
        }
        
        // Check for STOPF (STOP detection) - only in slave mode
        if ((cr2 & (1 << 10)) != 0) && ((sr1 & (1 << 4)) != 0) {
            // Clear STOPF - need to read SR1 (already done) and write to CR1
            self.reg.write_cr1(self.reg.read_cr1()?)?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::Stop);
        }
        
        Ok(())
    }

    // IRQ Handler for errors
    pub fn error_irq_handler(&mut self) -> Result<(), I2cError> {
        let sr1: u32 = self.reg.read_sr1()?.get();
        let cr2: u32 = self.reg.read_cr2()?.get();
        
        // Check if error interrupt is enabled
        if (cr2 & (1 << 8)) == 0 {
            return Ok(());
        }
        
        // Bus Error
        if (sr1 & (1 << 8)) != 0 {
            // Clear the bus error flag
            self.reg.modify_sr1(|mut reg: RegValue| {
                reg.clear_bits(1 << 8);
                reg
            })?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::BusError);
        }
        
        // Arbitration Loss Error
        if (sr1 & (1 << 9)) != 0 {
            // Clear the arbitration loss flag
            self.reg.modify_sr1(|mut reg: RegValue| {
                reg.clear_bits(1 << 9);
                reg
            })?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::ArbitrationLoss);
        }
        
        // ACK Failure Error
        if (sr1 & (1 << 10)) != 0 {
            // Clear the ACK failure flag
            self.reg.modify_sr1(|mut reg: RegValue| {
                reg.clear_bits(1 << 10);
                reg
            })?;
            
            // Generate STOP condition in case of ACK failure
            if !self.repeated_start {
                self.generate_stop_condition()?;
            }
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::AckFailure);
        }
        
        // Overrun/Underrun Error
        if (sr1 & (1 << 11)) != 0 {
            // Clear the overrun flag
            self.reg.modify_sr1(|mut reg: RegValue| {
                reg.clear_bits(1 << 11);
                reg
            })?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::Overrun);
        }
        
        // Timeout Error
        if (sr1 & (1 << 14)) != 0 {
            // Clear the timeout flag
            self.reg.modify_sr1(|mut reg: RegValue| {
                reg.clear_bits(1 << 14);
                reg
            })?;
            
            // Notify application
            I2c::application_event_callback(self.pi2cx, I2cEvent::Timeout);
        }
        
        Ok(())
    }

    // Slave mode functions
    pub fn slave_send_data(&self, data: u8) -> Result<(), I2cError> {
        self.reg.write_dr(RegValue::new(data as u32))
    }
    
    pub fn slave_receive_data(&self) -> Result<u8, I2cError> {
        let value: u32 = self.reg.read_dr()?.get();
        Ok(value as u8)
    }

    pub fn slave_enable_disable_callback_events(&self, enable: bool) -> Result<(), I2cError> {
        let state: i32 = if enable { 1 } else { 0 };
        
        self.reg.modify_cr2(|mut reg: RegValue| {
            // Event interrupt enable
            if state != 0 {
                reg.set_bits(1 << 10);
            } else {
                reg.clear_bits(1 << 10);
            }
            
            // Buffer interrupt enable
            if state != 0 {
                reg.set_bits(1 << 9);
            } else {
                reg.clear_bits(1 << 9);
            }
            
            // Error interrupt enable
            if state != 0 {
                reg.set_bits(1 << 8);
            } else {
                reg.clear_bits(1 << 8);
            }
            
            reg
        })
    }

}

pub struct I2c;

impl I2c {
    pub fn periph_clock_control(pi2cx: *mut I2CRegDef, state: bool) -> Result<(), I2cError> {
        if pi2cx.is_null() {
            return Err(I2cError::InvalidPeripheral);
        }

        let rcc_handle: RccHandle<'_> = RccHandle::new()?;
        use crate::stm32f4xx::{I2C1_BASE, I2C2_BASE, I2C3_BASE};
        match pi2cx as u32 {
            I2C1_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 21, state)?,
            I2C2_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 22, state)?,
            I2C3_BASE => rcc_handle.rcc_reg.enable_peripheral_clock("APB1", 23, state)?,
            _ => return Err(I2cError::InvalidPeripheral),
        }
        Ok(())
    }
    
    pub fn peripheral_control(pi2cx: *mut I2CRegDef, enable: bool) -> Result<(), I2cError> {
        if pi2cx.is_null() {
            return Err(I2cError::InvalidPeripheral);
        }
        
        let reg: I2cRegister = I2cRegister::new(pi2cx)?;
        
        reg.modify_cr1(|mut cr1: RegValue| {
            if enable {
                cr1.set_bits(1 << 0); // PE bit
            } else {
                cr1.clear_bits(1 << 0); // PE bit
            }
            cr1
        })
    }

    pub fn application_event_callback(_pi2cx: *mut I2CRegDef, _event: I2cEvent) {
        // TODO: This is a weak function
    }
    
    pub fn irq_config(irq_number: u8, irq_priority: u8, enable: bool) -> Result<(), I2cError> {
        //  TODO: 
        Ok(())
    }
    
    pub fn get_flag_status(pi2cx: *mut I2CRegDef, flag: I2cSr1Flag) -> Result<bool, I2cError> {
        if pi2cx.is_null() {
            return Err(I2cError::InvalidPeripheral);
        }
        
        let reg: I2cRegister = I2cRegister::new(pi2cx)?;
        let sr1_value: u32 = reg.read_sr1()?.get();
        
        let bit_pos: i32 = match flag {
            I2cSr1Flag::StartBit => 0,
            I2cSr1Flag::AddressSent => 1,
            I2cSr1Flag::ByteTransferFinished => 2,
            I2cSr1Flag::AddressSent10bit => 3,
            I2cSr1Flag::StopDetection => 4,
            I2cSr1Flag::RxNotEmpty => 6,
            I2cSr1Flag::TxEmpty => 7,
            I2cSr1Flag::BusError => 8,
            I2cSr1Flag::ArbitrationLoss => 9,
            I2cSr1Flag::AckFailure => 10,
            I2cSr1Flag::Overrun => 11,
            I2cSr1Flag::PecError => 12,
            I2cSr1Flag::Timeout => 14,
            I2cSr1Flag::SmbAlert => 15,
        };
        
        Ok((sr1_value & (1 << bit_pos)) != 0)
    }
}

pub fn init_i2c(
    pi2cx: *mut I2CRegDef,
    scl_speed: I2cSpeed,
    device_address: u8,
    ack_control: I2cAckControl,
    fm_duty_cycle: I2cFmDutyCycle,
) -> Result<I2cHandle<'static>, I2cError> {
    let mut handle: I2cHandle<'_> = I2cHandle::new(pi2cx)?;
    handle.config_i2c(scl_speed, device_address, ack_control, fm_duty_cycle)?;
    handle.init()?;
    Ok(handle)
}