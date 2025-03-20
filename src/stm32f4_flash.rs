use core::marker::PhantomData;
use crate::stm32f4xx::{FLASH_R, FLASH_BASE, FLASHRegDef, RegValue};

// Error types for Flash operations
#[derive(Debug, Clone, Copy)]
pub enum FlashError {
    InvalidConfiguration,
    HardwareFault,
    Timeout,
    WriteError,
    EraseError,
    Busy,
    InvalidSector,
    WrongAddress,
}

#[derive(Debug, Clone, Copy)]
pub enum FlashDataSize {
    Data8Bit = 0,
    Data16Bit = 1,
    Data32Bit = 2,
    Data64Bit = 3,
}

#[derive(Debug, Clone, Copy)]
pub enum FlashLatency {
    Lat0WS = 0,
    Lat1WS = 1,
    Lat2WS = 2,
    Lat3WS = 3,
    Lat4WS = 4,
    Lat5WS = 5,
    Lat6WS = 6,
    Lat7WS = 7,
}

// Flash constants
pub const FLASH_KEY1: u32 = 0x45670123;
pub const FLASH_KEY2: u32 = 0xCDEF89AB;
pub const FLASH_OPT_KEY1: u32 = 0x08192A3B;
pub const FLASH_OPT_KEY2: u32 = 0x4C5D6E7F;

pub const FLASH_APP_START_ADDRESS: u32 = 0x08008004;
pub const FLASH_TEMP_START_ADDRESS: u32 = 0x080A0000;
pub const FLASH_SECTOR_TOTAL: usize = 12;

// Flash sectors sizes in kB
pub const FLASH_SECTORS: [u32; FLASH_SECTOR_TOTAL] = [
    16,  // sector 0
    16,  // sector 1
    16,  // sector 2
    16,  // sector 3
    64,  // sector 4
    128, // sector 5
    128, // sector 6
    128, // sector 7
    128, // sector 8
    128, // sector 9
    128, // sector 10
    128  // sector 11
];

// Flash register access
pub struct FlashRegister {
    reg: *mut FLASHRegDef,
}

impl FlashRegister {
    pub fn new() -> Result<Self, FlashError> {
        if FLASH_R.is_null() {
            return Err(FlashError::HardwareFault);
        }

        Ok(FlashRegister { reg: FLASH_R })
    }

    // ACR register
    pub fn read_acr(&self) -> Result<RegValue, FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }

        let value: u32 = unsafe { (*self.reg).ACR };
        Ok(RegValue::new(value))
    }

    pub fn write_acr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }

        unsafe {
            (*self.reg).ACR = value.get();
        }

        Ok(())
    }

    pub fn modify_acr<F>(&self, f: F) -> Result<(), FlashError>
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_acr()?;
        let new_value: RegValue = f(value);
        self.write_acr(new_value)
    }

    // KEYR register
    pub fn write_keyr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).KEYR = value.get();
        }
        
        Ok(())
    }

    // OPTKEYR register
    pub fn write_optkeyr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).OPTKEYR = value.get();
        }
        
        Ok(())
    }

    // SR register
    pub fn read_sr(&self) -> Result<RegValue, FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }

        let value: u32 = unsafe { (*self.reg).SR };
        Ok(RegValue::new(value))
    }

    pub fn write_sr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }

        unsafe {
            (*self.reg).SR = value.get();
        }

        Ok(())
    }

    // CR register
    pub fn read_cr(&self) -> Result<RegValue, FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_cr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_cr<F>(&self, f: F) -> Result<(), FlashError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_cr()?;
        let new_value: RegValue = f(value);
        self.write_cr(new_value)
    }

    // OPTCR register
    pub fn read_optcr(&self) -> Result<RegValue, FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).OPTCR };
        Ok(RegValue::new(value))
    }
    
    pub fn write_optcr(&self, value: RegValue) -> Result<(), FlashError> {
        if self.reg.is_null() {
            return Err(FlashError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).OPTCR = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_optcr<F>(&self, f: F) -> Result<(), FlashError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value: RegValue = self.read_optcr()?;
        let new_value: RegValue = f(value);
        self.write_optcr(new_value)
    }

    // for direct memory operations
    pub fn get_raw_ptr(&self) -> *mut FLASHRegDef {
        self.reg
    }
}

pub struct FlashHandle<'a> {
    pub pflash: *mut FLASHRegDef,
    reg: FlashRegister,
    _marker: PhantomData<&'a ()>,
}

impl<'a> FlashHandle<'a> {
    pub fn new() -> Result<Self, FlashError> {
        let reg: FlashRegister = FlashRegister::new()?;
        
        Ok(FlashHandle {
            pflash: reg.get_raw_ptr(),
            reg,
            _marker: PhantomData,
        })
    }

    pub fn init(&self, block_size: FlashDataSize) -> Result<(), FlashError> {
        // Unlock
        self.reg.write_optkeyr(RegValue::new(FLASH_OPT_KEY1))?;
        self.reg.write_optkeyr(RegValue::new(FLASH_OPT_KEY2))?;
        
        // Set latency
        self.reg.modify_acr(|mut reg: RegValue| {
            // Clear latency bits
            reg.clear_bits(0x7);
            reg.set_bits(FlashLatency::Lat5WS as u32);
            reg
        })?;
        
        // Lock
        self.reg.modify_optcr(|mut reg: RegValue| {
            reg.set_bits(0x1);
            reg
        })?;
        
        // Unlock
        self.unlock()?;
        
        // Set block size
        self.reg.modify_cr(|mut reg: RegValue| {
            // Clear psize bits
            reg.clear_bits(0x3 << 8);
            reg.set_bits((block_size as u32) << 8);
            reg
        })?;
        
        self.lock()?;
        
        Ok(())
    }

    pub fn unlock(&self) -> Result<(), FlashError> {
        // Wait until not busy
        self.wait_for_last_operation()?;
        
        // Check if unlocked
        let cr_val: RegValue = self.reg.read_cr()?;
        if (cr_val.get() & 0x80000000) != 0 {
            // Write unlock sequence
            self.reg.write_keyr(RegValue::new(FLASH_KEY1))?;
            self.reg.write_keyr(RegValue::new(FLASH_KEY2))?;
        }
        
        Ok(())
    }

    pub fn lock(&self) -> Result<(), FlashError> {
        self.reg.modify_cr(|mut reg: RegValue| {
            // Set LOCK bit
            reg.set_bits(0x80000000);
            reg
        })
    }

    pub fn erase_sector(&self, sector_address: u32) -> Result<u32, FlashError> {
        // Wait until not busy
        self.wait_for_last_operation()?;
        
        // Find the sector number
        let mut addr: u32 = FLASH_BASE;
        let mut sector_num: usize = 0;
        
        for i in 0..FLASH_SECTOR_TOTAL {
            if addr == sector_address {
                sector_num = i;
                break;
            } else if addr > sector_address {
                return Err(FlashError::WrongAddress);
            }
            addr += FLASH_SECTORS[i] << 10;
        }
        
        if sector_num == FLASH_SECTOR_TOTAL {
            return Err(FlashError::InvalidSector);
        }
        
        // Unlock
        self.unlock()?;
        
        // Set sector erase
        self.reg.modify_cr(|mut reg: RegValue| {
            // Set SER bit
            reg.set_bits(0x2);
            // Clear SNB bits
            reg.clear_bits(0xF << 3);
            // Set SNB
            reg.set_bits((sector_num as u32) << 3);
            reg
        })?;
        
        // Start erase
        self.reg.modify_cr(|mut reg: RegValue| {
            reg.set_bits(0x10000);
            reg
        })?;
        
        // Wait for completion
        self.wait_for_last_operation()?;
        
        // Clear SER bit
        self.reg.modify_cr(|mut reg: RegValue| {
            reg.clear_bits(0x2);
            reg
        })?;
        
        // Lock
        self.lock()?;
        
        // Return sector size in bytes
        Ok(FLASH_SECTORS[sector_num] << 10)
    }

    pub fn erase(&self, start_address: u32) -> Result<(), FlashError> {
        // Wait until not busy
        self.wait_for_last_operation()?;
        
        // Find the starting sector number
        let mut addr: u32 = FLASH_BASE;
        let mut sector_num: usize = 0;
        
        for i in 0..FLASH_SECTOR_TOTAL {
            if addr == start_address {
                sector_num = i;
                break;
            } else if addr > start_address {
                return Err(FlashError::WrongAddress);
            }
            addr += FLASH_SECTORS[i] << 10;
        }
        
        if sector_num == FLASH_SECTOR_TOTAL {
            return Err(FlashError::InvalidSector);
        }
        
        // Unlock
        self.unlock()?;
        
        // Set sector erase
        self.reg.modify_cr(|mut reg: RegValue| {
            reg.set_bits(0x2);
            reg
        })?;
        
        // Erase all sectors starting from sector_num
        for i in sector_num..FLASH_SECTOR_TOTAL {
            self.reg.modify_cr(|mut reg: RegValue| {
                reg.clear_bits(0xF << 3);
                reg.set_bits((i as u32) << 3);
                reg
            })?;
            
            // Start erase
            self.reg.modify_cr(|mut reg: RegValue| {
                reg.set_bits(0x10000);
                reg
            })?;
            
            // Wait for completion
            self.wait_for_last_operation()?;
        }
        
        // Lock
        self.lock()?;
        
        Ok(())
    }

    pub fn write(&self, source_data: &[u8], destination: u32) -> Result<(), FlashError> {
        // Get block size from CR
        let cr_val: RegValue = self.reg.read_cr()?;
        let psize: u32 = (cr_val.get() >> 8) & 0x3;
        let block_size: u32 = 1 << psize;
        
        // Check if length is a multiple of block size
        if (source_data.len() as u32 & (block_size - 1)) != 0 {
            return Err(FlashError::InvalidConfiguration);
        }
        
        // Unlock
        self.unlock()?;
        
        let mut offset: u32 = 0;
        while offset < source_data.len() as u32 {
            // Set programming bit
            self.reg.modify_cr(|mut reg: RegValue| {
                reg.clear_bits(0x2);
                reg.set_bits(0x1);
                reg
            })?;
            
            // Write data based on block size
            unsafe {
                match block_size {
                    1 => { // 8-bit
                        let dst_ptr: *mut u8 = (destination + offset) as *mut u8;
                        *dst_ptr = source_data[offset as usize];
                    },
                    2 => { // 16-bit
                        let dst_ptr: *mut u16 = (destination + offset) as *mut u16;
                        let src_ptr: *const u16 = source_data.as_ptr().add(offset as usize) as *const u16;
                        *dst_ptr = *src_ptr;
                    },
                    4 => { // 32-bit
                        let dst_ptr: *mut u32 = (destination + offset) as *mut u32;
                        let src_ptr: *const u32 = source_data.as_ptr().add(offset as usize) as *const u32;
                        *dst_ptr = *src_ptr;
                    },
                    8 => { // 64-bit
                        let dst_ptr: *mut u64 = (destination + offset) as *mut u64;
                        let src_ptr: *const u64 = source_data.as_ptr().add(offset as usize) as *const u64;
                        *dst_ptr = *src_ptr;
                    },
                    _ => return Err(FlashError::WriteError),
                }
            }
            
            // Wait for completion
            self.wait_for_last_operation()?;
            
            offset += block_size;
        }
        
        // Lock
        self.lock()?;
        
        Ok(())
    }

    pub fn read(&self, source: u32, destination: &mut [u8]) -> Result<(), FlashError> {
        for i in 0..destination.len() {
            unsafe {
                let src_ptr: *const u8 = (source + i as u32) as *const u8;
                destination[i] = *src_ptr;
            }
        }
        
        Ok(())
    }
    
    fn wait_for_last_operation(&self) -> Result<(), FlashError> {
        let mut timeout: i32 = 50000;
        
        while timeout > 0 {
            let sr_val: RegValue = self.reg.read_sr()?;
            
            if (sr_val.get() & 0x1) == 0 { // BSY bit
                // If there are errors, return appropriate error
                let error_bits: u32 = sr_val.get() & 0xF2;
                if error_bits != 0 {
                    return Err(FlashError::WriteError);
                }
                
                return Ok(());
            }
            
            timeout -= 1;
        }
        
        Err(FlashError::Timeout)
    }
    
    // Get the current Flash data size setting
    pub fn get_data_size(&self) -> Result<FlashDataSize, FlashError> {
        let cr_val: RegValue = self.reg.read_cr()?;
        let psize: u32 = (cr_val.get() >> 8) & 0x3;
        
        match psize {
            0 => Ok(FlashDataSize::Data8Bit),
            1 => Ok(FlashDataSize::Data16Bit),
            2 => Ok(FlashDataSize::Data32Bit),
            3 => Ok(FlashDataSize::Data64Bit),
            _ => Err(FlashError::InvalidConfiguration),
        }
    }
    
    // Get the current Flash latency setting
    pub fn get_latency(&self) -> Result<FlashLatency, FlashError> {
        let acr_val: RegValue = self.reg.read_acr()?;
        let latency: u32 = acr_val.get() & 0x7;
        
        match latency {
            0 => Ok(FlashLatency::Lat0WS),
            1 => Ok(FlashLatency::Lat1WS),
            2 => Ok(FlashLatency::Lat2WS),
            3 => Ok(FlashLatency::Lat3WS),
            4 => Ok(FlashLatency::Lat4WS),
            5 => Ok(FlashLatency::Lat5WS),
            6 => Ok(FlashLatency::Lat6WS),
            7 => Ok(FlashLatency::Lat7WS),
            _ => Err(FlashError::InvalidConfiguration),
        }
    }
    
    // Set Flash latency
    pub fn set_latency(&self, latency: FlashLatency) -> Result<(), FlashError> {
        self.reg.modify_acr(|mut reg: RegValue| {
            reg.clear_bits(0x7);
            reg.set_bits(latency as u32);
            reg
        })
    }
}

pub fn init_flash(block_size: FlashDataSize) -> Result<FlashHandle<'static>, FlashError> {
    let handle = FlashHandle::new()?;
    handle.init(block_size)?;
    Ok(handle)
}