#![allow(non_upper_case_globals)]
#![allow(non_snake_case)]
use core::sync::atomic::{AtomicU32, Ordering};
use core::marker::PhantomData;
use crate::stm32f4xx::{SCS_BASE, RegValue};

// milis counter
static MILLIS: AtomicU32 = AtomicU32::new(0);

// Тип ошибок для SysTick
#[derive(Debug, Clone, Copy)]
pub enum SysTickError {
    InitializationFailed,
    HardwareFault,
}

// SysTick specific registers
#[repr(C)]
pub struct SysTickRegDef {
    pub CTRL: u32,    // Control and Status Register
    pub LOAD: u32,    // Reload Value Register
    pub VAL: u32,     // Current Value Register
    pub CALIB: u32,   // Calibration Value Register
}

// SysTick constants
pub const SysTick_BASE: u32 = SCS_BASE + 0x0010;
pub const SYSTICK_REGS: *mut SysTickRegDef = SysTick_BASE as *mut SysTickRegDef;

pub struct SysTickRegister {
    reg: *mut SysTickRegDef,
}

impl SysTickRegister {
    pub fn new() -> Result<Self, SysTickError> {
        if SYSTICK_REGS.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        Ok(SysTickRegister { reg: SYSTICK_REGS })
    }
    
    // CTRL register
    pub fn read_ctrl(&self) -> Result<RegValue, SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).CTRL };
        Ok(RegValue::new(value))
    }
    
    pub fn write_ctrl(&self, value: RegValue) -> Result<(), SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).CTRL = value.get();
        }
        
        Ok(())
    }
    
    pub fn modify_ctrl<F>(&self, f: F) -> Result<(), SysTickError> 
    where F: FnOnce(RegValue) -> RegValue {
        let value = self.read_ctrl()?;
        let new_value: RegValue = f(value);
        self.write_ctrl(new_value)
    }
    
    // LOAD register
    pub fn read_load(&self) -> Result<RegValue, SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).LOAD };
        Ok(RegValue::new(value))
    }
    
    pub fn write_load(&self, value: RegValue) -> Result<(), SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).LOAD = value.get();
        }
        
        Ok(())
    }
    
    // VAL register
    pub fn read_val(&self) -> Result<RegValue, SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        let value: u32 = unsafe { (*self.reg).VAL };
        Ok(RegValue::new(value))
    }
    
    pub fn write_val(&self, value: RegValue) -> Result<(), SysTickError> {
        if self.reg.is_null() {
            return Err(SysTickError::HardwareFault);
        }
        
        unsafe {
            (*self.reg).VAL = value.get();
        }
        
        Ok(())
    }
    
    // for operations that need direct access
    pub fn get_raw_ptr(&self) -> *mut SysTickRegDef {
        self.reg
    }
}

pub struct SysTickHandle<'a> {
    pub psystick: *mut SysTickRegDef,
    register: SysTickRegister,
    _marker: PhantomData<&'a ()>,
}

impl<'a> SysTickHandle<'a> {
    pub fn new() -> Result<Self, SysTickError> {
        let register: SysTickRegister = SysTickRegister::new()?;
        
        Ok(SysTickHandle {
            psystick: register.get_raw_ptr(),
            register,
            _marker: PhantomData,
        })
    }
    
    pub fn init(&self, system_core_clock: u32) -> Result<(), SysTickError> {
        // Calculate reload value
        let reload_value: u32 = system_core_clock / 1000 - 1;
        
        // Configure SysTick
        self.register.write_load(RegValue::new(reload_value))?;
        self.register.write_val(RegValue::new(0))?;
        
        // Enable SysTick, Enable SysTick Interrupt, Use processor clock
        self.register.write_ctrl(RegValue::new(0x07))?;
        
        Ok(())
    }
    
    pub fn get_tick(&self) -> u32 {
        SysTick::get_tick()
    }
}

pub struct SysTick;

impl SysTick {
    // call this from SysTick handler
    pub fn increment_tick() {
        let current = MILLIS.load(Ordering::Relaxed);
        MILLIS.store(current + 1, Ordering::Relaxed);
    }
    
    pub fn get_tick() -> u32 {
        MILLIS.load(Ordering::Relaxed)
    }
}

pub fn init_systick(system_core_clock: u32) -> Result<SysTickHandle<'static>, SysTickError> {
    let handle = SysTickHandle::new()?;
    handle.init(system_core_clock)?;
    Ok(handle)
}