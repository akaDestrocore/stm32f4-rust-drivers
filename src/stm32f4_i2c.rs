use core::marker::PhantomData;
use crate::stm32f4xx::{
    I2C1, I2C2, I2C3, I2CRegDef, RegValue
};

use crate::stm32f4_rcc::{
    RccHandle, RccRegister, RccError
};
