#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]


// IRQ numbers enum
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(i8)]
pub enum IRQn_Type {
    // Cortex-M4 Processor Exceptions Numbers
    NonMaskableInt_IRQn         = -14,    // Non Maskable Interrupt
    MemoryManagement_IRQn       = -12,    // Memory Management Interrupt
    BusFault_IRQn               = -11,    // Bus Fault Interrupt
    UsageFault_IRQn             = -10,    // Usage Fault Interrupt
    SVCall_IRQn                 = -5,     // SV Call Interrupt
    DebugMonitor_IRQn           = -4,     // Debug Monitor Interrupt
    PendSV_IRQn                 = -2,     // Pend SV Interrupt
    SysTick_IRQn                = -1,     // System Tick Interrupt
    
    // STM32 specific Interrupt Numbers
    WWDG_IRQn                   = 0,      // Window WatchDog Interrupt
    PVD_IRQn                    = 1,      // PVD through EXTI Line detection Interrupt
    TAMP_STAMP_IRQn             = 2,      // Tamper and TimeStamp interrupts through the EXTI line
    RTC_WKUP_IRQn               = 3,      // RTC Wakeup interrupt through the EXTI line
    FLASH_IRQn                  = 4,      // FLASH global Interrupt
    RCC_IRQn                    = 5,      // RCC global Interrupt
    EXTI0_IRQn                  = 6,      // EXTI Line0 Interrupt
    EXTI1_IRQn                  = 7,      // EXTI Line1 Interrupt
    EXTI2_IRQn                  = 8,      // EXTI Line2 Interrupt
    EXTI3_IRQn                  = 9,      // EXTI Line3 Interrupt
    EXTI4_IRQn                  = 10,     // EXTI Line4 Interrupt
    DMA1_Stream0_IRQn           = 11,     // DMA1 Stream 0 global Interrupt
    DMA1_Stream1_IRQn           = 12,     // DMA1 Stream 1 global Interrupt
    DMA1_Stream2_IRQn           = 13,     // DMA1 Stream 2 global Interrupt
    DMA1_Stream3_IRQn           = 14,     // DMA1 Stream 3 global Interrupt
    DMA1_Stream4_IRQn           = 15,     // DMA1 Stream 4 global Interrupt
    DMA1_Stream5_IRQn           = 16,     // DMA1 Stream 5 global Interrupt
    DMA1_Stream6_IRQn           = 17,     // DMA1 Stream 6 global Interrupt
    ADC_IRQn                    = 18,     // ADC1, ADC2 and ADC3 global Interrupts
    CAN1_TX_IRQn                = 19,     // CAN1 TX Interrupt
    CAN1_RX0_IRQn               = 20,     // CAN1 RX0 Interrupt
    CAN1_RX1_IRQn               = 21,     // CAN1 RX1 Interrupt
    CAN1_SCE_IRQn               = 22,     // CAN1 SCE Interrupt
    EXTI9_5_IRQn                = 23,     // External Line[9:5] Interrupts
    TIM1_BRK_TIM9_IRQn          = 24,     // TIM1 Break interrupt and TIM9 global interrupt
    TIM1_UP_TIM10_IRQn          = 25,     // TIM1 Update Interrupt and TIM10 global interrupt
    TIM1_TRG_COM_TIM11_IRQn     = 26,     // TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt
    TIM1_CC_IRQn                = 27,     // TIM1 Capture Compare Interrupt
    TIM2_IRQn                   = 28,     // TIM2 global Interrupt
    TIM3_IRQn                   = 29,     // TIM3 global Interrupt
    TIM4_IRQn                   = 30,     // TIM4 global Interrupt
    I2C1_EV_IRQn                = 31,     // I2C1 Event Interrupt
    I2C1_ER_IRQn                = 32,     // I2C1 Error Interrupt
    I2C2_EV_IRQn                = 33,     // I2C2 Event Interrupt
    I2C2_ER_IRQn                = 34,     // I2C2 Error Interrupt
    SPI1_IRQn                   = 35,     // SPI1 global Interrupt
    SPI2_IRQn                   = 36,     // SPI2 global Interrupt
    USART1_IRQn                 = 37,     // USART1 global Interrupt
    USART2_IRQn                 = 38,     // USART2 global Interrupt
    USART3_IRQn                 = 39,     // USART3 global Interrupt
    EXTI15_10_IRQn              = 40,     // External Line[15:10] Interrupts
    RTC_Alarm_IRQn              = 41,     // RTC Alarm (A and B) through EXTI Line Interrupt
    OTG_FS_WKUP_IRQn            = 42,     // USB OTG FS Wakeup through EXTI line interrupt
    TIM8_BRK_TIM12_IRQn         = 43,     // TIM8 Break Interrupt and TIM12 global interrupt
    TIM8_UP_TIM13_IRQn          = 44,     // TIM8 Update Interrupt and TIM13 global interrupt
    TIM8_TRG_COM_TIM14_IRQn     = 45,     // TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt
    TIM8_CC_IRQn                = 46,     // TIM8 Capture Compare global interrupt
    DMA1_Stream7_IRQn           = 47,     // DMA1 Stream7 Interrupt
    FSMC_IRQn                   = 48,     // FSMC global Interrupt
    SDIO_IRQn                   = 49,     // SDIO global Interrupt
    TIM5_IRQn                   = 50,     // TIM5 global Interrupt
    SPI3_IRQn                   = 51,     // SPI3 global Interrupt
    UART4_IRQn                  = 52,     // UART4 global Interrupt
    UART5_IRQn                  = 53,     // UART5 global Interrupt
    TIM6_DAC_IRQn               = 54,     // TIM6 global and DAC1&2 underrun error interrupts
    TIM7_IRQn                   = 55,     // TIM7 global interrupt
    DMA2_Stream0_IRQn           = 56,     // DMA2 Stream 0 global Interrupt
    DMA2_Stream1_IRQn           = 57,     // DMA2 Stream 1 global Interrupt
    DMA2_Stream2_IRQn           = 58,     // DMA2 Stream 2 global Interrupt
    DMA2_Stream3_IRQn           = 59,     // DMA2 Stream 3 global Interrupt
    DMA2_Stream4_IRQn           = 60,     // DMA2 Stream 4 global Interrupt
    ETH_IRQn                    = 61,     // Ethernet global Interrupt
    ETH_WKUP_IRQn               = 62,     // Ethernet Wakeup through EXTI line Interrupt
    CAN2_TX_IRQn                = 63,     // CAN2 TX Interrupt
    CAN2_RX0_IRQn               = 64,     // CAN2 RX0 Interrupt
    CAN2_RX1_IRQn               = 65,     // CAN2 RX1 Interrupt
    CAN2_SCE_IRQn               = 66,     // CAN2 SCE Interrupt
    OTG_FS_IRQn                 = 67,     // USB OTG FS global Interrupt
    DMA2_Stream5_IRQn           = 68,     // DMA2 Stream 5 global interrupt
    DMA2_Stream6_IRQn           = 69,     // DMA2 Stream 6 global interrupt
    DMA2_Stream7_IRQn           = 70,     // DMA2 Stream 7 global interrupt
    USART6_IRQn                 = 71,     // USART6 global interrupt
    I2C3_EV_IRQn                = 72,     // I2C3 event interrupt
    I2C3_ER_IRQn                = 73,     // I2C3 error interrupt
    OTG_HS_EP1_OUT_IRQn         = 74,     // USB OTG HS End Point 1 Out global interrupt
    OTG_HS_EP1_IN_IRQn          = 75,     // USB OTG HS End Point 1 In global interrupt
    OTG_HS_WKUP_IRQn            = 76,     // USB OTG HS Wakeup through EXTI interrupt
    OTG_HS_IRQn                 = 77,     // USB OTG HS global interrupt
    DCMI_IRQn                   = 78,     // DCMI global interrupt
    RNG_IRQn                    = 80,     // RNG global Interrupt
    FPU_IRQn                    = 81,     // FPU global interrupt
}

// STM32F4XX base addresses of flash and SRAM memories
pub const FLASH_BASE        : u32 = 0x08000000; // FLASHup to 1 MB) base address in the alias region                       
pub const CCMDATARAM_BASE   : u32 = 0x10000000; // CCMcore coupled memory) data RAM64 KB) base address in the alias region
pub const SRAM1_BASE        : u32 = 0x20000000; // SRAM1112 KB) base address in the alias region                           
pub const SRAM2_BASE        : u32 = 0x2001C000; // SRAM216 KB) base address in the alias region                            
pub const PERIPH_BASE       : u32 = 0x40000000; // Peripheral base address in the alias region                              
pub const BKPSRAM_BASE      : u32 = 0x40024000; // Backup SRAM4 KB) base address in the alias region                       
pub const FSMC_R_BASE       : u32 = 0xA0000000; // FSMC registers base address                                              
pub const SRAM1_BB_BASE     : u32 = 0x22000000; // SRAM1112 KB) base address in the bit-band region                        
pub const SRAM2_BB_BASE     : u32 = 0x22380000; // SRAM216 KB) base address in the bit-band region                         
pub const PERIPH_BB_BASE    : u32 = 0x42000000; // Peripheral base address in the bit-band region                           
pub const BKPSRAM_BB_BASE   : u32 = 0x42480000; // Backup SRAM4 KB) base address in the bit-band region                    
pub const FLASH_END         : u32 = 0x080FFFFF; // FLASH end address                                                        
pub const FLASH_OTP_BASE    : u32 = 0x1FFF7800; // Base address of :up to 528 Bytes) embedded FLASH OTP Area              
pub const FLASH_OTP_END     : u32 = 0x1FFF7A0F; // End address of :up to 528 Bytes) embedded FLASH OTP Area               
pub const CCMDATARAM_END    : u32 = 0x1000FFFF; // CCM data RAM end address                                                 

// STM32F4XX AHBx and APBx Bus Peripheral base addresses
pub const APB1PERIPH_BASE   : u32 = PERIPH_BASE;
pub const APB2PERIPH_BASE   : u32 = PERIPH_BASE + 0x00010000;
pub const AHB1PERIPH_BASE   : u32 = PERIPH_BASE + 0x00020000;
pub const AHB2PERIPH_BASE   : u32 = PERIPH_BASE + 0x10000000;

// STM32F4XX base addresses of peripherals which are hanging on AHB1 bus
pub const GPIOA_BASE        : u32 = AHB1PERIPH_BASE + 0x0000;
pub const GPIOB_BASE        : u32 = AHB1PERIPH_BASE + 0x0400;
pub const GPIOC_BASE        : u32 = AHB1PERIPH_BASE + 0x0800;
pub const GPIOD_BASE        : u32 = AHB1PERIPH_BASE + 0x0C00;
pub const GPIOE_BASE        : u32 = AHB1PERIPH_BASE + 0x1000;
pub const GPIOF_BASE        : u32 = AHB1PERIPH_BASE + 0x1400;
pub const GPIOG_BASE        : u32 = AHB1PERIPH_BASE + 0x1800;
pub const GPIOH_BASE        : u32 = AHB1PERIPH_BASE + 0x1C00;
pub const GPIOI_BASE        : u32 = AHB1PERIPH_BASE + 0x2000;
pub const CRC_BASE          : u32 = AHB1PERIPH_BASE + 0x3000;
pub const RCC_BASE          : u32 = AHB1PERIPH_BASE + 0x3800;
pub const FLASH_R_BASE      : u32 = AHB1PERIPH_BASE + 0x3C00;
pub const DMA1_BASE         : u32 = AHB1PERIPH_BASE + 0x6000;
pub const DMA1_Stream0_BASE : u32 = DMA1_BASE + 0x010;
pub const DMA1_Stream1_BASE : u32 = DMA1_BASE + 0x028;
pub const DMA1_Stream2_BASE : u32 = DMA1_BASE + 0x040;
pub const DMA1_Stream3_BASE : u32 = DMA1_BASE + 0x058;
pub const DMA1_Stream4_BASE : u32 = DMA1_BASE + 0x070;
pub const DMA1_Stream5_BASE : u32 = DMA1_BASE + 0x088;
pub const DMA1_Stream6_BASE : u32 = DMA1_BASE + 0x0A0;
pub const DMA1_Stream7_BASE : u32 = DMA1_BASE + 0x0B8;
pub const DMA2_BASE         : u32 = AHB1PERIPH_BASE + 0x6400;
pub const DMA2_Stream0_BASE : u32 = DMA2_BASE + 0x010;
pub const DMA2_Stream1_BASE : u32 = DMA2_BASE + 0x028;
pub const DMA2_Stream2_BASE : u32 = DMA2_BASE + 0x040;
pub const DMA2_Stream3_BASE : u32 = DMA2_BASE + 0x058;
pub const DMA2_Stream4_BASE : u32 = DMA2_BASE + 0x070;
pub const DMA2_Stream5_BASE : u32 = DMA2_BASE + 0x088;
pub const DMA2_Stream6_BASE : u32 = DMA2_BASE + 0x0A0;
pub const DMA2_Stream7_BASE : u32 = DMA2_BASE + 0x0B8;
pub const ETH_BASE          : u32 = AHB1PERIPH_BASE + 0x8000;
pub const ETH_MAC_BASE      : u32 = ETH_BASE;
pub const ETH_MMC_BASE      : u32 = ETH_BASE + 0x0100;
pub const ETH_PTP_BASE      : u32 = ETH_BASE + 0x0700;
pub const ETH_DMA_BASE      : u32 = ETH_BASE + 0x1000;

// STM32F4XX base addresses of peripherals which are hanging on AHB2 bus
pub const DCMI_BASE         : u32 = AHB2PERIPH_BASE + 0x50000;
pub const RNG_BASE          : u32 = AHB2PERIPH_BASE + 0x60800;

// STM32F4XX base addresses of peripherals which are hanging on APB1 bus
pub const TIM2_BASE         : u32 = APB1PERIPH_BASE + 0x0000;
pub const TIM3_BASE         : u32 = APB1PERIPH_BASE + 0x0400;
pub const TIM4_BASE         : u32 = APB1PERIPH_BASE + 0x0800;
pub const TIM5_BASE         : u32 = APB1PERIPH_BASE + 0x0C00;
pub const TIM6_BASE         : u32 = APB1PERIPH_BASE + 0x1000;
pub const TIM7_BASE         : u32 = APB1PERIPH_BASE + 0x1400;
pub const TIM12_BASE        : u32 = APB1PERIPH_BASE + 0x1800;
pub const TIM13_BASE        : u32 = APB1PERIPH_BASE + 0x1C00;
pub const TIM14_BASE        : u32 = APB1PERIPH_BASE + 0x2000;
pub const RTC_BASE          : u32 = APB1PERIPH_BASE + 0x2800;
pub const WWDG_BASE         : u32 = APB1PERIPH_BASE + 0x2C00;
pub const IWDG_BASE         : u32 = APB1PERIPH_BASE + 0x3000;
pub const I2S2ext_BASE      : u32 = APB1PERIPH_BASE + 0x3400;
pub const SPI2_BASE         : u32 = APB1PERIPH_BASE + 0x3800;
pub const SPI3_BASE         : u32 = APB1PERIPH_BASE + 0x3C00;
pub const I2S3ext_BASE      : u32 = APB1PERIPH_BASE + 0x4000;
pub const USART2_BASE       : u32 = APB1PERIPH_BASE + 0x4400;
pub const USART3_BASE       : u32 = APB1PERIPH_BASE + 0x4800;
pub const UART4_BASE        : u32 = APB1PERIPH_BASE + 0x4C00;
pub const UART5_BASE        : u32 = APB1PERIPH_BASE + 0x5000;
pub const I2C1_BASE         : u32 = APB1PERIPH_BASE + 0x5400;
pub const I2C2_BASE         : u32 = APB1PERIPH_BASE + 0x5800;
pub const I2C3_BASE         : u32 = APB1PERIPH_BASE + 0x5C00;
pub const CAN1_BASE         : u32 = APB1PERIPH_BASE + 0x6400;
pub const CAN2_BASE         : u32 = APB1PERIPH_BASE + 0x6800;
pub const PWR_BASE          : u32 = APB1PERIPH_BASE + 0x7000;
pub const DAC_BASE          : u32 = APB1PERIPH_BASE + 0x7400;

// STM32F4XX base addresses of peripherals which are hanging on APB2 bus
pub const TIM1_BASE         : u32 = APB2PERIPH_BASE + 0x0000;
pub const TIM8_BASE         : u32 = APB2PERIPH_BASE + 0x0400;
pub const USART1_BASE       : u32 = APB2PERIPH_BASE + 0x1000;
pub const USART6_BASE       : u32 = APB2PERIPH_BASE + 0x1400;
pub const ADC1_BASE         : u32 = APB2PERIPH_BASE + 0x2000;
pub const ADC2_BASE         : u32 = APB2PERIPH_BASE + 0x2100;
pub const ADC3_BASE         : u32 = APB2PERIPH_BASE + 0x2200;
pub const ADC123_COMMON_BASE : u32 = APB2PERIPH_BASE + 0x2300;
pub const ADC_BASE          : u32 = ADC123_COMMON_BASE;
pub const SDIO_BASE         : u32 = APB2PERIPH_BASE + 0x2C00;
pub const SPI1_BASE         : u32 = APB2PERIPH_BASE + 0x3000;
pub const SYSCFG_BASE       : u32 = APB2PERIPH_BASE + 0x3800;
pub const EXTI_BASE         : u32 = APB2PERIPH_BASE + 0x3C00;
pub const TIM9_BASE         : u32 = APB2PERIPH_BASE + 0x4000;
pub const TIM10_BASE        : u32 = APB2PERIPH_BASE + 0x4400;
pub const TIM11_BASE        : u32 = APB2PERIPH_BASE + 0x4800;

// FSMC BANKx registers base address
pub const FSMCBank1_R_BASE         : u32 = FSMC_R_BASE + 0x0000;
pub const FSMCBank1E_R_BASE        : u32 = FSMC_R_BASE + 0x0104;
pub const FSMCBank2_3_R_BASE       : u32 = FSMC_R_BASE + 0x0060;
pub const FSMCBank4_R_BASE         : u32 = FSMC_R_BASE + 0x00A0;

// Debug MCU registers base address
pub const DBGMCU_BASE               : u32 = 0xE0042000;

// USB registers base address
pub const USB_OTG_HS_PERIPH_BASE    : u32 = 0x40040000;
pub const USB_OTG_FS_PERIPH_BASE    : u32 = 0x50000000;

pub const USB_OTG_GLOBAL_BASE       : u32 = 0x000;
pub const USB_OTG_DEVICE_BASE       : u32 = 0x800;
pub const USB_OTG_IN_ENDPOINT_BASE  : u32 = 0x900;
pub const USB_OTG_OUT_ENDPOINT_BASE : u32 = 0xB00;
pub const USB_OTG_EP_REG_SIZE       : u32 = 0x20;
pub const USB_OTG_HOST_BASE         : u32 = 0x400;
pub const USB_OTG_HOST_PORT_BASE    : u32 = 0x440;
pub const USB_OTG_HOST_CHANNEL_BASE : u32 = 0x500;
pub const USB_OTG_HOST_CHANNEL_SIZE : u32 = 0x20;
pub const USB_OTG_PCGCCTL_BASE      : u32 = 0xE00;
pub const USB_OTG_FIFO_BASE         : u32 = 0x1000;
pub const USB_OTG_FIFO_SIZE         : u32 = 0x1000;

pub const UID_BASE                  : u32 = 0x1FFF7A10;     // Unique device ID register base address
pub const FLASHSIZE_BASE            : u32 = 0x1FFF7A22;     // FLASH Size register base address      
pub const PACKAGE_BASE              : u32 = 0x1FFF7BF0;     // Package size register base address    

// Memory mapping of Core Hardware
pub const SCS_BASE          : u32 = 0xE000E000;                             // System Control Space Base Address
pub const ITM_BASE          : u32 = 0xE0000000;                             // ITM Base Address
pub const DWT_BASE          : u32 = 0xE0001000;                             // DWT Base Address
pub const TPI_BASE          : u32 = 0xE0040000;                             // TPI Base Address
pub const CoreDebug_BASE    : u32 = 0xE000EDF0;                             // Core Debug Base Address
pub const SysTick_BASE      : u32 = SCS_BASE + 0x0010;                      // SysTick Base Address
pub const NVIC_BASE         : u32 = SCS_BASE + 0x0100;                      // NVIC Base Address
pub const SCB_BASE          : u32 = SCS_BASE + 0x0D00;                      // System Control Block Base Address

#[repr(C)]
pub struct TIM_2_5_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub SMCR        : u32,          // Slave mode control register     Address offset: 0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub CCMR1       : u32,          // Capture/compare mode register 1 Address offset: 0x18
    pub CCMR2       : u32,          // Capture/compare mode register 2 Address offset: 0x1C
    pub CCER        : u32,          // Capture/compare enable register Address offset: 0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
    pub RCR         : u32,          // Repetition counter register     Address offset: 0x30
    pub CCR1        : u32,          // Capture/compare register 1      Address offset: 0x34
    pub CCR2        : u32,          // Capture/compare register 2      Address offset: 0x38
    pub CCR3        : u32,          // Capture/compare register 3      Address offset: 0x3C
    pub CCR4        : u32,          // Capture/compare register 4      Address offset: 0x40
    pub BDTR        : u32,          // Break and dead-time register    Address offset: 0x44
    pub DCR         : u32,          // DMA control register            Address offset: 0x48
    pub DMAR        : u32,          // DMA address for full transfer   Address offset: 0x4C
    pub OR          : u32,          // Option register                 Address offset: 0x50
}

#[repr(C)]
pub struct TIM_1_8_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub SMCR        : u32,          // Slave mode control register     Address offset: 0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub CCMR1       : u32,          // Capture/compare mode register 1 Address offset: 0x18
    pub CCMR2       : u32,          // Capture/compare mode register 2 Address offset: 0x1C
    pub CCER        : u32,          // Capture/compare enable register Address offset: 0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
    pub RCR         : u32,          // Repetition counter register     Address offset: 0x30
    pub CCR1        : u32,          // Capture/compare register 1      Address offset: 0x34
    pub CCR2        : u32,          // Capture/compare register 2      Address offset: 0x38
    pub CCR3        : u32,          // Capture/compare register 3      Address offset: 0x3C
    pub CCR4        : u32,          // Capture/compare register 4      Address offset: 0x40
    pub BDTR        : u32,          // Break and dead-time register    Address offset: 0x44
    pub DCR         : u32,          // DMA control register            Address offset: 0x48
    pub DMAR        : u32,          // DMA address for full transfer   Address offset: 0x4C
}

#[repr(C)]
pub struct TIM_6_7_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub RESERVED0   : u32,          // Reserved                        Address offset: 0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub RESERVED1   : [u32; 3],     // Reserved                   Address offset: 0x18-0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
}

#[repr(C)]
pub struct TIM_9_12_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub SMCR        : u32,          // Slave mode control register     Address offset: 0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub CCMR1       : u32,          // Capture/compare mode register 1 Address offset: 0x18
    pub RESERVED0   : u32,          // Reserved                        Address offset: 0x1C
    pub CCER        : u32,          // Capture/compare enable register Address offset: 0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
    pub RESERVED1   : u32,          // Reserved                        Address offset: 0x30
    pub CCR1        : u32,          // Capture/compare register 1      Address offset: 0x34
    pub CCR2        : u32,          // Capture/compare register 2      Address offset: 0x38
}

#[repr(C)]
pub struct TIM_10_14_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub RESERVED0   : [u32; 2],     // Reserved                   Address offset: 0x04-0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub CCMR1       : u32,          // Capture/compare mode register 1 Address offset: 0x18
    pub RESERVED1   : u32,          // Reserved                        Address offset: 0x1C
    pub CCER        : u32,          // Capture/compare enable register Address offset: 0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
    pub RESERVED2   : u32,          // Reserved                        Address offset: 0x30
    pub CCR1        : u32,          // Capture/compare register 1      Address offset: 0x34
}

#[repr(C)]
pub struct TIM_11_RegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub RESERVED0   : [u32; 2],     // Reserved                   Address offset: 0x04-0x08
    pub DIER        : u32,          // DMA/Interrupt enable register   Address offset: 0x0C
    pub SR          : u32,          // Status register                 Address offset: 0x10
    pub EGR         : u32,          // Event generation register       Address offset: 0x14
    pub CCMR1       : u32,          // Capture/compare mode register 1 Address offset: 0x18
    pub RESERVED1   : u32,          // Reserved                        Address offset: 0x1C
    pub CCER        : u32,          // Capture/compare enable register Address offset: 0x20
    pub CNT         : u32,          // Counter                         Address offset: 0x24
    pub PSC         : u32,          // Prescaler                       Address offset: 0x28
    pub ARR         : u32,          // Auto-reload register            Address offset: 0x2C
    pub RESERVED2   : u32,          // Reserved                        Address offset: 0x30
    pub CCR1        : u32,          // Capture/compare register 1      Address offset: 0x34
    pub OR          : u32,          // Option register                 Address offset: 0x50
}

#[repr(C)]
pub struct RTCRegDef {
    pub TR          : u32,          // Time register                   Address offset: 0x00
    pub DR          : u32,          // Date register                   Address offset: 0x04
    pub CR          : u32,          // Control register                Address offset: 0x08
    pub ISR         : u32,          // Initialization and status register Address offset: 0x0C
    pub PRER        : u32,          // Prescaler register              Address offset: 0x10
    pub WUTR        : u32,          // Wakeup timer register           Address offset: 0x14
    pub CALIBR      : u32,          // Calibration register            Address offset: 0x18
    pub ALRMAR      : u32,          // Alarm A register                Address offset: 0x1C
    pub ALRMBR      : u32,          // Alarm B register                Address offset: 0x20
    pub WPR         : u32,          // Write protection register       Address offset: 0x24
    pub SSR         : u32,          // Sub second register             Address offset: 0x28
    pub SHIFTR      : u32,          // Shift control register          Address offset: 0x2C
    pub TSTR        : u32,          // Time stamp time register        Address offset: 0x30
    pub TSDR        : u32,          // Time stamp date register        Address offset: 0x34
    pub TSSSR       : u32,          // Timestamp sub second register   Address offset: 0x38
    pub CALR        : u32,          // Calibration register            Address offset: 0x3C
    pub TAFCR       : u32,          // Tamper and alternate function configuration register Address offset: 0x40
    pub ALRMASSR    : u32,          // Alarm A sub second register     Address offset: 0x44
    pub ALRMBSSR    : u32,          // Alarm B sub second register     Address offset: 0x48
    pub RESERVED    : u32,          // Reserved                         Address offset: 0x4C
    pub BKP0R       : u32,          // Backup register 0               Address offset: 0x50
    pub BKP1R       : u32,          // Backup register 1               Address offset: 0x54
    pub BKP2R       : u32,          // Backup register 2               Address offset: 0x58
    pub BKP3R       : u32,          // Backup register 3               Address offset: 0x5C
    pub BKP4R       : u32,          // Backup register 4               Address offset: 0x60
    pub BKP5R       : u32,          // Backup register 5               Address offset: 0x64
    pub BKP6R       : u32,          // Backup register 6               Address offset: 0x68
    pub BKP7R       : u32,          // Backup register 7               Address offset: 0x6C
    pub BKP8R       : u32,          // Backup register 8               Address offset: 0x70
    pub BKP9R       : u32,          // Backup register 9               Address offset: 0x74
    pub BKP10R      : u32,          // Backup register 10              Address offset: 0x78
    pub BKP11R      : u32,          // Backup register 11              Address offset: 0x7C
    pub BKP12R      : u32,          // Backup register 12              Address offset: 0x80
    pub BKP13R      : u32,          // Backup register 13              Address offset: 0x84
    pub BKP14R      : u32,          // Backup register 14              Address offset: 0x88
    pub BKP15R      : u32,          // Backup register 15              Address offset: 0x8C
    pub BKP16R      : u32,          // Backup register 16              Address offset: 0x90
    pub BKP17R      : u32,          // Backup register 17              Address offset: 0x94
    pub BKP18R      : u32,          // Backup register 18              Address offset: 0x98
    pub BKP19R      : u32,          // Backup register 19              Address offset: 0x9C
}

#[repr(C)]
pub struct WWDGRegDef {
    pub CR          : u32,          // Control register                Address offset: 0x00
    pub CFR         : u32,          // Configuration register          Address offset: 0x04
    pub SR          : u32,          // Status register                 Address offset: 0x08
}

#[repr(C)]
pub struct IWDGRegDef {
    pub KR          : u32,          // Key register                    Address offset: 0x00
    pub PR          : u32,          // Prescaler register              Address offset: 0x04
    pub RLR         : u32,          // Reload register                 Address offset: 0x08
    pub SR          : u32,          // Status register                 Address offset: 0x0C
}

#[repr(C)]
pub struct SPIRegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub SR          : u32,          // Status register                 Address offset: 0x08
    pub DR          : u32,          // Data register                   Address offset: 0x0C
    pub CRCPR       : u32,          // CRC polynomial register         Address offset: 0x10
    pub RXCRCR      : u32,          // RX CRC register                 Address offset: 0x14
    pub TXCRCR      : u32,          // TX CRC register                 Address offset: 0x18
    pub I2SCFGR     : u32,          // I2S configuration register      Address offset: 0x1C
    pub I2SPR       : u32,          // I2S prescaler register          Address offset: 0x20
}

#[repr(C)]
pub struct USARTRegDef {
pub SR          : u32,              // Status register                 Address offset: 0x00
    pub DR          : u32,          // Data register                   Address offset: 0x04
    pub BRR         : u32,          // Baud rate register              Address offset: 0x08
    pub CR1         : u32,          // Control register 1              Address offset: 0x0C
    pub CR2         : u32,          // Control register 2              Address offset: 0x10
    pub CR3         : u32,          // Control register 3              Address offset: 0x14
    pub GTPR        : u32,          // Guard time and prescaler register Address offset: 0x18
}

#[repr(C)]
pub struct I2CRegDef {
    pub CR1         : u32,          // Control register 1              Address offset: 0x00
    pub CR2         : u32,          // Control register 2              Address offset: 0x04
    pub OAR1        : u32,          // Own address register 1          Address offset: 0x08
    pub OAR2        : u32,          // Own address register 2          Address offset: 0x0C
    pub DR          : u32,          // Data register                   Address offset: 0x10
    pub SR1         : u32,          // Status register 1               Address offset: 0x14
    pub SR2         : u32,          // Status register 2               Address offset: 0x18
    pub CCR         : u32,          // Clock control register          Address offset: 0x1C
    pub TRISE       : u32,          // TRISE register                  Address offset: 0x20
    pub FLTR        : u32,          // FLTR register                   Address offset: 0x24
}

#[repr(C)]
pub struct CANRegDef {
    pub MCR         : u32,       // CAN master control register           Address offset: 0x00
    pub MSR         : u32,       // CAN master status register            Address offset: 0x04
    pub TSR         : u32,       // CAN transmit status register          Address offset: 0x08
    pub RF0R        : u32,       // CAN receive FIFO 0 register           Address offset: 0x0C
    pub RF1R        : u32,       // CAN receive FIFO 1 register           Address offset: 0x10
    pub IER         : u32,       // CAN interrupt enable register         Address offset: 0x14
    pub ESR         : u32,       // CAN error status register             Address offset: 0x18
    pub BTR         : u32,       // CAN bit timing register               Address offset: 0x1C
    pub RESERVED0   : [u32; 88], // Reserved                              Address offset: 0x20-0x17F
    pub TI0R        : u32,       // CAN TX mailbox identifier register 0  Address offset: 0x180
    pub TDT0R       : u32,       // CAN TX mailbox data length control and time stamp register 0  Address offset: 0x184
    pub TDL0R       : u32,       // CAN TX mailbox data low register 0    Address offset: 0x188
    pub TDH0R       : u32,       // CAN TX mailbox data high register 0   Address offset: 0x18C
    pub TI1R        : u32,       // CAN TX mailbox identifier register 1  Address offset: 0x190
    pub TDT1R       : u32,       // CAN TX mailbox data length control and time stamp register 1  Address offset: 0x194
    pub TDL1R       : u32,       // CAN TX mailbox data low register 1    Address offset: 0x198
    pub TDH1R       : u32,       // CAN TX mailbox data high register 1   Address offset: 0x19C
    pub TI2R        : u32,       // CAN TX mailbox identifier register 2  Address offset: 0x1A0
    pub TDT2R       : u32,       // CAN TX mailbox data length control and time stamp register 2  Address offset: 0x1A4
    pub TDL2R       : u32,       // CAN TX mailbox data low register 2    Address offset: 0x1A8
    pub TDH2R       : u32,       // CAN TX mailbox data high register 2   Address offset: 0x1AC
    pub RI0R        : u32,       // CAN receive FIFO mailbox identifier register 0  Address offset: 0x1B0
    pub RDT0R       : u32,       // CAN receive FIFO mailbox data length control and time stamp register 0  Address offset: 0x1B4
    pub RDL0R       : u32,       // CAN receive FIFO mailbox data low register 0    Address offset: 0x1B8
    pub RDH0R       : u32,       // CAN receive FIFO mailbox data high register 0   Address offset: 0x1BC
    pub RI1R        : u32,       // CAN receive FIFO mailbox identifier register 1  Address offset: 0x1C0
    pub RDT1R       : u32,       // CAN receive FIFO mailbox data length control and time stamp register 1  Address offset: 0x1C4
    pub RDL1R       : u32,       // CAN receive FIFO mailbox data low register 1    Address offset: 0x1C8
    pub RDH1R       : u32,       // CAN receive FIFO mailbox data high register 1   Address offset: 0x1CC
    pub RESERVED1   : [u32; 12], // Reserved                              Address offset: 0x1D0-0x1FF
    pub FMR         : u32,       // CAN filter master register            Address offset: 0x200
    pub FM1R        : u32,       // CAN filter mode register              Address offset: 0x204
    pub FS1R        : u32,       // CAN filter scale register             Address offset: 0x208
    pub FFA1R       : u32,       // CAN filter FIFO assignment register   Address offset: 0x20C
    pub FA1R        : u32,       // CAN filter activation register        Address offset: 0x210
    pub RESERVED2   : [u32; 8],  // Reserved                              Address offset: 0x214-0x233
    pub F0R1        : u32,       // CAN Filter bank 0 register 1          Address offset: 0x240
    pub F0R2        : u32,       // CAN Filter bank 0 register 2          Address offset: 0x244
    pub F1R1        : u32,       // CAN Filter bank 1 register 1          Address offset: 0x248
    pub F1R2        : u32,       // CAN Filter bank 1 register 2          Address offset: 0x24C
    pub F2R1        : u32,       // CAN Filter bank 2 register 1          Address offset: 0x250
    pub F2R2        : u32,       // CAN Filter bank 2 register 2          Address offset: 0x254
    pub F3R1        : u32,       // CAN Filter bank 3 register 1          Address offset: 0x258
    pub F3R2        : u32,       // CAN Filter bank 3 register 2          Address offset: 0x25C
    pub F4R1        : u32,       // CAN Filter bank 4 register 1          Address offset: 0x260
    pub F4R2        : u32,       // CAN Filter bank 4 register 2          Address offset: 0x264
    pub F5R1        : u32,       // CAN Filter bank 5 register 1          Address offset: 0x268
    pub F5R2        : u32,       // CAN Filter bank 5 register 2          Address offset: 0x26C
    pub F6R1        : u32,       // CAN Filter bank 6 register 1          Address offset: 0x270
    pub F6R2        : u32,       // CAN Filter bank 6 register 2          Address offset: 0x274
    pub F7R1        : u32,       // CAN Filter bank 7 register 1          Address offset: 0x278
    pub F7R2        : u32,       // CAN Filter bank 7 register 2          Address offset: 0x27C
    pub F8R1        : u32,       // CAN Filter bank 8 register 1          Address offset: 0x280
    pub F8R2        : u32,       // CAN Filter bank 8 register 2          Address offset: 0x284
    pub F9R1        : u32,       // CAN Filter bank 9 register 1          Address offset: 0x288
    pub F9R2        : u32,       // CAN Filter bank 9 register 2          Address offset: 0x28C
    pub F10R1       : u32,       // CAN Filter bank 10 register 1         Address offset: 0x290
    pub F10R2       : u32,       // CAN Filter bank 10 register 2         Address offset: 0x294
    pub F11R1       : u32,       // CAN Filter bank 11 register 1         Address offset: 0x298
    pub F11R2       : u32,       // CAN Filter bank 11 register 2         Address offset: 0x29C
    pub F12R1       : u32,       // CAN Filter bank 12 register 1         Address offset: 0x2A0
    pub F12R2       : u32,       // CAN Filter bank 12 register 2         Address offset: 0x2A4
    pub F13R1       : u32,       // CAN Filter bank 13 register 1         Address offset: 0x2A8
    pub F13R2       : u32,       // CAN Filter bank 13 register 2         Address offset: 0x2AC
    pub F14R1       : u32,       // CAN Filter bank 14 register 1         Address offset: 0x2B0
    pub F14R2       : u32,       // CAN Filter bank 14 register 2         Address offset: 0x2B4
    pub F15R1       : u32,       // CAN Filter bank 15 register 1         Address offset: 0x2B8
    pub F15R2       : u32,       // CAN Filter bank 15 register 2         Address offset: 0x2BC
    pub F16R1       : u32,       // CAN Filter bank 16 register 1         Address offset: 0x2C0
    pub F16R2       : u32,       // CAN Filter bank 16 register 2         Address offset: 0x2C4
    pub F17R1       : u32,       // CAN Filter bank 17 register 1         Address offset: 0x2C8
    pub F17R2       : u32,       // CAN Filter bank 17 register 2         Address offset: 0x2CC
    pub F18R1       : u32,       // CAN Filter bank 18 register 1         Address offset: 0x2D0
    pub F18R2       : u32,       // CAN Filter bank 18 register 2         Address offset: 0x2D4
    pub F19R1       : u32,       // CAN Filter bank 19 register 1         Address offset: 0x2D8
    pub F19R2       : u32,       // CAN Filter bank 19 register 2         Address offset: 0x2DC
    pub F20R1       : u32,       // CAN Filter bank 20 register 1         Address offset: 0x2E0
    pub F20R2       : u32,       // CAN Filter bank 20 register 2         Address offset: 0x2E4
    pub F21R1       : u32,       // CAN Filter bank 21 register 1         Address offset: 0x2E8
    pub F21R2       : u32,       // CAN Filter bank 21 register 2         Address offset: 0x2EC
    pub F22R1       : u32,       // CAN Filter bank 22 register 1         Address offset: 0x2F0
    pub F22R2       : u32,       // CAN Filter bank 22 register 2         Address offset: 0x2F4
    pub F23R1       : u32,       // CAN Filter bank 23 register 1         Address offset: 0x2F8
    pub F23R2       : u32,       // CAN Filter bank 23 register 2         Address offset: 0x2FC
    pub F24R1       : u32,       // CAN Filter bank 24 register 1         Address offset: 0x300
    pub F24R2       : u32,       // CAN Filter bank 24 register 2         Address offset: 0x304
    pub F25R1       : u32,       // CAN Filter bank 25 register 1         Address offset: 0x308
    pub F25R2       : u32,       // CAN Filter bank 25 register 2         Address offset: 0x30C
    pub F26R1       : u32,       // CAN Filter bank 26 register 1         Address offset: 0x310
    pub F26R2       : u32,       // CAN Filter bank 26 register 2         Address offset: 0x314
    pub F27R1       : u32,       // CAN Filter bank 27 register 1         Address offset: 0x318
    pub F27R2       : u32,       // CAN Filter bank 27 register 2         Address offset: 0x31C
}

#[repr(C)]
pub struct PWRRegDef {
    pub CR          : u32,        // Power control register          Address offset: 0x00
    pub CSR         : u32,        // Power control/status register   Address offset: 0x04
}

#[repr(C)]
pub struct DACRegDef {
    pub CR          : u32,      // Control register                Address offset: 0x00
    pub SWTRIGR     : u32,      // Software trigger register       Address offset: 0x04
    pub DHR12R1     : u32,      // Channel1 12-bit right-aligned data holding register Address offset: 0x08
    pub DHR12L1     : u32,      // Channel1 12-bit left-aligned data holding register Address offset: 0x0C
    pub DHR8R1      : u32,      // Channel1 8-bit right-aligned data holding register Address offset: 0x10
    pub DHR12R2     : u32,      // Channel2 12-bit right-aligned data holding register Address offset: 0x14
    pub DHR12L2     : u32,      // Channel2 12-bit left-aligned data holding register Address offset: 0x18
    pub DHR8R2      : u32,      // Channel2 8-bit right-aligned data holding register Address offset: 0x1C
    pub DHR12RD     : u32,      // Dual 12-bit right-aligned data holding register Address offset: 0x20
    pub DHR12LD     : u32,      // Dual 12-bit left-aligned data holding register Address offset: 0x24
    pub DHR8RD      : u32,      // Dual 8-bit right-aligned data holding register Address offset: 0x28
    pub DOR1        : u32,      // Channel1 data output register   Address offset: 0x2C
    pub DOR2        : u32,      // Channel2 data output register   Address offset: 0x30
    pub SR          : u32,      // Status register                 Address offset: 0x34
}

#[repr(C)]
pub struct ADCRegDef {
    pub SR          : u32,      // Status register                 Address offset: 0x00
    pub CR1         : u32,      // Control register 1              Address offset: 0x04
    pub CR2         : u32,      // Control register 2              Address offset: 0x08
    pub SMPR1       : u32,      // Sample time register 1          Address offset: 0x0C
    pub SMPR2       : u32,      // Sample time register 2          Address offset: 0x10
    pub JOFR1       : u32,      // Injected channel data offset register 1 Address offset: 0x14
    pub JOFR2       : u32,      // Injected channel data offset register 2 Address offset: 0x18
    pub JOFR3       : u32,      // Injected channel data offset register 3 Address offset: 0x1C
    pub JOFR4       : u32,      // Injected channel data offset register 4 Address offset: 0x20
    pub HTR         : u32,      // Watchdog higher threshold register Address offset: 0x24
    pub LTR         : u32,      // Watchdog lower threshold register Address offset: 0x28
    pub SQR1        : u32,      // Regular sequence register 1     Address offset: 0x2C
    pub SQR2        : u32,      // Regular sequence register 2     Address offset: 0x30
    pub SQR3        : u32,      // Regular sequence register 3     Address offset: 0x34
    pub JSQR        : u32,      // Injected sequence register      Address offset: 0x38
    pub JDR1        : u32,      // Injected data register 1        Address offset: 0x3C
    pub JDR2        : u32,      // Injected data register 2        Address offset: 0x40
    pub JDR3        : u32,      // Injected data register 3        Address offset: 0x44
    pub JDR4        : u32,      // Injected data register 4        Address offset: 0x48
    pub DR          : u32,      // Regular data register           Address offset: 0x4C
}

#[repr(C)]
pub struct ADCCommonRegDef {
    pub CSR         : u32,       // Common status register          Address offset: 0x00
    pub CCR         : u32,       // Common control register         Address offset: 0x04
    pub CDR         : u32,       // Common regular data register for dual and triple modes Address offset: 0x08
}

#[repr(C)]
pub struct SDIORegDef {
    pub POWER       : u32,        // Power control register         Address offset: 0x00
    pub CLKCR       : u32,        // Clock control register         Address offset: 0x04
    pub ARG         : u32,        // Argument register              Address offset: 0x08
    pub CMD         : u32,        // Command register               Address offset: 0x0C
    pub RESPCMD     : u32,        // Command response register      Address offset: 0x10
    pub RESP1       : u32,        // Response 1 register            Address offset: 0x14
    pub RESP2       : u32,        // Response 2 register            Address offset: 0x18
    pub RESP3       : u32,        // Response 3 register            Address offset: 0x1C
    pub RESP4       : u32,        // Response 4 register            Address offset: 0x20
    pub DTIMER      : u32,        // Data timer register            Address offset: 0x24
    pub DLEN        : u32,        // Data length register           Address offset: 0x28
    pub DCTRL       : u32,        // Data control register          Address offset: 0x2C
    pub DCOUNT      : u32,        // Data counter register          Address offset: 0x30
    pub STA         : u32,        // Status register                Address offset: 0x34
    pub ICR         : u32,        // Interrupt clear register       Address offset: 0x38
    pub MASK        : u32,        // Mask register                  Address offset: 0x3C
    pub RESERVED0   : [u32; 2],   // Reserved                       Address offset: 0x40-0x44
    pub FIFOCNT     : u32,        // FIFO counter register          Address offset: 0x48
    pub RESERVED1   : [u32; 13],  // Reserved                       Address offset: 0x4C-0x7C
    pub FIFO        : u32,        // Data FIFO register             Address offset: 0x80
}

#[repr(C)]
pub struct SYSCFGRegDef {
    pub MEMRMP      : u32,         // Memory remap register                     Address offset: 0x00
    pub PMC         : u32,         // Peripheral mode configuration register    Address offset: 0x04
    pub EXTICR      : [u32; 4],    // External interrupt configuration registers Address offset: 0x08-0x14
    pub RESERVED    : [u32; 2],    // Reserved                                  Address offset: 0x18-0x1C
    pub CMPCR       : u32,         // Compensation cell control register        Address offset: 0x20
}

#[repr(C)]
pub struct EXTIRegDef {
    pub IMR         : u32,         // Interrupt mask register            Address offset: 0x00
    pub EMR         : u32,         // Event mask register                Address offset: 0x04
    pub RTSR        : u32,         // Rising trigger selection register  Address offset: 0x08
    pub FTSR        : u32,         // Falling trigger selection register Address offset: 0x0C
    pub SWIER       : u32,         // Software interrupt event register  Address offset: 0x10
    pub PR          : u32,         // Pending register                   Address offset: 0x14
}

#[repr(C)]
pub struct FLASHRegDef {
    pub ACR         : u32,         // Access control register           Address offset: 0x00
    pub KEYR        : u32,         // Key register                      Address offset: 0x04
    pub OPTKEYR     : u32,         // Option key register               Address offset: 0x08
    pub SR          : u32,         // Status register                   Address offset: 0x0C
    pub CR          : u32,         // Control register                  Address offset: 0x10
    pub OPTCR       : u32,         // Option control register           Address offset: 0x14
    pub OPTCR1      : u32,         // Option control register 1         Address offset: 0x18
}

#[repr(C)]
pub struct GPIORegDef {
    pub MODER       : u32,         // Port mode register               Address offset: 0x00
    pub OTYPER      : u32,         // Port output type register        Address offset: 0x04
    pub OSPEEDR     : u32,         // Port output speed register       Address offset: 0x08
    pub PUPDR       : u32,         // Port pull-up/pull-down register  Address offset: 0x0C
    pub IDR         : u32,         // Port input data register         Address offset: 0x10
    pub ODR         : u32,         // Port output data register        Address offset: 0x14
    pub BSRR        : u32,         // Port bit set/reset register      Address offset: 0x18
    pub LCKR        : u32,         // Port configuration lock register Address offset: 0x1C
    pub AFR         : [u32; 2],    // Port alternate function register Address offset: 0x20-0x24
}

#[repr(C)]
pub struct RCCRegDef {
    pub CR          : u32,         // Clock control register                      Address offset: 0x00
    pub PLLCFGR     : u32,         // PLL configuration register                  Address offset: 0x04
    pub CFGR        : u32,         // Clock configuration register                Address offset: 0x08
    pub CIR         : u32,         // Clock interrupt register                    Address offset: 0x0C
    pub AHB1RSTR    : u32,         // AHB1 peripheral reset register              Address offset: 0x10
    pub AHB2RSTR    : u32,         // AHB2 peripheral reset register              Address offset: 0x14
    pub AHB3RSTR    : u32,         // AHB3 peripheral reset register              Address offset: 0x18
    pub RESERVED0   : u32,         // Reserved                                    Address offset: 0x1C
    pub APB1RSTR    : u32,         // APB1 peripheral reset register              Address offset: 0x20
    pub APB2RSTR    : u32,         // APB2 peripheral reset register              Address offset: 0x24
    pub RESERVED1   : [u32; 2],    // Reserved                                    Address offset: 0x28-0x2C
    pub AHB1ENR     : u32,         // AHB1 peripheral clock enable register       Address offset: 0x30
    pub AHB2ENR     : u32,         // AHB2 peripheral clock enable register       Address offset: 0x34
    pub AHB3ENR     : u32,         // AHB3 peripheral clock enable register       Address offset: 0x38
    pub RESERVED2   : u32,         // Reserved                                    Address offset: 0x3C
    pub APB1ENR     : u32,         // APB1 peripheral clock enable register       Address offset: 0x40
    pub APB2ENR     : u32,         // APB2 peripheral clock enable register       Address offset: 0x44
    pub RESERVED3   : [u32; 2],    // Reserved                                    Address offset: 0x48-0x4C
    pub AHB1LPENR   : u32,         // AHB1 clock enable in low power mode         Address offset: 0x50
    pub AHB2LPENR   : u32,         // AHB2 clock enable in low power mode         Address offset: 0x54
    pub AHB3LPENR   : u32,         // AHB3 clock enable in low power mode         Address offset: 0x58
    pub RESERVED4   : u32,         // Reserved                                    Address offset: 0x5C
    pub APB1LPENR   : u32,         // APB1 clock enable in low power mode         Address offset: 0x60
    pub APB2LPENR   : u32,         // APB2 clock enable in low power mode         Address offset: 0x64
    pub RESERVED5   : [u32; 2],    // Reserved                                    Address offset: 0x68-0x6C
    pub BDCR        : u32,         // Backup domain control register              Address offset: 0x70
    pub CSR         : u32,         // Clock control & status register             Address offset: 0x74
    pub RESERVED6   : [u32; 2],    // Reserved                                    Address offset: 0x78-0x7C
    pub SSCGR       : u32,         // Spread spectrum clock generation register   Address offset: 0x80
    pub PLLI2SCFGR  : u32,         // PLLI2S configuration register               Address offset: 0x84
}

#[repr(C)]
pub struct DMAStreamRegDef {
    pub CR          : u32,         // Configuration register           Address offset: 0x00
    pub NDTR        : u32,         // Number of data register          Address offset: 0x04
    pub PAR         : u32,         // Peripheral address register      Address offset: 0x08
    pub M0AR        : u32,         // Memory 0 address register        Address offset: 0x0C
    pub M1AR        : u32,         // Memory 1 address register        Address offset: 0x10
    pub FCR         : u32,         // FIFO control register            Address offset: 0x14
}

#[repr(C)]
pub struct DMARegDef {
    pub LISR        : u32,         // Low interrupt status register     Address offset: 0x00
    pub HISR        : u32,         // High interrupt status register    Address offset: 0x04
    pub LIFCR       : u32,         // Low interrupt flag clear register Address offset: 0x08
    pub HIFCR       : u32,         // High interrupt flag clear register Address offset: 0x0C
}

#[repr(C)]
pub struct FSMCBank1RegDef {
    pub BTCR        : [u32; 8],    // NOR/PSRAM chip-select control register(BCR) and chip-select timing register(BTR)
}

#[repr(C)]
pub struct FSMCBank1ERegDef {
    pub BWTR        : [u32; 7],    // NOR/PSRAM write timing registers
}

#[repr(C)]
pub struct FSMCBank2_3RegDef {
    pub PCR2        : u32,         // NAND Flash control register 2                        Address offset: 0x60
    pub SR2         : u32,         // NAND Flash FIFO status and interrupt register 2      Address offset: 0x64
    pub PMEM2       : u32,         // NAND Flash Common memory space timing register 2     Address offset: 0x68
    pub PATT2       : u32,         // NAND Flash Attribute memory space timing register 2  Address offset: 0x6C
    pub RESERVED0   : u32,         // Reserved                                             Address offset: 0x70
    pub ECCR2       : u32,         // NAND Flash ECC result registers 2                    Address offset: 0x74
    pub RESERVED1   : [u32; 2],    // Reserved                                             Address offset: 0x78-0x7C
    pub PCR3        : u32,         // NAND Flash control register 3                        Address offset: 0x80
    pub SR3         : u32,         // NAND Flash FIFO status and interrupt register 3      Address offset: 0x84
    pub PMEM3       : u32,         // NAND Flash Common memory space timing register 3     Address offset: 0x88
    pub PATT3       : u32,         // NAND Flash Attribute memory space timing register 3  Address offset: 0x8C
    pub RESERVED2   : u32,         // Reserved                                             Address offset: 0x90
    pub ECCR3       : u32,         // NAND Flash ECC result registers 3                    Address offset: 0x94
}

#[repr(C)]
pub struct FSMCBank4RegDef {
    pub PCR4        : u32,         // PC Card control register 4                         Address offset: 0xA0
    pub SR4         : u32,         // PC Card FIFO status and interrupt register 4       Address offset: 0xA4
    pub PMEM4       : u32,         // PC Card Common memory space timing register 4      Address offset: 0xA8
    pub PATT4       : u32,         // PC Card Attribute memory space timing register 4   Address offset: 0xAC
    pub PIO4        : u32,         // PC Card I/O space timing register 4                Address offset: 0xB0
}

#[repr(C)]
pub struct DBGMCURegDef {
    pub IDCODE      : u32,         // MCU device ID code                 Address offset: 0x00
    pub CR          : u32,         // Debug MCU configuration register   Address offset: 0x04
    pub APB1FZ      : u32,         // Debug MCU APB1 freeze register     Address offset: 0x08
    pub APB2FZ      : u32,         // Debug MCU APB2 freeze register     Address offset: 0x0C
}

#[repr(C)]
pub struct DCMIRegDef {
    pub CR          : u32,         // Control register 1                  Address offset: 0x00
    pub SR          : u32,         // Status register                     Address offset: 0x04
    pub RISR        : u32,         // Raw interrupt status register       Address offset: 0x08
    pub IER         : u32,         // Interrupt enable register           Address offset: 0x0C
    pub MISR        : u32,         // Masked interrupt status register    Address offset: 0x10
    pub ICR         : u32,         // Interrupt clear register            Address offset: 0x14
    pub ESCR        : u32,         // Embedded synchronization code reg.  Address offset: 0x18
    pub ESUR        : u32,         // Embedded synchronization unmask reg.Address offset: 0x1C
    pub CWSTRTR     : u32,         // Crop window start                   Address offset: 0x20
    pub CWSIZER     : u32,         // Crop window size                    Address offset: 0x24
    pub DR          : u32,         // Data register                       Address offset: 0x28
}

#[repr(C)]
pub struct USBOTGGlobalRegDef {
    pub GOTGCTL     : u32,         // Control and status register         Address offset: 0x000
    pub GOTGINT     : u32,         // Interrupt register                  Address offset: 0x004
    pub GAHBCFG     : u32,         // AHB configuration register          Address offset: 0x008
    pub GUSBCFG     : u32,         // USB configuration register          Address offset: 0x00C
    pub GRSTCTL     : u32,         // Reset register                      Address offset: 0x010
    pub GINTSTS     : u32,         // Interrupt register                  Address offset: 0x014
    pub GINTMSK     : u32,         // Interrupt mask register             Address offset: 0x018
    pub GRXSTSR     : u32,         // Receive status debug read register  Address offset: 0x01C
    pub GRXSTSP     : u32,         // Receive status debug pop register   Address offset: 0x020
    pub GRXFSIZ     : u32,         // Receive FIFO size register          Address offset: 0x024
    pub DIEPTXF0_HNPTXFSIZ : u32,  // Non-periodic TX FIFO size register  Address offset: 0x028
    pub HNPTXSTS    : u32,         // Non-periodic TX FIFO/queue status   Address offset: 0x02C
    pub RESERVED0   : [u32; 2],    // Reserved                            Address offset: 0x030-0x034
    pub GCCFG       : u32,         // General core configuration          Address offset: 0x038
    pub CID         : u32,         // Core ID register                    Address offset: 0x03C
    pub RESERVED1   : [u32; 48],   // Reserved                            Address offset: 0x040-0x0FC
    pub HPTXFSIZ    : u32,         // Host periodic TX FIFO size register Address offset: 0x100
    pub DIEPTXF     : [u32; 15],   // Device IN endpoint TX FIFO size     Address offset: 0x104-0x13C
}

#[repr(C)]
pub struct USBOTGDeviceRegDef {
    pub DCFG        : u32,         // Device configuration register       Address offset: 0x800
    pub DCTL        : u32,         // Device control register             Address offset: 0x804
    pub DSTS        : u32,         // Device status register              Address offset: 0x808
    pub RESERVED0   : u32,         // Reserved                            Address offset: 0x80C
    pub DIEPMSK     : u32,         // Device IN endpoint mask register    Address offset: 0x810
    pub DOEPMSK     : u32,         // Device OUT endpoint mask register   Address offset: 0x814
    pub DAINT       : u32,         // Device all endpoints interrupt reg. Address offset: 0x818
    pub DAINTMSK    : u32,         // Device all EP interrupt mask reg.   Address offset: 0x81C
    pub RESERVED1   : u32,         // Reserved                            Address offset: 0x820
    pub RESERVED2   : u32,         // Reserved                            Address offset: 0x824
    pub DVBUSDIS    : u32,         // Device VBUS discharge time register Address offset: 0x828
    pub DVBUSPULSE  : u32,         // Device VBUS pulsing time register   Address offset: 0x82C
    pub DTHRCTL     : u32,         // Device threshold control register   Address offset: 0x830
    pub DIEPEMPMSK  : u32,         // Device IN endpoint FIFO empty int.  Address offset: 0x834
    pub DEACHINT    : u32,         // Dedicated EP interrupt register     Address offset: 0x838
    pub DEACHMSK    : u32,         // Dedicated EP interrupt mask reg.    Address offset: 0x83C
    pub RESERVED3   : u32,         // Reserved                            Address offset: 0x840
    pub DINEP1MSK   : u32,         // Dedicated EP mask register          Address offset: 0x844
    pub RESERVED4   : [u32; 15],   // Reserved                            Address offset: 0x848-0x880
    pub DOUTEP1MSK  : u32,         // Dedicated EP mask register          Address offset: 0x884
}

#[repr(C)]
pub struct USB_OTG_INEndpointRegDef {
    pub DIEPCTL     : u32,         // Device IN endpoint control register Address offset: 0x900 + (ep_num * 0x20)
    pub RESERVED0   : u32,         // Reserved                            Address offset: 0x904 + (ep_num * 0x20)
    pub DIEPINT     : u32,         // Device IN endpoint interrupt reg.   Address offset: 0x908 + (ep_num * 0x20)
    pub RESERVED1   : u32,         // Reserved                            Address offset: 0x90C + (ep_num * 0x20)
    pub DIEPTSIZ    : u32,         // Device IN endpoint transfer size    Address offset: 0x910 + (ep_num * 0x20)
    pub DIEPDMA     : u32,         // Device IN endpoint DMA address      Address offset: 0x914 + (ep_num * 0x20)
    pub DTXFSTS     : u32,         // Device IN endpoint TX FIFO status   Address offset: 0x918 + (ep_num * 0x20)
    pub RESERVED2   : u32,         // Reserved                            Address offset: 0x91C + (ep_num * 0x20)
}

#[repr(C)]
pub struct USB_OTG_OUTEndpointRegDef {
    pub DOEPCTL     : u32,         // Device OUT endpoint control reg.    Address offset: 0xB00 + (ep_num * 0x20)
    pub RESERVED0   : u32,         // Reserved                            Address offset: 0xB04 + (ep_num * 0x20)
    pub DOEPINT     : u32,         // Device OUT endpoint interrupt reg.  Address offset: 0xB08 + (ep_num * 0x20)
    pub RESERVED1   : u32,         // Reserved                            Address offset: 0xB0C + (ep_num * 0x20)
    pub DOEPTSIZ    : u32,         // Device OUT endpoint transfer size   Address offset: 0xB10 + (ep_num * 0x20)
    pub DOEPDMA     : u32,         // Device OUT endpoint DMA address     Address offset: 0xB14 + (ep_num * 0x20)
    pub RESERVED2   : [u32; 2],    // Reserved                            Address offset: 0xB18-0xB1C + (ep_num * 0x20)
}

#[repr(C)]
pub struct USB_OTG_HostRegDef {
    pub HCFG        : u32,         // Host configuration register         Address offset: 0x400
    pub HFIR        : u32,         // Host frame interval register        Address offset: 0x404
    pub HFNUM       : u32,         // Host frame number/frame time        Address offset: 0x408
    pub RESERVED0   : u32,         // Reserved                            Address offset: 0x40C
    pub HPTXSTS     : u32,         // Host periodic TX FIFO/queue status  Address offset: 0x410
    pub HAINT       : u32,         // Host all channels interrupt reg.    Address offset: 0x414
    pub HAINTMSK    : u32,         // Host all channels interrupt mask    Address offset: 0x418
}

#[repr(C)]
pub struct USB_OTG_HostChannelRegDef {
    pub HCCHAR      : u32,         // Host channel characteristics        Address offset: 0x500 + (ch_num * 0x20)
    pub HCSPLT      : u32,         // Host channel split control reg.     Address offset: 0x504 + (ch_num * 0x20)
    pub HCINT       : u32,         // Host channel interrupt              Address offset: 0x508 + (ch_num * 0x20)
    pub HCINTMSK    : u32,         // Host channel interrupt mask         Address offset: 0x50C + (ch_num * 0x20)
    pub HCTSIZ      : u32,         // Host channel transfer size          Address offset: 0x510 + (ch_num * 0x20)
    pub HCDMA       : u32,         // Host channel DMA address            Address offset: 0x514 + (ch_num * 0x20)
    pub RESERVED0   : [u32; 2],    // Reserved                            Address offset: 0x518-0x51C + (ch_num * 0x20)
}

pub const GPIOA                                         : *mut GPIORegDef = GPIOA_BASE as *mut GPIORegDef;
pub const GPIOB                                         : *mut GPIORegDef = GPIOB_BASE as *mut GPIORegDef;
pub const GPIOC                                         : *mut GPIORegDef = GPIOC_BASE as *mut GPIORegDef;
pub const GPIOD                                         : *mut GPIORegDef = GPIOD_BASE as *mut GPIORegDef;
pub const GPIOE                                         : *mut GPIORegDef = GPIOE_BASE as *mut GPIORegDef;
pub const GPIOF                                         : *mut GPIORegDef = GPIOF_BASE as *mut GPIORegDef;
pub const GPIOG                                         : *mut GPIORegDef = GPIOG_BASE as *mut GPIORegDef;
pub const GPIOH                                         : *mut GPIORegDef = GPIOH_BASE as *mut GPIORegDef;
pub const GPIOI                                         : *mut GPIORegDef = GPIOI_BASE as *mut GPIORegDef;
pub const RCC                                           : *mut RCCRegDef = RCC_BASE as *mut RCCRegDef;
pub const FLASH_R                                       : *mut FLASHRegDef = FLASH_R_BASE as *mut FLASHRegDef;
pub const EXTI                                          : *mut EXTIRegDef = EXTI_BASE as *mut EXTIRegDef;
pub const SYSCFG                                        : *mut SYSCFGRegDef = SYSCFG_BASE as *mut SYSCFGRegDef;
pub const DMA1                                          : *mut DMARegDef = DMA1_BASE as *mut DMARegDef;
pub const DMA2                                          : *mut DMARegDef = DMA2_BASE as *mut DMARegDef;
pub const DMA1_Stream0                                  : *mut DMAStreamRegDef = DMA1_Stream0_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream1                                  : *mut DMAStreamRegDef = DMA1_Stream1_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream2                                  : *mut DMAStreamRegDef = DMA1_Stream2_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream3                                  : *mut DMAStreamRegDef = DMA1_Stream3_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream4                                  : *mut DMAStreamRegDef = DMA1_Stream4_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream5                                  : *mut DMAStreamRegDef = DMA1_Stream5_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream6                                  : *mut DMAStreamRegDef = DMA1_Stream6_BASE as *mut DMAStreamRegDef;
pub const DMA1_Stream7                                  : *mut DMAStreamRegDef = DMA1_Stream7_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream0                                  : *mut DMAStreamRegDef = DMA2_Stream0_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream1                                  : *mut DMAStreamRegDef = DMA2_Stream1_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream2                                  : *mut DMAStreamRegDef = DMA2_Stream2_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream3                                  : *mut DMAStreamRegDef = DMA2_Stream3_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream4                                  : *mut DMAStreamRegDef = DMA2_Stream4_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream5                                  : *mut DMAStreamRegDef = DMA2_Stream5_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream6                                  : *mut DMAStreamRegDef = DMA2_Stream6_BASE as *mut DMAStreamRegDef;
pub const DMA2_Stream7                                  : *mut DMAStreamRegDef = DMA2_Stream7_BASE as *mut DMAStreamRegDef;
pub const SDIO                                          : *mut SDIORegDef = SDIO_BASE as *mut SDIORegDef;
pub const FSMCBank1                                     : *mut FSMCBank1RegDef = FSMCBank1_R_BASE as *mut FSMCBank1RegDef;
pub const FSMCBank1E                                    : *mut FSMCBank1ERegDef = FSMCBank1E_R_BASE as *mut FSMCBank1ERegDef;
pub const FSMCBank2_3                                   : *mut FSMCBank2_3RegDef = FSMCBank2_3_R_BASE as *mut FSMCBank2_3RegDef;
pub const FSMCBank4                                     : *mut FSMCBank4RegDef = FSMCBank4_R_BASE as *mut FSMCBank4RegDef;
pub const DCMI                                          : *mut DCMIRegDef = DCMI_BASE as *mut DCMIRegDef;
pub const DBGMCU                                        : *mut DBGMCURegDef = DBGMCU_BASE as *mut DBGMCURegDef;
pub const USB_OTG_FS_GLOBAL                             : *mut USBOTGGlobalRegDef = (USB_OTG_FS_PERIPH_BASE + USB_OTG_GLOBAL_BASE) as *mut USBOTGGlobalRegDef;
pub const USB_OTG_HS_GLOBAL                             : *mut USBOTGGlobalRegDef = (USB_OTG_HS_PERIPH_BASE + USB_OTG_GLOBAL_BASE) as *mut USBOTGGlobalRegDef;
pub const USB_OTG_FS_DEVICE                             : *mut USBOTGDeviceRegDef = (USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE) as *mut USBOTGDeviceRegDef;
pub const USB_OTG_HS_DEVICE                             : *mut USBOTGDeviceRegDef = (USB_OTG_HS_PERIPH_BASE + USB_OTG_DEVICE_BASE) as *mut USBOTGDeviceRegDef;
pub const USB_OTG_FS_HOST                               : *mut USB_OTG_HostRegDef = (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE) as *mut USB_OTG_HostRegDef;
pub const USB_OTG_HS_HOST                               : *mut USB_OTG_HostRegDef = (USB_OTG_HS_PERIPH_BASE + USB_OTG_HOST_BASE) as *mut USB_OTG_HostRegDef;
pub const TIM6                                          : *mut TIM_6_7_RegDef = TIM6_BASE as *mut TIM_6_7_RegDef;
pub const TIM7                                          : *mut TIM_6_7_RegDef = TIM7_BASE as *mut TIM_6_7_RegDef;
pub const TIM8                                          : *mut TIM_1_8_RegDef = TIM8_BASE as *mut TIM_1_8_RegDef;
pub const TIM9                                          : *mut TIM_9_12_RegDef = TIM9_BASE as *mut TIM_9_12_RegDef;
pub const TIM10                                         : *mut TIM_10_14_RegDef = TIM10_BASE as *mut TIM_10_14_RegDef;
pub const TIM11                                         : *mut TIM_11_RegDef = TIM11_BASE as *mut TIM_11_RegDef;
pub const TIM12                                         : *mut TIM_9_12_RegDef = TIM12_BASE as *mut TIM_9_12_RegDef;
pub const TIM13                                         : *mut TIM_10_14_RegDef = TIM13_BASE as *mut TIM_10_14_RegDef;
pub const TIM14                                         : *mut TIM_10_14_RegDef = TIM14_BASE as *mut TIM_10_14_RegDef;
pub const RTC                                           : *mut RTCRegDef = RTC_BASE as *mut RTCRegDef;
pub const WWDG                                          : *mut WWDGRegDef = WWDG_BASE as *mut WWDGRegDef;
pub const IWDG                                          : *mut IWDGRegDef = IWDG_BASE as *mut IWDGRegDef;
pub const SPI1                                          : *mut SPIRegDef = SPI1_BASE as *mut SPIRegDef;
pub const SPI2                                          : *mut SPIRegDef = SPI2_BASE as *mut SPIRegDef;
pub const SPI3                                          : *mut SPIRegDef = SPI3_BASE as *mut SPIRegDef;
pub const I2S2ext                                       : *mut SPIRegDef = I2S2ext_BASE as *mut SPIRegDef;
pub const I2S3ext                                       : *mut SPIRegDef = I2S3ext_BASE as *mut SPIRegDef;
pub const USART1                                        : *mut USARTRegDef = USART1_BASE as *mut USARTRegDef;
pub const USART2                                        : *mut USARTRegDef = USART2_BASE as *mut USARTRegDef;
pub const USART3                                        : *mut USARTRegDef = USART3_BASE as *mut USARTRegDef;
pub const UART4                                         : *mut USARTRegDef = UART4_BASE as *mut USARTRegDef;
pub const UART5                                         : *mut USARTRegDef = UART5_BASE as *mut USARTRegDef;
pub const USART6                                        : *mut USARTRegDef = USART6_BASE as *mut USARTRegDef;
pub const I2C1                                          : *mut I2CRegDef = I2C1_BASE as *mut I2CRegDef;
pub const I2C2                                          : *mut I2CRegDef = I2C2_BASE as *mut I2CRegDef;
pub const I2C3                                          : *mut I2CRegDef = I2C3_BASE as *mut I2CRegDef;
pub const CAN1                                          : *mut CANRegDef = CAN1_BASE as *mut CANRegDef;
pub const CAN2                                          : *mut CANRegDef = CAN2_BASE as *mut CANRegDef;
pub const PWR                                           : *mut PWRRegDef = PWR_BASE as *mut PWRRegDef;
pub const DAC                                           : *mut DACRegDef = DAC_BASE as *mut DACRegDef;
pub const ADC1                                          : *mut ADCRegDef = ADC1_BASE as *mut ADCRegDef;
pub const ADC2                                          : *mut ADCRegDef = ADC2_BASE as *mut ADCRegDef;
pub const ADC3                                          : *mut ADCRegDef = ADC3_BASE as *mut ADCRegDef;
pub const ADC123_COMMON                                 : *mut ADCCommonRegDef = ADC123_COMMON_BASE as *mut ADCCommonRegDef;

#[derive(Copy, Clone)]
pub struct RegValue(pub u32);

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