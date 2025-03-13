/* STM32F407VGTx - 1024K FLASH, 192K RAM (128K + 64K CCMRAM) */
MEMORY
{
  /* Основная FLASH память */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* Основная RAM память */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* CCM (Core Coupled Memory) RAM */
  CCMRAM (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Расположение стека (конец RAM) */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* Секции в выходном файле */
SECTIONS
{
  /* Секция с таблицей векторов прерываний */
  .vector_table ORIGIN(FLASH) :
  {
    /* Первый элемент - начальное значение стека */
    LONG(_stack_start);
    
    /* Таблица векторов Cortex-M */
    KEEP(*(.vector_table.reset_vector));
    KEEP(*(.vector_table.exceptions));
    
    /* Таблица прерываний STM32 */
    KEEP(*(.vector_table.interrupts));
    . = ALIGN(4);
  } > FLASH

  /* Секция с кодом */
  .text :
  {
    . = ALIGN(4);
    *(.text .text.*);
    . = ALIGN(4);
    KEEP(*(.init));
    KEEP(*(.fini));
    . = ALIGN(4);
  } > FLASH

  /* Секция с константами (только для чтения) */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  /* Секция с инициализированными данными */
  .data : 
  {
    . = ALIGN(4);
    __sdata = .;        /* Символ для начала секции .data */
    *(.data .data.*);
    . = ALIGN(4);
    __edata = .;        /* Символ для конца секции .data */
  } > RAM AT> FLASH
  
  /* Адрес во FLASH, откуда копируются инициализированные данные */
  __sidata = LOADADDR(.data);

  /* Секция с неинициализированными данными */
  .bss :
  {
    . = ALIGN(4);
    __sbss = .;         /* Символ для начала секции .bss */
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    __ebss = .;         /* Символ для конца секции .bss */
  } > RAM
  
  /* Секция для данных в CCM RAM (если используется) */
  .ccmram :
  {
    . = ALIGN(4);
    __sccmram = .;      /* Символ для начала секции .ccmram */
    *(.ccmram .ccmram.*);
    . = ALIGN(4);
    __eccmram = .;      /* Символ для конца секции .ccmram */
  } > CCMRAM
}

/* Точка входа в программу */
ENTRY(Reset_Handler);