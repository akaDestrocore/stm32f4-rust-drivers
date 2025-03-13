use cortex_m_rt::exception;
use cortex_m_rt::ExceptionFrame;

#[exception]
unsafe fn NonMaskableInt() {
    loop {}
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    loop {}
}

#[exception]
unsafe fn MemoryManagement() {
    loop {}
}

#[exception]
unsafe fn BusFault() {
    loop {}
}

#[exception]
unsafe fn UsageFault() {
    loop {}
}

#[exception]
unsafe fn SVCall() {
    loop {}
}

#[exception]
unsafe fn DebugMonitor() {
    loop {}
}

#[exception]
unsafe fn PendSV() {
    loop {}
}

#[exception]
unsafe fn SysTick() {
    loop {}
}