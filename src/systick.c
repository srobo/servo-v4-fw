#include "systick.h"

#include <libopencm3/cm3/systick.h>

void systick_init(void) {
    // Generate a 1ms systick interrupt
    // 72MHz / 8 => 9000000 counts per second
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

    // 9000000/9000 = 1000 overflows per second
    // SysTick interrupt every N clock pulses: set reload to N-1
    systick_set_reload(8999);

    systick_interrupt_enable();

    // Start counting.
    systick_counter_enable();
}

void sys_tick_handler(void) {
    ;
    /// TODO if watchdog tripped re-init expander
}
