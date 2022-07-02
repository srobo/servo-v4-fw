#include "systick.h"
#include "servo.h"
#include "i2c.h"
#include "global_vars.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

uint8_t systick_servo_tick = 0;

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
    // Every 20 ms start servo pulse
    if (++systick_servo_tick == 20) {
        // if watchdog tripped re-init expander
        if (i2c_timed_out) {
            // reset watchdog
            reset_i2c_watchdog();
            init_expander(I2C_EXPANDER_ADDR);
            init_current_sense(CURRENT_SENSE_ADDR);
        }
        start_servo_period();
        current_sense_updated = false;
        systick_servo_tick = 0;
    }
}
