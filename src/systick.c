#include "systick.h"
#include "servo.h"
#include "i2c.h"
#include "global_vars.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

// externs in global_vars.h
int board_voltage_mv = 0;
int board_current_ma = 0;
bool detected_power_good = false;

uint32_t systick_tick = 0;
uint8_t systick_servo_tick = 0;
uint8_t servo_phase = 0;

void systick_init(void) {
    // Generate a 1ms systick interrupt
    // 72MHz / 8 => 9000000 counts per second
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

    // 9000000/9000 = 1000 overflows per second
    // SysTick interrupt every N clock pulses
    systick_set_reload(9000);

    systick_interrupt_enable();

    // Start counting.
    systick_counter_enable();
}

void sys_tick_handler(void) {
    // Each servo phase lasts 5 ms
    if (++systick_servo_tick == 5) {
        if (servo_phase == 0) {
            // if watchdog tripped re-init expander
            if (i2c_timed_out) {
                // reset watchdog
                reset_i2c_watchdog();
                init_i2c_devices(false);
            } else {
                // measure servo current in dead-time before all pulses
                // 532us is needed after setup so skip it if the watchdog failed last cycle
                INA219_meas_t res = measure_current_sense(CURRENT_SENSE_ADDR);
                if (res.success) {
                    board_voltage_mv = res.voltage;
                    board_current_ma = res.current;
                }
            }
            get_expander_status(I2C_EXPANDER_ADDR);
        } else {
            if (!i2c_timed_out) {
                start_servo_phase(servo_phase);
            }
        }
        servo_phase++;
        servo_phase %= (NUM_SERVO_PHASES + 1);
        systick_servo_tick = 0;
    }
}
