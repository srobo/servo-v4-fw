#include "servo.h"
#include "i2c.h"
#include "led.h"

#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>

#define US_TO_TICK(x) ((uint16_t)x / 5)
#define TICK_TO_US(x) (x * 5)

// in uS
#define MIN_SERVO_PULSE 200
#define MAX_SERVO_PULSE 10000

static const uint8_t servo_bit_mapping[] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3};

static volatile uint16_t current_pin_state = 0x0000;
static volatile uint8_t current_servo_step = 0;
volatile uint8_t processing_servo_pulses = 0;

typedef struct {
    uint8_t idx;
    uint8_t enabled;
    uint16_t pulse;  // in timer ticks
} servo_t;

static servo_t servo_state[NUM_SERVOS] = {};
// loaded from servo_state at the end of each period
static servo_t current_servo_state[NUM_SERVOS] = {};

static void init_timer(void) {
    // Enable TIM1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // Enable TIM1 compare interrupt line
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    // Reset TIM1 peripheral.
	rcc_periph_reset_pulse(RST_TIM1);

    // Up counting, edge triggered no divider
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 360);  // 200kHz
    timer_set_period(TIM1, 200000 / 50);  // 50 Hz
    timer_continuous_mode(TIM1);
    timer_disable_preload(TIM1);

    // Don't alter output on compare match
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_FROZEN);
    timer_enable_oc_preload(TIM1, TIM_OC1);

    // Start counting
    timer_enable_counter(TIM1);
}

void servo_init(void) {
    // configure i2c expander
    i2c_init();
    init_expander(I2C_EXPANDER_ADDR);

    // initialise servo state indexes
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servo_state[i].idx = i;
    }

    // setup timer
    init_timer();
}

void servo_set_pos(uint8_t idx, uint16_t pulse_us) {
    if (idx >= NUM_SERVOS) {
        return;
    }
    // constrain pulse high time to a reasonable range for servos
    if (pulse_us < MIN_SERVO_PULSE) {
        pulse_us = MIN_SERVO_PULSE;
    } else if (pulse_us > MAX_SERVO_PULSE) {
        pulse_us = MAX_SERVO_PULSE;
    }

    // servos are automatically enabled when they are set
    servo_state[idx].enabled = 1;
    set_led(idx);
    servo_state[idx].pulse = US_TO_TICK(pulse_us);
}

void servo_disable(uint8_t idx) {
    if (idx >= NUM_SERVOS) {
        return;
    }
    clear_led(idx);
    servo_state[idx].enabled = 0;
}

uint16_t servo_get_pos(uint8_t idx) {
    if (idx >= NUM_SERVOS) {
        return -1;
    }
    if (servo_state[idx].enabled == 0) {
        return 0;
    } else {
        return TICK_TO_US(servo_state[idx].pulse);
    }
}

void servo_reset(void) {
    for (uint8_t i=0; i < NUM_SERVOS; i++) {
        servo_disable(i);
    }
}

static void set_expander_output(uint16_t val) {
    // this uses an existing transaction with the expander in a special byte mode that
    // alternates between the AB register pairs to minimise the data sent over I2C
    // mask in fixed values, SMPS_EN=1, LINK_EN=0
    val &= 0xF7;
    val |= (1 << 5);
    i2c_send_byte((uint8_t)(val & 0xff));  // A-reg first
    i2c_send_byte((uint8_t)((val >> 8) & 0xff));
}

static int compare_servos(const void* a, const void* b) {
    // helper function to sort servos by pulse length
    return ((servo_t*)a)->pulse - ((servo_t*)b)->pulse;
}

static void load_servo_state(void) {
    servo_t active_servos[NUM_SERVOS];
    servo_t inactive_servos[NUM_SERVOS];
    uint8_t num_active = 0;
    uint8_t num_inactive = 0;

    // split out active servos
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if (servo_state[i].enabled) {
            active_servos[num_active] = servo_state[i];
            num_active++;
        } else {
            inactive_servos[num_inactive] = servo_state[i];
            num_inactive++;
        }
    }

    // sort active servos into ascending order of pulse
    qsort(active_servos, num_active, sizeof(servo_t), compare_servos);

    // store in current_servo_state
    memcpy(current_servo_state, active_servos, sizeof(servo_t) * num_active);
    memcpy(&(current_servo_state[num_active+1]), inactive_servos, sizeof(servo_t) * num_inactive);
}

void start_servo_period(void) {
    // disable timer interrupt
    timer_disable_irq(TIM1, TIM_DIER_CC1IE);
    load_servo_state();
    current_servo_step = 0;
    // if all servos are disabled
    if (current_servo_state[0].enabled == 0) {
        return;
    }
    // setup transaction to GPIO register
    i2c_start_message(0x21, I2C_WRITE);
    i2c_send_byte(0x12);
    // set flag for processing servo pulses
    processing_servo_pulses = 1;
    // set compare to first servo pulse
    timer_set_oc_value(TIM1, TIM_OC1, current_servo_state[0].pulse);
    // set all enabled servo bits high
    current_pin_state = 0x0000;
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        if(current_servo_state[i].enabled) {
            current_pin_state |= (1 << servo_bit_mapping[current_servo_state[i].idx]);
        }
    }
    // set timer val to 0
    timer_set_counter(TIM1, 0);
    // write bit val to expander
    set_expander_output(current_pin_state);
    // enable timer interrupt
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
}

void tim1_cc_isr(void) {
    uint8_t next_servo_step;
    uint8_t current_servo_index;

    set_led(LED_STATUS_RED);

    do {
        next_servo_step = current_servo_step + 1;

        current_servo_index = current_servo_state[current_servo_step].idx;
        // set current servo bit low (high pulse complete)
        current_pin_state &= ~((uint16_t)(1 << servo_bit_mapping[current_servo_index]));

        // multiple servos may be set to the same value, set all the bits together
        while (current_servo_state[next_servo_step].pulse <= (current_servo_state[current_servo_step].pulse + US_TO_TICK(20))) {
            if (current_servo_state[next_servo_step].enabled == 0) {
                break;
            }
            // set current servo bit low (high pulse complete)
            current_pin_state &= ~((uint16_t)(1 << servo_bit_mapping[current_servo_index]));
            if (++next_servo_step >= NUM_SERVOS) {
                break;
            }
        }

        // write bit val to expander
        set_expander_output(current_pin_state);

        if ((current_servo_state[next_servo_step].enabled == 0) || (next_servo_step >= NUM_SERVOS)) {
            break;
        }

    // since sending I2C messages is slow (20us/byte) we may have missed the next servo's pulse end
    // so we'll handle that right now
        current_servo_step = next_servo_step;
    } while ((uint16_t)(current_servo_state[next_servo_step].pulse + 1) > timer_get_counter(TIM1));

    // completed all active servos
    if(current_servo_state[next_servo_step].enabled == 0 || next_servo_step >= NUM_SERVOS) {
        // disable timer interrupt
        timer_disable_irq(TIM1, TIM_DIER_CC1IE);
        // clear flag for processing servo pulses
        processing_servo_pulses = 0;
        // stop expander transaction
        i2c_stop_message();
        // next period will be triggered by the systick interrupt
    } else {
        // set the timer compare to the next servo pulse end
        timer_set_oc_value(TIM1, TIM_OC1, current_servo_state[next_servo_step].pulse);
    }

    timer_clear_flag(TIM1, TIM_SR_CC1IF);
    clear_led(LED_STATUS_RED);
}
