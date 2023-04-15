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
#define TICKS_BETWEEN_EDGES US_TO_TICK(100)
#define START_TICKS_PADDING US_TO_TICK(20)

static const uint8_t servo_bit_mapping[] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3};
typedef enum {XFR_BOTH, XFR_LOW, XFR_HIGH} transfer_mode_t;

static volatile uint16_t current_pin_state = 0x0000;
static volatile uint8_t current_servo_step = 0;

typedef struct {
    uint8_t idx;
    bool enabled;
    uint16_t pulse;  // in timer ticks
} servo_t;

static volatile servo_t servo_state[NUM_SERVOS] = {};
// loaded from servo_state at the start of each phase
static servo_t current_servo_state[SERVOS_PER_PHASE] = {};

static void init_timer(void) {
    // Enable TIM1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // Enable TIM1 compare interrupt line
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    // Reset TIM1 peripheral.
	rcc_periph_reset_pulse(RST_TIM1);

    timer_disable_preload(TIM1);
    // Up counting, edge triggered no divider
    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 360);  // 200kHz
    timer_set_period(TIM1, UINT16_MAX);
    timer_continuous_mode(TIM1);

    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_value(TIM1, TIM_OC1, 0);
}

void servo_init(void) {
    // initialise servo state indexes
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servo_state[i].idx = i;
        servo_state[i].enabled = false;
    }

    // setup timer
    init_timer();
}

void servo_deinit(void) {
    // disable irq
    nvic_disable_irq(NVIC_TIM1_CC_IRQ);
    timer_disable_irq(TIM1, TIM_DIER_CC1IE);

    // stop counting
    timer_disable_counter(TIM1);
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
    servo_state[idx].enabled = true;
    set_led(idx);
    servo_state[idx].pulse = US_TO_TICK(pulse_us);
}

void servo_disable(uint8_t idx) {
    if (idx >= NUM_SERVOS) {
        return;
    }
    clear_led(idx);
    servo_state[idx].enabled = false;
}

uint16_t servo_get_pos(uint8_t idx) {
    if (idx >= NUM_SERVOS) {
        return -1;
    }
    if (servo_state[idx].enabled) {
        return TICK_TO_US(servo_state[idx].pulse);
    } else {
        return 0;
    }
}

void servo_reset(void) {
    for (uint8_t i=0; i < NUM_SERVOS; i++) {
        servo_disable(i);
    }
}

static void set_expander_output(uint16_t val, transfer_mode_t mode) {
    // Assumes IOCON.BANK=0 to write the A & B registers in a single transaction
    const uint8_t EXT_GPIOA = 0x12;
    const uint8_t EXT_GPIOB = 0x13;
    // mask in fixed values, n_SMPS_EN=0, LINK_EN=0
    val &= 0xFF9F;

    // setup transaction to GPIO register
    i2c_start_message(I2C_EXPANDER_ADDR);
    if (mode == XFR_HIGH) {
        i2c_send_byte(EXT_GPIOB);
    } else {
        i2c_send_byte(EXT_GPIOA);
    }

    if ((mode == XFR_BOTH) || (mode == XFR_LOW)) {
        i2c_send_byte((uint8_t)(val & 0xff));  // A-reg first
    }
    if ((mode == XFR_BOTH) || (mode == XFR_HIGH)) {
        i2c_send_byte((uint8_t)((val >> 8) & 0xff));
    }
    i2c_stop_message();
}

static int compare_servos(const void* a, const void* b) {
    // helper function to sort servos by pulse length
    return ((servo_t*)a)->pulse - ((servo_t*)b)->pulse;
}

static void load_servo_state(uint8_t phase) {
    // phase 0 is reserved for other uses
    const uint8_t offset = (phase - 1) * SERVOS_PER_PHASE;
    uint16_t max_pulse_len = 0;
    servo_t active_servos[SERVOS_PER_PHASE] = {0};
    uint8_t num_active = 0;

    // split out active servos
    for (uint8_t i = 0; i < SERVOS_PER_PHASE; i++) {
        if (servo_state[i + offset].enabled) {
            active_servos[num_active] = servo_state[i + offset];
            num_active++;
        }
    }

    if (num_active > 0) {
        // sort active servos into ascending order of pulse length
        qsort(active_servos, num_active, sizeof(servo_t), compare_servos);

        // store in current_servo_state
        memcpy(current_servo_state, active_servos, sizeof(servo_t) * num_active);
        max_pulse_len = current_servo_state[num_active - 1].pulse;
    }
    for (uint8_t i = num_active; i < SERVOS_PER_PHASE; i++) {
        current_servo_state[i].enabled = false;
        current_servo_state[i].pulse = max_pulse_len;
    }
}

void start_servo_phase(uint8_t phase) {
    // disable timer interrupt
    timer_disable_irq(TIM1, TIM_DIER_CC1IE);
    load_servo_state(phase);

    current_servo_step = 1;
    // if all servos in this phase are disabled, skip the phase
    if (current_servo_state[0].enabled == false) {
        return;
    }

    // do step 0
    // start each phase with all servo pins low except the first servo of the phase
    current_pin_state = (uint16_t)(1 << servo_bit_mapping[current_servo_state[0].idx]);

    // enable timer interrupt
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
    // set compare for first split, allow extra time to fully set expander
    timer_set_period(TIM1, TICKS_BETWEEN_EDGES + START_TICKS_PADDING);
    // set timer val to 0
    timer_set_counter(TIM1, 0);
    // Start counting
    timer_enable_counter(TIM1);

    // write bit val to expander
    set_expander_output(current_pin_state, XFR_BOTH);
}

void tim1_cc_isr(void) {
    uint8_t servo_step = (current_servo_step < SERVOS_PER_PHASE)?current_servo_step:(current_servo_step - SERVOS_PER_PHASE);
    uint8_t servo_index = current_servo_state[servo_step].idx;
    uint16_t bit_to_change = (uint16_t)(1 << servo_bit_mapping[servo_index]);

    if (current_servo_step == (SERVOS_PER_PHASE - 1)) {  // last rising edge of the phase
        // this gap is the remaining delay before the first falling edge
        const uint16_t ticks_passed = (TICKS_BETWEEN_EDGES * 3 + START_TICKS_PADDING);
        timer_set_period(TIM1, current_servo_state[0].pulse - ticks_passed);
        if (current_servo_state[current_servo_step].enabled) {
            // set the active servo's output high
            current_pin_state |= bit_to_change;
        }
    } else if (current_servo_step < SERVOS_PER_PHASE) {  // A rising edge phase
        // rising edges are equally spaced
        timer_set_period(TIM1, TICKS_BETWEEN_EDGES);
        if (current_servo_state[current_servo_step].enabled) {
            // set the active servo's output high
            current_pin_state |= bit_to_change;
        }
    } else {  // A falling edge phase
        if (current_servo_step == (SERVOS_PER_PHASE * 2 - 1)) {
            // this phase is complete, stop interrupts until the next phase
            timer_disable_irq(TIM1, TIM_DIER_CC1IE);
        } else {
            // We need to account for how far through this phase we are
            uint16_t ticks_to_next_edge = (
                current_servo_state[servo_step + 1].pulse
                - current_servo_state[servo_step].pulse
                + START_TICKS_PADDING + TICKS_BETWEEN_EDGES);
            timer_set_period(TIM1, ticks_to_next_edge);
        }
        if (current_servo_state[servo_step].enabled) {
            // set the active servo's output low
            current_pin_state &= ~bit_to_change;
        }
    }

    // write bit val to expander
    set_expander_output(current_pin_state, (servo_bit_mapping[servo_index] < 8) ? XFR_LOW : XFR_HIGH);

    current_servo_step++;
    timer_clear_flag(TIM1, TIM_SR_CC1IF);
}
