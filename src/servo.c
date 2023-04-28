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
#define TICKS_BETWEEN_EDGES US_TO_TICK(125)
static void calculate_phase_steps(uint8_t phase);

static const uint8_t servo_bit_mapping[] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3};

static volatile uint16_t current_pin_state = 0x0000;
static volatile uint8_t current_servo_step = 0;

typedef struct {
    uint8_t idx;
    bool enabled;
    uint16_t pulse;  // in timer ticks
} servo_t;

static servo_t servo_state[NUM_SERVOS] = {};
static servo_step_t servo_steps[NUM_SERVO_PHASES][SERVO_STEPS_PER_PHASE] = { 0 };

// loaded from servo_steps at the start of each phase
static servo_step_t current_servo_steps[SERVO_STEPS_PER_PHASE] = {};

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
        servo_state[i].pulse = US_TO_TICK(MIN_SERVO_PULSE);
    }
    for (uint8_t i = 0; i < NUM_SERVO_PHASES; i++) {
        calculate_phase_steps(i + 1);
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

    calculate_phase_steps((idx / SERVOS_PER_PHASE) + 1);
}

void servo_disable(uint8_t idx) {
    if (idx >= NUM_SERVOS) {
        return;
    }
    clear_led(idx);
    servo_state[idx].enabled = false;

    calculate_phase_steps((idx / SERVOS_PER_PHASE) + 1);
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

servo_step_t* get_servo_steps(uint8_t phase) {
    // phase 0 is reserved for other uses
    if ((phase == 0) || (phase > NUM_SERVO_PHASES)) {
        return NULL;
    }

    return servo_steps[phase - 1];
}

static int compare_servos(const void* a, const void* b) {
    // helper function to sort servos by pulse length
    return ((servo_t*)a)->pulse - ((servo_t*)b)->pulse;
}

static void calculate_phase_steps(uint8_t phase) {
    // phase 0 is reserved for other uses
    if ((phase == 0) || (phase > NUM_SERVO_PHASES)) {
        return;
    }

    servo_t sorted_servo_states[SERVOS_PER_PHASE] = {0};
    servo_step_t sorted_servo_steps[SERVO_STEPS_PER_PHASE] = {0};
    const uint8_t offset = (phase - 1) * SERVOS_PER_PHASE;
    uint8_t num_active = 0;
    uint8_t num_inactive = 0;
    uint16_t max_pulse_len = US_TO_TICK(MIN_SERVO_PULSE);

    // split out active servos
    for (uint8_t i = 0; i < SERVOS_PER_PHASE; i++) {
        if (servo_state[i + offset].enabled) {
            sorted_servo_states[num_active] = servo_state[i + offset];
            num_active++;
        }
    }
    // include disabled servos after the active servos
    for (uint8_t i = 0; i < SERVOS_PER_PHASE; i++) {
        if (!servo_state[i + offset].enabled) {
            sorted_servo_states[num_active + num_inactive] = servo_state[i + offset];
            num_inactive++;
        }
    }

    if (num_active > 0) {
        // sort active servos into ascending order of pulse length
        qsort(sorted_servo_states, num_active, sizeof(servo_t), compare_servos);
        max_pulse_len = sorted_servo_states[num_active - 1].pulse;
    }
    // set disabled servo pulses to the longest pulse length
    for (uint8_t i = num_active; i < SERVOS_PER_PHASE; i++) {
        sorted_servo_states[i].pulse = max_pulse_len;
    }

    // calculate this phase's rising steps
    uint8_t step;
    for (step = 0; step < SERVOS_PER_PHASE; step++) {
        sorted_servo_steps[step].idx = sorted_servo_states[step].idx;
        sorted_servo_steps[step].enabled = sorted_servo_states[step].enabled;
        sorted_servo_steps[step].rising = true;
        if (step == (SERVOS_PER_PHASE - 1)) {
            // this gap is the remaining delay before the first falling edge
            sorted_servo_steps[step].next_steps = sorted_servo_states[0].pulse - (TICKS_BETWEEN_EDGES * 3);
        } else {
            sorted_servo_steps[step].next_steps = TICKS_BETWEEN_EDGES;
        }
    }

    // calculate this phase's falling steps
    for (uint8_t idx = 0; idx < SERVOS_PER_PHASE; idx++) {
        step = idx + SERVOS_PER_PHASE;
        sorted_servo_steps[step].idx = sorted_servo_states[idx].idx;
        sorted_servo_steps[step].enabled = sorted_servo_states[idx].enabled;
        sorted_servo_steps[step].rising = false;
        if (idx == (SERVOS_PER_PHASE - 1)) {
            // this phase is complete so this step never happens
            sorted_servo_steps[step].next_steps = UINT16_MAX;
        } else {
            // We need to account for how far through this phase we are
            uint16_t ticks_to_next_edge = (
                sorted_servo_states[idx + 1].pulse
                - sorted_servo_states[idx].pulse
                + TICKS_BETWEEN_EDGES);
            sorted_servo_steps[step].next_steps = ticks_to_next_edge;
        }
    }

    /// TODO make sure this is atomic
    memcpy(servo_steps[phase - 1], sorted_servo_steps, sizeof(servo_step_t) * SERVO_STEPS_PER_PHASE);
}

static void set_expander_output(uint16_t val) {
    // Assumes IOCON.BANK=0 to write the A & B registers in a single transaction
    const uint8_t EXT_GPIOA = 0x12;
    // mask in fixed values, n_SMPS_EN=0, LINK_EN=0
    val &= 0xFF9F;

    // setup transaction to GPIO register
    i2c_start_message(I2C_EXPANDER_ADDR);
    i2c_send_byte(EXT_GPIOA);
    i2c_send_byte((uint8_t)(val & 0xff));  // A-reg first
    i2c_send_byte((uint8_t)((val >> 8) & 0xff));
    i2c_stop_message();
}

void start_servo_phase(uint8_t phase) {
    // phase 0 is reserved for other uses
    if ((phase == 0) || (phase > NUM_SERVO_PHASES)) {
        return;
    }
    // disable timer interrupt
    timer_disable_irq(TIM1, TIM_DIER_CC1IE);

    // store this phase's steps in current_servo_steps
    memcpy(current_servo_steps, servo_steps[phase - 1], sizeof(servo_step_t) * SERVO_STEPS_PER_PHASE);

    current_servo_step = 0;

    // Clear all servo outputs
    current_pin_state = 0x0000;

    // if all servos in this phase are disabled, skip the phase
    if (current_servo_steps[0].enabled == false) {
        // write bit val to expander
        set_expander_output(current_pin_state);
        return;
    }

    // set compare for first split
    timer_set_period(TIM1, TICKS_BETWEEN_EDGES - 1);
    // set timer val to 0
    timer_set_counter(TIM1, 0);
    // Start counting
    timer_enable_counter(TIM1);
    // enable timer interrupt
    // this causes the ISR to run as soon as this ISR completes
    timer_enable_irq(TIM1, TIM_DIER_CC1IE);
}

void tim1_cc_isr(void) {
    servo_step_t current_step = current_servo_steps[current_servo_step];
    uint16_t bit_to_change = (uint16_t)(1 << servo_bit_mapping[current_step.idx]);
    // Handle delays from systick interrupt
    if (current_servo_step == 0) {
        // set timer val to 0
        timer_set_counter(TIM1, 0);
    }
    timer_clear_flag(TIM1, TIM_SR_CC1IF);

    timer_set_period(TIM1, current_step.next_steps - 1);
    if (current_step.enabled) {
        if (current_step.rising) {
            // set the active servo's output high
            current_pin_state |= bit_to_change;
        } else {
            // set the active servo's output low
            current_pin_state &= ~bit_to_change;
        }
    }

     if (current_servo_step == (SERVO_STEPS_PER_PHASE - 1)) {
        // this phase is complete, stop interrupts until the next phase
        timer_disable_irq(TIM1, TIM_DIER_CC1IE);
    }

    // write bit val to expander
    set_expander_output(current_pin_state);

    current_servo_step++;
}
