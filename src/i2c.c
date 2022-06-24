#include "i2c.h"

/// TODO watchdog to prevent I2C lockups

void i2c_init(void){
    // Set I2C alternate functions on PB6 & PB7
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_I2C1_SCL | GPIO_I2C1_SDA);

    // Enable clock for I2C
    rcc_periph_clock_enable(RCC_I2C1);

    i2c_reset(I2C1);

    // Set APB1 to 14MHz
    i2c_set_clock_frequency(I2C1, 14);
    // Configure clocks for 400kHz I2C
    i2c_set_speed(I2C1, i2c_speed_fm_400k, 14);

    i2c_peripheral_enable(I2C1);
}

void i2c_start_message(uint8_t addr, uint8_t recv){
    uint32_t reg32 __attribute__((unused));

    // Send START condition.
    i2c_send_start(I2C1);

    // Waiting for START to send and switch to master mode.
    while (!((I2C_SR1(I2C1) & I2C_SR1_SB)
        && (I2C_SR2(I2C1) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    // Say to what address we want to talk to.
    i2c_send_7bit_address(I2C1, addr, recv);

    if (recv == I2C_READ) {
        // Set ack for future bytes.
        i2c_enable_ack(I2C1);
    }

    // Waiting for address to transfer.
    while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR));

    // Cleaning ADDR condition sequence.
    reg32 = I2C_SR2(I2C1);
}

void i2c_stop_message(void){
    // Wait for the data register to be empty.
    // This will not be set if a NACK is received.
    while (!(I2C_SR1(I2C1) & I2C_SR1_TxE));

    // Send STOP condition.
    i2c_send_stop(I2C1);
}

void i2c_send_byte(char c){
    i2c_send_data(I2C1, c);
    // Wait for byte to complete transferring
    while (!(I2C_SR1(I2C1) & I2C_SR1_BTF));
}

char i2c_recv_byte(bool last_byte){
    // Respond NACK to last byte
    if (last_byte) {
        i2c_disable_ack(I2C1);
    }

    // Wait for the receive register to not be empty
    while (!(I2C_SR1(I2C1) & I2C_SR1_RxNE));
    char res = i2c_get_data(I2C1);

    // End the transmission
    if (last_byte) {
        i2c_send_stop(I2C1);
    }

    return res;
}

// Set expander into byte/bank mode (IOCON.SEQOP=1, IOCON.BANK=0)
// configure pin directions and pullups
void init_expander(uint8_t addr){
    const uint8_t IOCON = 0x0A;
    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(IOCON);
    i2c_send_byte(1 << 5);  // SEQOP=1, BANK=0
    i2c_stop_message();

    // these values assume IOCON.BANK = 0
    const uint8_t EXT_GPIOA = 0x12;
    const uint8_t GPPUA = 0x0C;
    const uint8_t IODIRA = 0x00;

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(IODIRA);
    i2c_send_byte(0x90);  // PGOOD, VAUX_MON inputs
    i2c_send_byte(0x00);  // all outputs
    i2c_stop_message();

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(EXT_GPIOA);
    i2c_send_byte(0x20);  // SMPS_EN
    i2c_stop_message();

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(GPPUA);
    i2c_send_byte(0x90);  // PGOOD, VAUX_MON
    i2c_stop_message();
}
