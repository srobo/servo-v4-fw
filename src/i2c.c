#include "i2c.h"

/// TODO watchdog to prevent I2C lockups

void i2c_init(void){
    ;
}

void i2c_start_message(uint8_t addr, uint8_t recv){
    (void)addr;
    (void)recv;
    ;
}

void i2c_stop_message(void){
    ;
}

void i2c_send_byte(char c){
    (void)c;
    ;
}

char i2c_recv_byte(void){
    return 0;
    ;
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
    const uint8_t GPIOA = 0x12;
    const uint8_t GPPUA = 0x0C;
    const uint8_t IODIRA = 0x00;

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(IODIRA);
    i2c_send_byte(0x90);  // PGOOD, VAUX_MON inputs
    i2c_send_byte(0x00);  // all outputs
    i2c_stop_message();

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(GPIOA);
    i2c_send_byte(0x20);  // SMPS_EN
    i2c_stop_message();

    i2c_start_message(addr, I2C_WRITE);
    i2c_send_byte(GPPUA);
    i2c_send_byte(0x90);  // PGOOD, VAUX_MON
    i2c_stop_message();
}
