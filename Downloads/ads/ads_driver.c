/****************************************************************************
 * Project                     : shakti devt board
 * Name of the file            : custom_i2c_slave.c
 * Brief Description of file   : Reads/writes registers from custom Verilog I2C slave.
 * Author                      : (your name)
 * Email ID                    : (your email)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 ****************************************************************************/

/**
@file custom_i2c_slave.c
@brief Contains the driver routines to configure and read custom Verilog I2C slave.
@detail I2C based routines to configure and access 16-bit registers: conv, conf, lot, hit.
*/

#include "i2c.h"
#include "log.h"
#include "uart.h"

#define I2C                     i2c_instance[0]
#define SLAVE_ADDRESS           0x48
#define REG_SEL_ADDR            0x00   // register selector
#define REG_DATA_MSB            0x01
#define REG_DATA_LSB            0x02

#define REG_CONV                0x00
#define REG_CONF                0x01
#define REG_LOT                 0x02
#define REG_HIT                 0x03

#define PRESCALER_COUNT         0x1F
#define SCLK_COUNT              0x91

/** 
 * @fn int read_register16(i2c_struct *i2c_instance, unsigned char reg_sel, unsigned int *value, unsigned long delay)
 * @brief Reads a 16-bit register (conv, conf, lot, hit) from the I2C slave.
 */
int read_register16(i2c_struct *i2c_instance, unsigned char reg_sel, unsigned int *value, unsigned long delay)
{
    unsigned char temp;
    unsigned char msb, lsb;

    // Write slave address + write
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);

    // Point to reg_00 (selector)
    i2c_write_data(i2c_instance, REG_SEL_ADDR, delay);
    i2c_write_data(i2c_instance, reg_sel, delay);

    i2c_instance->control = I2C_STOP;

    // Write slave address + write again to set pointer
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);
    i2c_write_data(i2c_instance, REG_DATA_MSB, delay);
    i2c_instance->control = I2C_STOP;

    // Now read MSB
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_READ, delay);
    i2c_read_data(i2c_instance, &msb, delay);
    i2c_instance->control = I2C_STOP;

    // Point to LSB
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);
    i2c_write_data(i2c_instance, REG_DATA_LSB, delay);
    i2c_instance->control = I2C_STOP;

    // Read LSB
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_READ, delay);
    i2c_read_data(i2c_instance, &lsb, delay);
    i2c_instance->control = I2C_STOP;

    *value = ((msb << 8) | lsb);
    return 0;
}

/**
 * @fn int write_register16(i2c_struct *i2c_instance, unsigned char reg_sel, unsigned int value, unsigned long delay)
 * @brief Writes a 16-bit register (conf, lot, hit) in the I2C slave.
 */
int write_register16(i2c_struct *i2c_instance, unsigned char reg_sel, unsigned int value, unsigned long delay)
{
    unsigned char msb = (value >> 8) & 0xFF;
    unsigned char lsb = value & 0xFF;

    // Select register
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);
    i2c_write_data(i2c_instance, REG_SEL_ADDR, delay);
    i2c_write_data(i2c_instance, reg_sel, delay);
    i2c_instance->control = I2C_STOP;

    // Write MSB
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);
    i2c_write_data(i2c_instance, REG_DATA_MSB, delay);
    i2c_write_data(i2c_instance, msb, delay);
    i2c_instance->control = I2C_STOP;

    // Write LSB
    i2c_send_slave_address(i2c_instance, SLAVE_ADDRESS, I2C_WRITE, delay);
    i2c_write_data(i2c_instance, REG_DATA_LSB, delay);
    i2c_write_data(i2c_instance, lsb, delay);
    i2c_instance->control = I2C_STOP;

    return 0;
}

/**
 * @fn void main()
 * @brief Example program: Reads conv value and writes lot threshold.
 */
void main()
{
    unsigned long delay = 800;
    unsigned int conv_value = 0;
    unsigned int conf_value = 0;

    printf("\nHello from Custom I2C Slave Driver");

    i2c_init();

    if (config_i2c(I2C, PRESCALER_COUNT, SCLK_COUNT))
    {
        log_error("\tI2C Init Failed\n");
        return;
    }
    else
        log_info("\tI2C Init Success\n");

    // Example: write LOT = 0x1234
    write_register16(I2C, REG_LOT, 0x1234, delay);
    log_info("Wrote LOT = 0x1234");

    // Example: read CONV
    read_register16(I2C, REG_CONV, &conv_value, delay);
    log_info("Read CONV = 0x%04x", conv_value);

    // Example: read CONF
    read_register16(I2C, REG_CONF, &conf_value, delay);
    log_info("Read CONF = 0x%04x", conf_value);

    while(1)
    {
        read_register16(I2C, REG_CONV, &conv_value, delay);
        log_info("Loop: Conversion Value = 0x%04x", conv_value);
        delay_loop(1000, 1000);
    }
}
