/*
 * gfg_fpga.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#include "gfg_fpga.h"

static SPI_Handle spi_handle = NULL;

int_fast8_t gfg_fgpa_init(SPI_Handle spiHndl) {
    spi_handle = spiHndl;

    // See if a SPI works by trying to send a series of NO_OP commands
    if(!gfg_fpga_send_5_no_ops())
    {
        return -1;
    }

    // Check to see if a register can be written to
    if(!gfg_fpga_write_reg(GFG_FPGA_TRIANGLE_POINT_0_X_ADDR, 37))
    {
        return -2;
    }
    if(gfg_fpga_read_reg(GFG_FPGA_TRIANGLE_POINT_0_X_ADDR) != 37)
    {
        return -3;
    }

    // Check to see if a register can be re-written to
    if(!gfg_fpga_write_reg(GFG_FPGA_TRIANGLE_POINT_0_X_ADDR, 0))
    {
        return -4;
    }
    if(gfg_fpga_read_reg(GFG_FPGA_TRIANGLE_POINT_0_X_ADDR) != 0)
    {
        return -5;
    }

    // Try to clear the screen
    if(!gfg_fpga_write_triangle(0, 0, 0, 0, 0, 0, 0))
    {
        return -6;
    }

    return 0;
}


int_fast64_t gfg_fpga_read_reg(uint8_t reg_addr) {
    // Two read commands are sent in succession.
    // The correct value is returned on the second read, but depends on the
    // address requested by the first read.

    uint8_t tx_buf[10] = { 0xFF };

    // load the read command and register address twice
    tx_buf[0] = GFG_FPGA_READ_CMD | (reg_addr & 0x1F);
    tx_buf[5] = GFG_FPGA_READ_CMD | (reg_addr & 0x1F);

    uint8_t rx_buf[10] = { 0x00 };

    SPI_Transaction spiTransaction;
    spiTransaction.txBuf = tx_buf;
    spiTransaction.count = sizeof(tx_buf);
    spiTransaction.rxBuf = rx_buf;

    if(!SPI_transfer(spi_handle, &spiTransaction))
    {
        return -1;
    }

    uint32_t value_read = 0;
    value_read |= ((uint32_t)tx_buf[6]) << 24u;
    value_read |= ((uint32_t)tx_buf[7]) << 16u;
    value_read |= ((uint32_t)tx_buf[8]) <<  8u;
    value_read |= ((uint32_t)tx_buf[9]) <<  0u;

    return value_read;
}

int_fast8_t gfg_fpga_send_5_no_ops(void) {
    int_fast8_t ierror = 0;

    // See if a SPI works by trying to send a series of NO_OP commands
    uint8_t tx_buf[5] = { GFG_FPGA_NO_OP_CMD };

    SPI_Transaction spiTransaction;
    spiTransaction.txBuf = tx_buf;
    spiTransaction.count = sizeof(tx_buf);
    spiTransaction.rxBuf = NULL; // discard data received

    if(!SPI_transfer(spi_handle, &spiTransaction))
    {
        ierror = -1;
    }

    return ierror;
}

int_fast8_t gfg_fpga_write_reg(uint8_t reg_addr, uint32_t reg_val) {
    int_fast8_t ierror = 0;

    uint8_t tx_buf[5];

    // load the command and register address
    tx_buf[0] = GFG_FPGA_WRITE_CMD | (reg_addr & 0x1F);
    // load the value into the buffer to be transmitted in big endian order.
    tx_buf[1] = (uint8_t) ((reg_val & 0xff000000) >> 24u);
    tx_buf[2] = (uint8_t) ((reg_val & 0x00ff0000) >> 16u);
    tx_buf[3] = (uint8_t) ((reg_val & 0x0000ff00) >>  8u);
    tx_buf[4] = (uint8_t) ((reg_val & 0x000000ff) >>  0u);

    SPI_Transaction spiTransaction;
    spiTransaction.txBuf = tx_buf;
    spiTransaction.count = sizeof(tx_buf);
    spiTransaction.rxBuf = NULL; // discard data received

    if(!SPI_transfer(spi_handle, &spiTransaction))
    {
        ierror = -1;
    }

    return ierror;
}

int_fast8_t gfg_fpga_write_triangle(uint_fast16_t point_0_x,
                                    uint_fast16_t point_0_y,
                                    uint_fast16_t point_1_x,
                                    uint_fast16_t point_1_y,
                                    uint_fast16_t point_2_x,
                                    uint_fast16_t point_2_y,
                                    uint_fast16_t color) {
    int_fast8_t ierror = 0;

    uint8_t tx_buf[] =
    {
        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_0_X_ADDR,
        0,
        0,
        (uint8_t) (point_0_x & 0xff00) >> 8u,
        (uint8_t) (point_0_x & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_0_Y_ADDR,
        0,
        0,
        (uint8_t) (point_0_y & 0xff00) >> 8u,
        (uint8_t) (point_0_y & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_1_X_ADDR,
        0,
        0,
        (uint8_t) (point_1_x & 0xff00) >> 8u,
        (uint8_t) (point_1_x & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_1_Y_ADDR,
        0,
        0,
        (uint8_t) (point_1_y & 0xff00) >> 8u,
        (uint8_t) (point_1_y & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_2_X_ADDR,
        0,
        0,
        (uint8_t) (point_2_x & 0xff00) >> 8u,
        (uint8_t) (point_2_x & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_POINT_2_Y_ADDR,
        0,
        0,
        (uint8_t) (point_2_y & 0xff00) >> 8u,
        (uint8_t) (point_2_y & 0x00ff),

        GFG_FPGA_WRITE_CMD | GFG_FPGA_TRIANGLE_COLOR_ADDR,
        0,
        0,
        (uint8_t) (color & 0x0f00) >> 8u,
        (uint8_t) (color & 0x00ff)
    };

    SPI_Transaction spiTransaction;
    spiTransaction.txBuf = tx_buf;
    spiTransaction.count = sizeof(tx_buf);
    spiTransaction.rxBuf = NULL; // discard data received

    if(!SPI_transfer(spi_handle, &spiTransaction))
    {
        ierror = -1;
    }

    return ierror;
}
