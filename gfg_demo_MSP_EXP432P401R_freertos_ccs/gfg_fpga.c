/*
 * gfg_fpga.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#include "gfg_fpga.h"

static SPI_Handle spi_handle = NULL;

int_fast8_t gfg_fpga_write_reg(uint8_t reg_addr, uint32_t reg_val) {
    int_fast8_t ierror = 0;

    uint8_t tx_buf[5];

    // load the command and register address
    tx_buf[0] = 0x40 | (reg_addr & 0x1F);

    // swap the endianess of reg_val and load it into tx_buf
    int_fast8_t i;
    for(i = 1; i <= 4; i++) {
        tx_buf[i] = *(((uint8_t *) &reg_val) + (4-i));
    }

    uint8_t rx_buf[5] = {0};

    SPI_Transaction spiTransaction;
    spiTransaction.txBuf = tx_buf;
    spiTransaction.count = 5;
    spiTransaction.rxBuf = rx_buf;

    if(!SPI_transfer(spi_handle, &spiTransaction))
    {
        ierror = -1;
    }

    return ierror;
}

int_fast8_t gfg_fgpa_init(SPI_Handle spiHndl) {
    spi_handle = spiHndl;
}
