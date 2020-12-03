/*
 * gfg_fpga_support.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#include "gfg_fpga.h"
#include "gfg_fpga_support.h"

void gfg_fpga_initialize_driver(SPI_Handle spiHndl) {
    gfg_fgpa_init(spiHndl);
}
