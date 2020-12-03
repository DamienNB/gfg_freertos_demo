/*
 * gfg_fpga.h
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#ifndef GFG_FPGA_H_
#define GFG_FPGA_H_

#include <stdint.h>
#include <ti/drivers/SPI.h>

int_fast8_t gfg_fpga_write_reg(uint8_t reg_addr, uint32_t reg_val);

int_fast8_t gfg_fgpa_init(SPI_Handle spiHndl);

#endif /* GFG_FPGA_H_ */
