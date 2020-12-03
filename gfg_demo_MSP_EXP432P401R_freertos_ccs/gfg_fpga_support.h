/*
 * gfg_fpga_support.h
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#ifndef GFG_FPGA_SUPPORT_H_
#define GFG_FPGA_SUPPORT_H_

#include <ti/drivers/SPI.h>

void gfg_fpga_initialize_driver(SPI_Handle spiHndl);

#endif /* GFG_FPGA_SUPPORT_H_ */
