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

/*******************/
/**\name GFG_FPGA COMMANDS */
/*******************/
#define GFG_FPGA_NO_OP_CMD                      (0x00)
#define GFG_FPGA_WRITE_CMD                      (0x80)
#define GFG_FPGA_READ_CMD                       (0x40)

/*******************/
/**\name GFG_FPGA REGISTERS */
/*******************/
#define GFG_FPGA_TRIANGLE_POINT_0_X_ADDR        (0x00)
#define GFG_FPGA_TRIANGLE_POINT_0_Y_ADDR        (0x01)
#define GFG_FPGA_TRIANGLE_POINT_1_X_ADDR        (0x02)
#define GFG_FPGA_TRIANGLE_POINT_1_Y_ADDR        (0x03)
#define GFG_FPGA_TRIANGLE_POINT_2_X_ADDR        (0x04)
#define GFG_FPGA_TRIANGLE_POINT_2_Y_ADDR        (0x05)
#define GFG_FPGA_TRIANGLE_COLOR_ADDR            (0X06)

int_fast8_t gfg_fgpa_init(SPI_Handle spiHndl);

int_fast64_t gfg_fpga_read_reg(uint8_t reg_addr);

int_fast8_t gfg_fpga_send_5_no_ops(void);

int_fast8_t gfg_fpga_write_reg(uint8_t reg_addr, uint32_t reg_val);

int_fast8_t gfg_fpga_write_triangle(uint_fast16_t point_0_x,
                                    uint_fast16_t point_0_y,
                                    uint_fast16_t point_1_x,
                                    uint_fast16_t point_1_y,
                                    uint_fast16_t point_2_x,
                                    uint_fast16_t point_2_y,
                                    uint_fast16_t color);

#endif /* GFG_FPGA_H_ */
