/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Source Code Derivative Author: Damien Nikola Bobrek
 */

/*
 *    ======== gfg.c ========
 */
#include <stdint.h>
#include <stddef.h>

/* POSIX Header file */
#include <semaphore.h>
#include <mqueue.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>

#include <ti/display/Display.h>

/* Module Header */
#include <ti/sail/bmi160/bmi160.h>

/* Demo header files */
#include "mqueue_settings.h"
#include "bmi160_support.h"
#include "gfg_fpga_support.h"

/* Driver configuration */
#include "ti_drivers_config.h"

I2C_Handle      i2c;
I2C_Params      i2cParams;

SPI_Handle spi;
SPI_Params spiParams;

Display_Handle display;

mqd_t magnetometer_data_queue;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    SPI_init();

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }
    Display_print0(display, 0, 0, "Starting the i2cbmi160 sensor example...\n\n");

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.transferCallbackFxn = NULL;
    i2c = I2C_open(CONFIG_I2C_BMI, &i2cParams);
    if (i2c == NULL) {
        Display_print0(display, 0, 0, "Error Initializing I2C\n");
    }
    else {
        Display_print0(display, 0, 0, "I2C Initialized!\n");
    }

    SPI_Params_init(&spiParams);
    spiParams.transferMode        = SPI_MODE_BLOCKING;
    spiParams.mode                = SPI_MASTER;
    spiParams.bitRate             = 8000000; // Hz
    spiParams.dataSize            = 8;
    spiParams.frameFormat         = SPI_POL1_PHA1; //SPI_POL1_PHA0;
    spi = SPI_open(GFG_SPI, &spiParams);
    if (spi == NULL)
    {
        Display_print0(display, 0, 0, "Error Initializing SPI\n");
    }
    else
    {
        Display_print0(display, 0, 0, "SPI Initialized!\n");
    }

    mqd_t accelerometer_queue;
    struct mq_attr queue_attr;

    queue_attr.mq_flags   = 0;
    queue_attr.mq_maxmsg  = MAX_QUEUE_MESSAGES;
    queue_attr.mq_msgsize = MAX_QUEUE_MESSAGE_SIZE;
    queue_attr.mq_curmsgs = 0;

    accelerometer_queue = mq_open(QUEUE_NAME, O_CREAT | O_RDONLY, 0644, &queue_attr);

    bmi160_initialize_sensor(i2c);

    gfg_fpga_initialize_driver(spi);

    return (0);
}
