/*
 * gfg_fpga_support.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

/* POSIX Header files */
#include <mqueue.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* TI Driver Header files */
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* Support code includes */
#include "mqueue_settings.h"
#include "gfg_fpga.h"
#include "gfg_fpga_support.h"

#define GFG_FPGA_TASK_STACK_SIZE   (1024)
#define GFG_FPGA_TASK_PRIORITY        (3)

sem_t gfg_fpga_spi_sem;

extern Display_Handle display;

pthread_t gfg_task;

/*
 *  ======== bmiInterruptHandlerTask ========
 *  This task sends out new data for drawing a triangle to the GFG FPGA after
 *  receiving new magnetometer data
 */
static void* gfg_fpga_task(void *arg0) {

    mqd_t magnetometer_queue;

    magnetometer_queue = mq_open(QUEUE_NAME, O_RDONLY);

    uint_fast16_t color = 0;
    while(1) {
        char buffer[MAX_QUEUE_MESSAGE_SIZE+1] = { 0 };

        mq_receive(magnetometer_queue, (char *)buffer, MAX_QUEUE_MESSAGE_SIZE, NULL);

        signed int magnetometer_x = ((signed int *)buffer)[0];
        signed int magnetometer_y = ((signed int *)buffer)[1];
        signed int magnetometer_z = ((signed int *)buffer)[2];

        Display_print3(display, 0, 0, "magno message : x = %d, y = %d,z = %d\n",
                       magnetometer_x, magnetometer_y, magnetometer_z);

        color += 37;

        if(color > 0x0fff)
        {
            color %= 37;
        }

        gfg_fpga_write_triangle(11, 10, 50, 10, 30, 60, color);

//        sleep(1);
//        usleep(33333u);
    }
}


void gfg_fpga_initialize_driver(SPI_Handle spi_hndl) {
    pthread_attr_t       p_attrs;
    struct sched_param   pri_param;
    int retc;

    /* The semaphore is used by the GFG FPGA task
    * The task sends out new data for drawing a triangle to the GFG FPGA
    */
    if(0 != sem_init(&gfg_fpga_spi_sem,0,0))
    {
        /* sem_init() failed */
        Display_print0(display, 0, 0, "gfg_fpga_spi_sem Semaphore creation failed");
        while (1);
    }

    if(!gfg_fgpa_init(spi_hndl))
    {
        Display_print0(display, 0, 0, "gfg_fpga_init failed");
        while(1);
    }

    pthread_attr_init(&p_attrs);
    /* Set priority and stack size attributes */
    pthread_attr_setstacksize(&p_attrs, GFG_FPGA_TASK_STACK_SIZE);
    pri_param.sched_priority = 3;
    pthread_attr_setschedparam(&p_attrs, &pri_param);
    retc = pthread_create(&gfg_task, &p_attrs, gfg_fpga_task, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        Display_print0(display, 0, 0, "GFG FPGA Task creation failed");
        while (1);
    }
}
