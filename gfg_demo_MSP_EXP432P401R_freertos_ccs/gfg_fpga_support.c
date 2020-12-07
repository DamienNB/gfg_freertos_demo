/*
 * gfg_fpga_support.c
 *
 *  Created on: Dec 2, 2020
 *      Author: Damien
 */

#include <math.h>

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

    mqd_t accelerometer_queue;

    accelerometer_queue = mq_open(QUEUE_NAME, O_RDONLY);

    uint_fast16_t color = 0;

    while(1) {
        char buffer[MAX_QUEUE_MESSAGE_SIZE+1] = { 0 };

        mq_receive(accelerometer_queue, (char *)buffer, MAX_QUEUE_MESSAGE_SIZE, NULL);

        signed short int accelerometer_x = ((signed short int *)buffer)[0];
        signed short int accelerometer_y = ((signed short int *)buffer)[1];
        signed short int accelerometer_z = ((signed short int *)buffer)[2];

        float cos_theta =  accelerometer_z;
        float sin_theta =  -(accelerometer_y + accelerometer_x)/2;

        // normalize the vector
        float magnitude = sqrtf((cos_theta*cos_theta) + (sin_theta*sin_theta));
        cos_theta /= magnitude;
        sin_theta /= magnitude;

        const float point_0_scalar_x =   00.0f;
        const float point_0_scalar_y =  -15.0f;
        const float point_1_scalar_x =  -20.0f;
        const float point_1_scalar_y =   15.0f;
        const float point_2_scalar_x =   20.0f;
        const float point_2_scalar_y =   15.0f;

        const unsigned int offset_x = 40;
        const unsigned int offset_y = 30;

        long point_0_x = lroundf( (cos_theta     * point_0_scalar_x) + (sin_theta * point_0_scalar_y)) + offset_x;
        long point_0_y = lroundf(((-1*sin_theta) * point_0_scalar_x) + (cos_theta * point_0_scalar_y)) + offset_y;
        long point_1_x = lroundf( (cos_theta     * point_1_scalar_x) + (sin_theta * point_1_scalar_y)) + offset_x;
        long point_1_y = lroundf(((-1*sin_theta) * point_1_scalar_x) + (cos_theta * point_1_scalar_y)) + offset_y;
        long point_2_x = lroundf( (cos_theta     * point_2_scalar_x) + (sin_theta * point_2_scalar_y)) + offset_x;
        long point_2_y = lroundf(((-1*sin_theta) * point_2_scalar_x) + (cos_theta * point_2_scalar_y)) + offset_y;

        Display_print3(display, 0, 0, "\naccelo message   : x = %d, y = %d, z = %d",
                       accelerometer_x, accelerometer_y, accelerometer_z);
        Display_print2(display, 0, 0, "Triangle point 0 : x0 = %d, y0 = %d",
                       point_0_x, point_0_y);
        Display_print2(display, 0, 0, "Triangle point 1 : x1 = %d, y1 = %d",
                       point_1_x, point_1_y);
        Display_print2(display, 0, 0, "Triangle point 2 : x2 = %d, y2 = %d\n",
                       point_2_x, point_2_y);

        color += 37;

        if(color > 0x0fff)
        {
            color %= 37;
        }

        gfg_fpga_write_triangle(point_0_x, point_0_y, point_1_x, point_1_y,
                                point_2_x, point_2_y, color);
//        gfg_fpga_write_triangle(11, 10, 50, 10, 30, 60, color);

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
