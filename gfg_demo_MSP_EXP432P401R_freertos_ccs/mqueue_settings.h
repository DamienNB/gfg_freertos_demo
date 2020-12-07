/*
 * mqueue_settings.h
 *
 *  Created on: Dec 6, 2020
 *      Author: Damien
 */

#ifndef MQUEUE_SETTINGS_H_
#define MQUEUE_SETTINGS_H_

#define QUEUE_NAME               "/accelerometer_data"
#define MAX_QUEUE_MESSAGES       5
#define MAX_QUEUE_MESSAGE_SIZE   ((3*sizeof(signed short int))+1)

#endif /* MQUEUE_SETTINGS_H_ */
