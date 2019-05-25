#ifndef __IMU_H
#define __IMU_H

#include "includes.h"

#define IMULength	19

extern volatile uint32_t g_imu_time;

typedef struct {
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t x_acc;
    int16_t y_acc;
    int16_t z_acc;
	uint32_t timestamp;
} IMU_DATA_TypeDef;

extern IMU_DATA_TypeDef imu_data;
extern SemaphoreHandle_t xSemaphore_imu_h;

/* 供外部调用的函数声明 */
void IMUSend(void);

#endif
																					 