
#ifndef __CSRO_COMMON_H
#define __CSRO_COMMON_H

#include "stm32f1xx_hal.h"
#include "mb_config.h"

typedef struct
{
    uint8_t input[6];
    uint8_t relay[8];
    int16_t ntc[4];
    int16_t temp[2];
    uint16_t humi[2];
    uint16_t pwm[4];
} csro_info;

extern csro_info sysinfo;

float Csro_Calculate_ntc3950_Temperature_from_Resvalue(float res_value);
float Csro_Calculate_ntc3380_Temperature_from_Resvalue(float res_value);

#endif
