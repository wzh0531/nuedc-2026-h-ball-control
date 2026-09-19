#ifndef _CAR_MOTOR_H_
#define _CAR_MOTOR_H_

/* L2 BSP - 扩展板板载 TB6612 双路电机输出。 */
#include "zf_common_typedef.h"

typedef enum
{
    CAR_MOTOR_LEFT = 0,
    CAR_MOTOR_RIGHT,
    CAR_MOTOR_COUNT,
}car_motor_id_enum;

void car_motor_init(void);
void car_motor_set(car_motor_id_enum motor, int16 output);
void car_motor_set_pair(int16 left, int16 right);
void car_motor_stop(void);
int16 car_motor_get_output(car_motor_id_enum motor);

#endif
