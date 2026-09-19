#ifndef _CAR_ENCODER_H_
#define _CAR_ENCODER_H_

/* L2 BSP - 双路 AB 正交编码器 GPIO 四倍频解码。 */
#include "zf_common_typedef.h"

typedef enum
{
    CAR_ENCODER_LEFT = 0,
    CAR_ENCODER_RIGHT,
    CAR_ENCODER_COUNT,
}car_encoder_id_enum;

void car_encoder_init(void);
void car_encoder_sample(void);
int16 car_encoder_get_delta(car_encoder_id_enum encoder);
int32 car_encoder_get_total(car_encoder_id_enum encoder);
uint32 car_encoder_get_invalid(car_encoder_id_enum encoder);
void car_encoder_reset(void);

#endif
