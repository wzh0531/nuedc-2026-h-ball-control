#ifndef _CAR_SAFETY_H_
#define _CAR_SAFETY_H_

/* L5 Service - 底盘故障判据和锁定。 */
#include "zf_common_typedef.h"

#define CHASSIS_FAULT_NONE                     (0x00000000UL)
#define CHASSIS_FAULT_LEFT_STALL               (0x00000001UL)
#define CHASSIS_FAULT_RIGHT_STALL              (0x00000002UL)
#define CHASSIS_FAULT_LINE_LOST                (0x00000004UL)
#define CHASSIS_FAULT_IMU                      (0x00000008UL)
#define CHASSIS_FAULT_CONTROL_OVERRUN          (0x00000010UL)
#define CHASSIS_FAULT_ENCODER_SIGNAL           (0x00000020UL)
#define CHASSIS_FAULT_EMERGENCY_KEY            (0x00000040UL)

void car_safety_init(void);
void car_safety_reset(void);
uint32 car_safety_update(uint8 line_valid);
void car_safety_latch(uint32 fault_bits);
uint32 car_safety_get_faults(void);
uint8 car_safety_get_line_lost_ticks(void);

#endif
