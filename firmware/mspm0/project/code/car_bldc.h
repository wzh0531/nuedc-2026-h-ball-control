#ifndef _CAR_BLDC_H_
#define _CAR_BLDC_H_

/* L3 Driver - 串口无刷电机协议，移植自 trackball/mspcar/gimbal。 */
#include "zf_common_typedef.h"

#define CAR_BLDC_HEADER                    (0x7AU)
#define CAR_BLDC_TAIL                      (0x7BU)
#define CAR_BLDC_ADDR_1                    (0x01U)
#define CAR_BLDC_ADDR_2                    (0x02U)

#define CAR_BLDC_CMD_MODE                  (0x00U)
#define CAR_BLDC_CMD_SPEED                 (0x01U)
#define CAR_BLDC_CMD_MULTI_POSITION        (0x02U)
#define CAR_BLDC_CMD_SINGLE_POSITION       (0x03U)
#define CAR_BLDC_CMD_DISABLE               (0x05U)
#define CAR_BLDC_CMD_ENABLE                (0x06U)
#define CAR_BLDC_CMD_ACCELERATION          (0x07U)
#define CAR_BLDC_CMD_SAVE                  (0x08U)
#define CAR_BLDC_CMD_CLEAR_MULTI           (0x09U)
#define CAR_BLDC_CMD_SET_ZERO              (0x0AU)
#define CAR_BLDC_CMD_FACTORY_RESET         (0x0BU)
#define CAR_BLDC_CMD_SET_ADDRESS           (0x0DU)
#define CAR_BLDC_CMD_FEEDBACK              (0x0EU)

#define CAR_BLDC_MODE_SPEED                (0x0000U)
#define CAR_BLDC_MODE_MULTI_POSITION       (0x0001U)
#define CAR_BLDC_MODE_SINGLE_POSITION      (0x0002U)
#define CAR_BLDC_MODE_MULTI_POSITION_L     (0x0003U)
#define CAR_BLDC_MODE_SINGLE_POSITION_L    (0x0004U)

typedef enum
{
    CAR_BLDC_FEEDBACK_SPEED = 0,
    CAR_BLDC_FEEDBACK_MULTI_ANGLE,
    CAR_BLDC_FEEDBACK_SINGLE_ANGLE,
    CAR_BLDC_FEEDBACK_ACCELERATION,
    CAR_BLDC_FEEDBACK_VOLTAGE,
}car_bldc_feedback_enum;

typedef struct
{
    int16 speed_rpm;
    int32 multi_angle_x10;
    uint16 single_angle_x10;
    int16 acceleration_rps2;
    uint16 voltage_x100;
    uint8 data_ready;
}car_bldc_motor_data_struct;

typedef void (*car_bldc_send_function)(const uint8 *data, uint8 length);

extern volatile car_bldc_motor_data_struct car_bldc_motor_1;
extern volatile car_bldc_motor_data_struct car_bldc_motor_2;

void car_bldc_set_send_function(car_bldc_send_function function);
uint8 car_bldc_calculate_bcc(const uint8 *data, uint8 length);
void car_bldc_parse_rx_byte(uint8 data);

void car_bldc_enable(uint8 address);
void car_bldc_disable(uint8 address);
void car_bldc_set_mode(uint8 address, uint16 mode);
void car_bldc_set_speed(uint8 address, int16 rpm);
void car_bldc_set_acceleration(uint8 address, uint16 acceleration_rps2);
void car_bldc_set_multi_angle(uint8 address, int32 angle_x10);
void car_bldc_set_single_angle(uint8 address, uint16 angle_x10);
void car_bldc_request_feedback(uint8 address,
    car_bldc_feedback_enum feedback);
void car_bldc_save_parameters(uint8 address);
void car_bldc_clear_multi_angle(uint8 address);
void car_bldc_set_single_angle_zero(uint8 address);
void car_bldc_factory_reset(uint8 address);
void car_bldc_set_address(uint8 address, uint8 new_address);

#endif
