#include "car_bldc.h"

#include <string.h>

#define CAR_BLDC_RX_FRAME_LENGTH            (9U)
#define CAR_BLDC_TX_BUFFER_LENGTH           (20U)
#define CAR_BLDC_TX_MAX_DATA_LENGTH         \
    (CAR_BLDC_TX_BUFFER_LENGTH - 5U)

volatile car_bldc_motor_data_struct car_bldc_motor_1;
volatile car_bldc_motor_data_struct car_bldc_motor_2;

static car_bldc_send_function bldc_send;
static uint8 bldc_rx_buffer[CAR_BLDC_RX_FRAME_LENGTH];
static uint8 bldc_rx_index;

static uint8 car_bldc_address_is_valid(uint8 address)
{
    return (uint8)((CAR_BLDC_ADDR_1 == address) ||
        (CAR_BLDC_ADDR_2 == address));
}

static void car_bldc_send_command(uint8 address, uint8 command,
    const uint8 *data, uint8 length)
{
    uint8 buffer[CAR_BLDC_TX_BUFFER_LENGTH];
    uint8 index = 0U;

    if(!car_bldc_address_is_valid(address) ||
        (length > CAR_BLDC_TX_MAX_DATA_LENGTH) ||
        ((length > 0U) && (NULL == data)))
    {
        return;
    }

    buffer[index++] = CAR_BLDC_HEADER;
    buffer[index++] = address;
    buffer[index++] = command;
    if(length > 0U)
    {
        memcpy(&buffer[index], data, length);
        index += length;
    }
    buffer[index] = car_bldc_calculate_bcc(buffer, index);
    index++;
    buffer[index++] = CAR_BLDC_TAIL;

    if(NULL != bldc_send)
    {
        bldc_send(buffer, index);
    }
}

static void car_bldc_store_feedback(void)
{
    uint8 address = bldc_rx_buffer[1];
    uint8 type = bldc_rx_buffer[2];
    uint32 raw_value = ((uint32)bldc_rx_buffer[3] << 24) |
        ((uint32)bldc_rx_buffer[4] << 16) |
        ((uint32)bldc_rx_buffer[5] << 8) |
        (uint32)bldc_rx_buffer[6];
    int32 value = (int32)raw_value;
    volatile car_bldc_motor_data_struct *motor =
        (CAR_BLDC_ADDR_1 == address) ?
        &car_bldc_motor_1 : &car_bldc_motor_2;

    switch(type)
    {
        case CAR_BLDC_FEEDBACK_SPEED:
            motor->speed_rpm = (int16)(value & 0xFFFF);
            break;
        case CAR_BLDC_FEEDBACK_MULTI_ANGLE:
            motor->multi_angle_x10 = value;
            break;
        case CAR_BLDC_FEEDBACK_SINGLE_ANGLE:
            motor->single_angle_x10 = (uint16)(value & 0xFFFF);
            break;
        case CAR_BLDC_FEEDBACK_ACCELERATION:
            motor->acceleration_rps2 = (int16)(value & 0xFFFF);
            break;
        case CAR_BLDC_FEEDBACK_VOLTAGE:
            motor->voltage_x100 = (uint16)(value & 0xFFFF);
            break;
        default:
            return;
    }
    motor->data_ready = 1U;
}

void car_bldc_set_send_function(car_bldc_send_function function)
{
    bldc_send = function;
}

uint8 car_bldc_calculate_bcc(const uint8 *data, uint8 length)
{
    uint8 bcc = 0U;
    uint8 index;

    if(NULL == data)
    {
        return 0U;
    }
    for(index = 0U; index < length; index++)
    {
        bcc ^= data[index];
    }
    return bcc;
}

void car_bldc_parse_rx_byte(uint8 data)
{
    if(0U == bldc_rx_index)
    {
        if(CAR_BLDC_HEADER == data)
        {
            bldc_rx_buffer[bldc_rx_index++] = data;
        }
        return;
    }

    if((1U == bldc_rx_index) && !car_bldc_address_is_valid(data))
    {
        bldc_rx_index = (CAR_BLDC_HEADER == data) ? 1U : 0U;
        return;
    }
    if((2U == bldc_rx_index) &&
        (data > (uint8)CAR_BLDC_FEEDBACK_VOLTAGE))
    {
        bldc_rx_index = (CAR_BLDC_HEADER == data) ? 1U : 0U;
        return;
    }

    bldc_rx_buffer[bldc_rx_index++] = data;
    if(bldc_rx_index < CAR_BLDC_RX_FRAME_LENGTH)
    {
        return;
    }

    if((CAR_BLDC_TAIL == bldc_rx_buffer[8]) &&
        (bldc_rx_buffer[7] ==
        car_bldc_calculate_bcc(bldc_rx_buffer, 7U)))
    {
        car_bldc_store_feedback();
    }
    bldc_rx_index = 0U;
}

void car_bldc_enable(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_ENABLE, NULL, 0U);
}

void car_bldc_disable(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_DISABLE, NULL, 0U);
}

void car_bldc_set_mode(uint8 address, uint16 mode)
{
    uint8 data[2] = {(uint8)(mode >> 8), (uint8)mode};
    car_bldc_send_command(address, CAR_BLDC_CMD_MODE, data, sizeof(data));
}

void car_bldc_set_speed(uint8 address, int16 rpm)
{
    uint16 raw = (uint16)rpm;
    uint8 data[2] = {(uint8)(raw >> 8), (uint8)raw};
    car_bldc_send_command(address, CAR_BLDC_CMD_SPEED, data, sizeof(data));
}

void car_bldc_set_acceleration(uint8 address, uint16 acceleration_rps2)
{
    uint8 data[2] = {(uint8)(acceleration_rps2 >> 8),
        (uint8)acceleration_rps2};
    car_bldc_send_command(address, CAR_BLDC_CMD_ACCELERATION,
        data, sizeof(data));
}

void car_bldc_set_multi_angle(uint8 address, int32 angle_x10)
{
    uint32 raw = (uint32)angle_x10;
    uint8 data[4] = {(uint8)(raw >> 24), (uint8)(raw >> 16),
        (uint8)(raw >> 8), (uint8)raw};
    car_bldc_send_command(address, CAR_BLDC_CMD_MULTI_POSITION,
        data, sizeof(data));
}

void car_bldc_set_single_angle(uint8 address, uint16 angle_x10)
{
    uint8 data[2];

    if(angle_x10 > 3599U)
    {
        angle_x10 = 3599U;
    }
    data[0] = (uint8)(angle_x10 >> 8);
    data[1] = (uint8)angle_x10;
    car_bldc_send_command(address, CAR_BLDC_CMD_SINGLE_POSITION,
        data, sizeof(data));
}

void car_bldc_request_feedback(uint8 address,
    car_bldc_feedback_enum feedback)
{
    uint8 data = (uint8)feedback;
    car_bldc_send_command(address, CAR_BLDC_CMD_FEEDBACK, &data, 1U);
}

void car_bldc_save_parameters(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_SAVE, NULL, 0U);
}

void car_bldc_clear_multi_angle(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_CLEAR_MULTI, NULL, 0U);
}

void car_bldc_set_single_angle_zero(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_SET_ZERO, NULL, 0U);
}

void car_bldc_factory_reset(uint8 address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_FACTORY_RESET, NULL, 0U);
}

void car_bldc_set_address(uint8 address, uint8 new_address)
{
    car_bldc_send_command(address, CAR_BLDC_CMD_SET_ADDRESS,
        &new_address, 1U);
}
