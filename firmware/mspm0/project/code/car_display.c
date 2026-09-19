#include "car_display.h"

#include <stdio.h>
#include <string.h>

#include "car_board.h"
#include "car_config.h"
#include "zf_common_font.h"
#include "zf_driver_delay.h"
#include "zf_driver_soft_iic.h"

#define CAR_OLED_WIDTH                  (128U)
#define CAR_OLED_PAGES                  (8U)
#define CAR_OLED_TIME_PAGE              (2U)

static soft_iic_info_struct display_i2c;
static uint8 display_buffer[CAR_OLED_WIDTH * CAR_OLED_PAGES];
static uint8 display_page_to_flush;

static void car_display_write_commands(const uint8 *commands, uint8 count)
{
    uint8 packet[32];
    uint8 index;

    if(count > 31U)
    {
        count = 31U;
    }
    packet[0] = 0x00U;
    for(index = 0U; index < count; index++)
    {
        packet[index + 1U] = commands[index];
    }
    soft_iic_write_8bit_array(&display_i2c, packet, (uint32)count + 1U);
}

static void car_display_clear_buffer(void)
{
    memset(display_buffer, 0, sizeof(display_buffer));
}

static void car_display_draw_text(uint8 column, uint8 page, const char *text)
{
    uint8 character;
    uint8 font_column;
    uint16 offset;

    while((*text != '\0') && (page < CAR_OLED_PAGES) &&
        (column <= (CAR_OLED_WIDTH - 6U)))
    {
        character = (uint8)*text++;
        if((character < 32U) || (character > 126U))
        {
            character = (uint8)'?';
        }
        offset = (uint16)page * CAR_OLED_WIDTH + column;
        for(font_column = 0U; font_column < 6U; font_column++)
        {
            display_buffer[offset + font_column] =
                ascii_font_6x8[character - 32U][font_column];
        }
        column += 6U;
    }
}

static const char *car_display_mode_text(uint8 mode)
{
    static const char *const text[] =
    {
        "INIT", "CAL", "READY", "RUN", "BRAKE", "FINISH", "FAULT"
    };

    return (mode < 7U) ? text[mode] : "UNKNOWN";
}

void car_display_init(void)
{
    static const uint8 init_commands[] =
    {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0x7F, 0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6,
        0xAF,
    };
    uint8 page;

    soft_iic_init(&display_i2c, CAR_DISPLAY_I2C_ADDRESS,
        CAR_DISPLAY_SOFT_I2C_DELAY, CAR_BOARD_OLED_SCL,
        CAR_BOARD_OLED_SDA);
    system_delay_ms(50U);
    car_display_write_commands(init_commands,
        (uint8)sizeof(init_commands));
    car_display_clear_buffer();
    display_page_to_flush = 0U;

    /* 上电阶段无实时控制，允许同步清空八页。 */
    for(page = 0U; page < CAR_OLED_PAGES; page++)
    {
        car_display_flush_step();
    }
}

void car_display_update(uint8 task, uint8 mode, uint32 runtime_ms,
    uint32 lap_time_ms, float distance_m, float left_speed_mps,
    float right_speed_mps, uint8 gray_mask, float line_error,
    int16 left_pwm, int16 right_pwm, uint32 fault_bits)
{
    char line[24];

    car_display_clear_buffer();
    (void)snprintf(line, sizeof(line), "H CHASSIS TASK %u",
        (unsigned int)task);
    car_display_draw_text(0U, 0U, line);
    (void)snprintf(line, sizeof(line), "STATE %-7s",
        car_display_mode_text(mode));
    car_display_draw_text(0U, 1U, line);
    (void)snprintf(line, sizeof(line), "TIME %3lu.%03lus",
        (unsigned long)(runtime_ms / 1000U),
        (unsigned long)(runtime_ms % 1000U));
    car_display_draw_text(0U, 2U, line);
    (void)snprintf(line, sizeof(line), "RES  %3lu.%03lus",
        (unsigned long)(lap_time_ms / 1000U),
        (unsigned long)(lap_time_ms % 1000U));
    car_display_draw_text(0U, 3U, line);
    (void)snprintf(line, sizeof(line), "D %.3fm V%.2f/%.2f",
        distance_m, left_speed_mps, right_speed_mps);
    car_display_draw_text(0U, 4U, line);
    (void)snprintf(line, sizeof(line), "G %02X E%+.2f",
        (unsigned int)gray_mask, line_error);
    car_display_draw_text(0U, 5U, line);
    (void)snprintf(line, sizeof(line), "PWM %d/%d",
        (int)left_pwm, (int)right_pwm);
    car_display_draw_text(0U, 6U, line);
    (void)snprintf(line, sizeof(line), "FAULT %08lX",
        (unsigned long)fault_bits);
    car_display_draw_text(0U, 7U, line);
}

static void car_display_flush_page(uint8 page)
{
    uint8 command[3];
    uint8 packet[CAR_OLED_WIDTH + 1U];
    uint16 offset;

    command[0] = (uint8)(0xB0U + page);
    command[1] = 0x00U;
    command[2] = 0x10U;
    car_display_write_commands(command, 3U);

    packet[0] = 0x40U;
    offset = (uint16)page * CAR_OLED_WIDTH;
    memcpy(&packet[1], &display_buffer[offset], CAR_OLED_WIDTH);
    soft_iic_write_8bit_array(&display_i2c, packet,
        CAR_OLED_WIDTH + 1U);
}

void car_display_flush_step(void)
{
    car_display_flush_page(display_page_to_flush);

    display_page_to_flush++;
    if(display_page_to_flush >= CAR_OLED_PAGES)
    {
        display_page_to_flush = 0U;
    }
}

void car_display_flush_time_page(void)
{
    car_display_flush_page(CAR_OLED_TIME_PAGE);
}
