#include "car_command.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_app.h"
#include "car_ball_tracker.h"
#include "car_bldc.h"
#include "car_chassis.h"
#include "car_config.h"
#include "car_control.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_imu.h"
#include "car_safety.h"
#include "car_telemetry.h"
#include "zf_common_debug.h"
#include "zf_common_typedef.h"

static char command_buffer[CAR_COMMAND_BUFFER_SIZE];
static uint8 command_length;
static uint8 command_discard_line;

static void car_command_print_help(void)
{
    printf("Commands:\r\n");
    printf("  STATUS | STATS | ALOG | START | STOP | RESET\r\n");
    printf("  TASK <2..6>       select H problem item\r\n");
    printf("  SPEED <0|0.10..0.37 m/s>\r\n");
    printf("  PID SPEED <kp> <ki> [kd]\r\n");
    printf("  PID LINE <kp> <kd>\r\n");
    printf("  PID YAW <kp> <ki> [kd]\r\n");
    printf("  PID BALL <kp> <ki> [kd]\r\n");
    printf("  PID BALL3 <kp> <ki> [kd]\r\n");
    printf("  PID BALL3L|BALL3R <kp> <ki> [kd]\r\n");
    printf("  BALL STATUS | BALL TARGET <-1000..1000 pixels>\r\n");
    printf("  MOTOR <left> <right> [ms<=1500]\r\n");
    printf("  VEL <left_mps> <right_mps> <200..3000ms>\r\n");
    printf("  YAW <base_mps> <rate_dps> <200..3000ms>\r\n");
    printf("  IMUCAL | ENCZ | GRAY\r\n");
}

static uint8 car_command_parse_int(const char *text, int32 *value)
{
    char *end;
    long parsed;

    if(NULL == text)
    {
        return 0U;
    }
    parsed = strtol(text, &end, 10);
    if((end == text) || ('\0' != *end))
    {
        return 0U;
    }
    *value = (int32)parsed;
    return 1U;
}

static uint8 car_command_parse_float(const char *text, float *value)
{
    char *end;
    float parsed;

    if(NULL == text)
    {
        return 0U;
    }
    parsed = strtof(text, &end);
    if((end == text) || ('\0' != *end) || !isfinite(parsed))
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

void car_command_print_status(void)
{
    const ChassisState *state = chassis_get_state();

    printf("task=%u mode=%u time=%lu result=%lu fault=0x%08lX ",
        (unsigned int)state->task, (unsigned int)state->mode,
        (unsigned long)state->run_time_ms,
        (unsigned long)state->lap_time_ms,
        (unsigned long)state->fault_bits);
    printf("dist=%.4f speed=%.3f,%.3f ref=%.3f,%.3f ",
        state->distance_m, state->left_speed_mps,
        state->right_speed_mps, state->left_speed_ref_mps,
        state->right_speed_ref_mps);
    printf("gray=0x%02X black=%u line=%u err=%.3f yaw_rate=%.2f ",
        (unsigned int)state->gray_mask,
        (unsigned int)state->gray_black_count,
        (unsigned int)state->line_valid, state->line_error,
        state->yaw_rate_dps);
    printf("pwm=%d,%d enc=%ld,%ld invalid=%lu,%lu overrun=%lu/%u\r\n",
        (int)state->left_pwm, (int)state->right_pwm,
        (long)car_encoder_get_total(CAR_ENCODER_LEFT),
        (long)car_encoder_get_total(CAR_ENCODER_RIGHT),
        (unsigned long)car_encoder_get_invalid(CAR_ENCODER_LEFT),
        (unsigned long)car_encoder_get_invalid(CAR_ENCODER_RIGHT),
        (unsigned long)car_app_get_overrun_count(),
        (unsigned int)car_app_get_max_pending_ticks());
}

static void car_command_print_stats(void)
{
    const car_telemetry_stats_struct *stats =
        car_telemetry_get_stats();
    uint32 duration_ms = stats->sample_count * CAR_CONTROL_PERIOD_MS;

    printf("stats samples=%lu duration=%lums ",
        (unsigned long)stats->sample_count,
        (unsigned long)duration_ms);
    printf("speed_mae=%.4f,%.4f speed_max=%.4f,%.4f\r\n",
        stats->left_speed_error_mean_abs_mps,
        stats->right_speed_error_mean_abs_mps,
        stats->left_speed_error_max_abs_mps,
        stats->right_speed_error_max_abs_mps);
    printf("wheel_diff=%.4f/%.4f line_max=%.3f ",
        stats->wheel_speed_difference_mean_abs_mps,
        stats->wheel_speed_difference_max_abs_mps,
        stats->line_error_max_abs);
    printf("yaw_err=%.2f/%.2f yaw_max=%.2f\r\n",
        stats->yaw_rate_error_mean_abs_dps,
        stats->yaw_rate_error_max_abs_dps,
        stats->yaw_rate_max_abs_dps);
    printf("pwm_max=%d sat=%lu line_lost=%lu\r\n",
        (int)stats->pwm_max_abs,
        (unsigned long)stats->pwm_saturation_samples,
        (unsigned long)stats->line_lost_samples);
}

static void car_command_print_a_log(void)
{
    car_a_log_info_struct info = car_telemetry_get_a_log_info();
    car_a_log_sample_struct sample;
    uint16 index;

    printf("ALOG samples=%u triggered=%u frozen=%u\r\n",
        (unsigned int)info.sample_count,
        (unsigned int)info.triggered,
        (unsigned int)info.frozen);
    printf("ALOG time_ms,dist_m,mask,black,line_valid,line_error,"
        "yaw_rate_dps\r\n");
    for(index = 0U; index < info.sample_count; index++)
    {
        if(car_telemetry_get_a_log_sample(index, &sample))
        {
            printf("A,%lu,%.4f,0x%02X,%u,%u,%.3f,%.2f\r\n",
                (unsigned long)sample.time_ms,
                sample.distance_m,
                (unsigned int)sample.gray_mask,
                (unsigned int)sample.black_count,
                (unsigned int)sample.line_valid,
                sample.line_error,
                sample.yaw_rate_dps);
        }
    }
    car_app_clear_pending_ticks();
}

static void car_command_pid(char *loop_name)
{
    char *arg1 = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    char *arg3 = strtok(NULL, " \t");
    float first;
    float second;
    float third = 0.0f;

    if((NULL == loop_name) ||
        !car_command_parse_float(arg1, &first) ||
        !car_command_parse_float(arg2, &second) ||
        ((NULL != arg3) && !car_command_parse_float(arg3, &third)) ||
        (first < 0.0f) || (second < 0.0f) || (third < 0.0f))
    {
        printf("ERR PID\r\n");
        return;
    }

    if(0 == strcmp(loop_name, "SPEED"))
    {
        car_control_set_speed_pid(first, second, third);
    }
    else if(0 == strcmp(loop_name, "LINE"))
    {
        car_control_set_line_pd(first, second);
    }
    else if(0 == strcmp(loop_name, "YAW"))
    {
        car_control_set_yaw_pid(first, second, third);
    }
    else if(0 == strcmp(loop_name, "BALL"))
    {
        car_ball_tracker_set_pid(first, second, third);
    }
    else if(0 == strcmp(loop_name, "BALL3"))
    {
        car_ball_tracker_set_task3_pid(first, second, third);
    }
    else if(0 == strcmp(loop_name, "BALL3L"))
    {
        car_ball_tracker_set_task3_left_pid(first, second, third);
    }
    else if(0 == strcmp(loop_name, "BALL3R"))
    {
        car_ball_tracker_set_task3_right_pid(first, second, third);
    }
    else
    {
        printf("ERR PID LOOP\r\n");
        return;
    }
    printf("OK PID %s %.5f %.5f %.5f\r\n",
        loop_name, first, second, third);
}

static void car_command_ball(void)
{
    char *subcommand = strtok(NULL, " \t");
    char *argument;
    float target;

    if((NULL != subcommand) && (0 == strcmp(subcommand, "STATUS")))
    {
        printf("BALL tracking=%u dx=%d target=%.2f cmd=%.1fdeg frames=%lu ",
            (unsigned int)car_ball_tracker_is_tracking(),
            (int)car_ball_tracker_get_dx(),
            car_ball_tracker_get_target(),
            (float)car_ball_tracker_get_command_angle_x10() * 0.1f,
            (unsigned long)car_ball_tracker_get_frame_count());
        printf("motor2 rpm=%d angle=%.1fdeg voltage=%.2fV\r\n",
            (int)car_bldc_motor_2.speed_rpm,
            (float)car_bldc_motor_2.multi_angle_x10 * 0.1f,
            (float)car_bldc_motor_2.voltage_x100 * 0.01f);
        return;
    }

    if((NULL != subcommand) && (0 == strcmp(subcommand, "TARGET")))
    {
        argument = strtok(NULL, " \t");
        if(car_command_parse_float(argument, &target) &&
            (target >= -CAR_BALL_TARGET_LIMIT_PIXELS) &&
            (target <= CAR_BALL_TARGET_LIMIT_PIXELS))
        {
            car_ball_tracker_set_target(target);
            printf("OK BALL TARGET %.2f\r\n", target);
            return;
        }
    }
    printf("ERR BALL\r\n");
}

static void car_command_motor(void)
{
    char *arg1 = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    char *arg3 = strtok(NULL, " \t");
    int32 left;
    int32 right;
    int32 duration = (int32)CAR_MOTOR_TEST_DEFAULT_MS;

    if(!car_command_parse_int(arg1, &left) ||
        !car_command_parse_int(arg2, &right) ||
        ((NULL != arg3) && !car_command_parse_int(arg3, &duration)) ||
        (left > CAR_MOTOR_OUTPUT_LIMIT) ||
        (left < -CAR_MOTOR_OUTPUT_LIMIT) ||
        (right > CAR_MOTOR_OUTPUT_LIMIT) ||
        (right < -CAR_MOTOR_OUTPUT_LIMIT) ||
        (duration <= 0) ||
        !chassis_start_motor_test((int16)left, (int16)right,
            (uint32)duration))
    {
        printf("ERR MOTOR\r\n");
        return;
    }
    printf("OK MOTOR %ld %ld %ldms\r\n",
        (long)left, (long)right, (long)duration);
}

static void car_command_velocity(void)
{
    char *arg1 = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    char *arg3 = strtok(NULL, " \t");
    float left;
    float right;
    int32 duration;

    if(!car_command_parse_float(arg1, &left) ||
        !car_command_parse_float(arg2, &right) ||
        !car_command_parse_int(arg3, &duration) ||
        (duration < (int32)CAR_VELOCITY_TEST_MIN_MS) ||
        (duration > (int32)CAR_VELOCITY_TEST_MAX_MS) ||
        !chassis_start_velocity_test(left, right, (uint32)duration))
    {
        printf("ERR VEL\r\n");
        return;
    }
    printf("OK VEL %.3f %.3f %ldms\r\n",
        left, right, (long)duration);
}

static void car_command_yaw(void)
{
    char *arg1 = strtok(NULL, " \t");
    char *arg2 = strtok(NULL, " \t");
    char *arg3 = strtok(NULL, " \t");
    float base_speed;
    float yaw_target;
    int32 duration;

    if(!car_command_parse_float(arg1, &base_speed) ||
        !car_command_parse_float(arg2, &yaw_target) ||
        !car_command_parse_int(arg3, &duration) ||
        (duration < (int32)CAR_YAW_TEST_MIN_MS) ||
        (duration > (int32)CAR_YAW_TEST_MAX_MS) ||
        !chassis_start_yaw_test(base_speed, yaw_target, (uint32)duration))
    {
        printf("ERR YAW\r\n");
        return;
    }
    printf("OK YAW %.3f %.2f %ldms\r\n",
        base_speed, yaw_target, (long)duration);
}

static void car_command_execute(char *line)
{
    const ChassisState *state;
    char *command = strtok(line, " \t");
    char *argument;
    int32 integer;
    float value;

    if(NULL == command)
    {
        return;
    }

    /*
     * 逐飞 UART0 的 printf 为阻塞发送。运行中执行 STATUS/HELP/IMUCAL
     * 会占用多个 5 ms 周期，因此只允许短小的 STOP 命令。
     * PID 和速度参数统一在停车状态修改，避免比赛中参数突变。
     */
    state = chassis_get_state();
    if(((CHASSIS_MODE_RUNNING == state->mode) ||
        (CHASSIS_MODE_PRE_BRAKE == state->mode)) &&
        (0 != strcmp(command, "STOP")))
    {
        printf("ERR BUSY RUN\r\n");
        return;
    }

    if(0 == strcmp(command, "HELP"))
    {
        car_command_print_help();
    }
    else if(0 == strcmp(command, "STATUS"))
    {
        car_command_print_status();
    }
    else if(0 == strcmp(command, "STATS"))
    {
        car_command_print_stats();
    }
    else if(0 == strcmp(command, "ALOG"))
    {
        car_command_print_a_log();
    }
    else if(0 == strcmp(command, "START"))
    {
        printf(chassis_start() ? "OK START\r\n" : "ERR START\r\n");
    }
    else if(0 == strcmp(command, "STOP"))
    {
        chassis_stop();
        printf("OK STOP WAIT FINISH\r\n");
    }
    else if(0 == strcmp(command, "RESET"))
    {
        printf(chassis_reset() ? "OK RESET\r\n" : "ERR RESET\r\n");
    }
    else if(0 == strcmp(command, "TASK"))
    {
        argument = strtok(NULL, " \t");
        if(!car_command_parse_int(argument, &integer) ||
            !chassis_select_task((ChassisTask)integer))
        {
            printf("ERR TASK\r\n");
        }
        else
        {
            printf("OK TASK %ld\r\n", (long)integer);
        }
    }
    else if(0 == strcmp(command, "SPEED"))
    {
        argument = strtok(NULL, " \t");
        if(!car_command_parse_float(argument, &value) ||
            (value < 0.0f) || (value > CAR_RACE_MAX_SPEED_MPS))
        {
            printf("ERR SPEED\r\n");
        }
        else
        {
            chassis_set_base_speed(value);
            printf("OK SPEED %.3f\r\n", value);
        }
    }
    else if(0 == strcmp(command, "PID"))
    {
        car_command_pid(strtok(NULL, " \t"));
    }
    else if(0 == strcmp(command, "MOTOR"))
    {
        car_command_motor();
    }
    else if(0 == strcmp(command, "VEL"))
    {
        car_command_velocity();
    }
    else if(0 == strcmp(command, "YAW"))
    {
        car_command_yaw();
    }
    else if(0 == strcmp(command, "BALL"))
    {
        car_command_ball();
    }
    else if(0 == strcmp(command, "IMUCAL"))
    {
        if((CHASSIS_MODE_READY != state->mode) &&
            (CHASSIS_MODE_FINISHED != state->mode) &&
            (CHASSIS_MODE_FAULT != state->mode))
        {
            printf("ERR IMUCAL STATE\r\n");
        }
        else
        {
            printf("IMUCAL KEEP STILL...\r\n");
            if(car_imu_calibrate())
            {
                if((CHASSIS_MODE_FAULT == state->mode) &&
                    (CHASSIS_FAULT_IMU == state->fault_bits))
                {
                    (void)chassis_reset();
                }
                printf("OK IMUCAL bias=%.4f residual=%.4f\r\n",
                    car_imu_get_bias(), car_imu_get_residual());
            }
            else
            {
                printf("ERR IMUCAL residual=%.4f limit=%.4f\r\n",
                    car_imu_get_residual(),
                    (double)CAR_IMU_RESIDUAL_LIMIT_DPS);
            }
            car_app_clear_pending_ticks();
        }
    }
    else if(0 == strcmp(command, "ENCZ"))
    {
        if(CHASSIS_MODE_READY == state->mode)
        {
            car_encoder_reset();
            printf("OK ENCZ\r\n");
        }
        else
        {
            printf("ERR ENCZ STATE\r\n");
        }
    }
    else if(0 == strcmp(command, "GRAY"))
    {
        (void)car_gray_read();
        printf("GRAY mask=0x%02X black=%u\r\n",
            (unsigned int)car_gray_get_mask(),
            (unsigned int)car_gray_get_black_count());
    }
    else
    {
        printf("ERR UNKNOWN\r\n");
    }
}

void car_command_init(void)
{
    command_length = 0U;
    command_discard_line = 0U;
    memset(command_buffer, 0, sizeof(command_buffer));
}

void car_command_process(void)
{
    uint8 data[16];
    uint32 length = debug_read_ring_buffer(data, sizeof(data));
    uint32 index;
    char character;

    for(index = 0U; index < length; index++)
    {
        character = (char)data[index];
        if(('\r' == character) || ('\n' == character))
        {
            if(command_discard_line)
            {
                command_discard_line = 0U;
                command_length = 0U;
            }
            else if(command_length > 0U)
            {
                command_buffer[command_length] = '\0';
                car_command_execute(command_buffer);
                command_length = 0U;
            }
        }
        else if(command_discard_line)
        {
        }
        else if(command_length < (CAR_COMMAND_BUFFER_SIZE - 1U))
        {
            if((character >= 'a') && (character <= 'z'))
            {
                character = (char)(character - 'a' + 'A');
            }
            command_buffer[command_length++] = character;
        }
        else
        {
            command_length = 0U;
            command_discard_line = 1U;
            printf("ERR TOO LONG\r\n");
        }
    }
}
