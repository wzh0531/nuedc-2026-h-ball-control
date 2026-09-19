#ifndef _CAR_CONFIG_H_
#define _CAR_CONFIG_H_

/*
 * L4 Middleware configuration
 *
 * 这里只保存与算法和整车参数有关的常量，不包含 GPIO/PWM 等板级枚举。
 * 板级资源统一放在 car_board.h，避免业务层间接包含逐飞总头文件。
 */
#include "zf_common_typedef.h"

/* 固定 5 ms 控制周期，对应 200 Hz 轮速、灰度和角速度控制。 */
#define CAR_CONTROL_PERIOD_MS                 (5U)
#define CAR_CONTROL_DT_S                      (0.005f)
#define CAR_UI_REFRESH_PERIOD_MS              (100U)

/* MG513XP28_12V：13 PPR × AB 四倍频 × 28:1 = 1456 count/rev。 */
#define CAR_ENCODER_COUNTS_PER_REV            (1456.0f)
#define CAR_WHEEL_DIAMETER_M                  (0.065f)
#define CAR_WHEEL_TRACK_M                     (0.170f)
#define CAR_METERS_PER_COUNT                  (0.0001402497f)
#define CAR_ENCODER_LEFT_SIGN                 (-1)
#define CAR_ENCODER_RIGHT_SIGN                (1)
#define CAR_ENCODER_INVALID_LIMIT             (24U)

/* TB6612：PWM 标尺为逐飞库的 0..10000，本项目按风险约束限制到 65%。 */
#define CAR_MOTOR_PWM_FREQ_HZ                 (20000U)
#define CAR_MOTOR_OUTPUT_LIMIT                (6500)
#define CAR_MOTOR_STALL_PWM_THRESHOLD         (2500)
#define CAR_MOTOR_REVERSE_ZERO_TICKS          (1U)   /* 至少 1×5 ms 零输出 */
#define CAR_MOTOR_SHORT_BRAKE_MAX_TICKS       (10U)  /* 最多 50 ms，默认不用 */
#define CAR_MOTOR_LEFT_SIGN                   (1)
#define CAR_MOTOR_RIGHT_SIGN                  (1)

/* 编码器堵转与丢线保护均以 5 ms 控制周期计数。 */
#define CAR_STALL_NO_EDGE_TICKS               (3U)
#define CAR_LINE_SEARCH_TICKS                 (20U)  /* 100 ms */
#define CAR_LINE_FAULT_TICKS                  (40U)  /* 200 ms */
#define CAR_CONTROL_OVERRUN_LIMIT             (2U)

/* 实车标定：辅助板原始输出 0=黑、1=白，bit0 对应车体物理最左探头。 */
#define CAR_GRAY_BLACK_LEVEL                  (0U)
#define CAR_GRAY_REVERSE_ORDER                (0U)
#define CAR_GRAY_CLOCK_DELAY_US               (5U)
#define CAR_GRAY_STOP_MIN_BLACK               (4U)
#define CAR_GRAY_STOP_CONFIRM_FRAMES          (3U)
/*
 * A 点动态灰度证据缓冲：门控打开后持续滚动保留最近约 1.28 s。
 * black>=3 只设置候选标记；车辆最终停稳或故障时才冻结，运行中不打印。
 */
#define CAR_A_LOG_CAPACITY                    (256U)
#define CAR_A_LOG_TRIGGER_MIN_BLACK           (3U)

/* IMU660RA 静止零偏标定和角速度低通。 */
#define CAR_IMU_CALIBRATION_SAMPLES           (400U)
#define CAR_IMU_CALIBRATION_DELAY_MS          (2U)
#define CAR_IMU_GYRO_DEADZONE_DPS             (0.35f)
#define CAR_IMU_RATE_FILTER_ALPHA              (0.25f)
#define CAR_IMU_YAW_SIGN                      (1.0f)
#define CAR_IMU_RESIDUAL_LIMIT_DPS             (0.5f)

/* 初始速度环参数。单位：速度 m/s，输出为 0..6500 占空比标尺。 */
#define CAR_SPEED_FF_GAIN                     (8000.0f)
#define CAR_SPEED_KP_DEFAULT                  (7000.0f)
#define CAR_SPEED_KI_DEFAULT                  (5000.0f)
#define CAR_SPEED_KD_DEFAULT                  (0.0f)
#define CAR_SPEED_INTEGRAL_LIMIT              (0.35f)
#define CAR_SPEED_OUTPUT_LIMIT                (6500.0f)
#define CAR_SPEED_SAMPLE_COUNT                (3U)

/* 灰度外环：物理左侧误差为负，左转为正角速度，所以默认符号为 -1。 */
#define CAR_LINE_CONTROL_SIGN                 (-1.0f)
#define CAR_LINE_KP_DEFAULT                   (65.0f)
#define CAR_LINE_KD_DEFAULT                   (0.3f)
#define CAR_LINE_RATE_FILTER_ALPHA            (0.25f)
#define CAR_LINE_YAW_RATE_LIMIT_DPS           (70.0f)
#define CAR_LINE_SEARCH_YAW_RATE_DPS          (28.0f)

/* IMU 角速度内环输出左右轮差速量，单位 m/s。 */
#define CAR_YAW_KP_DEFAULT                    (0.0024f)
#define CAR_YAW_KI_DEFAULT                    (0.0f)
#define CAR_YAW_KD_DEFAULT                    (0.0f)
#define CAR_YAW_INTEGRAL_LIMIT                (30.0f)
#define CAR_YAW_DELTA_SPEED_LIMIT_MPS         (0.16f)

/* 速度规划与比赛速度。初次上车可通过 SPEED 命令降低基础速度。 */
#define CAR_ACCEL_LIMIT_MPS2                  (0.30f)
#define CAR_BALANCE_ACCEL_LIMIT_MPS2          (0.15f)
#define CAR_DECEL_LIMIT_MPS2                  (0.35f)
#define CAR_JERK_LIMIT_MPS3                   (1.20f)
#define CAR_BRAKE_JERK_MARGIN_M               (0.075f)
#define CAR_WHEEL_SPEED_LIMIT_MPS             (0.65f)
#define CAR_DEBUG_BASE_SPEED_MPS              (0.22f)
#define CAR_RACE_TASK2_SPEED_MPS              (0.37f)
#define CAR_RACE_TASK4_SPEED_MPS              (0.26f)
#define CAR_RACE_TASK56_SPEED_MPS             (0.25f)
#define CAR_RACE_MAX_SPEED_MPS                (CAR_RACE_TASK2_SPEED_MPS)
#define CAR_PRE_BRAKE_SPEED_MPS               (0.22f)
#define CAR_LINE_SEARCH_SPEED_MPS             (0.10f)
#define CAR_FINISH_APPROACH_SPEED_MPS         (0.04f)
#define CAR_FINISH_SPEED_EPSILON_MPS          (0.035f)
/*
 * 跃度受限规划在目标前约 1.4 cm 停稳；以内部 1.5 cm 停车目标作为
 * 完成窗口，仍严于题目任务 2 的 2 cm 上限。
 */
#define CAR_FINISH_DISTANCE_TOLERANCE_M       (0.015f)

/* H 题赛道与停车标定参数。 */
#define CAR_A_GATE_DISTANCE_M                 (5.50f)
#define CAR_A_GATE_TIME_MS                    (10000U)
#define CAR_LAP_NOMINAL_DISTANCE_M            (6.1416f)
#define CAR_SENSOR_TO_TEST_POINT_M            (0.090f)
/* 任务 4/5/6 到达计时点后保持行驶，再前进 1 m 才平滑停车。 */
#define CAR_POST_RESULT_DISTANCE_M            (1.000f)
#define CAR_PASS_A_EXTRA_DISTANCE_M           (CAR_POST_RESULT_DISTANCE_M)
#define CAR_TASK4_B_DISTANCE_M                (1.500f)
#define CAR_TASK4_PRE_BRAKE_DISTANCE_M        (CAR_TASK4_B_DISTANCE_M)
/*
 * 任务 4 在测试点到达 B 时锁存成绩，再以 B 后 1 m 为停车目标。
 * 状态机对任务 4 使用零距离完成容差，避免全局 1.5 cm 完成窗口导致提前结束。
 */
#define CAR_TASK4_PASS_B_EXTRA_DISTANCE_M     (CAR_POST_RESULT_DISTANCE_M)
#define CAR_TASK4_STOP_DISTANCE_M             \
    (CAR_TASK4_B_DISTANCE_M + CAR_TASK4_PASS_B_EXTRA_DISTANCE_M)

/* 串口命令和电机台架测试。 */
#define CAR_COMMAND_BUFFER_SIZE               (96U)
#define CAR_MOTOR_TEST_MAX_MS                 (1500U)
#define CAR_MOTOR_TEST_DEFAULT_MS             (500U)
#define CAR_VELOCITY_TEST_MAX_SPEED_MPS       (0.35f)
#define CAR_VELOCITY_TEST_MIN_MS              (200U)
#define CAR_VELOCITY_TEST_MAX_MS              (3000U)
#define CAR_YAW_TEST_BASE_MIN_MPS             (0.18f)
#define CAR_YAW_TEST_BASE_MAX_MPS             (0.25f)
#define CAR_YAW_TEST_MAX_RATE_DPS             (30.0f)
#define CAR_YAW_TEST_MIN_MS                   (200U)
#define CAR_YAW_TEST_MAX_MS                   (3000U)

/* MaixCAM 小球追踪与串口无刷云台。视觉帧：7B tracking dxH dxL 7D。 */
#define CAR_BALL_UART_BAUDRATE                 (115200U)
#define CAR_BALL_UART_IRQ_PRIORITY             (1U)
#define CAR_BALL_MOTOR_ADDRESS                 (2U)
#define CAR_BALL_MOTOR_ACCELERATION_RPS2       (100U)
#define CAR_BALL_MOTOR_SPEED_RPM               (100)
#define CAR_BALL_COMMAND_GAP_US                 (250U)
#define CAR_BALL_ZERO_SETTLE_MS                 (500U)
#define CAR_BALL_TARGET_DEFAULT_PIXELS         (0.0f)
#define CAR_BALL_TARGET_LIMIT_PIXELS           (1000.0f)
#define CAR_BALL_KP_DEFAULT                    (0.11f)
#define CAR_BALL_KI_DEFAULT                    (0.21f)
#define CAR_BALL_KD_DEFAULT                    (0.012f)
#define CAR_BALL_ANGLE_LIMIT_DEG               (19.0f)
#define CAR_BALL_INTEGRAL_LIMIT                (38.0f)
/*
 * Positive BLDC angle raises the rod front.  Positive chassis acceleration
 * therefore needs a negative angle feedforward (front down).
 */
#define CAR_BALL_ACCEL_FF_SIGN                 (-1.0f)
#define CAR_BALL_ACCEL_FF_GAIN                 (3.00f)
#define CAR_BALL_ACCEL_FF_PREVIEW_S            (0.150f)
#define CAR_BALL_ACCEL_FF_MAX_ACCEL_MPS2       (CAR_BALANCE_ACCEL_LIMIT_MPS2)
#define CAR_BALL_ACCEL_FF_MAX_DEG              (2.50f)
#define CAR_GRAVITY_MPS2                       (9.80665f)
#define CAR_RAD_TO_DEG                         (57.2957795f)
/* MaixCAM sends (position_cm - selected_zero_cm) * 8 as signed int16. */
#define CAR_BALL_UART_UNITS_PER_CM             (8.0f)
/* Task 3 uses independent gains for center -> -5 cm and -5 cm -> +5 cm. */
#define CAR_BALL_TASK3_LEFT_KP                 (0.17f)
#define CAR_BALL_TASK3_LEFT_KI                 (0.20f)
#define CAR_BALL_TASK3_LEFT_KD                 (0.012f)
#define CAR_BALL_TASK3_RIGHT_KP                (0.14f)
#define CAR_BALL_TASK3_RIGHT_KI                (0.21f)
#define CAR_BALL_TASK3_RIGHT_KD                (0.012f)
#define CAR_BALL_TASK3_POSITIVE_TARGET_UNITS   (5.0f * CAR_BALL_UART_UNITS_PER_CM)
#define CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS   (-5.0f * CAR_BALL_UART_UNITS_PER_CM)
#define CAR_BALL_TASK3_TOLERANCE_UNITS         (0.5f * CAR_BALL_UART_UNITS_PER_CM)
#define CAR_BALL_TASK3_LEFT_STABLE_FRAMES      (5U)
#define CAR_BALL_TASK3_RIGHT_STABLE_FRAMES     (5U)

/* SSD1306 I2C OLED 常用 7 位地址。 */
#define CAR_DISPLAY_I2C_ADDRESS               (0x3CU)
#define CAR_DISPLAY_SOFT_I2C_DELAY            (20U)

#endif
