#include "car_chassis.h"

#include <math.h>
#include <string.h>

#include "car_config.h"
#include "car_control.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_imu.h"
#include "car_motor.h"
#include "car_planner.h"
#include "car_safety.h"
#include "car_telemetry.h"

static ChassisState chassis_state;
static car_planner_struct speed_planner;
static float base_speed_override_mps;
static uint8 stop_line_frames;
static uint8 a_line_latched;
static uint8 stop_requested;
static uint8 distance_target_valid;
static float distance_target_m;
static uint8 result_capture_pending;
static float result_capture_distance_m;
static uint8 motor_test_active;
static int16 motor_test_left;
static int16 motor_test_right;
static uint32 motor_test_duration_ms;
static uint8 velocity_test_active;
static float velocity_test_left_mps;
static float velocity_test_right_mps;
static uint32 velocity_test_duration_ms;
static uint8 yaw_test_active;
static float yaw_test_base_mps;
static float yaw_test_target_dps;
static uint32 yaw_test_duration_ms;

static float chassis_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float chassis_min(float first, float second)
{
    return (first < second) ? first : second;
}

static float chassis_finish_distance_tolerance(void)
{
    /*
     * 任务 4 必须先到达 B 点。若沿用全局 1.5 cm 完成窗口，即使配置
     * 零过 B 余量，也会在 B 前提前结束。其余任务仍保留原停车容差。
     */
    if(CHASSIS_TASK_4_A_TO_B == chassis_state.task)
    {
        return 0.0f;
    }
    return CAR_FINISH_DISTANCE_TOLERANCE_M;
}

static float chassis_nominal_speed(void)
{
    if(base_speed_override_mps > 0.0f)
    {
        return base_speed_override_mps;
    }
    if(CHASSIS_TASK_4_A_TO_B == chassis_state.task)
    {
        return CAR_RACE_TASK4_SPEED_MPS;
    }
    if((CHASSIS_TASK_5_LAP_CENTER == chassis_state.task) ||
        (CHASSIS_TASK_6_LAP_POSITION == chassis_state.task))
    {
        return CAR_RACE_TASK56_SPEED_MPS;
    }
    return CAR_RACE_TASK2_SPEED_MPS;
}

static float chassis_acceleration_limit(void)
{
    if((CHASSIS_TASK_4_A_TO_B == chassis_state.task) ||
        (CHASSIS_TASK_5_LAP_CENTER == chassis_state.task) ||
        (CHASSIS_TASK_6_LAP_POSITION == chassis_state.task))
    {
        return CAR_BALANCE_ACCEL_LIMIT_MPS2;
    }
    return CAR_ACCEL_LIMIT_MPS2;
}

static void chassis_refresh_public_state(void)
{
    const car_control_state_struct *control = car_control_get_state();
    int32 left_total = car_encoder_get_total(CAR_ENCODER_LEFT);
    int32 right_total = car_encoder_get_total(CAR_ENCODER_RIGHT);

    chassis_state.left_speed_mps = control->left_speed_mps;
    chassis_state.right_speed_mps = control->right_speed_mps;
    chassis_state.body_speed_mps =
        (control->left_speed_mps + control->right_speed_mps) * 0.5f;
    chassis_state.distance_m = ((float)left_total + (float)right_total) *
        0.5f * CAR_METERS_PER_COUNT;
    chassis_state.gray_mask = car_gray_get_mask();
    chassis_state.gray_black_count = car_gray_get_black_count();
    chassis_state.yaw_rate_dps = car_imu_get_rate();
    chassis_state.vehicle_accel_ref_mps2 = speed_planner.acceleration_mps2;
    chassis_state.left_speed_ref_mps = control->left_target_mps;
    chassis_state.right_speed_ref_mps = control->right_target_mps;
    chassis_state.left_pwm = car_motor_get_output(CAR_MOTOR_LEFT);
    chassis_state.right_pwm = car_motor_get_output(CAR_MOTOR_RIGHT);
    chassis_state.fault_bits = car_safety_get_faults();
}

static void chassis_finish(void)
{
    /*
     * 任务 2/3 的有效时间就是完成时刻。任务 4/5/6 通常已在过线时
     * 锁存；这里保留兜底，避免异常参数令结果时间永久为 0。
     * 电机台架测试不是赛题运行，不写入比赛时间。
     */
    if(!motor_test_active && !velocity_test_active && !yaw_test_active &&
        (0U == chassis_state.lap_time_ms))
    {
        chassis_state.lap_time_ms = chassis_state.run_time_ms;
    }
    car_motor_stop();
    car_telemetry_freeze_a_log();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    chassis_state.base_speed_ref_mps = 0.0f;
    chassis_state.mode = CHASSIS_MODE_FINISHED;
    stop_requested = 0U;
    distance_target_valid = 0U;
    result_capture_pending = 0U;
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_active = 0U;
    chassis_refresh_public_state();
}

static uint8 chassis_a_line_confirmed(void)
{
    uint8 gate_open = (uint8)(
        (chassis_state.distance_m >= CAR_A_GATE_DISTANCE_M) &&
        (chassis_state.run_time_ms >= CAR_A_GATE_TIME_MS));

    if(a_line_latched)
    {
        return 0U;
    }

    if(gate_open &&
        (chassis_state.gray_black_count >= CAR_GRAY_STOP_MIN_BLACK))
    {
        if(stop_line_frames < 255U)
        {
            stop_line_frames++;
        }
    }
    else
    {
        stop_line_frames = 0U;
    }
    return (uint8)(stop_line_frames >= CAR_GRAY_STOP_CONFIRM_FRAMES);
}

static void chassis_handle_task_transition(void)
{
    /*
     * 前置灰度看到 A 时，车体指定测试点尚未过线。用实测的
     * “灰度阵列到测试点”距离延迟锁存计时，避免提前约 90 mm 停表。
     */
    if(result_capture_pending &&
        (chassis_state.distance_m >= result_capture_distance_m))
    {
        chassis_state.lap_time_ms = chassis_state.run_time_ms;
        result_capture_pending = 0U;
    }

    if(CHASSIS_TASK_4_A_TO_B == chassis_state.task)
    {
        /* 第 4 项要求通过 B：在测试点越过 B 时停表，随后再停车。 */
        if((0U == chassis_state.lap_time_ms) &&
            (chassis_state.distance_m >= CAR_TASK4_B_DISTANCE_M))
        {
            chassis_state.lap_time_ms = chassis_state.run_time_ms;
        }
        if((CHASSIS_MODE_RUNNING == chassis_state.mode) &&
            (chassis_state.distance_m >= CAR_TASK4_PRE_BRAKE_DISTANCE_M))
        {
            chassis_state.mode = CHASSIS_MODE_PRE_BRAKE;
            distance_target_m = CAR_TASK4_STOP_DISTANCE_M;
            distance_target_valid = 1U;
        }
        return;
    }

    if((CHASSIS_TASK_2_LAP_STOP_A == chassis_state.task) &&
        (CHASSIS_MODE_RUNNING == chassis_state.mode) &&
        (chassis_state.distance_m >= CAR_A_GATE_DISTANCE_M))
    {
        chassis_state.mode = CHASSIS_MODE_PRE_BRAKE;
    }

    if(chassis_a_line_confirmed())
    {
        stop_line_frames = 0U;
        a_line_latched = 1U;
        if(CHASSIS_TASK_2_LAP_STOP_A == chassis_state.task)
        {
            distance_target_m = chassis_state.distance_m +
                CAR_SENSOR_TO_TEST_POINT_M;
            distance_target_valid = 1U;
            chassis_state.mode = CHASSIS_MODE_PRE_BRAKE;
        }
        else if((CHASSIS_TASK_5_LAP_CENTER == chassis_state.task) ||
            (CHASSIS_TASK_6_LAP_POSITION == chassis_state.task))
        {
            result_capture_distance_m = chassis_state.distance_m +
                CAR_SENSOR_TO_TEST_POINT_M;
            result_capture_pending = 1U;
            distance_target_m = result_capture_distance_m +
                CAR_PASS_A_EXTRA_DISTANCE_M;
            distance_target_valid = 1U;
            chassis_state.mode = CHASSIS_MODE_PRE_BRAKE;
        }
    }
}

static float chassis_target_for_remaining_distance(float normal_target)
{
    float remaining;
    float usable_remaining;
    float braking_speed;

    if(!distance_target_valid)
    {
        return stop_requested ? 0.0f : normal_target;
    }

    remaining = distance_target_m - chassis_state.distance_m;
    if(remaining <= chassis_finish_distance_tolerance())
    {
        return 0.0f;
    }

    /*
     * S 曲线的减速度不能瞬时从 0 跳到上限，因此先扣除 jerk 制动裕量，
     * 再使用 v <= sqrt(2*a*s) 的恒减速度包络。
     */
    usable_remaining = remaining - CAR_BRAKE_JERK_MARGIN_M;
    if(usable_remaining <= 0.0f)
    {
        return chassis_min(normal_target, CAR_FINISH_APPROACH_SPEED_MPS);
    }
    braking_speed = sqrtf(2.0f * CAR_DECEL_LIMIT_MPS2 *
        usable_remaining);
    if(braking_speed < CAR_FINISH_APPROACH_SPEED_MPS)
    {
        braking_speed = CAR_FINISH_APPROACH_SPEED_MPS;
    }
    return chassis_min(normal_target, braking_speed);
}

static uint8 chassis_reached_finish_condition(void)
{
    uint8 speed_is_zero = (uint8)(
        (chassis_abs(chassis_state.left_speed_mps) <=
            CAR_FINISH_SPEED_EPSILON_MPS) &&
        (chassis_abs(chassis_state.right_speed_mps) <=
            CAR_FINISH_SPEED_EPSILON_MPS));

    if(stop_requested)
    {
        return speed_is_zero;
    }
    if(distance_target_valid &&
        ((distance_target_m - chassis_state.distance_m) <=
            chassis_finish_distance_tolerance()))
    {
        return speed_is_zero;
    }
    return 0U;
}

void chassis_init(void)
{
    memset(&chassis_state, 0, sizeof(chassis_state));
    car_control_init();
    car_planner_init(&speed_planner);
    car_safety_init();
    car_telemetry_init();
    car_motor_stop();
    chassis_state.task = CHASSIS_TASK_2_LAP_STOP_A;
    if(car_imu_is_ready())
    {
        chassis_state.mode = CHASSIS_MODE_READY;
    }
    else if(car_imu_is_calibrating())
    {
        chassis_state.mode = CHASSIS_MODE_CALIBRATE;
    }
    else
    {
        chassis_state.mode = CHASSIS_MODE_FAULT;
        car_safety_latch(CHASSIS_FAULT_IMU);
    }
    base_speed_override_mps = 0.0f;
    stop_line_frames = 0U;
    a_line_latched = 0U;
    stop_requested = 0U;
    distance_target_valid = 0U;
    result_capture_pending = 0U;
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_active = 0U;
    chassis_refresh_public_state();
}

uint8 chassis_start(void)
{
    if((CHASSIS_MODE_READY != chassis_state.mode) &&
        (CHASSIS_MODE_FINISHED != chassis_state.mode))
    {
        return 0U;
    }
    if(!car_imu_is_ready())
    {
        chassis_emergency_stop(CHASSIS_FAULT_IMU);
        return 0U;
    }

    car_motor_stop();
    car_encoder_reset();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    car_planner_set_acceleration_limit(&speed_planner,
        chassis_acceleration_limit());
    car_safety_reset();
    car_telemetry_reset();
    chassis_state.run_time_ms = 0U;
    chassis_state.lap_time_ms = 0U;
    chassis_state.distance_m = 0.0f;
    chassis_state.mode = CHASSIS_MODE_RUNNING;
    stop_line_frames = 0U;
    a_line_latched = 0U;
    stop_requested = 0U;
    distance_target_valid = 0U;
    result_capture_pending = 0U;
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_active = 0U;
    return 1U;
}

void chassis_stop(void)
{
    if((CHASSIS_MODE_RUNNING == chassis_state.mode) ||
        (CHASSIS_MODE_PRE_BRAKE == chassis_state.mode))
    {
        stop_requested = 1U;
        distance_target_valid = 0U;
        chassis_state.mode = CHASSIS_MODE_PRE_BRAKE;
    }
}

void chassis_update_5ms(void)
{
    const car_control_state_struct *control;
    float requested_speed;
    float planned_speed;
    uint8 safety_line_valid;
    uint32 faults;

    car_encoder_sample();
    (void)car_gray_read();
    chassis_state.line_valid = car_gray_get_error(&chassis_state.line_error);

    /*
     * 上电标定每个 5 ms 周期只读取一次陀螺仪。标定期间电机始终为零，
     * 运行计时尚未开始，OLED 可持续显示 CAL 提醒保持车体静止。
     */
    if(CHASSIS_MODE_CALIBRATE == chassis_state.mode)
    {
        car_motor_stop();
        if(car_imu_calibration_step())
        {
            if(car_imu_is_ready())
            {
                chassis_state.mode = CHASSIS_MODE_READY;
            }
            else
            {
                car_safety_latch(CHASSIS_FAULT_IMU);
                chassis_state.mode = CHASSIS_MODE_FAULT;
            }
        }
        chassis_refresh_public_state();
        return;
    }

    car_imu_update(CAR_CONTROL_DT_S);
    chassis_refresh_public_state();

    if((CHASSIS_MODE_RUNNING != chassis_state.mode) &&
        (CHASSIS_MODE_PRE_BRAKE != chassis_state.mode))
    {
        car_motor_stop();
        return;
    }

    chassis_state.run_time_ms += CAR_CONTROL_PERIOD_MS;

    if(motor_test_active)
    {
        /* 先检查上一周期输出，再施加本周期 MOTOR 测试输出。 */
        faults = car_safety_update(1U);
        if(CHASSIS_FAULT_NONE != faults)
        {
            chassis_emergency_stop(faults);
            return;
        }
        car_motor_set_pair(motor_test_left, motor_test_right);
        if(chassis_state.run_time_ms >= motor_test_duration_ms)
        {
            chassis_finish();
            return;
        }
        chassis_refresh_public_state();
        return;
    }

    if(velocity_test_active)
    {
        if(stop_requested)
        {
            chassis_finish();
            return;
        }
        /*
         * 与正式运行保持相同顺序：先用“上一周期已实际施加的 PWM”和本周期
         * 编码器增量做安全判断，再计算下一周期输出。否则 VEL 首帧会在电机
         * 尚未来得及响应时提前累计一次无边沿，缩短堵转判据。
         */
        faults = car_safety_update(1U);
        if(CHASSIS_FAULT_NONE != faults)
        {
            chassis_emergency_stop(faults);
            return;
        }
        car_control_update_wheel_targets(velocity_test_left_mps,
            velocity_test_right_mps, CAR_CONTROL_DT_S);
        chassis_state.base_speed_ref_mps =
            (velocity_test_left_mps + velocity_test_right_mps) * 0.5f;
        chassis_refresh_public_state();
        control = car_control_get_state();
        car_telemetry_update(chassis_state.left_speed_mps,
            chassis_state.right_speed_mps, chassis_state.left_speed_ref_mps,
            chassis_state.right_speed_ref_mps, 0.0f, 1U, 0.0f, 0.0f,
            chassis_state.left_pwm, chassis_state.right_pwm);
        if(chassis_state.run_time_ms >= velocity_test_duration_ms)
        {
            chassis_finish();
        }
        return;
    }

    if(yaw_test_active)
    {
        if(stop_requested)
        {
            chassis_finish();
            return;
        }
        /*
         * 与正式运行及 VEL 保持相同时序：先检查上一周期已施加的 PWM，
         * 再计算当前周期的偏航角速度闭环输出。
         */
        faults = car_safety_update(1U);
        if(CHASSIS_FAULT_NONE != faults)
        {
            chassis_emergency_stop(faults);
            return;
        }
        car_control_update_yaw_target(yaw_test_base_mps,
            yaw_test_target_dps, CAR_CONTROL_DT_S);
        chassis_state.base_speed_ref_mps = yaw_test_base_mps;
        chassis_refresh_public_state();
        control = car_control_get_state();
        car_telemetry_update(chassis_state.left_speed_mps,
            chassis_state.right_speed_mps, chassis_state.left_speed_ref_mps,
            chassis_state.right_speed_ref_mps, 0.0f, 1U,
            control->yaw_rate_target_dps, control->yaw_rate_feedback_dps,
            chassis_state.left_pwm, chassis_state.right_pwm);
        if(chassis_state.run_time_ms >= yaw_test_duration_ms)
        {
            chassis_finish();
        }
        return;
    }

    if(CHASSIS_TASK_3_STATIONARY == chassis_state.task)
    {
        if(stop_requested ||
            (CHASSIS_MODE_PRE_BRAKE == chassis_state.mode))
        {
            chassis_finish();
            return;
        }
        car_motor_stop();
        chassis_refresh_public_state();
        return;
    }

    safety_line_valid = chassis_state.line_valid;
    faults = car_safety_update(safety_line_valid);
    if(CHASSIS_FAULT_NONE != faults)
    {
        chassis_emergency_stop(faults);
        return;
    }

    /*
     * 只在需要识别 A 点的任务和既有里程/时间门控打开后采样。
     * 控制周期内仅写 RAM；停车后由 ALOG 命令阻塞式导出。
     */
    if((CHASSIS_TASK_4_A_TO_B != chassis_state.task) &&
        (CHASSIS_TASK_3_STATIONARY != chassis_state.task) &&
        (chassis_state.distance_m >= CAR_A_GATE_DISTANCE_M) &&
        (chassis_state.run_time_ms >= CAR_A_GATE_TIME_MS))
    {
        car_telemetry_update_a_log(chassis_state.run_time_ms,
            chassis_state.distance_m, chassis_state.gray_mask,
            chassis_state.gray_black_count, chassis_state.line_valid,
            chassis_state.line_error, chassis_state.yaw_rate_dps);
    }

    chassis_handle_task_transition();
    requested_speed = chassis_nominal_speed();

    if(!chassis_state.line_valid)
    {
        if(car_safety_get_line_lost_ticks() < CAR_LINE_SEARCH_TICKS)
        {
            requested_speed = CAR_LINE_SEARCH_SPEED_MPS;
        }
        else
        {
            requested_speed = 0.0f;
        }
    }

    if((CHASSIS_MODE_PRE_BRAKE == chassis_state.mode) &&
        !distance_target_valid && !stop_requested)
    {
        requested_speed = chassis_min(requested_speed,
            CAR_PRE_BRAKE_SPEED_MPS);
    }
    requested_speed = chassis_target_for_remaining_distance(requested_speed);
    car_planner_set_target(&speed_planner, requested_speed);
    planned_speed = car_planner_update(&speed_planner, CAR_CONTROL_DT_S);
    chassis_state.base_speed_ref_mps = planned_speed;

    car_control_update(planned_speed, chassis_state.line_error,
        chassis_state.line_valid, CAR_CONTROL_DT_S);
    chassis_refresh_public_state();
    control = car_control_get_state();
    car_telemetry_update(chassis_state.left_speed_mps,
        chassis_state.right_speed_mps, chassis_state.left_speed_ref_mps,
        chassis_state.right_speed_ref_mps, chassis_state.line_error,
        chassis_state.line_valid, control->yaw_rate_target_dps,
        control->yaw_rate_feedback_dps, chassis_state.left_pwm,
        chassis_state.right_pwm);

    if(chassis_reached_finish_condition())
    {
        chassis_finish();
    }
}

void chassis_set_base_speed(float speed_mps)
{
    if(speed_mps <= 0.0f)
    {
        base_speed_override_mps = 0.0f;
    }
    else
    {
        if(speed_mps < CAR_LINE_SEARCH_SPEED_MPS)
        {
            speed_mps = CAR_LINE_SEARCH_SPEED_MPS;
        }
        if(speed_mps > CAR_RACE_MAX_SPEED_MPS)
        {
            speed_mps = CAR_RACE_MAX_SPEED_MPS;
        }
        base_speed_override_mps = speed_mps;
    }
}

const ChassisState *chassis_get_state(void)
{
    return &chassis_state;
}

void chassis_emergency_stop(uint32 fault_bits)
{
    car_safety_latch(fault_bits);
    car_motor_stop();
    car_telemetry_freeze_a_log();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    chassis_state.mode = CHASSIS_MODE_FAULT;
    chassis_state.base_speed_ref_mps = 0.0f;
    chassis_state.fault_bits = car_safety_get_faults();
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_active = 0U;
    stop_requested = 0U;
    result_capture_pending = 0U;
}

uint8 chassis_reset(void)
{
    if((CHASSIS_MODE_FAULT != chassis_state.mode) &&
        (CHASSIS_MODE_FINISHED != chassis_state.mode))
    {
        return 0U;
    }
    if(!car_imu_is_ready())
    {
        return 0U;
    }

    car_motor_stop();
    car_encoder_reset();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    car_safety_reset();
    chassis_state.mode = CHASSIS_MODE_READY;
    chassis_state.fault_bits = CHASSIS_FAULT_NONE;
    chassis_state.run_time_ms = 0U;
    chassis_state.lap_time_ms = 0U;
    stop_line_frames = 0U;
    a_line_latched = 0U;
    stop_requested = 0U;
    distance_target_valid = 0U;
    result_capture_pending = 0U;
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_active = 0U;
    chassis_refresh_public_state();
    return 1U;
}

uint8 chassis_select_task(ChassisTask task)
{
    if((task < CHASSIS_TASK_2_LAP_STOP_A) ||
        (task > CHASSIS_TASK_6_LAP_POSITION))
    {
        return 0U;
    }
    if((CHASSIS_MODE_READY != chassis_state.mode) &&
        (CHASSIS_MODE_FINISHED != chassis_state.mode))
    {
        return 0U;
    }
    chassis_state.task = task;
    return 1U;
}

uint8 chassis_start_motor_test(int16 left, int16 right, uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != chassis_state.mode) ||
        (duration_ms == 0U) || (duration_ms > CAR_MOTOR_TEST_MAX_MS))
    {
        return 0U;
    }
    if((left > CAR_MOTOR_OUTPUT_LIMIT) ||
        (left < -CAR_MOTOR_OUTPUT_LIMIT) ||
        (right > CAR_MOTOR_OUTPUT_LIMIT) ||
        (right < -CAR_MOTOR_OUTPUT_LIMIT))
    {
        return 0U;
    }

    car_encoder_reset();
    car_control_reset();
    car_safety_reset();
    car_telemetry_reset();
    chassis_state.run_time_ms = 0U;
    chassis_state.mode = CHASSIS_MODE_RUNNING;
    motor_test_left = left;
    motor_test_right = right;
    motor_test_duration_ms = duration_ms;
    motor_test_active = 1U;
    return 1U;
}

uint8 chassis_start_velocity_test(float left_mps, float right_mps,
    uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != chassis_state.mode) ||
        (duration_ms < CAR_VELOCITY_TEST_MIN_MS) ||
        (duration_ms > CAR_VELOCITY_TEST_MAX_MS) ||
        (left_mps < 0.0f) ||
        (left_mps > CAR_VELOCITY_TEST_MAX_SPEED_MPS) ||
        (right_mps < 0.0f) ||
        (right_mps > CAR_VELOCITY_TEST_MAX_SPEED_MPS))
    {
        return 0U;
    }

    car_motor_stop();
    car_encoder_reset();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    car_safety_reset();
    car_telemetry_reset();
    chassis_state.run_time_ms = 0U;
    chassis_state.lap_time_ms = 0U;
    chassis_state.distance_m = 0.0f;
    chassis_state.mode = CHASSIS_MODE_RUNNING;
    stop_requested = 0U;
    motor_test_active = 0U;
    yaw_test_active = 0U;
    velocity_test_left_mps = left_mps;
    velocity_test_right_mps = right_mps;
    velocity_test_duration_ms = duration_ms;
    velocity_test_active = 1U;
    return 1U;
}

uint8 chassis_start_yaw_test(float base_speed_mps, float yaw_target_dps,
    uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != chassis_state.mode) ||
        !isfinite(base_speed_mps) || !isfinite(yaw_target_dps) ||
        (base_speed_mps < CAR_YAW_TEST_BASE_MIN_MPS) ||
        (base_speed_mps > CAR_YAW_TEST_BASE_MAX_MPS) ||
        (chassis_abs(yaw_target_dps) > CAR_YAW_TEST_MAX_RATE_DPS) ||
        (duration_ms < CAR_YAW_TEST_MIN_MS) ||
        (duration_ms > CAR_YAW_TEST_MAX_MS))
    {
        return 0U;
    }

    car_motor_stop();
    car_encoder_reset();
    car_control_reset();
    car_planner_force_zero(&speed_planner);
    car_safety_reset();
    car_telemetry_reset();
    chassis_state.run_time_ms = 0U;
    chassis_state.lap_time_ms = 0U;
    chassis_state.distance_m = 0.0f;
    chassis_state.mode = CHASSIS_MODE_RUNNING;
    stop_requested = 0U;
    motor_test_active = 0U;
    velocity_test_active = 0U;
    yaw_test_base_mps = base_speed_mps;
    yaw_test_target_dps = yaw_target_dps;
    yaw_test_duration_ms = duration_ms;
    yaw_test_active = 1U;
    return 1U;
}
