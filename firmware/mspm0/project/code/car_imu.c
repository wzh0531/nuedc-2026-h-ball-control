#include "car_imu.h"

#include "car_config.h"
#include "zf_device_imu660ra.h"
#include "zf_driver_delay.h"

static uint8 imu_device_ready;
static uint8 imu_ready;
static float gyro_bias;
static float gyro_residual;
static float yaw_rate;
static uint8 calibration_phase;
static uint16 calibration_count;
static float calibration_sum;

enum
{
    CAR_IMU_CALIBRATION_IDLE = 0,
    CAR_IMU_CALIBRATION_BIAS,
    CAR_IMU_CALIBRATION_RESIDUAL,
};

static float car_imu_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

uint8 car_imu_init(void)
{
    imu_device_ready = (uint8)(ZF_NO_ERROR == imu660ra_init());
    imu_ready = 0U;
    gyro_bias = 0.0f;
    gyro_residual = 999.0f;
    yaw_rate = 0.0f;
    calibration_phase = CAR_IMU_CALIBRATION_IDLE;
    calibration_count = 0U;
    calibration_sum = 0.0f;

    if(imu_device_ready)
    {
        /*
         * 只启动标定，不在初始化阶段连续 delay。随后由 chassis 的
         * CALIBRATE 状态每 5 ms 采一帧，使 OLED 和按键调度仍可运行。
         */
        calibration_phase = CAR_IMU_CALIBRATION_BIAS;
    }
    return imu_device_ready;
}

uint8 car_imu_is_calibrating(void)
{
    return (uint8)(CAR_IMU_CALIBRATION_IDLE != calibration_phase);
}

uint8 car_imu_calibration_step(void)
{
    float sample;

    if(CAR_IMU_CALIBRATION_IDLE == calibration_phase)
    {
        return 1U;
    }

    imu660ra_get_gyro();
    sample = imu660ra_gyro_transition(imu660ra_gyro_z);

    if(CAR_IMU_CALIBRATION_BIAS == calibration_phase)
    {
        calibration_sum += sample;
        calibration_count++;
        if(calibration_count < CAR_IMU_CALIBRATION_SAMPLES)
        {
            return 0U;
        }

        gyro_bias = calibration_sum /
            (float)CAR_IMU_CALIBRATION_SAMPLES;
        calibration_phase = CAR_IMU_CALIBRATION_RESIDUAL;
        calibration_count = 0U;
        calibration_sum = 0.0f;
        return 0U;
    }

    calibration_sum += car_imu_abs(sample - gyro_bias);
    calibration_count++;
    if(calibration_count < 50U)
    {
        return 0U;
    }

    gyro_residual = calibration_sum / 50.0f;
    yaw_rate = 0.0f;
    /*
     * 残差过大通常表示标定期间车体被移动或存在强振动。
     * 此时保留器件在线状态，允许停车后再次执行 IMUCAL，但禁止底盘启动。
     */
    imu_ready = (uint8)(gyro_residual <= CAR_IMU_RESIDUAL_LIMIT_DPS);
    calibration_phase = CAR_IMU_CALIBRATION_IDLE;
    return 1U;
}

uint8 car_imu_calibrate(void)
{
    if(!imu_device_ready)
    {
        return 0U;
    }

    imu_ready = 0U;
    gyro_residual = 999.0f;
    calibration_phase = CAR_IMU_CALIBRATION_BIAS;
    calibration_count = 0U;
    calibration_sum = 0.0f;

    /*
     * 该同步接口只供停车后的 IMUCAL 命令使用；上电流程使用
     * car_imu_calibration_step()，不会阻塞控制调度。
     */
    while(car_imu_is_calibrating())
    {
        (void)car_imu_calibration_step();
        if(car_imu_is_calibrating())
        {
            system_delay_ms(CAR_IMU_CALIBRATION_DELAY_MS);
        }
    }
    return imu_ready;
}

void car_imu_update(float dt_s)
{
    float measured_rate;

    /* 接口保留周期参数，便于未来滤波按实际周期换算；当前只做角速度滤波。 */
    (void)dt_s;
    if(!imu_ready)
    {
        return;
    }

    imu660ra_get_gyro();
    measured_rate = (imu660ra_gyro_transition(imu660ra_gyro_z) - gyro_bias) *
        CAR_IMU_YAW_SIGN;
    if(car_imu_abs(measured_rate) < CAR_IMU_GYRO_DEADZONE_DPS)
    {
        measured_rate = 0.0f;
    }

    yaw_rate += CAR_IMU_RATE_FILTER_ALPHA * (measured_rate - yaw_rate);
}

uint8 car_imu_is_ready(void)
{
    return imu_ready;
}

float car_imu_get_rate(void)
{
    return yaw_rate;
}

float car_imu_get_bias(void)
{
    return gyro_bias;
}

float car_imu_get_residual(void)
{
    return gyro_residual;
}
