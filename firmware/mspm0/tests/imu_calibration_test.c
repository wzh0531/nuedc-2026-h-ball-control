#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "car_config.h"
#include "car_imu.h"
#include "zf_device_imu660ra.h"

/*
 * 直接链接正式 car_imu.c，只替换 IMU660RA 寄存器读取和 delay。
 * 用它验证上电非阻塞标定与停车后同步 IMUCAL 共用同一套采样状态机。
 */
int16 imu660ra_gyro_z;
float imu660ra_transition_factor[2] = {1.0f, 100.0f};

static uint8 stub_init_result;
static uint32 stub_sample_index;
static uint32 stub_delay_ms_total;
static uint8 stub_noisy_residual;

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] IMU calibration line %d: %s\n",
        line, expression);
    exit(1);
}

#define TEST_ASSERT(expression) \
    do \
    { \
        if(!(expression)) \
        { \
            test_fail(#expression, __LINE__); \
        } \
    } while(0)

static uint8 test_near(float actual, float expected, float tolerance)
{
    return (uint8)(fabsf(actual - expected) <= tolerance);
}

uint8 imu660ra_init(void)
{
    return stub_init_result;
}

void imu660ra_get_gyro(void)
{
    /*
     * 前 400 帧恒为 10 dps 零偏；后 50 帧用于残差验证。
     * noisy=0 时残差为 0.01 dps，noisy=1 时为 1 dps。
     */
    if(stub_sample_index < CAR_IMU_CALIBRATION_SAMPLES)
    {
        imu660ra_gyro_z = 1000;
    }
    else if(stub_noisy_residual)
    {
        imu660ra_gyro_z = 1100;
    }
    else
    {
        imu660ra_gyro_z =
            (0U == (stub_sample_index & 1U)) ? 1001 : 999;
    }
    stub_sample_index++;
}

void system_delay_ms(uint32 time_ms)
{
    stub_delay_ms_total += time_ms;
}

void system_delay_us(uint32 time_us)
{
    (void)time_us;
}

static void test_device_failure(void)
{
    stub_init_result = 1U;
    stub_sample_index = 0U;
    TEST_ASSERT(!car_imu_init());
    TEST_ASSERT(!car_imu_is_calibrating());
    TEST_ASSERT(!car_imu_is_ready());
}

static void test_non_blocking_startup_calibration(void)
{
    uint16 index;

    stub_init_result = ZF_NO_ERROR;
    stub_sample_index = 0U;
    stub_delay_ms_total = 0U;
    stub_noisy_residual = 0U;

    TEST_ASSERT(car_imu_init());
    TEST_ASSERT(car_imu_is_calibrating());
    TEST_ASSERT(!car_imu_is_ready());

    for(index = 0U;
        index < (CAR_IMU_CALIBRATION_SAMPLES + 49U);
        index++)
    {
        TEST_ASSERT(!car_imu_calibration_step());
    }
    TEST_ASSERT(car_imu_is_calibrating());
    TEST_ASSERT(car_imu_calibration_step());
    TEST_ASSERT(!car_imu_is_calibrating());
    TEST_ASSERT(car_imu_is_ready());
    TEST_ASSERT(test_near(car_imu_get_bias(), 10.0f, 0.0001f));
    TEST_ASSERT(car_imu_get_residual() < 0.02f);
    TEST_ASSERT(0U == stub_delay_ms_total);

    /* 标定后验证死区之前的 1 dps 输入和 0.25 一阶滤波。 */
    stub_sample_index = 0U;
    imu660ra_gyro_z = 1100;
    stub_noisy_residual = 0U;
    /*
     * update 内会再次调用 get_gyro，因此临时把样本索引放到残差段，
     * 并用 noisy 模式固定输出 11 dps。
     */
    stub_sample_index = CAR_IMU_CALIBRATION_SAMPLES;
    stub_noisy_residual = 1U;
    car_imu_update(CAR_CONTROL_DT_S);
    TEST_ASSERT(test_near(car_imu_get_rate(), 0.25f, 0.0001f));
}

static void test_blocking_recalibration_rejects_motion(void)
{
    stub_sample_index = 0U;
    stub_delay_ms_total = 0U;
    stub_noisy_residual = 1U;

    TEST_ASSERT(!car_imu_calibrate());
    TEST_ASSERT(!car_imu_is_calibrating());
    TEST_ASSERT(!car_imu_is_ready());
    TEST_ASSERT(test_near(car_imu_get_residual(), 1.0f, 0.0001f));
    TEST_ASSERT((CAR_IMU_CALIBRATION_SAMPLES + 49U) *
        CAR_IMU_CALIBRATION_DELAY_MS == stub_delay_ms_total);
}

int main(void)
{
    test_device_failure();
    test_non_blocking_startup_calibration();
    test_blocking_recalibration_rejects_motion();
    printf("[PASS] real IMU adapter non-blocking and blocking calibration\n");
    return 0;
}
