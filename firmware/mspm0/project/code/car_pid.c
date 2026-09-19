#include "car_pid.h"

static float car_pid_limit(float value, float limit)
{
    if(value > limit)
    {
        value = limit;
    }
    else if(value < -limit)
    {
        value = -limit;
    }
    return value;
}

void car_pid_init(car_pid_struct *pid, float kp, float ki, float kd,
    float integral_limit, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
    car_pid_reset(pid);
}

void car_pid_set_gains(car_pid_struct *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    car_pid_reset(pid);
}

void car_pid_reset(car_pid_struct *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->initialized = 0U;
}

float car_pid_update(car_pid_struct *pid, float target, float measurement,
    float dt_s, float feedforward)
{
    float error = target - measurement;
    float derivative = 0.0f;
    float candidate_integral;
    float unsaturated;
    float output;
    uint8 drives_further_into_saturation;

    if(pid->initialized && (dt_s > 0.0f))
    {
        derivative = (error - pid->previous_error) / dt_s;
    }
    candidate_integral = car_pid_limit(pid->integral + error * dt_s,
        pid->integral_limit);
    unsaturated = feedforward + pid->kp * error +
        pid->ki * candidate_integral + pid->kd * derivative;
    output = car_pid_limit(unsaturated, pid->output_limit);

    drives_further_into_saturation =
        (uint8)(((unsaturated > pid->output_limit) && (error > 0.0f)) ||
        ((unsaturated < -pid->output_limit) && (error < 0.0f)));
    if(!drives_further_into_saturation)
    {
        pid->integral = candidate_integral;
    }

    pid->previous_error = error;
    pid->initialized = 1U;
    return output;
}
