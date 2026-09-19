#include "car_planner.h"

#include "car_config.h"

static float car_planner_move_toward(float value, float target, float step)
{
    if(value < target)
    {
        value += step;
        if(value > target)
        {
            value = target;
        }
    }
    else if(value > target)
    {
        value -= step;
        if(value < target)
        {
            value = target;
        }
    }
    return value;
}

static float car_planner_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

void car_planner_init(car_planner_struct *planner)
{
    planner->current_mps = 0.0f;
    planner->target_mps = 0.0f;
    planner->acceleration_mps2 = 0.0f;
    planner->acceleration_limit_mps2 = CAR_ACCEL_LIMIT_MPS2;
    planner->releasing_acceleration = 0U;
}

void car_planner_set_target(car_planner_struct *planner, float target_mps)
{
    if(target_mps > CAR_WHEEL_SPEED_LIMIT_MPS)
    {
        target_mps = CAR_WHEEL_SPEED_LIMIT_MPS;
    }
    else if(target_mps < -CAR_WHEEL_SPEED_LIMIT_MPS)
    {
        target_mps = -CAR_WHEEL_SPEED_LIMIT_MPS;
    }
    planner->target_mps = target_mps;
}

void car_planner_set_acceleration_limit(car_planner_struct *planner,
    float acceleration_limit_mps2)
{
    if(acceleration_limit_mps2 <= 0.0f)
    {
        acceleration_limit_mps2 = CAR_ACCEL_LIMIT_MPS2;
    }
    else if(acceleration_limit_mps2 > CAR_ACCEL_LIMIT_MPS2)
    {
        acceleration_limit_mps2 = CAR_ACCEL_LIMIT_MPS2;
    }
    planner->acceleration_limit_mps2 = acceleration_limit_mps2;
}

float car_planner_update(car_planner_struct *planner, float dt_s)
{
    float error;
    float direction;
    float acceleration_limit;
    float desired_acceleration;
    float braking_delta_speed;
    float next_speed;

    if(dt_s <= 0.0f)
    {
        return planner->current_mps;
    }

    error = planner->target_mps - planner->current_mps;
    if(car_planner_abs(error) < 0.000001f)
    {
        planner->current_mps = planner->target_mps;
        planner->acceleration_mps2 = car_planner_move_toward(
            planner->acceleration_mps2, 0.0f,
            CAR_JERK_LIMIT_MPS3 * dt_s);
        if(car_planner_abs(planner->acceleration_mps2) < 0.000001f)
        {
            planner->acceleration_mps2 = 0.0f;
            planner->releasing_acceleration = 0U;
        }
        return planner->current_mps;
    }

    direction = (error >= 0.0f) ? 1.0f : -1.0f;
    acceleration_limit = planner->acceleration_limit_mps2;
    if((0.0f == planner->target_mps) ||
        (planner->current_mps * planner->target_mps < 0.0f) ||
        (car_planner_abs(planner->target_mps) <
        car_planner_abs(planner->current_mps)))
    {
        acceleration_limit = CAR_DECEL_LIMIT_MPS2;
    }
    desired_acceleration = direction * acceleration_limit;

    /*
     * 当剩余速度差只够把当前加速度按 jerk 降到零时，提前撤掉加速度。
     * 这样起步和停车的加速度均连续，速度轮廓为离散 S 曲线。
     */
    braking_delta_speed =
        planner->acceleration_mps2 * planner->acceleration_mps2 /
        (2.0f * CAR_JERK_LIMIT_MPS3);
    if(planner->releasing_acceleration)
    {
        desired_acceleration = 0.0f;
    }
    else if((planner->acceleration_mps2 * direction > 0.0f) &&
        (car_planner_abs(error) <= braking_delta_speed))
    {
        planner->releasing_acceleration = 1U;
        desired_acceleration = 0.0f;
    }

    planner->acceleration_mps2 = car_planner_move_toward(
        planner->acceleration_mps2, desired_acceleration,
        CAR_JERK_LIMIT_MPS3 * dt_s);
    if(planner->releasing_acceleration &&
        (car_planner_abs(planner->acceleration_mps2) < 0.000001f))
    {
        planner->acceleration_mps2 = 0.0f;
        planner->releasing_acceleration = 0U;
    }
    next_speed = planner->current_mps +
        planner->acceleration_mps2 * dt_s;

    /* 防止离散积分跨过目标后在目标两侧来回修正。 */
    if(((error > 0.0f) && (next_speed >= planner->target_mps)) ||
        ((error < 0.0f) && (next_speed <= planner->target_mps)))
    {
        next_speed = planner->target_mps;
        if(car_planner_abs(planner->acceleration_mps2) > 0.000001f)
        {
            planner->releasing_acceleration = 1U;
        }
    }
    planner->current_mps = next_speed;
    return planner->current_mps;
}

void car_planner_force_zero(car_planner_struct *planner)
{
    planner->current_mps = 0.0f;
    planner->target_mps = 0.0f;
    planner->acceleration_mps2 = 0.0f;
    planner->releasing_acceleration = 0U;
}
