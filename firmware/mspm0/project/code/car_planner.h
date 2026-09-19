#ifndef _CAR_PLANNER_H_
#define _CAR_PLANNER_H_

/* L4 Middleware - 基础速度限加速度、限加加速度 S 曲线规划器。 */
typedef struct
{
    float current_mps;
    float target_mps;
    float acceleration_mps2;
    float acceleration_limit_mps2;
    unsigned char releasing_acceleration;
}car_planner_struct;

void car_planner_init(car_planner_struct *planner);
void car_planner_set_target(car_planner_struct *planner, float target_mps);
void car_planner_set_acceleration_limit(car_planner_struct *planner,
    float acceleration_limit_mps2);
float car_planner_update(car_planner_struct *planner, float dt_s);
void car_planner_force_zero(car_planner_struct *planner);

#endif
