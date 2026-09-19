#include "car_scheduler.h"

#include "car_board.h"
#include "car_config.h"
#include "zf_common_interrupt.h"

static volatile uint32 scheduler_time_ms;
static volatile uint8 pending_control_ticks;
static volatile uint8 control_divider;

static void car_scheduler_pit_callback(uint32 state, void *ptr)
{
    (void)state;
    (void)ptr;
    scheduler_time_ms++;
    control_divider++;
    if(control_divider >= CAR_CONTROL_PERIOD_MS)
    {
        control_divider = 0U;
        if(pending_control_ticks < 250U)
        {
            pending_control_ticks++;
        }
    }
}

void car_scheduler_init(void)
{
    scheduler_time_ms = 0U;
    pending_control_ticks = 0U;
    control_divider = 0U;
    pit_ms_init(CAR_BOARD_SCHEDULER_PIT, 1U,
        car_scheduler_pit_callback, NULL);
    /*
     * 编码器 GPIO=0，调试 UART0=1，调度时基=2。
     * 调度回调只置标志，因此允许被编码器和调试串口抢占。
     */
    interrupt_set_priority(TIMG12_INT_IRQn, 2U);
}

uint8 car_scheduler_take_control_ticks(void)
{
    uint8 ticks;
    uint32 interrupt_state = interrupt_global_disable();

    ticks = pending_control_ticks;
    pending_control_ticks = 0U;
    interrupt_global_enable(interrupt_state);
    return ticks;
}

uint32 car_scheduler_get_time_ms(void)
{
    uint32 value;
    uint32 interrupt_state = interrupt_global_disable();

    value = scheduler_time_ms;
    interrupt_global_enable(interrupt_state);
    return value;
}
