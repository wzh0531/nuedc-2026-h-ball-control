#ifndef _CAR_COMMAND_H_
#define _CAR_COMMAND_H_

/* L6 App - UART0 底盘调试与 PID 参数命令。 */
void car_command_init(void);
void car_command_process(void);
void car_command_print_status(void);

#endif
