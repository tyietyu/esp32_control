#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_

#include <stdio.h>
#include <string.h>

void motor_init(void);
void motor_reverse_for_duration(uint8_t motor_index, uint32_t duration_ms);
void motor_forward_for_duration(uint8_t motor_index, uint32_t duration_ms);
void motor_stop(uint8_t motor_index);
void test(void);

void motor_start_forward(uint8_t motor_index);
void motor_start_reverse(uint8_t motor_index);

#endif // !_MOTOR_CONTROL_H_

