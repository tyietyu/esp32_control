#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_


void motor_init(void);
void motor_forward_for_duration(int motor_index, uint32_t duration_ms); 
void motor_reverse_for_duration(int motor_index, uint32_t duration_ms);
void motor_stop(int motor_index);

#endif // !_MOTOR_CONTROL_H_

