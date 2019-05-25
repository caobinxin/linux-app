#ifndef __MOTORCONTROL_H
#define __MOTORCONTROL_H

#define LEFT_MOTOR_COM  COM4
#define RIGHT_MOTOR_COM COM5

#define MAX_ROTARY_SPEED  60*10   // MAX:1000RPM，50mm/s = 56.18RPM
#define MIN_ROTARY_SPEED  0
#define DEAD_ROTARY_SPEED 1

#define MAX_LINE_SPEED    60*10   // MAX:1000RPM，50mm/s = 56.18RPM
#define MIN_LINE_SPEED    0
#define DEAD_LINE_SPEED   1

extern volatile uint32_t g_encoder_time;


/* 供外部调用的函数声明 */
void MotorReInit(void);
void MotorControl(void);
void MotorEncode(void);
void MotorStop(int motor);
void MotorMove(int motor,int speed);
void engine_driven(int16_t speed,int16_t omega);
void get_encoder(void);
uint16_t speed_frequency(uint16_t speed);

void pid_init(void);
void set_pid(void);
uint8_t get_overcurrent_status(void);
void get_motor_current(uint8_t *current);


extern uint8_t request_flag;


#endif
