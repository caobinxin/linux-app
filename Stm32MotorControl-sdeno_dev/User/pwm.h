#ifndef  __PWM_H
#define  __PWM_H

extern __IO uint32_t pulse_cnt_l;
extern __IO uint32_t pulse_cnt_r;

extern int pwm_init(void);
extern void set_pwm_frequency(uint8_t deviceId,uint16_t f);
extern uint16_t get_pwm_frequency(uint8_t deviceId);
extern void tim_pwm_start(uint8_t deviceId);
extern void tim_pwm_stop(uint8_t deviceId);

extern void delay_us(u32 time);
extern void delay_ms(u32 time);

#endif

