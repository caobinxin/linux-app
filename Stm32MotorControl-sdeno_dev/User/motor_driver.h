#ifndef  __MOTOR_DRIVER_H
#define  __MOTOR_DRIVER_H

#include "stm32f10x.h"
#include  "motor.h"
#include  "pwm.h"
#include  "motorcontrol.h"
#include  "bsp.h"

// Motor left gpio
#define  MOTOR_DRV_L_PUL_PORT          (GPIOB)
#define  MOTOR_DRV_L_PUL_PIN           (GPIO_Pin_0)

#define  MOTOR_DRV_L_ERC_PORT           (GPIOA)
#define  MOTOR_DRV_L_ERC_PIN            (GPIO_Pin_6)

#define  MOTOR_DRV_L_DIR_PORT        (GPIOA)
#define  MOTOR_DRV_L_DIR_PIN         (GPIO_Pin_7)

// Motor right gpio
#define  MOTOR_DRV_R_PUL_PORT          (GPIOB)
#define  MOTOR_DRV_R_PUL_PIN           (GPIO_Pin_8)

#define  MOTOR_DRV_R_ERC_PORT           (GPIOB)
#define  MOTOR_DRV_R_ERC_PIN            (GPIO_Pin_6)

#define  MOTOR_DRV_R_DIR_PORT        (GPIOB)
#define  MOTOR_DRV_R_DIR_PIN         (GPIO_Pin_7)

#define  MOTOR_DRV_L_PUL_HIGH()       (GPIO_SetBits(MOTOR_DRV_L_PUL_PORT,MOTOR_DRV_L_PUL_PIN))
#define  MOTOR_DRV_L_PUL_LOW()        (GPIO_ResetBits(MOTOR_DRV_L_PUL_PORT,MOTOR_DRV_L_PUL_PIN))

#define  MOTOR_DRV_L_ERC_HIGH()       (GPIO_SetBits(MOTOR_DRV_L_ERC_PORT,MOTOR_DRV_L_ERC_PIN))
#define  MOTOR_DRV_L_ERC_LOW()        (GPIO_ResetBits(MOTOR_DRV_L_ERC_PORT,MOTOR_DRV_L_ERC_PIN))

#define  MOTOR_DRV_L_DIR_HIGH()       (GPIO_SetBits(MOTOR_DRV_L_DIR_PORT,MOTOR_DRV_L_DIR_PIN))
#define  MOTOR_DRV_L_DIR_LOW()        (GPIO_ResetBits(MOTOR_DRV_L_DIR_PORT,MOTOR_DRV_L_DIR_PIN))

#define  MOTOR_DRV_R_PUL_HIGH()       (GPIO_SetBits(MOTOR_DRV_R_PUL_PORT,MOTOR_DRV_R_PUL_PIN))
#define  MOTOR_DRV_R_PUL_LOW()        (GPIO_ResetBits(MOTOR_DRV_R_PUL_PORT,MOTOR_DRV_R_PUL_PIN))

#define  MOTOR_DRV_R_ERC_HIGH()       (GPIO_SetBits(MOTOR_DRV_R_ERC_PORT,MOTOR_DRV_R_ERC_PIN))
#define  MOTOR_DRV_R_ERC_LOW()        (GPIO_ResetBits(MOTOR_DRV_R_ERC_PORT,MOTOR_DRV_R_ERC_PIN))

#define  MOTOR_DRV_R_DIR_HIGH()       (GPIO_SetBits(MOTOR_DRV_R_DIR_PORT,MOTOR_DRV_R_DIR_PIN))
#define  MOTOR_DRV_R_DIR_LOW()        (GPIO_ResetBits(MOTOR_DRV_R_DIR_PORT,MOTOR_DRV_R_DIR_PIN))


///Max number of Brush DC motors
#define  NB_MAX_MOTORS (2)
///Number of Bridges
#define  NB_BRIDGES (2)


/// Device Parameters Structure Type
typedef struct {
    /// Pwm frequency of the bridge input
    uint32_t bridgePwmFreq[NB_BRIDGES];      
    /// Pwm frequency of the ref pin
    uint32_t refPwmFreq;      
    /// Pwm Duty Cycle of the ref pin
    uint8_t refPwmDc;      
    /// Speed% (from 0 to 100) of the corresponding motor
     uint16_t speed[NB_MAX_MOTORS];  
    /// FORWARD or BACKWARD direction of the motors
    motorDir_t direction[ NB_MAX_MOTORS];                 
    /// Current State of the motors
     motorState_t motionState[NB_MAX_MOTORS];       
    /// Current State of the bridges
    bool bridgeEnabled[NB_BRIDGES];    
    /// Enabling of a dual bridge configuration
    bool dualBridgeEnabled;    
}deviceParams_t; 

extern deviceParams_t devicePrm;

extern void motor_driver_init(void);
extern void MotorControl_SetMaxSpeed(uint8_t motorId, int16_t newMaxSpeed);
extern int16_t MotorControl_GetMaxSpeed(uint8_t motorId);
extern void MotorControl_SetDirection(uint8_t motorId, motorDir_t dir);
//extern void MotorControl_Run(uint8_t motorId, motorDir_t direction);
extern void MotorControl_Stop(uint8_t motorId);

extern void MotorControl_Run(uint8_t motorId, motorDir_t direction,int16_t speed);
extern void MotorControl_SetERC(uint8_t motorId, FunctionalState erc);

#endif
