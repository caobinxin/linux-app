#include  "motor_driver.h"
#include  "bsp.h"

/// Device Paramaters structure
deviceParams_t devicePrm;

/******************************************************//**
 * @brief Motor driver init
 * @param[in] None
 * @retval None
 * @note
 **********************************************************/
void motor_driver_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);

    /* Output function push-pull */
    GPIO_InitStructure.GPIO_Pin = MOTOR_DRV_L_ERC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_DRV_L_ERC_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_DRV_L_DIR_PIN;
    GPIO_Init(MOTOR_DRV_L_DIR_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_DRV_R_ERC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_DRV_R_ERC_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MOTOR_DRV_R_DIR_PIN;
    GPIO_Init(MOTOR_DRV_R_DIR_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(MOTOR_DRV_L_ERC_PORT,MOTOR_DRV_L_ERC_PIN);
    GPIO_ResetBits(MOTOR_DRV_L_DIR_PORT,MOTOR_DRV_L_DIR_PIN);

    GPIO_ResetBits(MOTOR_DRV_R_ERC_PORT,MOTOR_DRV_R_ERC_PIN);
    GPIO_ResetBits(MOTOR_DRV_R_DIR_PORT,MOTOR_DRV_R_DIR_PIN);

	 /* Set direction */
     MotorControl_SetDirection(0, FORWARD);
     MotorControl_SetDirection(1, BACKWARD);
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] newMaxSpeed New max speed  to apply in % for Brush DC motor (0-100)
 * @retval true if the command is successfully executed, else false
 * @note
 **********************************************************/
void MotorControl_SetMaxSpeed(uint8_t motorId, int16_t newMaxSpeed)
{
    devicePrm.speed[motorId] = newMaxSpeed;
    if (devicePrm.motionState[motorId] != INACTIVE) {
        /* Set PWM frequency*/
        //Set_PWM_DC(motorId, devicePrm.speed[motorId]);

        set_pwm_frequency(motorId, speed_frequency(newMaxSpeed));
    }
}

/******************************************************//**
 * @brief  Changes the max speed of the specified device
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in]
 * @retval  New max speed  to apply in % for Brush DC motor (0-100)
 * @note
 **********************************************************/
int16_t MotorControl_GetMaxSpeed(uint8_t motorId)
{
    uint16_t speed = 0;
    if(motorId>=NB_MAX_MOTORS) {

    } else {
        speed =  devicePrm.speed[motorId];
    }
    return speed;
}

/******************************************************//**
 * @brief  Specifies the direction
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void MotorControl_SetDirection(uint8_t motorId, motorDir_t dir)
{
    if(motorId==0) {
        if(dir==BACKWARD) {
            MOTOR_DRV_L_DIR_HIGH();
            devicePrm.direction[motorId] = BACKWARD;
        } else {
            MOTOR_DRV_L_DIR_LOW();
            devicePrm.direction[motorId] = FORWARD;
        }
    } else {
        if(dir==BACKWARD) {
            MOTOR_DRV_R_DIR_HIGH();
            devicePrm.direction[motorId] = BACKWARD;
        } else {
            MOTOR_DRV_R_DIR_LOW();
            devicePrm.direction[motorId] = FORWARD;
        }
    }
}

/******************************************************//**
 * @brief  Specifies the direction
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] dir FORWARD or BACKWARD
 * @note The direction change is only applied if the device
 * is in INACTIVE state
 * @retval None
 **********************************************************/
motorDir_t MotorControl_GetDirection(uint8_t motorId)
{
    motorDir_t dir = BACKWARD;
    if(motorId>=NB_MAX_MOTORS) {

    } else {
        dir =  devicePrm.direction[motorId];
    }
    return dir;
}

/******************************************************//**
 * @brief  Specifies the ERC
 * @param[in] motorId from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] erc ENABLE or DISABLE
 * @note The direction change is only applied if the device
 * is in INACTIVE state
 * @retval None
 **********************************************************/
void MotorControl_SetERC(uint8_t motorId, FunctionalState erc)
{
    if(motorId==0) {
        if(erc==DISABLE) {
            MOTOR_DRV_L_ERC_HIGH();
        } else {
            MOTOR_DRV_L_ERC_LOW();
        }
    } else {
        if(erc==DISABLE) {
            MOTOR_DRV_R_ERC_HIGH();
        } else {
            MOTOR_DRV_R_ERC_LOW();
        }
    }
}


/******************************************************//**
 * @brief  Runs the motor. It will accelerate from the min
 * speed up to the max speed by using the device acceleration.
 * @param[in] motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] direction FORWARD or BACKWARD
 * @retval None
 * @note For unidirectionnal brush DC motor, direction parameter
 * has no effect
 **********************************************************/
void MotorControl_Run(uint8_t motorId, motorDir_t direction,int16_t speed)
{
    if(motorId>=NB_MAX_MOTORS) {

    } else if ((devicePrm.motionState[motorId] == INACTIVE) ||
               (devicePrm.direction[motorId] != direction)) {
        /* Eventually deactivate motor */
        if (devicePrm.motionState[motorId] != INACTIVE) { // 反向运行时先停止
            //MotorControl_Stop(motorId);
            MotorControl_Stop(0);
            MotorControl_Stop(1);

            vTaskDelay(75);  // 减速延时
        }
		
        //MotorControl_SetERC(motorId,ENABLE); /* Set ERC enable*/
        /* Set direction */
        MotorControl_SetDirection(motorId, direction);
        delay_us(10); // 设置方向，先于脉冲信号至少5us建立
        
        /* Switch to steady state */
        devicePrm.motionState[motorId] = STEADY;

        /* Set PWM */
        //MotorControl_SetMaxSpeed(motorId,devicePrm.speed[motorId]);

        tim_pwm_start(motorId);  // 开始频率输出

    }
    /* Set speed(rpm) */
    MotorControl_SetMaxSpeed(motorId,speed);
    if(speed==0) {
        if(devicePrm.motionState[motorId] != INACTIVE) {
            MotorControl_Stop(motorId);
        }
    }
}


/******************************************************//**
 * @brief  Stop the motor.
 * @param[in] motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @retval None
 * @note None
 **********************************************************/
void MotorControl_Stop(uint8_t motorId)
{
    if(devicePrm.motionState[motorId] != INACTIVE) {
        /* Disable corresponding PWM */

        if(motorId==0) {
            MOTOR_DRV_L_PUL_LOW();
        }
        if(motorId==1) {
            MOTOR_DRV_R_PUL_LOW();
        }

        tim_pwm_stop(motorId);  // 停止频率输出

		 //MotorControl_SetERC(motorId,DISABLE); /* Set ERC disable*/

        /* Set inactive state */
        devicePrm.motionState[motorId] = INACTIVE;

    }
}


/******************************************************//**
 * @brief Select the motor decay mode
 * @param[in] motorId  from 0 to MAX_NUMBER_OF_BRUSH_DC_MOTORS for Brush DC motor
 * @param[in] decayMode (SLOW_DECAY or FAST_DECAY)
 * @retval None
 **********************************************************/
void MotorControl_SetDecayMode(uint8_t motorId, motorDecayMode_t decayMode)
{

}
