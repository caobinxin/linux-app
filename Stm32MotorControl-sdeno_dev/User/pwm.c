/**
  ******************************************************************************
  * @file    TIM_OCToggle
  * @author
  * @version
  * @date
  * @brief
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "bsp.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define  TIM_COUNTER_CLOCK   200000  // Frequency = TIM_COUNTER_CLOCK/CCRx_Val/2

/* Private variables ---------------------------------------------------------*/
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
__IO uint16_t CCR1_Val = 0;
__IO uint16_t CCR2_Val = 0;
__IO uint16_t CCR3_Val = 0;
__IO uint16_t CCR4_Val = 0;
uint16_t PrescalerValue = 0;
uint16_t capture = 0;

__IO uint32_t pulse_cnt_l = 0;  // 输出脉冲数
__IO uint32_t pulse_cnt_r = 0;

__IO uint16_t CCR3_Val_l = 0;   // TIM重新载入值
__IO uint16_t CCR3_Val_r = 0;


/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void GPIO_Configuration(void);
static void NVIC_Configuration(void);
static void TIM3_Configuration(void);
static void TIM4_Configuration(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int pwm_init(void)
{
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f10x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f10x.c file
       */

    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC Configuration */
    NVIC_Configuration();

    /* GPIO Configuration */
    GPIO_Configuration();

    TIM3_Configuration();

    TIM4_Configuration();

}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
static void RCC_Configuration(void)
{
    /* PCLK1 = HCLK/4 */
    //RCC_PCLK1Config(RCC_HCLK_Div4);

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* TIM4 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* GPIOA clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

/**
  * @brief  Configure the TIM2 Pins.
  * @param  None
  * @retval None
  */
static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* GPIOB Configuration:TIM3 Channel3 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* GPIOB Configuration:TIM4 Channel3 as alternate function push-pull */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the TIM4 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configure tim3 OCToggle.
  * @param  None
  * @retval None
  */
static void TIM3_Configuration(void)
{
    /* ---------------------------------------------------------------------------
      TIMx Configuration: Output Compare Toggle Mode:
      TIMxCLK = SystemCoreClock,
      The objective is to get TIMx counter clock at 10KHz:
       - Prescaler = (TIMxCLK / TIMx counter clock) - 1
      CC1 update rate = TIMx counter clock / CCR1_Val = 1000 Hz
      CC2 update rate = TIMx counter clock / CCR2_Val = 1000 Hz
      CC3 update rate = TIMx counter clock / CCR3_Val = 1000 Hz
      CC4 update rate = TIMx counter clock / CCR4_Val = 1000 Hz
    ----------------------------------------------------------------------------*/
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNTER_CLOCK) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* Output Compare Toggle Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Disable);

    /* TIM disable counter */
    TIM_Cmd(TIM3, DISABLE);

    /* TIM IT enable */
    TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
}

/**
  * @brief  Configure tim4 OCToggle.
  * @param  None
  * @retval None
  */
static void TIM4_Configuration(void)
{
    /* ---------------------------------------------------------------------------
      TIMx Configuration: Output Compare Toggle Mode:
      TIMxCLK = SystemCoreClock,
      The objective is to get TIMx counter clock at 10KHz:
       - Prescaler = (TIMxCLK / TIMx counter clock) - 1
      CC1 update rate = TIMx counter clock / CCR1_Val = 1000 Hz
      CC2 update rate = TIMx counter clock / CCR2_Val = 1000 Hz
      CC3 update rate = TIMx counter clock / CCR3_Val = 1000 Hz
      CC4 update rate = TIMx counter clock / CCR4_Val = 1000 Hz
    ----------------------------------------------------------------------------*/
    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNTER_CLOCK) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 65535;
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* Output Compare Toggle Mode configuration: Channel3 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Toggle;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);

    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);

    /* TIM disable counter */
    TIM_Cmd(TIM4, DISABLE);

    /* TIM IT enable */
    TIM_ITConfig(TIM4, TIM_IT_CC3, ENABLE);

}

/**
  * @brief  Configure tim3 OCToggle.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
        capture = TIM_GetCapture3(TIM3);
        TIM_SetCompare3(TIM3, capture + CCR3_Val_l);
        pulse_cnt_l++;
    }

}

/**
  * @brief  Configure tim4 OCToggle.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{

    if (TIM_GetITStatus(TIM4, TIM_IT_CC3) != RESET) {
        TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
        capture = TIM_GetCapture3(TIM4);
        TIM_SetCompare3(TIM4, capture + CCR3_Val_r);
        pulse_cnt_r++;
    }

}

/**
  * @brief  set frequency.
  * @param  None
  * @retval None
  */
void set_pwm_frequency(uint8_t deviceId,uint16_t f)
{
	if(f != 0){
	if(deviceId == 0){
		CCR3_Val_l = TIM_COUNTER_CLOCK/2/f;
	}else{
		CCR3_Val_r = TIM_COUNTER_CLOCK/2/f;
	}
	}
}

/**
  * @brief  get frequency.
  * @param  None
  * @retval None
  */
uint16_t get_pwm_frequency(uint8_t deviceId)
{
	uint16_t frequency = 0;
	if(deviceId == 0){
		if(CCR3_Val_l != 0 ){
		frequency = TIM_COUNTER_CLOCK/2/CCR3_Val_l;
		}
	}else{
		if(CCR3_Val_r != 0 ){
		frequency = TIM_COUNTER_CLOCK/2/CCR3_Val_r;
		}
	}
	return frequency;
}



/**
  * @brief  stop time frequency output
  * @param  None
  * @retval None
  */
void tim_pwm_start(uint8_t deviceId)
{
	if(deviceId == 0){
		TIM3->CNT = 0;
		TIM_Cmd(TIM3, ENABLE);
		//bsp_ConfigTimGpio(MOTOR_DRV_L_PUL_PORT, MOTOR_DRV_L_PUL_PIN, TIM3, 3);	/* 使能GPIO和TIM时钟，并连接TIM通道到GPIO */		
	}else{	
		TIM4->CNT = 0;    
		TIM_Cmd(TIM4, ENABLE);
		//bsp_ConfigTimGpio(MOTOR_DRV_L_PUL_PORT, MOTOR_DRV_L_PUL_PIN, TIM3, 3);	/* 使能GPIO和TIM时钟，并连接TIM通道到GPIO */	
	}
}


/**
  * @brief  stop time frequency output
  * @param  None
  * @retval None
  */
void tim_pwm_stop(uint8_t deviceId)
{
    if(deviceId == 0){
		TIM3->CNT = 0;
		TIM_Cmd(TIM3, DISABLE);
		//bsp_ConfigGpioOut(MOTOR_DRV_L_PUL_PORT, MOTOR_DRV_L_PUL_PIN);	/* 配置GPIO为推挽输出 */		
		//GPIO_WriteBit(MOTOR_DRV_L_PUL_PORT, MOTOR_DRV_L_PUL_PIN, Bit_RESET);	/* PWM = 0 */
	}else{	
		TIM4->CNT = 0;    
		TIM_Cmd(TIM4, DISABLE);
		//bsp_ConfigGpioOut(MOTOR_DRV_R_PUL_PORT, MOTOR_DRV_R_PUL_PIN);	/* 配置GPIO为推挽输出 */		
		//GPIO_WriteBit(MOTOR_DRV_R_PUL_PORT, MOTOR_DRV_R_PUL_PIN, Bit_RESET);	/* PWM = 0 */
	}
}

/**
  * @brief  delay_us
  * @param  time:us count
  * @retval None
  */
void delay_us(u32 time)
{
  u32 i=8*time;
   while(i--);
}

/**
  * @brief  delay_ms
  * @param  time:ms count
  * @retval None
  */
void delay_ms(u32 time)
{
  u32 i=8000*time;
  while(i--);
}


/**********************************************************END OF FILE****/

