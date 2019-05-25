#include  "bsp.h"
#include  "tim.h"

uint32_t system_time = 0;    // 开机后系统累加时间：最小单位为10us

/**
  * @brief  Configure the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_TIM5_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM5 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

/********************************************************************************
 * @fn           TIM5_Encoder_Init
 * @brief        TIM5 Encoder PWM input
 *
 * @param[in]    None
 * @param[in]    None
 * @return       None
********************************************************************************/
void TIM5_Configuration(void)
{
    uint16_t PrescalerValue = 0;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    /* TIM5 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    NVIC_TIM5_Configuration();

    /* Compute the prescaler value */
    PrescalerValue = (uint16_t) (SystemCoreClock / 100000) - 1;

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 50000-1;  // 500ms
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM5,ENABLE);

    /* TIM IT enable */
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

    TIM_SetCounter(TIM5,0);
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_Cmd(TIM5, ENABLE);     //开启TIM5定时器
}

/**
  * @brief  This function handles TIM5 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
	  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	  system_time++;  // 系统时间间隔500ms
       
//        led_toggle(LED1_R_PIN);   // test timer period
    }
}


/********************************************************************************
 * @fn           get_system_time
 * @brief        Get system time
 *
 * @param[in]    NULL
 * @return       time
********************************************************************************/
uint32_t get_system_time(void)
{
    uint32_t time = 0;
	//__disable_irq();
    time = TIM5->CNT;              // 10us counts
    time += system_time*50000;     // 500ms counts
    //__enable_irq();
    return time;
}

