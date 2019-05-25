/********************************************************************************
 * @Copyright (C),   2018, Slightech Inc    All rights reserved.
 *
 * @file         : main.c
 * @brief        : server Robot main program
 * @author       : River Li
 * @version      : 1.1.0
 * @date         : 2018-10-09
 *
 *
 * @note History:
 * @note         :
 * @note         : Modification: Created file
********************************************************************************/

#include "includes.h"

/*
**********************************************************************************************************
                                            函数声明
**********************************************************************************************************
*/
static void vTaskUserProtocol(void *pvParameters);
static void vTaskMotorControl(void *pvParameters);
static void vTaskIMU(void *pvParameters);
static void vTaskBridge(void *pvParameters);
static void AppTaskCreate (void);

/*
**********************************************************************************************************
                                            变量声明
**********************************************************************************************************
*/
static TaskHandle_t xHandlevTaskUserProtocol = NULL;
static TaskHandle_t xHandleTaskMotorControl = NULL;
static TaskHandle_t xHandleTaskIMU = NULL;
static TaskHandle_t xHandleBridge = NULL;

/*
*********************************************************************************************************
*   函 数 名: main
*   功能说明: 标准c程序入口。
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
    /* 硬件初始化 */
    bsp_Init();

    vSemaphoreCreateBinary(xSemaphore_imu_h);
    /* 创建任务 */
    AppTaskCreate();

    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

    /*
      如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
      heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
      #define configTOTAL_HEAP_SIZE       ( ( size_t ) ( 17 * 1024 ) )
    */
    while(1);
}

/*
*********************************************************************************************************
*   函 数 名: vTaskUserProtocol
*   功能说明: 上位机交互
*   形    参: pvParameters 是在创建该任务时传递的形参
*   返 回 值: 无
*   优 先 级: 1  (数值越小优先级越低，这个跟uCOS相反)
*********************************************************************************************************
*/
static void vTaskUserProtocol(void *pvParameters)
{
    u32 xLastWakeTime;
    const u32 xFrequency = 40;  // 与上位机通信时间间隔
    
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();//获取当前tick
    
    while(1) {
		vTaskDelayUntil( &xLastWakeTime, xFrequency/2 );
        get_encoder();  // 发送获取编码器命令
        g_encoder_time = get_system_time();  // 更新编码器时间戳
        
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency/2 );

        MotorEncode();   // 解析编码器数据

        UserProtocol();  // 打印测试数据

        SendCom();       // 发送传感器数据
			
        bsp_LedToggle(1); 
    }
}

/*
*********************************************************************************************************
*   函 数 名: vTaskMotorContol
*   功能说明: 电机控制
*   形    参: pvParameters 是在创建该任务时传递的形参
*   返 回 值: 无
*   优 先 级: 2
*********************************************************************************************************
*/
static void vTaskMotorControl(void *pvParameters)
{
    while(1) {
        MotorControl();  // 控制底盘运动
        vTaskDelay(3);
    }
}

/*
*********************************************************************************************************
*   函 数 名: vTaskIMU
*   功能说明: IMU操作
*   形    参: pvParameters 是在创建该任务时传递的形参
*   返 回 值: 无
*   优 先 级: 3
*********************************************************************************************************
*/
static void vTaskIMU(void *pvParameters)
{
    while(1) {
        IMUSend();  // 获取IMU数据
        //vTaskDelay(2);
    }
}

/*
*********************************************************************************************************
*   函 数 名: vTaskBridge
*   功能说明: 桥接外部数据
*   形    参: pvParameters 是在创建该任务时传递的形参
*   返 回 值: 无
*   优 先 级: 4
*********************************************************************************************************
*/
static void vTaskBridge(void *pvParameters)
{
    while(1) {
        bridge();  // 获取传感器数据
        vTaskDelay(10);
    }
}

/*
*********************************************************************************************************
*   函 数 名: AppTaskCreate
*   功能说明: 创建应用任务
*   形    参：无
*   返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
    xTaskCreate( vTaskUserProtocol,     /* 任务函数  */
                 "vTaskUserProtocol",       /* 任务名    */
                 512,                   /* 任务栈大小，单位word，也就是4字节 */
                 NULL,                  /* 任务参数  */
                 1,                     /* 任务优先级*/
                 &xHandlevTaskUserProtocol );  /* 任务句柄  */


    xTaskCreate( vTaskMotorControl,         /* 任务函数  */
                 "vTaskMotorControl",       /* 任务名    */
                 512,               /* 任务栈大小，单位word，也就是4字节 */
                 NULL,              /* 任务参数  */
                 2,                 /* 任务优先级*/
                 &xHandleTaskMotorControl ); /* 任务句柄  */

    xTaskCreate( vTaskIMU,          /* 任务函数  */
                 "vTaskIMU",        /* 任务名    */
                 512,                   /* 任务栈大小，单位word，也就是4字节 */
                 NULL,                  /* 任务参数  */
                 3,                     /* 任务优先级*/
                 &xHandleTaskIMU );  /* 任务句柄  */


    xTaskCreate( vTaskBridge,           /* 任务函数  */
                 "vTaskBridge",         /* 任务名    */
                 512,                   /* 任务栈大小，单位word，也就是4字节 */
                 NULL,                  /* 任务参数  */
                 4,                     /* 任务优先级*/
                 &xHandleBridge );   /* 任务句柄  */
}

/************************************* (END OF FILE) *********************************/
