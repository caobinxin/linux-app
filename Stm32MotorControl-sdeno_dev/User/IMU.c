#include "bsp.h"
#include "includes.h"

IMU_DATA_TypeDef imu_data = {0};
volatile uint32_t g_imu_time = 0;
SemaphoreHandle_t xSemaphore_imu_h = NULL;


extern  UART_T g_tUart3;
u8 IMUProtocol[IMULength];

void IMUSend(void)
{
  if(xSemaphoreTake(xSemaphore_imu_h,portMAX_DELAY)) {
    u8 i,IMUCheck=0,pos=0;
    if(g_tUart3.usRxCount >=IMULength ) {
        //printf("%d\r\n",g_tUart3.usRxCount);
        if(g_tUart3.pRxBuf[g_tUart3.usRxRead]==0xAA&&g_tUart3.pRxBuf[g_tUart3.usRxRead+1]==0xAA) {
            if (g_tUart3.usRxRead>=g_tUart3.usRxBufSize-IMULength) {
                //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
                //comSendBuf(COM1,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
                memcpy(IMUProtocol,g_tUart3.pRxBuf+g_tUart3.usRxRead,g_tUart3.usRxBufSize-g_tUart3.usRxRead);
                memcpy(IMUProtocol+g_tUart3.usRxBufSize-g_tUart3.usRxRead,g_tUart3.pRxBuf,g_tUart3.usRxRead+IMULength-g_tUart3.usRxBufSize);

                for(i=2; i<18; i++)IMUCheck=IMUCheck+IMUProtocol[i];
                if(IMUProtocol[18]==IMUCheck) {
                    g_tUart3.usRxRead=g_tUart3.usRxRead+IMULength-g_tUart3.usRxBufSize;
                    g_tUart3.usRxCount-=IMULength;
                    pos=3;
                    imu_data.yaw   = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.pitch  = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.roll     = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.x_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.y_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.z_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);

					imu_data.timestamp = g_imu_time;
                } else {
                    g_tUart3.usRxRead++;
                    g_tUart3.usRxCount--;
                    if(g_tUart3.usRxRead==g_tUart3.usRxBufSize)g_tUart3.usRxRead=0;
                }
            } else { //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,UserProtocolLength);
                memcpy(IMUProtocol,g_tUart3.pRxBuf+g_tUart3.usRxRead,IMULength);
                for(i=2; i<18; i++)IMUCheck=IMUCheck+IMUProtocol[i];
                if(IMUProtocol[18]==IMUCheck) {
                    g_tUart3.usRxRead+=IMULength;
                    g_tUart3.usRxCount-=IMULength;
                    pos=3;
                    imu_data.yaw   = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.pitch  = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.roll     = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.x_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.y_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);
                    imu_data.z_acc = IMUProtocol[pos++] | (IMUProtocol[pos++] << 8);

					imu_data.timestamp = g_imu_time;
                } else {
                    g_tUart3.usRxRead++;
                    g_tUart3.usRxCount--;
                    if(g_tUart3.usRxRead==g_tUart3.usRxBufSize)g_tUart3.usRxRead=0;
                }
            }
        }

        comClearRxFifo(COM3);
    }
 }

}