#include "bsp.h"

#define BridgeLength 128  // 50
extern UART_T g_tUart2;
extern uint8_t g_TxBuf2[];      /* 发送缓冲区 */
extern uint8_t g_RxBuf2[];      /* 接收缓冲区 */

u8 charger = 0;
u8 BatteryData[2] = {0};
u8 IRData[DORKING_IR_DATA_LENGTH] = {0};
u8 UltrasonicData[ULTRASONIC_DATA_LENGTH] = {0};
u8 CurrentData[CURRENT_DATA_LENGTH] = {0};


#if 0
void bridge( void )
{
    u32 xLastWakeTime;
    const u32 xFrequency = 25;
    u8 BridgeCheck=0;
    u8 offset = 0,i=0;
    u8 BatteryTemp[2] = {0};
    u8 IRTemp[DORKING_IR_DATA_LENGTH] = {0};
    u8 UltrasonicTemp[ULTRASONIC_DATA_LENGTH] = {0};
    u8 BridgeProtocol[BridgeLength];

    /*
        // Initialise the xLastWakeTime variable with the current time.
        xLastWakeTime = xTaskGetTickCount();//获取当前tick
        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    */
    if(g_tUart2.usRxCount >=BridgeLength ) {
        if(g_tUart2.pRxBuf[g_tUart2.usRxRead]==0xAA&&g_tUart2.pRxBuf[g_tUart2.usRxRead+1]==0x55) {

            if (g_tUart2.usRxRead>=g_tUart2.usRxBufSize-BridgeLength) {
                //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
                //comSendBuf(COM1,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
                memcpy(BridgeProtocol,g_tUart2.pRxBuf+g_tUart2.usRxRead,g_tUart2.usRxBufSize-g_tUart2.usRxRead);
                memcpy(BridgeProtocol+g_tUart2.usRxBufSize-g_tUart2.usRxRead,g_tUart2.pRxBuf,g_tUart2.usRxRead+BridgeLength-g_tUart2.usRxBufSize);
                for(i=2; i<BridgeLength-1; i++)BridgeCheck=BridgeCheck^BridgeProtocol[i];
                if(BridgeProtocol[BridgeLength-1]==BridgeCheck) {
                    g_tUart2.usRxRead=g_tUart2.usRxRead+BridgeLength-g_tUart2.usRxBufSize;
                    g_tUart2.usRxCount-=BridgeLength;//end
                    offset = 0;
                    charger = g_tUart2.pRxBuf[CHARGER + HEADER_LENTH + CMD_LENTH + 1];
                    BatteryTemp[0] = g_tUart2.pRxBuf[BATTERY_LOW + HEADER_LENTH + CMD_LENTH + 1];
                    BatteryTemp[1] = g_tUart2.pRxBuf[BATTERY_HIGH + HEADER_LENTH + CMD_LENTH + 1];
                    offset += (HEADER_LENTH + 2 * CMD_LENTH + BASE_SENSOR_DATA_LENGTH + 1);
                    for(int i = 0; i < DORKING_IR_DATA_LENGTH; i++) {
                        IRTemp[i] = g_tUart2.pRxBuf[offset + i];
                    }
                    offset += (DORKING_IR_DATA_LENGTH + 2 * CMD_LENTH + OLD_INERTIAL_SENSOR_DATA_LENGTH-1);
                    for(int i = 0; i < ULTRASONIC_DATA_LENGTH; i++) {
                        UltrasonicTemp[i] = g_tUart2.pRxBuf[offset + i];
                    }
                    memcpy(BatteryData,BatteryTemp,2);
                    memcpy(IRData,IRTemp,DORKING_IR_DATA_LENGTH);
                    memcpy(UltrasonicData,UltrasonicTemp,ULTRASONIC_DATA_LENGTH);
                }

            } else {
                //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,UserProtocolLength);
                memcpy(BridgeProtocol,g_tUart2.pRxBuf+g_tUart2.usRxRead,BridgeLength);
                for(i=2; i<BridgeLength-1; i++)BridgeCheck=BridgeCheck^BridgeProtocol[i];
                if(BridgeProtocol[BridgeLength-1]==BridgeCheck) {
                    g_tUart2.usRxRead+=BridgeLength;
                    g_tUart2.usRxCount-=BridgeLength;//end
                    offset = 0;
                    charger = g_tUart2.pRxBuf[CHARGER + HEADER_LENTH + CMD_LENTH + 1];
                    BatteryTemp[0] = g_tUart2.pRxBuf[BATTERY_LOW + HEADER_LENTH + CMD_LENTH + 1];
                    BatteryTemp[1] = g_tUart2.pRxBuf[BATTERY_HIGH + HEADER_LENTH + CMD_LENTH + 1];
                    offset += (HEADER_LENTH + 2 * CMD_LENTH + BASE_SENSOR_DATA_LENGTH + 1);
                    for(int i = 0; i < DORKING_IR_DATA_LENGTH; i++) {
                        IRTemp[i] = g_tUart2.pRxBuf[offset + i];
                    }
                    offset += (DORKING_IR_DATA_LENGTH + 2 * CMD_LENTH + OLD_INERTIAL_SENSOR_DATA_LENGTH-1);
                    for(int i = 0; i < ULTRASONIC_DATA_LENGTH; i++) {
                        UltrasonicTemp[i] = g_tUart2.pRxBuf[offset + i];
                    }
                    memcpy(BatteryData,BatteryTemp,2);
                    memcpy(IRData,IRTemp,DORKING_IR_DATA_LENGTH);
                    memcpy(UltrasonicData,UltrasonicTemp,ULTRASONIC_DATA_LENGTH);
                    /*
                    u16 battery = 0;
                    u16 ultra_distance = 0;
                    battery = BatteryData[0]+(BatteryData[1]<<8);
                    ultra_distance = UltrasonicData[2]+(UltrasonicData[3]<<8);
                    printf("battery: %d  ultra: %d\r\n",battery,ultra_distance);
                    */
                }
            }
        }
        comClearRxFifo(COM2);
    }
}

#else
void bridge( void )
{
    u32 xLastWakeTime;
    const u32 xFrequency = 25;
    u8 BridgeCheck=0;
    u8 offset = 0,i=0;
    u8 BatteryTemp[2] = {0};
    u8 IRTemp[DORKING_IR_DATA_LENGTH] = {0};
    u8 UltrasonicTemp[ULTRASONIC_DATA_LENGTH] = {0};
    u8 BatteryCurrentTemp[CURRENT_DATA_LENGTH] = {0};
    u8 BridgeProtocol[BridgeLength];
    u8 data_len = 0;
    u8 sub_payload_header = 0;
    u8 sub_payload_length = 0;
    u8 *control_CpuID_p = (u8 *)control_CpuID;

    if(uart2_rx_flag) {  // 接收到一帧数据
        uart2_rx_flag = 0;
        if(g_tUart2.pRxBuf[g_tUart2.usRxRead]==0xAA&&g_tUart2.pRxBuf[g_tUart2.usRxRead+1]==0x55) {
            data_len = g_tUart2.pRxBuf[g_tUart2.usRxRead+2];

            BridgeCheck = 0;
            for(i=2; i<(data_len+3); i++) {
                BridgeCheck ^=  g_tUart2.pRxBuf[g_tUart2.usRxRead+i];
            }
            if(g_tUart2.pRxBuf[g_tUart2.usRxRead+data_len+3]==BridgeCheck) {   // 校验通过
                memcpy(BridgeProtocol,g_tUart2.pRxBuf+g_tUart2.usRxRead+HEADER_LENTH+1,data_len);
                offset = 0;
                while(offset<data_len) {
                    sub_payload_header = BridgeProtocol[offset++];
                    sub_payload_length = BridgeProtocol[offset++];
                    switch(sub_payload_header) {
                        case BASE_SENSOR_HEAD:
                            charger = BridgeProtocol[CHARGER + CMD_LENTH-2];// 兼容以前两字节时间戳协议

                            BatteryTemp[0] = BridgeProtocol[BATTERY_LOW + CMD_LENTH-2];
                            BatteryTemp[1] = BridgeProtocol[BATTERY_HIGH + CMD_LENTH-2];
                            break;
                        case DORKING_IR_HEAD:
                            for(i = 0; i < DORKING_IR_DATA_LENGTH; i++) {
                                IRTemp[i] = BridgeProtocol[offset + i];
                            }
                            break;
                        case INERTIAL_SENSOR_HEAD:
                            break;
                        case CLIFF_HEAD:
                            break;
                        case CURRENT_HEAD:
                            for(i = 0; i < sub_payload_length; i++) {
                                BatteryCurrentTemp[i] = BridgeProtocol[offset + i];
                            }
                            break;
                        case ULTRASONIC_HEAD:
                            for(i = 0; i < ULTRASONIC_DATA_LENGTH; i++) {
                                UltrasonicTemp[i] = BridgeProtocol[offset + i];
                            }
                            break;
                        case HARDWARE_HEAD:
                            control_hardware_version.patch = BridgeProtocol[offset + 0];
                            control_hardware_version.minor = BridgeProtocol[offset + 1];
                            control_hardware_version.major = BridgeProtocol[offset + 2];
                            control_hardware_version.unused = BridgeProtocol[offset + 3];
                            break;
                        case FIRMWARE_HEAD:
                            control_firmware_version.patch = BridgeProtocol[offset + 0];
                            control_firmware_version.minor = BridgeProtocol[offset + 1];
                            control_firmware_version.major = BridgeProtocol[offset + 2];
                            control_firmware_version.unused = BridgeProtocol[offset + 3];
                            break;
                        case UDID_HEAD:
                            for(i = 0; i < UDID_DATA_LENGTH; i++) {
                                *control_CpuID_p++ = BridgeProtocol[offset + i];
                            }
                            break;
                        default:
                            break;
                    }
                    offset += sub_payload_length;
                }

                memcpy(BatteryData,BatteryTemp,2);
                memcpy(IRData,IRTemp,DORKING_IR_DATA_LENGTH);
                memcpy(UltrasonicData,UltrasonicTemp,ULTRASONIC_DATA_LENGTH);
                memcpy(&CurrentData[4],BatteryCurrentTemp,4);
                /*
                u16 battery = 0;
                u16 ultra_distance = 0;
                battery = BatteryData[0]+(BatteryData[1]<<8);
                ultra_distance = UltrasonicData[2]+(UltrasonicData[3]<<8);
                printf("battery: %d  ultra: %d\r\n",battery,ultra_distance);
                */
            }
        }
        comClearRxFifo(COM2);
    }
}

#endif
