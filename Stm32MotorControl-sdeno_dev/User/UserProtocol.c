#include "bsp.h"

#pragma anon_unions

typedef struct {
    u8  head;
    u8  len;
    u8 data[21];   //uint8 * data :    error
} sub_payload_t;//unused

typedef struct {
    u8 len;
    u8 sub_head;
    u8 sub_len;
    u8 data[94];
} checksum_t;

typedef struct {
    u8  head[2];
    union {
        checksum_t checksum;
        u8 data[97];
    };
} send_data_t;

typedef struct {
    u8 len;
    union {
        send_data_t send_data;
        u8 data[99];
    };
} send_cmd_data_t;

send_cmd_data_t send_cmd_data;
extern u8 charger ;
extern u8 BatteryData[] ;
extern u8 IRData[] ;
extern u8 UltrasonicData[] ;
extern int32_t encode_l, encode_r ;

extern UART_T g_tUart1;
extern int16_t set_l,set_r ;
extern int16_t speedL,speedR;
void UserProtocol(void)
{
    /*  if(g_tUart1.usRxCount >=UserProtocolLength)
        {
            if (g_tUart1.usRxRead>= g_tUart1.usRxBufSize-UserProtocolLength)
            {
                comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
                comSendBuf(COM1,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
                g_tUart1.usRxRead=g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize;
            }
            else {comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,UserProtocolLength);
            g_tUart1.usRxRead+=UserProtocolLength;}
            g_tUart1.usRxCount-=UserProtocolLength;
        }*/
    //printf("%d,%d,%d,%d,%d,%d\r\n",imu_data.yaw,imu_data.pitch,imu_data.roll,imu_data.x_acc,imu_data.y_acc,imu_data.z_acc);
    //printf("%d,%d,%d,%d,%d,%d,%d,%d\r\n",pulse_cnt_l,pulse_cnt_r,encode_l,encode_r,set_l,set_r,speedL,speedR);
	//printf("%d,%d,%d\r\n",g_encoder_time,imu_data.timestamp,imu_data.timestamp-g_encoder_time);
}

//extern float  Pitch,Roll,Yaw;
void SendCom( void )
{

    u8 cur_len = 0;
    static u32 time = 0;
    u8 BatteryTemp[2] = {0};
    u8 IRTemp[DORKING_IR_DATA_LENGTH] = {0};
    u8 UltrasonicTemp[ULTRASONIC_DATA_LENGTH] = {0};
	u8 CurrentTemp[CURRENT_DATA_LENGTH] = {0};
    send_cmd_data.send_data.head[0] = 0xaa;
    send_cmd_data.send_data.head[1] = 0x55;
    send_cmd_data.send_data.checksum.sub_head = 0x01;
    send_cmd_data.send_data.checksum.sub_len = BASE_SENSOR_DATA_LENGTH; //base len

	
    memcpy(BatteryTemp,BatteryData,2);
    memcpy(IRTemp,IRData,DORKING_IR_DATA_LENGTH);
    memcpy(UltrasonicTemp,UltrasonicData,ULTRASONIC_DATA_LENGTH);
	get_motor_current(CurrentData);  // 获取电机电流
	memcpy(CurrentTemp,CurrentData,CURRENT_DATA_LENGTH);
    cur_len = send_cmd_data.send_data.checksum.sub_len + 5;

    time = g_encoder_time;
	send_cmd_data.send_data.checksum.data[TIMER_STAMP_LOW_LOW]   = (time >> 0) & 0x00ff;  // 编码器时间戳
    send_cmd_data.send_data.checksum.data[TIMER_STAMP_LOW_HIGH]  = (time >> 8) & 0x00ff;
	send_cmd_data.send_data.checksum.data[TIMER_STAMP_HIGH_LOW]  = (time >> 16) & 0x00ff;
    send_cmd_data.send_data.checksum.data[TIMER_STAMP_HIGH_HIGH] = (time >> 24) & 0x00ff;
    send_cmd_data.send_data.checksum.data[BUMPER] = (u8)0x00;          //  碰撞传感：bit0--right，bit1--central，bit2--left
    send_cmd_data.send_data.checksum.data[WHEEL_DROP] = (u8)0x00;  //  轮子跌落：bit0--right，bit1--left
    send_cmd_data.send_data.checksum.data[BUTTON] = (u8)0x00;             //  按键状态：bit0--key1， bit1--key2
    send_cmd_data.send_data.checksum.data[CLIFF] = (u8)0x00;             //  红外跌落：bit0--left_front，bit1--right_front，bit2--left_back，bit4--right_back
    send_cmd_data.send_data.checksum.data[LEFT_PWM] = 0x00;                          //  左电机PWM
    send_cmd_data.send_data.checksum.data[RIGHT_PWM] = 0x00;                         //  右电机PWM

    send_cmd_data.send_data.checksum.data[LEFT_ENCODER_LOW_LOW] = (u8)((encode_l & 0x000000FF) >> 0);
    send_cmd_data.send_data.checksum.data[LEFT_ENCODER_LOW_HIGH] =(u8)((encode_l & 0x0000FF00) >> 8);
    send_cmd_data.send_data.checksum.data[LEFT_ENCODER_HIGH_LOW] =(u8)((encode_l & 0x00FF0000) >> 16);
    send_cmd_data.send_data.checksum.data[LEFT_ENCODER_HIGH_HIGH] = (u8)((encode_l & 0xFF000000) >> 24);
    send_cmd_data.send_data.checksum.data[RIGHT_ENCODER_LOW_LOW] = (u8)((encode_r & 0x000000FF) >> 0);
    send_cmd_data.send_data.checksum.data[RIGHT_ENCODER_LOW_HIGH] = (u8)((encode_r & 0x0000FF00) >> 8);
    send_cmd_data.send_data.checksum.data[RIGHT_ENCODER_HIGH_LOW] = (u8)((encode_r & 0x00FF0000) >> 16);
    send_cmd_data.send_data.checksum.data[RIGHT_ENCODER_HIGH_HIGH] = (u8)((encode_r & 0xFF000000) >> 24);
    send_cmd_data.send_data.checksum.data[CHARGER] = charger;
    send_cmd_data.send_data.checksum.data[BATTERY_LOW] = BatteryTemp[0];
    send_cmd_data.send_data.checksum.data[BATTERY_HIGH] = BatteryTemp[1];
    send_cmd_data.send_data.checksum.data[OVER_CURRENT_FLAGS] = get_overcurrent_status();

    send_cmd_data.data[cur_len++] = 0x03;
    send_cmd_data.data[cur_len++] = DORKING_IR_DATA_LENGTH;
    for(int i = 0; i < DORKING_IR_DATA_LENGTH; i++) {
        send_cmd_data.data[cur_len++] = IRTemp[i];
    }

    send_cmd_data.data[cur_len++] = 0x04;
    send_cmd_data.data[cur_len++] = INERTIAL_SENSOR_DATA_LENGTH+4;
    send_cmd_data.data[cur_len++] = imu_data.yaw & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.yaw>>8) & 0xFF;
    send_cmd_data.data[cur_len++] = imu_data.pitch & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.pitch>>8) & 0xFF;
    send_cmd_data.data[cur_len++] = imu_data.roll & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.roll>>8) & 0xFF;
    send_cmd_data.data[cur_len++] = imu_data.x_acc & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.x_acc>>8) & 0xFF;
    send_cmd_data.data[cur_len++] = imu_data.y_acc & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.y_acc>>8) & 0xFF;
    send_cmd_data.data[cur_len++] = imu_data.z_acc & 0xFF;
    send_cmd_data.data[cur_len++] = (imu_data.z_acc>>8) & 0xFF;

	send_cmd_data.data[cur_len++] = (imu_data.timestamp >> 0) & 0xFF;  // IMU时间戳
    send_cmd_data.data[cur_len++] = (imu_data.timestamp >> 8) & 0xFF;
	send_cmd_data.data[cur_len++] = (imu_data.timestamp >> 16) & 0xFF;
	send_cmd_data.data[cur_len++] = (imu_data.timestamp >> 24) & 0xFF;


    send_cmd_data.data[cur_len++] = 0x09;
    send_cmd_data.data[cur_len++] = ULTRASONIC_DATA_LENGTH;
    for(int i = 0; i < ULTRASONIC_DATA_LENGTH; i++) {
        send_cmd_data.data[cur_len++] = UltrasonicTemp[i];
    }

    if(request_flag & Hardware) {  // 只有请求时才发送
        send_cmd_data.data[cur_len++] = 0x11;    // Hardware version
        send_cmd_data.data[cur_len++] = HARDWARE_DATA_LENGTH;

        send_cmd_data.data[cur_len++] = hardware_version.patch;
        send_cmd_data.data[cur_len++] = hardware_version.minor;
        send_cmd_data.data[cur_len++] = hardware_version.major;
        send_cmd_data.data[cur_len++] = hardware_version.unused;
    }

    if(request_flag & Firmware) {
        send_cmd_data.data[cur_len++] = 0x12;    // Firmware version
        send_cmd_data.data[cur_len++] = FIRMWARE_DATA_LENGTH;

        send_cmd_data.data[cur_len++] = firmware_version.patch;
        send_cmd_data.data[cur_len++] = firmware_version.minor;
        send_cmd_data.data[cur_len++] = firmware_version.major;
        send_cmd_data.data[cur_len++] = firmware_version.unused;		
    }

    if(request_flag & UniqueDeviceID) {
        send_cmd_data.data[cur_len++] = 0x13;    // UDID
        send_cmd_data.data[cur_len++] = UDID_DATA_LENGTH;
        for(int i = 0; i < UDID_DATA_LENGTH; i++) {
            send_cmd_data.data[cur_len++] = *((u8 *)CpuID+i);
        }
    }

    send_cmd_data.len = cur_len;
    send_cmd_data.send_data.checksum.len = send_cmd_data.len - 3;
    send_cmd_data.data[send_cmd_data.len] = 0;  // 计算校验前清零
    for (unsigned int i = 2; i < send_cmd_data.len; i++) {
        send_cmd_data.data[send_cmd_data.len] ^= send_cmd_data.data[i];
    }

    for (unsigned int i = 0; i <= send_cmd_data.len; i++) {
        printf("%c", send_cmd_data.data[i]);
    }

    //time = time + COMM_RATE;  // 更新时间戳

}

/**/
