#ifndef __USERPROTOCOL_H
#define __USERPROTOCOL_H

#define COMM_RATE          40   // 与上位机通信时间间隔：ms

#define UserProtocolLength 12

//PayloadType
#define BASE_SENSOR_HEAD                 1
#define DORKING_IR_HEAD                  3
#define INERTIAL_SENSOR_HEAD             4
#define CLIFF_HEAD                       5
#define CURRENT_HEAD                     6
#define ULTRASONIC_HEAD                  9
#define HARDWARE_HEAD                    17
#define FIRMWARE_HEAD                    18
#define UDID_HEAD                        19

enum RequestExtra{
    Hardware = 0x01,
	Firmware = 0x02,
	UniqueDeviceID = 0x08
};


#define HEADER_LENTH                0x02
#define CMD_LENTH                   0x02
#define BASE_SENSOR_DATA_LENGTH     0x14+2  // 编码器时间戳增加两个字节
#define INERTIAL_SENSOR_DATA_LENGTH 0x0C
#define DORKING_IR_DATA_LENGTH      0x03
#define ULTRASONIC_DATA_LENGTH      0x08
#define HARDWARE_DATA_LENGTH        0x04
#define FIRMWARE_DATA_LENGTH        0x04
#define UDID_DATA_LENGTH            0x0C

#define CURRENT_DATA_LENGTH         0x08


#define OLD_INERTIAL_SENSOR_DATA_LENGTH 0x08


typedef enum {
    TIMER_STAMP_LOW_LOW=0,
    TIMER_STAMP_LOW_HIGH,
    TIMER_STAMP_HIGH_LOW,
    TIMER_STAMP_HIGH_HIGH,
    BUMPER,
    WHEEL_DROP,
    CLIFF,
    LEFT_ENCODER_LOW_LOW,
    LEFT_ENCODER_LOW_HIGH,
    LEFT_ENCODER_HIGH_LOW,
    LEFT_ENCODER_HIGH_HIGH,
    RIGHT_ENCODER_LOW_LOW,
    RIGHT_ENCODER_LOW_HIGH,
    RIGHT_ENCODER_HIGH_LOW,
    RIGHT_ENCODER_HIGH_HIGH,
    LEFT_PWM,
    RIGHT_PWM,
    BUTTON,
    CHARGER,
    BATTERY_LOW,
    BATTERY_HIGH,
    OVER_CURRENT_FLAGS
} basic_sensor_data_t;


/* 供外部调用的函数声明 */
void UserProtocol(void);
void SendCom( void );

#endif
