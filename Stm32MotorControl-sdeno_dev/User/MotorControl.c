#include "bsp.h"

// 返回电机参数的地址偏移
typedef enum {
    ERROR_STATUS_CMD = 0,
    ERROR_STATUS_NULL,
    ERROR_STATUS_DAT,
    ERROR_STATUS_CS,
    VOLTAGE_CMD,
    VOLTAGE_HIGH,
    VOLTAGE_LOW,
    VOLTAGE_CS,
    CURRENT_CMD,
    CURRENT_HIGH,
    CURRENT_LOW,
    CURRENT_CS,
    SPEED_CMD,
    SPEED_HIGH,
    SPEED_LOW,
    SPEED_CS,
    POS_SET_MSB_CMD,
    POS_SET_MSB_HIGH,
    POS_SET_MSB_LOW,
    POS_SET_MSB_CS,
    POS_SET_LSB_CMD,
    POS_SET_LSB_HIGH,
    POS_SET_LSB_LOW,
    POS_SET_LSB_CS,
    POS_GET_MSB_CMD,
    POS_GET_MSB_HIGH,
    POS_GET_MSB_LOW,
    POS_GET_MSB_CS,
    POS_GET_LSB_CMD,
    POS_GET_LSB_HIGH,
    POS_GET_LSB_LOW,
    POS_GET_LSB_CS
} motor_para_t;

// 电机参数
typedef struct {
    uint8_t error_status;
    uint16_t voltage;
    uint16_t current;
    uint16_t speed;
    int32_t pos_set;
    int32_t pos_get
} motor_para_data_t;

motor_para_data_t motor_para_l = {0},motor_para_r = {0};

#define  POS_Kp        2000    // 位置PID  --数值越大刹车距离越短
#define  POS_Ki        0
#define  POS_Kd        1

#define  SPEED_Kp      4000   // 速度PID --数值大了容易引起共振，声音大
#define  SPEED_Ki      0      // 默认300
#define  SPEED_Kd      0

#define  POS_PID_CMD    0x1a
#define  SPEED_PID_CMD  0x40

// 电机PID参数
typedef struct {
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;
} motor_pid_t;

motor_pid_t pos_pid,speed_pid;


#define MOTOR_CURRENT_MAX    1500   // 10mA:电机过流限值

typedef enum {
    OVERCURRENT_NULL    = 0,
    OVERCURRENT_LEFT    = 0x01,
    OVERCURRENT_RIGHT   = 0x02,
} overcurrent_mask_bit_t;

volatile uint8_t motor_overcurrent_status = 0;



/**
  * @brief  check_sum
  * @param  *buf
  * @param  len
  * @retval cs
  */
uint8_t check_sum(uint8_t *buf,uint8_t len)
{
    uint8_t cs = 0;
    for(uint8_t i=0; i<len; i++) {
        cs += buf[i];
    }
    return cs;
}


#define  PULSES_CIRCLE    4096   // 轮子转一圈的脉冲数


int32_t encode_l = 0, encode_r = 0;
int16_t speedL=0;
int16_t speedR=0;

#define EncodeLength_ML 32
#define EncodeLength_MR 32

const float Perimeter = 534.0;   // 轮子半径R=85mm
const int ReductionRation = 1;   // 减速比
const int Seconds = 60*10;   // 确保精度rpm*10
const float ratio = Seconds * ReductionRation / Perimeter;
const int wheelHaseHalf = 171;

int16_t set_l = 0,set_r = 0;
motorDir_t left_motor_dir = 0,right_motor_dir = 0;

int16_t CommuOverTime = 0;
int8_t MotorInitFlag = 0;
volatile uint32_t g_encoder_time = 0;

extern UART_T g_tUart1;
extern uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];     /* 接收缓冲区 */
extern UART_T g_tUart4;
extern UART_T g_tUart5;

u8 pulse_mode[4] = {0x02, 0x00, 0xc0, 0xc2};/////外部脉冲模式
u8 current_set[4] = {0x2d, 0x13, 0x88, 0xc8}; // 5000mA

u8 ClearFault[4]= {0x4a,0x00,0x00,0x4a};
u8 speed_mode[4] = {0x02, 0x00, 0xc4, 0xc6};/////速度模式
u8 AD_speed_time[4] = {0x0A, 0x14, 0x14, 0x32};////加减速时间
u8 AbsoluteBack[4]= {0x52,0x00,0x01,0x53};
u8 speed1[4] = {0x06, 0x00, 0x88, 0x8E};//////////////50转
u8 speed2[4] = {0x06, 0x01, 0x11, 0x18};//////////////100转
u8 enable_motor[4] = {0x00, 0x00, 0x01, 0x01};//////////////使能电机
u8 back[4] = {0x80, 0x00, 0x80};//////////////防止锁轴


void MotorReInit(void)
{
    comSendBuf(LEFT_MOTOR_COM,ClearFault,4);
    comSendBuf(RIGHT_MOTOR_COM,ClearFault,4);
    vTaskDelay(1);
    //comSendBuf(LEFT_MOTOR_COM,speed_mode,4);
    //comSendBuf(RIGHT_MOTOR_COM,speed_mode,4);
    comSendBuf(LEFT_MOTOR_COM,pulse_mode,4);
    comSendBuf(RIGHT_MOTOR_COM,pulse_mode,4);
    vTaskDelay(1);
    comSendBuf(LEFT_MOTOR_COM,AD_speed_time,4);
    comSendBuf(RIGHT_MOTOR_COM,AD_speed_time,4);
    vTaskDelay(1);
    comSendBuf(LEFT_MOTOR_COM,enable_motor,4);
    comSendBuf(RIGHT_MOTOR_COM,enable_motor,4);
    vTaskDelay(1);
}

void MotorInit(void)
{
    //comSendBuf(LEFT_MOTOR_COM,speed_mode,4);
    //comSendBuf(RIGHT_MOTOR_COM,speed_mode,4);
    comSendBuf(LEFT_MOTOR_COM,pulse_mode,4);
    comSendBuf(RIGHT_MOTOR_COM,pulse_mode,4);
    vTaskDelay(100);
    comSendBuf(LEFT_MOTOR_COM,AD_speed_time,4);
    comSendBuf(RIGHT_MOTOR_COM,AD_speed_time,4);
    vTaskDelay(100);
    comSendBuf(LEFT_MOTOR_COM,AbsoluteBack,4);
    comSendBuf(RIGHT_MOTOR_COM,AbsoluteBack,4);
    vTaskDelay(100);

    pid_init();  // 初始化PID参数
    set_pid();   // 设置PID参数
}

/********************************************************************************
 * @fn           MotorControl
 * @brief        解析命令控制底盘运动
 *
 * @param[in]    None
 * @return       None
********************************************************************************/
#if 0
void MotorControl(void)
{
    int16_t velocityX = 0,velocityY = 0, omega = 0;
    u8 code,check=0,isCorrection=0;
    int i;
    float targetYaw;

    u8 UserProtocol[UserProtocolLength];

    if(MotorInitFlag==0) {
        MotorInitFlag = 1;
        vTaskDelay(3000);
        MotorInit();
    }

    if(g_tUart1.usRxCount >= UserProtocolLength) {
        if (g_tUart1.usRxRead>= g_tUart1.usRxBufSize-UserProtocolLength) {
            memcpy(UserProtocol,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
            memcpy(UserProtocol+g_tUart1.usRxBufSize-g_tUart1.usRxRead,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
            g_tUart1.usRxRead=g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize;
        } else { //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,UserProtocolLength);
            memcpy(UserProtocol,g_tUart1.pRxBuf+g_tUart1.usRxRead,UserProtocolLength);
            g_tUart1.usRxRead+=UserProtocolLength;
        }
        g_tUart1.usRxCount-=UserProtocolLength;

        if(UserProtocol[0] == 0xAA && UserProtocol[1] == 0x55 &&UserProtocol[2] == 0x08) {
            code = UserProtocol[11];
            for(i = 0 ; i < 9 ; i++) {
                check = check ^ UserProtocol[i + 2];
            }
            if(check == code) {
                switch(UserProtocol[3]) {
                    case 0X01:
                        velocityX = UserProtocol[5] | (UserProtocol[6] << 8);
                        velocityY = UserProtocol[7] | (UserProtocol[8] << 8);
                        omega = UserProtocol[9] | (UserProtocol[10] << 8);
                        engine_driven(velocityX,omega);   // 驱动底盘电机运动
                        CommuOverTime = 50;  // 串口无通信时，1s超时后停止电机运动
                        break;
                }
            }
        }
        comClearRxFifo(COM1);
    }

    if(CommuOverTime>0) { // 串口无通信时，1s超时后停止电机运动
        if(--CommuOverTime==0) {
            engine_driven(0,0);
        }
    }

}
#else

uint8_t request_flag = 0;

void MotorControl(void)
{
    int16_t velocityX = 0,velocityY = 0, omega = 0;
    u8 code,check=0,isCorrection=0,len=0;
    int i;
    float targetYaw;

    u8 UserProtocol[64];

    if(MotorInitFlag==0) {
        MotorInitFlag = 1;
        vTaskDelay(3000);
        MotorInit();
    }

    if(uart1_rx_flag) { // 接收到一帧数据
        uart1_rx_flag = 0;
        while(g_tUart1.usRxRead != g_tUart1.usRxWrite) {
            if((g_RxBuf1[g_tUart1.usRxRead]== 0xAA)&&(g_RxBuf1[g_tUart1.usRxRead+1]== 0x55)) {
                len = g_RxBuf1[g_tUart1.usRxRead+2];
                memcpy(UserProtocol,g_tUart1.pRxBuf+g_tUart1.usRxRead,len+4);
                code = UserProtocol[len+3];
                check = 0;
                for(i = 0 ; i <= len ; i++) {
                    check = check ^ UserProtocol[i + 2];
                }
                if(check == code) {
                    switch(UserProtocol[3]) {
                        case 0x01:  // Basic core sensor data
                            velocityX = UserProtocol[5] | (UserProtocol[6] << 8);
                            velocityY = UserProtocol[7] | (UserProtocol[8] << 8);
                            omega = UserProtocol[9] | (UserProtocol[10] << 8);
                            engine_driven(velocityX,omega);   // 驱动底盘电机运动
                            CommuOverTime = 50;  // 串口无通信时，1s超时后停止电机运动
                            break;
                        case 0x0E:  // Request PID gain

                            break;
						case 0x09:  // Request extra data: 01-harware,02-firmware,08-udid
                            request_flag = UserProtocol[4];
                            break;
                    }
                }
            }
            if(++g_tUart1.usRxRead >= g_tUart1.usRxBufSize) {
                g_tUart1.usRxRead = 0;
            }
        }
        comClearRxFifo(COM1);
    }

    if(CommuOverTime>0) { // 串口无通信时，1s超时后停止电机运动
        if(--CommuOverTime==0) {
            engine_driven(0,0);
        }
    }

}

#endif

// 解析返回数据获取编码器计数step--2
void MotorEncode(void)
{
    u8 MotorLProtocol[EncodeLength_ML];
    u8 MotorRProtocol[EncodeLength_MR];

    if(g_tUart4.usRxCount >= EncodeLength_ML) {
//        //printf("%d\r\n",g_tUart4.usRxCount);
//        if (g_tUart4.usRxRead>=g_tUart4.usRxBufSize-EncodeLength_ML) {
//            //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
//            //comSendBuf(COM1,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
//            memcpy(MotorLProtocol,g_tUart4.pRxBuf+g_tUart4.usRxRead,g_tUart4.usRxBufSize-g_tUart4.usRxRead);
//            memcpy(MotorLProtocol+g_tUart4.usRxBufSize-g_tUart4.usRxRead,g_tUart4.pRxBuf,g_tUart4.usRxRead+EncodeLength_ML-g_tUart4.usRxBufSize);
//            g_tUart4.usRxRead=g_tUart4.usRxRead+EncodeLength_ML-g_tUart4.usRxBufSize;
//        } else {

        memcpy(MotorLProtocol,g_tUart4.pRxBuf+g_tUart4.usRxRead,EncodeLength_ML);
        g_tUart4.usRxRead+=EncodeLength_ML;
        //printf("RxCount: %d\r\n",g_tUart4.usRxCount);
//        }
        g_tUart4.usRxCount-=EncodeLength_ML;
        comClearRxFifo(COM4);

        if((MotorLProtocol[POS_GET_MSB_CMD]==0xe8)&&(MotorLProtocol[POS_GET_LSB_CMD]==0xe9)) { // Encoder count
            if((MotorLProtocol[POS_GET_MSB_CS]==check_sum(&MotorLProtocol[POS_GET_MSB_CMD],3))&&(MotorLProtocol[POS_GET_LSB_CS]==check_sum(&MotorLProtocol[POS_GET_LSB_CMD],3))) {
                motor_para_l.pos_get = (MotorLProtocol[POS_GET_MSB_HIGH]<<24) + (MotorLProtocol[POS_GET_MSB_LOW]<<16) + (MotorLProtocol[POS_GET_LSB_HIGH]<<8) + MotorLProtocol[POS_GET_LSB_LOW];
                encode_l = motor_para_l.pos_get;			    
            }
        }

        if(MotorLProtocol[SPEED_CMD]==0xe4) {  // Speed:RPM
            if(MotorLProtocol[SPEED_CS]==check_sum(&MotorLProtocol[SPEED_CMD],3)) {
                motor_para_l.speed = (MotorLProtocol[SPEED_HIGH]<<8) + MotorLProtocol[SPEED_LOW];
                speedL = motor_para_l.speed;
            }
        }

        if(MotorLProtocol[CURRENT_CMD]==0xe2) {  // Current:10mA
            if(MotorLProtocol[CURRENT_CS]==check_sum(&MotorLProtocol[CURRENT_CMD],3)) {
                motor_para_l.current = (MotorLProtocol[CURRENT_HIGH]<<8) + MotorLProtocol[CURRENT_LOW];
                if(motor_para_l.current>MOTOR_CURRENT_MAX) {
                    motor_overcurrent_status |= OVERCURRENT_LEFT;
                } else {
                    motor_overcurrent_status &= ~OVERCURRENT_LEFT;
                }
            }
        }

        if(MotorLProtocol[ERROR_STATUS_CMD]==0x80) {  // error status
            if(MotorLProtocol[ERROR_STATUS_CS]==check_sum(&MotorLProtocol[ERROR_STATUS_CMD],3)) {
                motor_para_l.error_status = MotorLProtocol[ERROR_STATUS_DAT];
            }
        }

        speedL = motor_para_l.current;

    }
    if(g_tUart5.usRxCount >= EncodeLength_MR) {
//    if(g_tUart5.usRxCount >=EncodeLength_MR&&MotorMoveFlagR==1&&MotorStep<=1) {
//        if (g_tUart5.usRxRead>= g_tUart5.usRxBufSize-EncodeLength_MR) {
//            //comSendBuf(COM1,g_tUart1.pRxBuf+g_tUart1.usRxRead,g_tUart1.usRxBufSize-g_tUart1.usRxRead);
//            //comSendBuf(COM1,g_tUart1.pRxBuf,g_tUart1.usRxRead+UserProtocolLength-g_tUart1.usRxBufSize);
//            memcpy(MotorRProtocol,g_tUart5.pRxBuf+g_tUart5.usRxRead,g_tUart5.usRxBufSize-g_tUart5.usRxRead);
//            memcpy(MotorRProtocol+g_tUart5.usRxBufSize-g_tUart5.usRxRead,g_tUart5.pRxBuf,g_tUart5.usRxRead+EncodeLength_MR-g_tUart5.usRxBufSize);
//            g_tUart5.usRxRead=g_tUart5.usRxRead+EncodeLength_MR-g_tUart5.usRxBufSize;
//        } else {
        memcpy(MotorRProtocol,g_tUart5.pRxBuf+g_tUart5.usRxRead,EncodeLength_MR);
        g_tUart5.usRxRead+=EncodeLength_MR;
//        }
        g_tUart5.usRxCount-=EncodeLength_MR;
        comClearRxFifo(COM5);

        if((MotorRProtocol[POS_GET_MSB_CMD]==0xe8)&&(MotorRProtocol[POS_GET_LSB_CMD]==0xe9)) { // Encoder count
            if((MotorRProtocol[POS_GET_MSB_CS]==check_sum(&MotorRProtocol[POS_GET_MSB_CMD],3))&&(MotorRProtocol[POS_GET_LSB_CS]==check_sum(&MotorRProtocol[POS_GET_LSB_CMD],3))) {
                motor_para_r.pos_get = (MotorRProtocol[POS_GET_MSB_HIGH]<<24) + (MotorRProtocol[POS_GET_MSB_LOW]<<16) + (MotorRProtocol[POS_GET_LSB_HIGH]<<8) + MotorRProtocol[POS_GET_LSB_LOW];
                encode_r = motor_para_r.pos_get;
            }
        }

        if(MotorRProtocol[SPEED_CMD]==0xe4) {  // Speed:RPM
            if(MotorRProtocol[SPEED_CS]==check_sum(&MotorRProtocol[SPEED_CMD],3)) {
                motor_para_r.speed = (MotorRProtocol[SPEED_HIGH]<<8) + MotorRProtocol[SPEED_LOW];
                speedR = motor_para_r.speed;
            }
        }

        if(MotorRProtocol[CURRENT_CMD]==0xe2) {  // Current:10mA
            if(MotorRProtocol[CURRENT_CS]==check_sum(&MotorRProtocol[CURRENT_CMD],3)) {
                motor_para_r.current = (MotorRProtocol[CURRENT_HIGH]<<8) + MotorRProtocol[CURRENT_LOW];
                if(motor_para_r.current>MOTOR_CURRENT_MAX) {
                    motor_overcurrent_status |= OVERCURRENT_RIGHT;
                } else {
                    motor_overcurrent_status &= ~OVERCURRENT_RIGHT;
                }
            }
        }

        if(MotorRProtocol[ERROR_STATUS_CMD]==0x80) {  // error status
            if(MotorRProtocol[ERROR_STATUS_CS]==check_sum(&MotorRProtocol[ERROR_STATUS_CMD],3)) {
                motor_para_r.error_status = MotorRProtocol[ERROR_STATUS_DAT];
            }
        }
        speedR = motor_para_r.current;

    }
}

void MotorMove(int motor,int speed)
{
//    u8 speedtemp[4]= {0x06, 0x00, 0x88, 0x8E};
//    u8 back[3]= {0x80,0x00,0x80};
//    u8 EnableLock[4]= {0x00,0x00,0x01,0x01};
//    speedtemp[0] = 06;
//    speedtemp[1] = (speed>>8)&0xFF;
//    speedtemp[2] = speed&0xFF;
//    speedtemp[3] = (speedtemp[0]+speedtemp[1]+speedtemp[2])&0xFF;
//    comSendBuf(motor,speedtemp,4);
//    vTaskDelay(1);
//    comSendBuf(motor,EnableLock,4);
//    vTaskDelay(1);
    comSendBuf(motor,back,3);
    vTaskDelay(1);
}

// 限制最大速度
int16_t speedConstraint(int16_t speed)
{
    int16_t target = 0;
    if(speed == 0) target = 0;
    else if(abs(speed) < MIN_ROTARY_SPEED) target = speed > 0 ? MIN_ROTARY_SPEED : -MIN_ROTARY_SPEED;
    else if(abs(speed) > MAX_ROTARY_SPEED) target = speed > 0 ? MAX_ROTARY_SPEED : -MAX_ROTARY_SPEED;
    else target = speed;
    return target;
}

/********************************************************************************
 * @fn           engine_driven
 * @brief        驱动底盘运动
 *
 * @param[in]    speed: mm/s
 * @param[in]    omega: mrad/s
 * @return       None
********************************************************************************/
void engine_driven(int16_t speed,int16_t omega)
{
    float radius,radius_f;

    if(omega == 0 && speed == 0) {                   // 停止
        set_l = 0;
        set_r = 0;
    } else if(omega == 0 && speed != 0) {            // 直行
        set_l= speed * ratio;
        set_r= speed * ratio;
    } else if(omega != 0 && speed == 0) {            // 原地旋转
        set_r = omega * 1.0 / 1000 * wheelHaseHalf * ratio;
        set_l = - omega * 1.0 / 1000 * wheelHaseHalf * ratio;
    } else {
        radius_f = (speed * 1.0) / (omega * 1.0);
        radius = 1000 * radius_f;
        if(omega > 0) {                              // 左拐弯
            set_r = (int)(speed * ratio * (radius + wheelHaseHalf) / radius);
            set_l = (int)(speed * ratio * (radius - wheelHaseHalf) / radius);
        } else {                                     // 右拐弯
            set_l = (int)(speed * ratio * (radius - wheelHaseHalf) / radius);
            set_r = (int)(speed * ratio * (radius + wheelHaseHalf) / radius);
        }
    }

    left_motor_dir  = set_l >= 0 ? FORWARD : BACKWARD;
    right_motor_dir = set_r >= 0 ? BACKWARD: FORWARD;
    if((set_l==0)&&(set_r==0)) {
        if(devicePrm.motionState[0] != INACTIVE) {
            MotorControl_Stop(0);
            MotorControl_SetMaxSpeed(0,0);
        }
        if(devicePrm.motionState[1] != INACTIVE) {
            MotorControl_Stop(1);
            MotorControl_SetMaxSpeed(1,0);
        }
    } else {
        MotorControl_Run(0,left_motor_dir,speedConstraint(abs(set_l)));
        MotorControl_Run(1,right_motor_dir,speedConstraint(abs(set_r)));
    }

}
// 发送命令获取编码器计数step--1
void get_encoder(void)
{
    comSendBuf(LEFT_MOTOR_COM,back,3);  // 需要32ms才能返回完整的32bytes数据
    comSendBuf(RIGHT_MOTOR_COM,back,3);
    //vTaskDelay(40);
    //MotorEncode();
}

// 速度转化为相应的频率
uint16_t speed_frequency(uint16_t speed)
{
    uint16_t frequency = 0;
    frequency = speed*PULSES_CIRCLE/Seconds;  // convert rpm into Hz
    return frequency;
}

// PID参数初始化
void pid_init(void)
{
    pos_pid.kp = POS_Kp;
    pos_pid.ki = POS_Ki;
    pos_pid.kd = POS_Kd;

    speed_pid.kp = SPEED_Kp;
    speed_pid.ki = SPEED_Ki;
    speed_pid.kd = SPEED_Kd;
}

// 设置PID
void set_pid(void)
{
    uint8_t temp[4];
    temp[0] = POS_PID_CMD;
    temp[1] = (pos_pid.kp&0xFF00)>>8;
    temp[2] = (pos_pid.kp&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(100);

    temp[0] = POS_PID_CMD+1;
    temp[1] = (pos_pid.ki&0xFF00)>>8;
    temp[2] = (pos_pid.ki&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(100);

    temp[0] = POS_PID_CMD+2;
    temp[1] = (pos_pid.kd&0xFF00)>>8;
    temp[2] = (pos_pid.kd&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(100);

    temp[0] = SPEED_PID_CMD;
    temp[1] = (speed_pid.kp&0xFF00)>>8;
    temp[2] = (speed_pid.kp&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(100);

    temp[0] = SPEED_PID_CMD+1;
    temp[1] = (speed_pid.ki&0xFF00)>>8;
    temp[2] = (speed_pid.ki&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(100);

    temp[0] = SPEED_PID_CMD+2;
    temp[1] = (speed_pid.kd&0xFF00)>>8;
    temp[2] = (speed_pid.kd&0x00FF);
    temp[3] = check_sum(temp,3);
    comSendBuf(LEFT_MOTOR_COM,temp,4);
    comSendBuf(RIGHT_MOTOR_COM,temp,4);
    vTaskDelay(200);
}

// 读取PID
void get_pid(void)
{

}

/********************************************************************************
 * @fn           get_overcurrent_status
 * @brief        获取电机过流状态
 *
 * @param[in]    None
 * @return       None
********************************************************************************/
uint8_t get_overcurrent_status(void)
{
    return motor_overcurrent_status;
}


/********************************************************************************
 * @fn           get_motor_current
 * @brief        获取左、右电机相电流
 *
 * @param[in]    None
 * @return       None
********************************************************************************/
void get_motor_current(uint8_t *current)
{
	uint8_t i = 0;
    for(i = 0;i<2;i++){
		*current++ = *((uint8_t *)&motor_para_l.current+i);
	}
	for(i = 0;i<2;i++){
		*current++ = *((uint8_t *)&motor_para_r.current+i);
	}
}

