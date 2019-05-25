#ifndef __BRIDGE_H
#define __BRIDGE_H


extern u8 BatteryData[2];
extern u8 UltrasonicData[];
extern u8 CurrentData[];



/* 供外部调用的函数声明 */
void bridge( void );

#endif
