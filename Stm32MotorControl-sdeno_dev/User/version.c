#include<bsp.h>

// 电机控制板
version_t hardware_version= {0,1,5,0}; // 硬件版本
version_t firmware_version= {0,1,2,1}; // 固件版本
u32 CpuID[3];

// 总控制板
version_t control_hardware_version= {0,0,0,0}; // 硬件版本V1.0.0
version_t control_firmware_version= {0,0,0,0}; // 固件版本V1.0.0
u32 control_CpuID[3];                          // UDID


/********************************************************************************
 * @fn           Get_CpuID
 * @brief        获取CpuID作为UDID
 *
 * @param[in]    None
 * @return       None
********************************************************************************/
void Get_CpuID(void)
{
    //获取CPU唯一ID
    CpuID[0]=*(vu32*)(0x1ffff7e8);
    CpuID[1]=*(vu32*)(0x1ffff7ec);
    CpuID[2]=*(vu32*)(0x1ffff7f0);
}

