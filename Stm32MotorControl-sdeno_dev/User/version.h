#ifndef  __VERSION_H
#define  __VERSION_H

typedef struct {
    uint8_t unused;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} version_t;

extern version_t hardware_version;
extern version_t firmware_version;
extern u32 CpuID[3];

extern version_t control_hardware_version;
extern version_t control_firmware_version;
extern u32 control_CpuID[3];

void Get_CpuID(void);
#endif
