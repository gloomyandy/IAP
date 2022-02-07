#ifndef IAPPARAMS_H
#define IAPPARAMS_H
#include <Core.h>
constexpr uint32_t IAPInvalidSig = 0xBAD0BAD0;
constexpr uint32_t SBCIAPParamSig = 0xA1B20001;
typedef struct{
    uint32_t sig1;
    Pin clk;
    Pin miso;
    Pin mosi;
    Pin cs;
    Pin tfrRdy;
    SSPChannel dev;
    uint32_t sig2;
} SBCIAPParams;

constexpr uint32_t BOOTIAPParamSig = 0xB0010001;
typedef struct{
    uint32_t sig1;
    uint32_t cmd;
    uint32_t sig2;
} BOOTIAPParams;

typedef enum {
    NormalBoot,
    ExecFirmware
} BootCmd;

#endif