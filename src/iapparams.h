#ifndef IAPPARAMS_H
#define IAPPARAMS_H
#include <Core.h>
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
#endif