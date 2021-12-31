#include <USBSerial.h>
#define SERIAL_MAIN_DEVICE  SerialUSB
const NvicPriority NvicPrioritySpi = 8;
constexpr size_t IAP_BUFFER_SIZE = 2048;
constexpr size_t IAP_FLASH_START = 0x8008000;
constexpr size_t IAP_FLASH_END = 0x80fffff;
constexpr uint32_t IAP_BAD_SECTOR = 0xffffffff;
