#include <USBSerial.h>
#define SERIAL_MAIN_DEVICE  SerialUSB
const NvicPriority NvicPrioritySpi = 8;
constexpr size_t IAP_BUFFER_SIZE = 2048;
