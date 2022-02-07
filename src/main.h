#include <USBSerial.h>
#define SERIAL_MAIN_DEVICE  SerialUSB
const NvicPriority NvicPrioritySpi = 8;
const NvicPriority NvicPrioritySDIO = 8;

#ifdef FLASH_SIZE
#undef FLASH_SIZE
#endif

#if STM32H7
constexpr size_t FirmwareFlashStart = 0x8020000;
constexpr size_t FLASH_ADDR = FLASH_BASE;
// For now we only use one bank of flash
#undef FLASH_SIZE
constexpr size_t FLASH_SIZE = 0x0100000;
#elif STM32F4
constexpr size_t FirmwareFlashStart = 0x8008000;
constexpr size_t FLASH_ADDR = FLASH_BASE;
constexpr size_t FLASH_SIZE = (FLASH_END + 1 - FLASH_BASE);
#else
#error "unknown mcu"
#endif

constexpr uint32_t IAP_BAD_SECTOR = 0xffffffff;
constexpr size_t IAP_BUFFER_SIZE = 2048;
constexpr char firmwarePath[] = "0:/firmware.bin";
constexpr char goodFirmwarePath[] = "0:/firmware.cur";
constexpr char badFirmwarePath[] = "0:/firmware.bad";

typedef struct
{
  /* Stack pointer */
  void* pvStack;
  
  /* Cortex-M handlers */
  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnMemManage_Handler;
  void* pfnBusFault_Handler;
  void* pfnUsageFault_Handler;
  void* pfnReserved1_Handler;
  void* pfnReserved2_Handler;
  void* pfnReserved3_Handler;
  void* pfnReserved4_Handler;
  void* pfnSVC_Handler;
  void* pfnDebugMon_Handler;
  void* pfnReserved5_Handler;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;
} DeviceVectors;

