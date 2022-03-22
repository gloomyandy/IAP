/*
 * main.cpp
 *
 *  Created on: 16/Dec/2021
 *  Author: GA
 */

#include <Core.h>
#include <HardwareSPI.h>
#include <Cache.h>
#include "main.h"
#include <malloc.h>
#include <General/SafeVsnprintf.h>
#include <General/StringFunctions.h>
#include "iapparams.h"
// Define replacement standard library functions
#include <syscalls.h>
//#define USB_DEBUG 1
// This is the string that identifies the board type and firmware version, that the vector at 0x20 points to.
// The characters after the last space must be the firmware version in standard format, e.g. "3.3.0" or "3.4.0beta4". The firmware build date/time is not included.
extern const char VersionText[] = FIRMWARE_NAME " version " VERSION;

extern "C" void SysTick_Handler(void)
{
	CoreSysTick();
	WatchdogReset();
}

extern "C" void WWDG_IRQHandler() noexcept __attribute__((naked));
void WWDG_IRQHandler() noexcept
{
}

#if USB_DEBUG
static char formatBuffer[100];
#endif

// Write message to USB
void MessageF(const char *fmt, ...) noexcept
{
#if USB_DEBUG
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);
	SERIAL_MAIN_DEVICE.print(formatBuffer);
#endif
}

extern "C" void debugPrintf(const char *fmt, ...) noexcept
{
#if USB_DEBUG
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);
	SERIAL_MAIN_DEVICE.print(formatBuffer);
#endif
}

void debugFlush()
{
#if USB_DEBUG
	SERIAL_MAIN_DEVICE.flush();
#endif
}


[[noreturn]] void OutOfMemoryHandler() noexcept
{
	debugPrintf("Out of memory\n");
	debugFlush();
	for(;;);
}

void assert_failed(uint8_t *file, uint32_t line)
{
	debugPrintf("Assert failed file %s line %d\n", file, (int)line);
	debugFlush();
	for(;;);
}

extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute((naked));
void vAssertCalled(uint32_t line, const char *file) noexcept
{
	debugPrintf("ASSERTION FAILED IN %s on LINE %d\n", file, line);
	debugFlush();
	for(;;);
}

extern "C" void GetMallocMutex() noexcept
{
}

extern "C" void ReleaseMallocMutex() noexcept
{
}

void FlushECC(void *ptr, int bytes) noexcept
{
	// On some mcus we need to flush write to RAM before doing a reset if we want to read
	// that data after the reset.
#if STM32H7
	uint32_t addr = (uint32_t)ptr;
	/* Check if accessing AXI SRAM => 64-bit words*/
	if(addr >= 0x24000000 && addr < 0x24080000){
		volatile uint64_t temp;
		volatile uint64_t* flush_ptr = (uint64_t*) (addr & 0xFFFFFFF8);
		uint64_t *end_ptr = (uint64_t*) ((addr+bytes) & 0xFFFFFFF8) + 1;

		do{
			temp = *flush_ptr;
			*flush_ptr = temp;
			flush_ptr++;
		}while(flush_ptr != end_ptr);
	}
	/* Otherwise 32-bit words */
	else {
		volatile uint32_t temp;
		volatile uint32_t* flush_ptr = (uint32_t*) (addr & 0xFFFFFFFC);
		uint32_t *end_ptr = (uint32_t*) ((addr+bytes) & 0xFFFFFFFC) + 1;

		do{
			temp = *flush_ptr;
			*flush_ptr = temp;
			flush_ptr++;
		}while(flush_ptr != end_ptr);
	}
#endif
}

void *GetParamsPtr() noexcept
{
	// We use a paramter block just above the top of the stack. Return a pointer to it
	const uint32_t vtab = SCB->VTOR & SCB_VTOR_TBLOFF_Msk;
	return reinterpret_cast<void *>(*reinterpret_cast<uint32_t *>(vtab));
}

void AppInit() noexcept
{
	// Some bootloaders leave UASRT3 enabled, make sure it does not cause problems
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}

void Init(bool watchdog) noexcept
{
	// Make sue Arm ints are enabled
	__asm volatile(
					" cpsie i				\n"
					" cpsie f				\n"
				);
	CoreInit();
	// Initialise systick (needed for delay calls) - CoreNG initialises it in non-interrupt mode
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
	if (watchdog)
		WatchdogInit();

	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
	Cache::Init();					// initialise the cache and/or the MPU, if applicable to this processor
	Cache::Disable();				// Make sure it is off to avoid DMA issues
	IrqEnable();
#if USB_DEBUG
	SERIAL_MAIN_DEVICE.begin(9600);
	delay(5000);
#else
	delay(500);
#endif
	debugPrintf("IAP running....\n");
	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
}

#if STM32H7
// We write in 256 bit alignment!
#define IS_FLASH_ALIGNED(addr) (((uint32_t)(addr) & (32-1)) == 0)
#else
// we write with 32bit alignment
#define IS_FLASH_PROGRAM_ADDRESS(addr) (((addr) >= FLASH_BASE) && ((addr) <= FLASH_END))
#define IS_FLASH_ALIGNED(addr) (((uint32_t)(addr) & (sizeof(uint32_t)-1)) == 0)
#endif
#define IS_ALIGNED(addr) (((uint32_t)(addr) & (sizeof(uint32_t)-1)) == 0)
constexpr uint32_t IAP_BAD_SECTOR = 0xffffffff;

static void FlashClearError()
{
	// Clear pending flags (if any)
#if STM32H7
	__HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_WRPERR_BANK1 | FLASH_FLAG_PGSERR_BANK1 | FLASH_FLAG_STRBERR_BANK1 | \
                            		FLASH_FLAG_INCERR_BANK1 | FLASH_FLAG_OPERR_BANK1 | FLASH_FLAG_SNECCERR_BANK1 | \
                                    FLASH_IT_DBECCERR_BANK1);
	__HAL_FLASH_CLEAR_FLAG_BANK2((FLASH_FLAG_WRPERR_BANK2 | FLASH_FLAG_PGSERR_BANK2 | FLASH_FLAG_STRBERR_BANK2 | \
									FLASH_FLAG_INCERR_BANK2 | FLASH_FLAG_SNECCERR_BANK2 | FLASH_IT_DBECCERR_BANK2) & 0x7FFFFFFFU);
#else
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
							FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);
#endif
}


static bool isErased(const uint32_t addr, const size_t len) noexcept
{
#if STM32H7
    // On the STM32H7 if the flash has not been correctly erased then simply reading
    // it can cause a bus fault (due to multiple ECC errors). We avoid this by disaabling
    // the fault mechanism while checking the flash memory.
    const irqflags_t flags = IrqSave();

    __set_FAULTMASK(1);
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
#endif
	HAL_FLASH_Unlock();
	FlashClearError();

    bool blank = true;
	// Check that the sector really is erased
	for (uint32_t p = addr; p < addr + len && blank; p += sizeof(uint32_t))
	{
		if (*reinterpret_cast<const uint32_t*>(p) != 0xFFFFFFFF)
		{
			blank = false;
		}
	}

	FlashClearError();
	HAL_FLASH_Lock();

#if STM32H7
    // restore bus fault logic
	__set_FAULTMASK(0);
	SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();
	IrqRestore(flags);
#endif
	return blank;
}

static uint32_t FlashGetSector(const uint32_t addr) noexcept
{
	if (!IS_FLASH_PROGRAM_ADDRESS(addr))
	{
		debugPrintf("Bad flash address %x\n", (unsigned)addr);
		return IAP_BAD_SECTOR;
	}
	// Flash memory on STM32F4 is 4 sectors of 16K + 1 sector of 64K + 8 sectors of 128K
    // on the H7 all sectors are 128Kb
	uint32_t offset = addr - FLASH_BASE;
#if STM32H7
	return offset/0x20000;
#else
	if (offset < 4*0x4000)
		return offset / 0x4000;
	else if (offset < 4*0x4000 + 0x10000)
		return 4;
	else
		return offset / 0x20000 + 4;
#endif
}

static size_t FlashGetSectorLength(const uint32_t addr) noexcept
{
	uint32_t sector = FlashGetSector(addr);
	if (sector == IAP_BAD_SECTOR)
		return 0;
#if STM32H7
	return 0x20000;
#else
	if (sector < 4)
		return 0x4000;
	else if (sector < 5)
		return 0x10000;
	else
		return 0x20000;
#endif
}

static bool FlashEraseSector(const uint32_t sector) noexcept
{
	WatchdogReset();
	FLASH_EraseInitTypeDef eraseInfo;
	uint32_t SectorError;
	bool ret = true;
	eraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
#if STM32H7
    if (sector < FLASH_SECTOR_TOTAL)
    {
	    eraseInfo.Banks = FLASH_BANK_1;
        eraseInfo.Sector = sector;
    }
    else
    {
	    eraseInfo.Banks = FLASH_BANK_2;
        eraseInfo.Sector = sector - FLASH_SECTOR_TOTAL;
    }
    debugPrintf("Erase %d bank %d sector %d\n", sector, eraseInfo.Banks, eraseInfo.Sector);
#else
	eraseInfo.Sector = sector;
#endif
	eraseInfo.NbSectors = 1;
	eraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	HAL_FLASH_Unlock();
	FlashClearError();
	if (HAL_FLASHEx_Erase(&eraseInfo, &SectorError) != HAL_OK)
	{
		ret = false;
	}
	HAL_FLASH_Lock();
	if (!ret)
		debugPrintf("Flash erase failed sector %d error %x\n", (int)sector, (unsigned)SectorError);
	return ret;
}

static bool FlashWrite(const uint32_t addr, const uint8_t *data, const size_t len) noexcept
{
	uint32_t *dst = (uint32_t *)addr;
	uint32_t *src = (uint32_t *)data;
	if (!IS_FLASH_ALIGNED(dst) || !IS_ALIGNED(src) || !IS_ALIGNED(len))
	{
		debugPrintf("FlashWrite alignment error dst %x, data %d len %d\n", (unsigned)dst, (unsigned)src, (int)len);
		return false;
	}
	bool ret = true;
	debugPrintf("Write flash addr %x len %d\n", (unsigned)addr, (int)len);
	WatchdogReset();
    bool cacheEnabled = Cache::Disable();
	HAL_FLASH_Unlock();
	FlashClearError();
	uint32_t cnt = 0;
	while(cnt < len)
	{
#if STM32H7
#define FLASH_TYPEPROGRAM_WORD FLASH_TYPEPROGRAM_FLASHWORD
		// We write 256 bits == 8 32bit words at a time
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, (uint32_t) dst, (uint64_t) src) != HAL_OK)
		{
			ret = false;
			break;
		}
		dst += 8;
		src += 8;
		cnt += 32;
#else
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) dst, (uint64_t) *src) != HAL_OK)
		{
			ret = false;
			break;
		}
		dst++;
		src++;
		cnt += 4;
#endif
	}
	HAL_FLASH_Lock();
    if (cacheEnabled) Cache::Enable();
	if (!ret)
		debugPrintf("Flash write failed cnt %d\n", (int)((int)dst - addr));

	return ret; 
}

static bool FlashRead(const uint32_t addr, uint8_t *data, const size_t len) noexcept
{
#if STM32H7
    // On the STM32H7 if the flash has not been correctly erased then simply reading
    // it can cause a bus fault (due to multiple ECC errors). We avoid this by disaabling
    // the fault mechanism while checking the flash memory.
    const irqflags_t flags = IrqSave();

    __set_FAULTMASK(1);
    SCB->CCR |= SCB_CCR_BFHFNMIGN_Msk;
    __DSB();
    __ISB();
#endif
	HAL_FLASH_Unlock();
	FlashClearError();
    // Do the actual read from flash
    memcpy((void *)data, (void *)addr, len);
    // Clear any errors
	FlashClearError();
	HAL_FLASH_Lock();
#if STM32H7
    // restore bus fault logic
	__set_FAULTMASK(0);
	SCB->CCR &= ~SCB_CCR_BFHFNMIGN_Msk;
	__DSB();
	__ISB();
	IrqRestore(flags);
#endif
    return true;
}

bool FlashVerify(const uint32_t addr, const uint8_t *data, const size_t len)
{
	uint32_t *dst = (uint32_t *)addr;
	uint32_t *src = (uint32_t *)data;
	uint32_t cnt = len/sizeof(uint32_t);

	for(uint32_t i = 0; i < cnt; i++)
	{
		if (*dst != *src)
		{
			debugPrintf("Verify failed address %x %x != %x\n", (unsigned)dst, (unsigned)*dst, (unsigned)*src);
			return false;
		}
		dst++;
		src++;
	}
	return true;
}

bool FlashEraseAll() noexcept
{
	uint32_t addr = FirmwareFlashStart;
	while (addr < FLASH_ADDR + FLASH_SIZE)
	{
		uint32_t sector = FlashGetSector(addr);
		uint32_t len = FlashGetSectorLength(addr);
		debugPrintf("Erasing address %x sector %d length %d\n", (unsigned)addr, (int)sector, (int)len);
		if (isErased(addr, len))
			debugPrintf("Allready erased\n");
		else
		{
			if (!FlashEraseSector(sector))
			{
				debugPrintf("Erase failed\n");
				return false;
			}
		}
		addr += len;
	}
	return true;
}

#if IAP_SPI_LOADER
struct FlashVerifyRequest
{
	uint32_t firmwareLength;
	uint16_t crc16;
	uint16_t dummy;
};
const uint32_t TransferCompleteDelay = 400;								// DCS waits 500ms when the firmware image has been transferred
const uint32_t TransferTimeout = 2000;									// How long to wait before timing out
alignas(4) uint8_t rxBuffer[IAP_BUFFER_SIZE];
alignas(4) uint8_t txBuffer[IAP_BUFFER_SIZE];
bool transferReadyHigh = false;
Pin transferReadyPin = NoPin;
bool dataReceived = false;

// interrupt handler
void SpiInterrupt(HardwareSPI *spi) noexcept
{
	dataReceived = true;
}


HardwareSPI *InitSPI(SSPChannel chan, Pin clk, Pin miso, Pin mosi, Pin cs, Pin tfrRdy) noexcept
{
	transferReadyPin = tfrRdy;
	transferReadyHigh = false;
	pinMode(transferReadyPin, OUTPUT_LOW);
	HardwareSPI *dev = (HardwareSPI*)SPI::getSSPDevice(chan);
	if (dev == nullptr) 
	{
		debugPrintf("Failed to get SPI device %d\n", (int)chan);
		return nullptr;
	}
	dev->initPins(clk, miso, mosi, NoPin, NvicPrioritySpi);
	dev->configureDevice(SPI_MODE_SLAVE, 8, (uint8_t)0, 100000000, false);
	return dev;
}

spi_status_t SPITransfer(HardwareSPI *dev, const uint8_t *tx_data, uint8_t *rx_data, size_t len, uint32_t timeout)
{
	uint32_t start = millis();
	dataReceived = false;
	dev->startTransfer(tx_data, rx_data, len, SpiInterrupt);
	// Tell the SBC we are ready to go
	transferReadyHigh = !transferReadyHigh;
	digitalWrite(transferReadyPin, transferReadyHigh);
	while (!dataReceived && millis() - start < timeout)
	{
	}
	if (!dataReceived)
	{
		dev->stopTransfer();
		dataReceived = false;
		debugPrintf("Transfer timeout\n");
		return SPI_TIMEOUT;
	}
	return SPI_OK;
}

int TransferDataToFlash(HardwareSPI *dev)
{
	if (!FlashEraseAll())
	{
		debugPrintf("Flash erase failed\n");
		return -1;
	}
	//SBC expects 0x1a to be sent back
	memset(txBuffer, 0x1a, sizeof(txBuffer));
	uint32_t retryCnt = 0;
	uint32_t blockCnt = 0;
	uint32_t flashAddr = FirmwareFlashStart;
	for(;;)
	{
		spi_status_t status = SPITransfer(dev, txBuffer, rxBuffer, sizeof(rxBuffer), blockCnt == 0 ? TransferTimeout : TransferCompleteDelay );
		debugPrintf("SPI trans returns %d\n", (int)status);
		if (status == SPI_TIMEOUT)
		{
			if (blockCnt == 0 && ++retryCnt < 10) continue;
			// A timeout indicates that the main transfer is complete
			debugPrintf("Transfer complete after %d blocks\n", blockCnt);
			return blockCnt;
		}
		if (status == SPI_ERROR)
		{
			debugPrintf("Terminating transfer on error\n");
			return -1;
		}
		blockCnt++;
		debugPrintf("Read block %d\n", blockCnt);
		if (!FlashWrite(flashAddr, rxBuffer, sizeof(rxBuffer)))
		{
			debugPrintf("Flash write failed\n");
			return -1;
		}
		if (!FlashVerify(flashAddr, rxBuffer, sizeof(rxBuffer)))
		{
			debugPrintf("Flash verify failed\n");
			return -1;
		}
		flashAddr += sizeof(rxBuffer);
	}
}

uint16_t CRC16(const char *buffer, size_t length) noexcept
{
	static const uint16_t crc16_table[] =
	{
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
		0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
		0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
		0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
		0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
		0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
		0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
		0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
		0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
		0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
		0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
		0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
		0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
		0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
		0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
		0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
		0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
		0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
		0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
		0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
		0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
		0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
		0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
		0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
		0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
		0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
	};

	uint16_t Crc = 65535;
	uint16_t x;
	for (size_t i = 0; i < length; i++)
	{
		x = (uint16_t)(Crc ^ buffer[i]);
		Crc = (uint16_t)((Crc >> 8) ^ crc16_table[x & 0x00FF]);
	}

	return Crc;
}

bool ProcessChecksum(HardwareSPI *dev, int cnt) noexcept
{
	// Read the checksum
	spi_status_t status = SPITransfer(dev, txBuffer, rxBuffer, sizeof(FlashVerifyRequest), TransferTimeout);
	if (status != SPI_OK)
	{
		debugPrintf("Error reading CRC %d\n", (int) status);
		return false;
	}
	const FlashVerifyRequest *request = reinterpret_cast<const FlashVerifyRequest*>(rxBuffer);
	const uint16_t actualCRC = CRC16((char *)FirmwareFlashStart, request->firmwareLength);
	debugPrintf("Got verify request flash len %d CRC %x actual CRC %x\n", request->firmwareLength, request->crc16, (unsigned)actualCRC);
	// send response
	if (request->crc16 == actualCRC)
		txBuffer[0] = 0x0c;
	else
		txBuffer[0] = 0xff;
	SPITransfer(dev,txBuffer, rxBuffer, 1, TransferTimeout);
	return true;
}

const SBCIAPParams *const GetParams()
{
	const SBCIAPParams *const paramsPtr = (const SBCIAPParams *const) GetParamsPtr();
	if (paramsPtr->sig1 != SBCIAPParamSig || paramsPtr->sig2 != SBCIAPParamSig)
	{
		debugPrintf("Invalid parameter block sig1 %x sig2 %x\n", (unsigned)paramsPtr->sig1, (unsigned)paramsPtr->sig2);
		return nullptr;
	}
	else
		return paramsPtr;
}	

// Application entry point
[[noreturn]] void AppMain() noexcept
{
	Init(true);

	const SBCIAPParams *const params = GetParams();
	if (params != nullptr)
	{
		HardwareSPI *dev = InitSPI(params->dev, params->clk, params->miso, params->mosi, params->cs, params->tfrRdy);
		int blockCnt = TransferDataToFlash(dev);
		if (blockCnt > 0) ProcessChecksum(dev, blockCnt);
		debugPrintf("IAP complete\n");
		digitalWrite(transferReadyPin, false);
	}

	delay(1000);
	ResetProcessor();
}

#elif IAP_BOOT_LOADER


#include <integer.h>
#include <ff.h>
#include <sd_mmc.h>
#include <HardwareSDIO.h>


typedef struct {
    SSPChannel device;
    Pin pins[6];
} SDCardConfig;

// These are our known SD card configurations
static constexpr SDCardConfig SDCardConfigs[] = {
    {SSP1, {PA_5, PA_6, PB_5, PA_4, NoPin, NoPin}}, // SKR Pro
    {SSP1, {PA_5, PA_6, PA_7, PA_4, NoPin, NoPin}}, // GTR
    {SSPSDIO, {PC_8, PC_9, PC_10, PC_11, PC_12, PD_2}}, // Fly/SDIO
    {SSP3, {PC_10, PC_11, PC_12, PC_9, NoPin, NoPin}}, // MKS?
    {SSP3, {PC_10, PC_11, PC_12, PA_15, NoPin, NoPin}}, // BTT BX
};

static bool MountSDCard(uint32_t config, FATFS *fs)
{
    const SDCardConfig *conf = &SDCardConfigs[config];
    if (conf->device != SSPSDIO)
    {
        SPI::getSSPDevice(conf->device)->initPins(conf->pins[0], conf->pins[1], conf->pins[2]);
        sd_mmc_setSSPChannel(0, conf->device, conf->pins[3]);
    }
    else
    {
        HardwareSDIO::SDIO1.InitPins(NvicPrioritySDIO);
        sd_mmc_setSSPChannel(0, conf->device, NoPin);
    }

    FRESULT rslt= f_mount (fs, "0:", 1);
    if (rslt == FR_OK)
    {
        return true;
    }

    // mount failed reset things
    if (conf->device != SSPSDIO)
        ((HardwareSPI *)(SPI::getSSPDevice(conf->device)))->disable();
    sd_mmc_setSSPChannel(0, SSPNONE, NoPin);
    return false;
}

BOOTIAPParams *const GetParams()
{
	BOOTIAPParams *const paramsPtr =  (BOOTIAPParams *const) GetParamsPtr();
	if (paramsPtr->sig1 != BOOTIAPParamSig || paramsPtr->sig2 != BOOTIAPParamSig)
		return nullptr;
	else
		return paramsPtr;
}

void SetParams(uint32_t val)
{
	BOOTIAPParams *const paramsPtr =  (BOOTIAPParams *const) GetParamsPtr();
	paramsPtr->sig1 = BOOTIAPParamSig;
	paramsPtr->sig2 = BOOTIAPParamSig;
	paramsPtr->state = val;
	FlushECC(paramsPtr, sizeof(BOOTIAPParams));
}

// Execute the main firmware we call this just after a processor reset
// so pretty everything is in the default state.
[[noreturn]] void StartFirmware()
{
	// Modify vector table location
	__DSB();
	__ISB();
	SCB->VTOR = FirmwareFlashStart & SCB_VTOR_TBLOFF_Msk;
	__DSB();
	__ISB();

	__asm volatile ("mov r3, %0" : : "r" (FirmwareFlashStart) : "r3");

	__asm volatile ("ldr r1, [r3]");
	__asm volatile ("msr msp, r1");
	__asm volatile ("mov sp, r1");

	__asm volatile ("isb");

	__asm volatile ("ldr r1, [r3, #4]");
	__asm volatile ("orr r1, r1, #1");
	__asm volatile ("bx r1");

	// This point is unreachable, but gcc doesn't seem to know that
	for (;;) { }
}


// Check that we have valid firmware.
bool CheckValidFirmware(const DeviceVectors * const vectors)
{
	if (   reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) < FirmwareFlashStart
		|| reinterpret_cast<uint32_t>(vectors->pfnReset_Handler) >= FLASH_ADDR + FLASH_SIZE
		|| reinterpret_cast<uint32_t>(vectors->pvStack) < IRAM_ADDR
		|| reinterpret_cast<uint32_t>(vectors->pvStack) > IRAM_ADDR + IRAM_SIZE
//		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) < FirmwareFlashStart
//		|| reinterpret_cast<uint32_t>(vectors->pvReservedM9) > FirmwareFlashStart + FLASH_SIZE - 4
	   )
	{
		return false;
	}

#if 0
	// Fetch the CRC-32 from the file
	const uint32_t *crcAddr = (const uint32_t*)(vectors->pvReservedM9);
	const uint32_t storedCRC = *crcAddr;

	// Compute the CRC-32 of the file
#if SAME5x
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_DISABLE | DMAC_CRCCTRL_CRCPOLY_CRC32;	// disable the CRC unit
#elif SAMC21
	DMAC->CTRL.bit.CRCENABLE = 0;
#else
# error Unsupported processor
#endif
	DMAC->CRCCHKSUM.reg = 0xFFFFFFFF;
	DMAC->CRCCTRL.reg = DMAC_CRCCTRL_CRCBEATSIZE_WORD | DMAC_CRCCTRL_CRCSRC_IO | DMAC_CRCCTRL_CRCPOLY_CRC32;
#if SAMC21
	DMAC->CTRL.bit.CRCENABLE = 1;
#endif
	for (const uint32_t *p = reinterpret_cast<const uint32_t*>(FirmwareFlashStart); p < crcAddr; ++p)
	{
		DMAC->CRCDATAIN.reg = *p;
		asm volatile("nop");
		asm volatile("nop");
	}

	DMAC->CRCSTATUS.reg = DMAC_CRCSTATUS_CRCBUSY;
	asm volatile("nop");
	const uint32_t actualCRC = DMAC->CRCCHKSUM.reg;
	if (actualCRC == storedCRC)
	{
		return true;
	}

	String<100> message;
	message.printf("CRC error: stored %08" PRIx32 ", actual %" PRIx32, storedCRC, actualCRC);
	ReportError(message.c_str(), ErrorCode::badCRC);
	return false;
#endif
	return true;
}


void AppPreInit() noexcept
{
	// Called before clocks configured hardware in default state. Be very careful what we do here!
	// Check to see if we should try and start the main firmware
	BOOTIAPParams *const paramsPtr = GetParams();
	if (paramsPtr != nullptr && paramsPtr->state == BootState::ExecFirmware)
	{
		// Set things so we check for new firmware if this does not work
		SetParams(BootState::FirmwareRunning);
		// Make sure that what we are about jump to looks ok
		const DeviceVectors * const vectors = reinterpret_cast<const DeviceVectors*>(FirmwareFlashStart);
		if (CheckValidFirmware(vectors))
			StartFirmware();
	}	
}	

alignas(4) uint8_t ioBuffer[IAP_BUFFER_SIZE];

bool TransferDataToFlash(FIL *imageFile)
{
	UINT cnt;
	// read first part of file and check it is valid
	FRESULT rslt = f_read(imageFile, ioBuffer, sizeof(ioBuffer), &cnt);
	if (rslt != FR_OK || cnt != sizeof(ioBuffer))
	{
		debugPrintf("Initial read failed rslt %d cnt %u\n", rslt, cnt);
		return false;
	}
	if (!CheckValidFirmware((const DeviceVectors * const) ioBuffer))
	{
		debugPrintf("Invalid firmware image\n");
		return false;
	}
	// we have what looks like a good image
	if (!FlashEraseAll())
	{
		debugPrintf("Flash erase failed\n");
		return false;
	}
	uint32_t flashAddr = FirmwareFlashStart;
	do {
		if (!FlashWrite(flashAddr, ioBuffer, cnt))
		{
			debugPrintf("Flash write failed\n");
			return false;
		}
		if (!FlashVerify(flashAddr, ioBuffer, cnt))
		{
			debugPrintf("Flash verify failed\n");
			return false;
		}
		debugPrintf("Written %d bytes to address %x\n", cnt, flashAddr);
		flashAddr += cnt;
		rslt = f_read(imageFile, ioBuffer, sizeof(ioBuffer), &cnt);
		if (rslt != FR_OK)
		{
			debugPrintf("Read failed rslt %d cnt %u\n", rslt, cnt);
			return false;
		}

	} while (cnt > 0);
	debugPrintf("Flash complete\n");
	return true;
}

// Application entry point
[[noreturn]] void AppMain() noexcept
{
	FIL imageFile;
    FATFS fs;
#if STM32H7
	alignas(4) static uint8_t sectorBuffer[512];
	fs.win = sectorBuffer;
# endif

	Init(false);
	const DeviceVectors * const vectors = reinterpret_cast<const DeviceVectors*>(FirmwareFlashStart);
	if (CheckValidFirmware(vectors))
		debugPrintf("Current firmware is valid\n");
	else
		debugPrintf("Current firmware is not valid\n");
	if (MountSDCard(SDTYPE, &fs))
	{
		debugPrintf("Mounted SD card\n");
		if (f_open(&imageFile, firmwarePath, FA_READ) == FR_OK)
		{
			debugPrintf("Opened image file\n");
			bool transferOk = TransferDataToFlash(&imageFile);
			f_close(&imageFile);
			// rename the file so we don't do this again
			f_unlink((transferOk ? goodFirmwarePath : badFirmwarePath));
			f_rename(firmwarePath, (transferOk ? goodFirmwarePath : badFirmwarePath));
		}
		else
			debugPrintf("Image file not found\n");
		f_unmount("0:");

	}
	else
		debugPrintf("Failed to mount SD card\n");
	SetParams(BootState::ExecFirmware);
	debugPrintf("rebooting....\n");
#if USB_DEBUG
	delay(2000);
#endif
	ResetProcessor();
}

#else
#error "Unknown build configuration"
#endif

// End
