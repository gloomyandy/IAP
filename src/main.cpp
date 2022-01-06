/*
 * main.cpp
 *
 *  Created on: 16/Dec/2021
 *      Author: GA
 */

#include <Core.h>
#include <HardwareSPI.h>
#include <Cache.h>
#include "main.h"
#include <malloc.h>
#include <General/SafeVsnprintf.h>
#include <General/StringFunctions.h>
#include "iapparams.h"

extern char _end;						// defined in linker script
extern char _estack;					// defined in linker script

// Define replacement standard library functions
#include <syscalls.h>

struct FlashVerifyRequest
{
	uint32_t firmwareLength;
	uint16_t crc16;
	uint16_t dummy;
};

#ifndef DEBUG
extern uint32_t _firmware_crc;			// defined in linker script
#endif

extern "C" void SysTick_Handler(void)
{
	CoreSysTick();
	WatchdogReset();
}

extern "C" void WWDG_IRQHandler() noexcept __attribute__((naked));
void WWDG_IRQHandler() noexcept
{
}

static char formatBuffer[100];

// Write message to USB
void MessageF(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);

#if 0
	SERIAL_AUX_DEVICE.print("{\"message\":\"");
	SERIAL_AUX_DEVICE.print(formatBuffer);
	SERIAL_AUX_DEVICE.print("\"}\n");
	delay_ms(10);
#endif
	SERIAL_MAIN_DEVICE.print(formatBuffer);
}

extern "C" void debugPrintf(const char *fmt, ...) noexcept
{
	va_list vargs;
	va_start(vargs, fmt);
	SafeVsnprintf(formatBuffer, ARRAY_SIZE(formatBuffer), fmt, vargs);
	va_end(vargs);
	SERIAL_MAIN_DEVICE.print(formatBuffer);
}

#if defined(DEBUG) && DEBUG
#else
# define debugPrintf(...)		do { } while (false)
#endif


[[noreturn]] void OutOfMemoryHandler() noexcept
{
	debugPrintf("Out of memory\n");
    for(;;);
}

void assert_failed(uint8_t *file, uint32_t line)
{
    debugPrintf("Assert failed file %s line %d\n", file, (int)line);
	SERIAL_MAIN_DEVICE.flush();
    for(;;);
}

extern "C" [[noreturn]] void vAssertCalled(uint32_t line, const char *file) noexcept __attribute((naked));
void vAssertCalled(uint32_t line, const char *file) noexcept
{
	debugPrintf("ASSERTION FAILED IN %s on LINE %d\n", file, line);
	SERIAL_MAIN_DEVICE.flush();
    for(;;);
}

extern "C" void GetMallocMutex() noexcept
{
}

extern "C" void ReleaseMallocMutex() noexcept
{
}

void AppInit() noexcept
{
	// Some bootloaders leave UASRT3 enabled, make sure it does not cause problems
	HAL_NVIC_DisableIRQ(USART3_IRQn);
}

#define IS_FLASH_PROGRAM_ADDRESS(addr) (((addr) >= FLASH_BASE) && ((addr) <= FLASH_END))
#define IS_ALIGNED(addr) (((uint32_t)(addr) & (sizeof(uint32_t)-1)) == 0)

bool isErased(const uint32_t addr, const size_t len) noexcept
{
	// Check that the sector really is erased
	for (uint32_t p = addr; p < addr + len; p += sizeof(uint32_t))
	{
		if (*reinterpret_cast<const uint32_t*>(p) != 0xFFFFFFFF)
		{
			return false;
		}
	}
	return true;
}

uint32_t FlashGetSector(const uint32_t addr) noexcept
{
	if (!IS_FLASH_PROGRAM_ADDRESS(addr))
	{
		debugPrintf("Bad flash address %x\n", (unsigned)addr);
		return IAP_BAD_SECTOR;
	}
	// Flash memory on STM32F4 is 4 sectors of 16K + 1 sector of 64K + 8 sectors of 128K
	uint32_t offset = addr - FLASH_BASE;
	if (offset < 4*0x4000)
		return offset / 0x4000;
	else if (offset < 4*0x4000 + 0x10000)
		return 4;
	else
		return offset / 0x20000 + 4;
}

size_t FlashGetSectorLength(const uint32_t addr) noexcept
{
	uint32_t sector = FlashGetSector(addr);
	if (sector == IAP_BAD_SECTOR)
		return 0;
	if (sector < 4)
		return 0x4000;
	else if (sector < 5)
		return 0x10000;
	else
		return 0x20000;
}

bool FlashEraseSector(const uint32_t sector) noexcept
{
	WatchdogReset();
    FLASH_EraseInitTypeDef eraseInfo;
    uint32_t SectorError;
    bool ret = true;
    eraseInfo.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInfo.Sector = sector;
    eraseInfo.NbSectors = 1;
    eraseInfo.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    HAL_FLASH_Unlock();
    // Clear pending flags (if any)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
                            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);
    if (HAL_FLASHEx_Erase(&eraseInfo, &SectorError) != HAL_OK)
    {
        ret = false;
    }
    HAL_FLASH_Lock();
	if (!ret)
	    debugPrintf("Flash erase failed sector %d error %x\n", (int)sector, (unsigned)SectorError);
	return ret;
}

bool FlashWrite(const uint32_t addr, const uint8_t *data, const size_t len) noexcept
{
	uint32_t *dst = (uint32_t *)addr;
	uint32_t *src = (uint32_t *)data;
	uint32_t cnt = len/sizeof(uint32_t);
	if (!IS_ALIGNED(dst) || !IS_ALIGNED(src) || !IS_ALIGNED(len))
	{
		debugPrintf("FlashWrite alignment error dst %x, data %d len %d\n", (unsigned)dst, (unsigned)src, (int)len);
		return false;
	}
    bool ret = true;
	debugPrintf("Write flash addr %x len %d\n", (unsigned)addr, (int)len);
	WatchdogReset();
    HAL_FLASH_Unlock();
    // Clear pending flags (if any)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP    | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |\
                            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);
    for(uint32_t i = 0; i < cnt; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t) dst, (uint64_t) *src) != HAL_OK)
        {
            ret = false;
            break;
        }
        dst++;
        src++;
    }
    HAL_FLASH_Lock();      

	if (!ret)
    	debugPrintf("Flash write failed cnt %d\n", (int)dst - addr);

    return ret; 
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
	uint32_t addr = IAP_FLASH_START;
	while (addr <= IAP_FLASH_END)
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
	NVIC_SetPriority(DMA1_Stream3_IRQn, NvicPrioritySpi);
    NVIC_SetPriority(DMA1_Stream4_IRQn, NvicPrioritySpi);
    NVIC_SetPriority(DMA1_Stream0_IRQn, NvicPrioritySpi);
    NVIC_SetPriority(DMA1_Stream5_IRQn, NvicPrioritySpi);
	HardwareSPI *dev = (HardwareSPI*)SPI::getSSPDevice(chan);
	if (dev == nullptr) 
	{
		debugPrintf("Failed to get SPI device %d\n", (int)chan);
		return nullptr;
	}
	dev->initPins(clk, miso, mosi, NoPin);
	dev->configureDevice(SPI_MODE_SLAVE, 8, (uint8_t)0, 100000000, false);
	delay(2000);
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
	uint32_t flashAddr = IAP_FLASH_START;
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
	// wait for dcs to be ready
	//delay(1000);
	// Read the checksum
	spi_status_t status = SPITransfer(dev, txBuffer, rxBuffer, sizeof(FlashVerifyRequest), TransferTimeout);
	if (status != SPI_OK)
	{
		debugPrintf("Error reading CRC %d\n", (int) status);
		return false;
	}
	const FlashVerifyRequest *request = reinterpret_cast<const FlashVerifyRequest*>(rxBuffer);
	const uint16_t actualCRC = CRC16((char *)IAP_FLASH_START, request->firmwareLength);
	debugPrintf("Got verify request flash len %d CRC %x actual CRC %x\n", request->firmwareLength, request->crc16, (unsigned)actualCRC);
	// send Ok response
	txBuffer[0] = 0x0c;
	SPITransfer(dev,txBuffer, rxBuffer, 1, TransferTimeout);
	return true;
}

void Init() noexcept
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
    WatchdogInit();
	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
	Cache::Init();					// initialise the cache and/or the MPU, if applicable to this processor
	Cache::Disable();				// Make sure it is off to avoid DMA issues
    IrqEnable();
    SERIAL_MAIN_DEVICE.begin(9600);
	//delay(2000);
    debugPrintf("IAP running....\n");
	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
}

const SBCIAPParams *const GetParams()
{
	// RRF should have placed a parameter structure in an area at the top of RAM just above the start
	// of the stack.
	const uint32_t vtab = SCB->VTOR & SCB_VTOR_TBLOFF_Msk;
	const uint32_t stackTop = *reinterpret_cast<const uint32_t*>(vtab);
	const SBCIAPParams* const paramsPtr = reinterpret_cast<const SBCIAPParams*>(stackTop);

	// Check it is valid
	if (paramsPtr->sig1 != SBCIAPParamSig || paramsPtr->sig2 != SBCIAPParamSig)
	{
		debugPrintf("Invalid parameter block sig1 %x sig2 %x\n", (unsigned)paramsPtr->sig1, (unsigned)paramsPtr->sig2);
		return nullptr;
	}
	return paramsPtr;
}

// Application entry point
[[noreturn]] void AppMain() noexcept
{
	Init();

	const SBCIAPParams *const params = GetParams();
	if (params != nullptr)
	{
		HardwareSPI *dev = InitSPI(params->dev, params->clk, params->miso, params->mosi, params->cs, params->tfrRdy);
		int blockCnt = TransferDataToFlash(dev);
		if (blockCnt > 0) ProcessChecksum(dev, blockCnt);
		debugPrintf("IAP complete\n");
		digitalWrite(transferReadyPin, false);
		//transferReadyHigh = false;
		//digitalWrite(transferReadyPin, transferReadyHigh);
	}

	delay(1000);
	ResetProcessor();
}


// End
