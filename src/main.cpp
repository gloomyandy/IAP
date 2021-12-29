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

const uint8_t memPattern = 0xA5;		// this must be the same pattern as FreeRTOS because we use common code for checking for stack overflow

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


constexpr uint32_t SPITransferTimeout = 500;
uint8_t rxBuffer[IAP_BUFFER_SIZE];
uint8_t txBuffer[IAP_BUFFER_SIZE];
bool transferReadyHigh = false;
Pin transferReadyPin = PB_3;
bool dataReceived = false;

// interrupt handler
void SpiInterrupt(HardwareSPI *spi) noexcept
{
    dataReceived = true;
}


HardwareSPI *InitSPI(SSPChannel chan, Pin clk, Pin miso, Pin mosi) noexcept
{
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
	return dev;
}

spi_status_t SPITransfer(HardwareSPI *dev, const uint8_t *tx_data, uint8_t *rx_data, size_t len)
{
	uint32_t start = millis();
	dataReceived = false;
	dev->startTransfer(tx_data, rx_data, len, SpiInterrupt);
	// Tell the SBC we are ready to go
	transferReadyHigh = !transferReadyHigh;
	digitalWrite(PB_3, transferReadyHigh);
	while (!dataReceived && millis() - start < SPITransferTimeout)
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

int TransferData(HardwareSPI *dev)
{
	//SBC expects 0x1a to be sent back
	memset(txBuffer, 0x1a, sizeof(txBuffer));
	uint32_t retryCnt = 0;
	uint32_t blockCnt = 0;
	for(;;)
	{
		spi_status_t status = SPITransfer(dev, txBuffer, rxBuffer, sizeof(rxBuffer));
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
	}
}

bool ProcessChecksum(HardwareSPI *dev, int cnt) noexcept
{
	// wait fir dcs to be ready
	delay(1000);
	// Read the checksum
	spi_status_t status = SPITransfer(dev, txBuffer, rxBuffer, sizeof(FlashVerifyRequest));
	if (status != SPI_OK)
	{
		debugPrintf("Error reading CRC %d\n", (int) status);
		return false;
	}
	const FlashVerifyRequest *request = reinterpret_cast<const FlashVerifyRequest*>(rxBuffer);
	debugPrintf("Got verify request flash len %d CRC %x\n", request->firmwareLength, request->crc16);
	// send Ok response
	txBuffer[0] = 0x0c;
	SPITransfer(dev,txBuffer, rxBuffer, 1);
	return true;
}


// Application entry point
[[noreturn]] void AppMain() noexcept
{
	__asm volatile(
					" cpsie i				\n" /* Globally enable interrupts. */
					" cpsie f				\n"
				);
#if 0
	// Fill the free memory with a pattern so that we can check for stack usage and memory corruption
	char *_ecv_array heapend = heapTop;
	const char *_ecv_array stack_ptr = (const char*_ecv_array)GetStackPointer();
	while (heapend + 16 < stack_ptr)
	{
		*heapend++ = memPattern;
	}
#endif

	CoreInit();
	// Initialise systick (needed for delay calls) - CoreNG initialises it in non-interrupt mode
	SysTick->LOAD = ((SystemCoreClockFreq/1000) - 1) << SysTick_LOAD_RELOAD_Pos;
	SysTick->CTRL = (1 << SysTick_CTRL_ENABLE_Pos) | (1 << SysTick_CTRL_TICKINT_Pos) | (1 << SysTick_CTRL_CLKSOURCE_Pos);
	NVIC_SetPriority (SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL); /* set Priority for Systick Interrupt */
    WatchdogInit();
	transferReadyHigh = false;
	digitalWrite(PB_3, transferReadyHigh);
    IrqEnable(); 
    SERIAL_MAIN_DEVICE.begin(9600);
	delay(2000);
    debugPrintf("IAP running....");
	// Trap integer divide-by-zero.
	// We could also trap unaligned memory access, if we change the gcc options to not generate code that uses unaligned memory access.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	Cache::Init();					// initialise the cache and/or the MPU, if applicable to this processor
	Cache::Disable();
	HardwareSPI *dev = InitSPI(SSPChannel::SSP2, PB_13, PB_14, PB_15);
	digitalWrite(PB_3, transferReadyHigh);
	int blockCnt = TransferData(dev);
	if (blockCnt > 0) ProcessChecksum(dev, blockCnt);
	debugPrintf("IAP complete\n");
	transferReadyHigh = false;
	digitalWrite(PB_3, transferReadyHigh);
	delay(1000);
	ResetProcessor();
}


// End