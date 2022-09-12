#include "Core.h"
#include "SharedSpiDevice.h"

#include "SoftwareSPI.h"
#include "HardwareSPI.h"

constexpr uint32_t DefaultSharedSpiClockFrequency = 2000000;
constexpr uint32_t SpiTimeout = 10000;

// SharedSpiDevice members

SharedSpiDevice::SharedSpiDevice(SSPChannel chan) noexcept
    : hardware(SPI::getSSPDevice(chan))
{
}

// SharedSpiClient members

void SharedSpiDevice::Disable() const noexcept
{

}

void SharedSpiDevice::Enable() const noexcept
{

}

// Wait for transmitter ready returning true if timed out
inline bool SharedSpiDevice::waitForTxReady() const noexcept
{
	return false;
}

// Wait for transmitter empty returning true if timed out
inline bool SharedSpiDevice::waitForTxEmpty() const noexcept
{
	return false;
}

// Wait for receive data available returning true if timed out
inline bool SharedSpiDevice::waitForRxReady() const noexcept
{
	return false;
}

void SharedSpiDevice::SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept
{
	if (hardware != nullptr)
    	hardware->configureDevice(8, (uint8_t)mode, freq); 
}

bool SharedSpiDevice::TransceivePacket(const uint8_t* tx_data, uint8_t* rx_data, size_t len) const noexcept
{
	if (hardware != nullptr)
		return hardware->transceivePacket(tx_data, rx_data, len) == SPI_OK;
	else
	{
		debugPrintf("Warning: Attempt to use an undefined shared SPI device\n");
		return false;
	}
}

// Static members

SharedSpiDevice *SharedSpiDevice::Devices[NumSPIDevices];
SharedSpiDevice *SharedSpiDevice::invalidDevice;

void SharedSpiDevice::Init() noexcept
{
	for(size_t i = 0; i < NumSPIDevices; i++)
		Devices[i] = new SharedSpiDevice((SSPChannel)i);
	invalidDevice = new SharedSpiDevice(SSPNONE);
}
