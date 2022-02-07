/*
 * SharedSpiDevice.h
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_
#define SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_

#include "RepRapFirmware.h"
#include "SpiMode.h"
#include "SPI.h"

class SharedSpiDevice
{
public:
	SharedSpiDevice(SSPChannel chan) noexcept;

	void Disable() const noexcept;
	void Enable() const noexcept;
	void SetClockFrequencyAndMode(uint32_t freq, SpiMode mode) const noexcept;

	// Send and receive data returning true if successful. Caller must already own the mutex and have asserted CS.
	bool TransceivePacket(const uint8_t *tx_data, uint8_t *rx_data, size_t len) const noexcept;

	// Get ownership of this SPI, return true if successful
	bool Take(uint32_t timeout) noexcept {return true;}

	// Release ownership of this SPI
	void Release() noexcept {}

	static void Init() noexcept;
	static SharedSpiDevice& GetSharedSpiDevice(SSPChannel chan) noexcept { return (chan < NumSPIDevices) ? *Devices[chan] : *invalidDevice;  }

private:
	bool waitForTxReady() const noexcept;
	bool waitForTxEmpty() const noexcept;
	bool waitForRxReady() const noexcept;

	SPI * const hardware;

	static SharedSpiDevice *Devices[];
	static SharedSpiDevice *invalidDevice;
};

#endif /* SRC_HARDWARE_SHAREDSPI_SHAREDSPIDEVICE_H_ */
