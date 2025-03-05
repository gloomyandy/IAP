/*
 * Can.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 * Modified for STM32 by Andy 22/01/2025
 */

#define SUPPORT_CAN		1		// needed by CanDevice.h
#include "CanInterface.h"

#include <CanSettings.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>

#include <CanDevice.h>

static CanDevice *can0dev = nullptr;

static CanAddress boardAddress;

constexpr unsigned int CanDeviceNumber = 0;			// we use FDCAN1 (which is device 0)
#if STM32H7
constexpr Pin CanReadPin = CAN_READ;
constexpr Pin CanWritePin = CAN_WRITE;
#endif

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
	.numTxBuffers = 2,
	.txFifoSize = 4,
	.numRxBuffers = 0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 3,
	.txEventFifoSize = 2
};

static_assert(Can0Config.IsValid());

// Initialise the CAN interface
void CanInterface::Init(CanAddress defaultBoardAddress)
{
	CanTiming timing;
	timing.SetDefaults_1Mb();									// we only support default timing when a main board is used as an expansion board

	// Set up the CAN pins

	// Initialise the CAN hardware, using the timing data if it was valid
#if STM32H7
	can0dev = CanDevice::Init(0, CanDeviceNumber, Can0Config, nullptr, timing, nullptr, CanReadPin, CanWritePin);
#else
	can0dev = CanDevice::Init(0, CanDeviceNumber, Can0Config, nullptr, timing, nullptr);
#endif

	boardAddress = defaultBoardAddress;

	// Set up a CAN receive filter to receive all messages addressed to us in FIFO 0
	can0dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										(uint32_t)boardAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);
	// We ignore broadcast messages so no need to set up a filter for them
	can0dev->Enable();
}

// Close down the CAN interface
void CanInterface::Shutdown()
{
	if (can0dev != nullptr)
	{
		can0dev->DeInit();
	}
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

// Get a received CAN message if there is one
bool CanInterface::GetCanMessage(CanMessageBuffer *buf)
{
	return can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, 0, buf);
}

// Send a CAN message and free the buffer
void CanInterface::Send(CanMessageBuffer *buf)
{
	(void)can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
}

// End
