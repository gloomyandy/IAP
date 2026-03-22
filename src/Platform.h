#include <SPI/SharedSpiDevice.h>
class Platform final
{
public:
	static SharedSpiDevice& GetSharedSpiDevice(SSPChannel chan) noexcept { return *_ecv_not_null(SharedSpiDevices[chan]); }
	static void SetSharedSpiDevice(SSPChannel chan, SharedSpiDevice& device) { SharedSpiDevices[chan] = &device; }

private:
	static SharedSpiDevice *_ecv_null SharedSpiDevices[NumSPIDevices];
};