
/**
 * \brief Current time returned is packed into a DWORD value.
 *
 * The bit field is as follows:
 *
 * bit31:25  Year from 1980 (0..127)
 *
 * bit24:21  Month (1..12)
 *
 * bit20:16  Day in month(1..31)
 *
 * bit15:11  Hour (0..23)
 *
 * bit10:5   Minute (0..59)
 *
 * bit4:0    Second/2 (0..29)
 *
 * \return Current time.
 */
#include "Core.h"
extern "C" uint32_t get_fattime() noexcept
{
		// Date and time have not been set, return default timestamp instead
		return 0x210001;
}

