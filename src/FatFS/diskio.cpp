/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
//SD :: Modified to work with RRF
//SD :: Updated for RTOS

#include "diskio.h"
#include <stdio.h>
#include <string.h>
#include "SDCard.h"
#include "sd_mmc_wrapper.h"

#include "RepRapFirmware.h"
constexpr unsigned int MaxSdCardTries = 5;
constexpr uint32_t SdCardRetryDelay = 20;


extern SDCard *_ffs[_DRIVES];


/* drv - Physical drive nmuber (0..) */
DSTATUS disk_initialize (BYTE drv) noexcept
{
	return (DSTATUS)_ffs[drv]->disk_initialize();
}

/* drv - Physical drive nmuber (0..) */
DSTATUS disk_status (BYTE drv) noexcept
{
	return (DSTATUS)_ffs[drv]->disk_status();
}

/* drv - Physical drive nmuber (0..) */
/* buff - Data buffer to store read data */
/* sector - Sector address (LBA) */
/* count - Number of sectors to read (1..255) */
DRESULT disk_read (BYTE drv, BYTE *buff, DWORD sector, BYTE count) noexcept
{
    unsigned int retryNumber = 0;
    uint32_t retryDelay = SdCardRetryDelay;
    for(;;)
    {
        DRESULT res = _ffs[drv]->disk_read(buff, sector, count);
        if (res == RES_OK) break;
        ++retryNumber;
        if (retryNumber == MaxSdCardTries)
        {
            delay(retryDelay);
            _ffs[drv]->disk_initialize();
        }           
        if (retryNumber > MaxSdCardTries)
        {
            return RES_ERROR;
        }
        delay(retryDelay);
        retryDelay *= 2;
    }
    return RES_OK;
}

#if _READONLY == 0
/* drv - Physical drive nmuber (0..) */
/* buff - Data to be written */
/* sector - Sector address (LBA) */
/* count - Number of sectors to write (1..255) */

DRESULT disk_write (BYTE drv, const BYTE *buff, DWORD sector, BYTE count) noexcept
{
    /* Write the data */
    unsigned int retryNumber = 0;
    uint32_t retryDelay = SdCardRetryDelay;
    for(;;)
    {
        DRESULT res = _ffs[drv]->disk_write(buff, sector, count);
        if (res == RES_OK) break;
        ++retryNumber;
        if (retryNumber == MaxSdCardTries)
        {
            delay(retryDelay);
            _ffs[drv]->disk_initialize();
        }           
        if (retryNumber > MaxSdCardTries)
        {
            return RES_ERROR;
        }
        delay(retryDelay);
        retryDelay *= 2;
    }
    return RES_OK;
}
#endif /* _READONLY */

/* drv - Physical drive nmuber (0..) */
/* ctrl - Control code */
/* buff - Buffer to send/receive control data */

DRESULT disk_ioctl (BYTE drv, BYTE ctrl, void *buff) noexcept
{
    //MutexLocker lock(Tasks::GetSpiMutex());
    return _ffs[drv]->disk_ioctl(ctrl, buff);
}
