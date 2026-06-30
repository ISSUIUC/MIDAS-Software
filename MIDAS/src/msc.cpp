#include "msc.h"

#include <Arduino.h>
#include "USB.h"
#include "USBMSC.h"
#include "SD_MMC.h"

#include "sdmmc_cmd.h"

static USBMSC usb_msc;
static bool g_mounted = false;
static sdmmc_card_t* g_card = nullptr;

struct SDMMCMSC : fs::SDMMCFS { sdmmc_card_t* c() { return _card; } };

bool msc_begin() {

    if (g_mounted) return true;
    g_card = (*((SDMMCMSC*)&SD_MMC)).c();
    if (!g_card) return false;

    uint64_t sec = SD_MMC.cardSize() / 512ULL;

    usb_msc.vendorID("MIDAS");
    usb_msc.productID("FLIGHTLOG");
    usb_msc.productRevision("1.0");

    usb_msc.onRead([](uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize) -> int32_t {
        (void)offset;
        return sdmmc_read_sectors(g_card, buffer, lba, bufsize / 512) == ESP_OK ? bufsize : -1;
    });

    usb_msc.onWrite([](uint32_t, uint32_t, uint8_t*, uint32_t) -> int32_t { return -1; });

    usb_msc.mediaPresent(true);

    USB.begin();
    usb_msc.begin(sec, 512);

    g_mounted = true;
    return true;
}

bool msc_end(void)
{
    if (!g_mounted) return true;

    usb_msc.end();

    SD_MMC.begin();

    g_mounted = false;
    return true;
}
