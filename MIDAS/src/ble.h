#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "rocket_state.h"

#define MIDAS_BLE_SERVICE_UUID "c4558284-48c6-4808-a4d4-437bacd0b2e2"
#define MIDAS_BLE_CHAR_UUID    "6e79501b-2bab-4504-9049-1ba734276cd4"

struct BLEPacket {
    int32_t lat;
    int32_t lon;
    uint16_t gps_alt;
    uint16_t baro_alt;
    uint16_t highg_ax; 
    uint16_t highg_ay; 
    uint16_t highg_az;
    uint16_t tilt;
    uint8_t batt_volt;
    uint8_t fsm_callsign_satcount; //4 bit fsm state, 1 bit is_sustainer_callsign, 3 bits sat count
    uint32_t pyro; // 8 bit continuity
};


class MIDASBLE {
    BLECharacteristic* pchar;
    bool cur_adv_state = false;
    BLEPacket make_packet(RocketData& data);

    public:
    MIDASBLE();
    void set_advertising(bool adv_en);
    void update_char_data(RocketData& rd);
};

constexpr size_t ble_pkt_size = sizeof(BLEPacket);