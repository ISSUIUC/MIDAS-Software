#include "ble.h"


class MIDASServerCB : public BLEServerCallbacks {
  void onDisconnect(BLEServer* s) override {
    s->getAdvertising()->start(); 
  }
};

MIDASServerCB cbs;

MIDASBLE::MIDASBLE() {

    #ifdef IS_SUSTAINER
        BLEDevice::init("MIDASBLE-S");
    #else
        BLEDevice::init("MIDASBLE-B");
    #endif

    BLEServer* sv = BLEDevice::createServer();
    BLEService* pservice = sv->createService(MIDAS_BLE_SERVICE_UUID);
    pchar = pservice->createCharacteristic(MIDAS_BLE_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    pchar->setWriteProperty(false);
    pchar->setNotifyProperty(true);
    pchar->setIndicateProperty(true);
    pchar->setReadProperty(true);
    
    sv->setCallbacks(&cbs);
    pservice->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(MIDAS_BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x0);
    pAdvertising->setMinPreferred(0x12);
}

void MIDASBLE::set_advertising(bool adv_en) {
    if (adv_en == cur_adv_state) { return; }
    cur_adv_state = adv_en;
    if(adv_en) {
        BLEDevice::startAdvertising();
        return;
    }
    BLEDevice::stopAdvertising();
}

BLEPacket MIDASBLE::make_packet(RocketData& data) {
    BLEPacket packet = BLEPacket();

    GPS gps = data.gps.getRecentUnsync();
    Voltage voltage = data.voltage.getRecentUnsync();
    Barometer barometer = data.barometer.getRecentUnsync();
    FSMState fsm = data.fsm_state.getRecentUnsync();
    Continuity continuity = data.continuity.getRecentUnsync();
    HighGData highg = data.high_g.getRecentUnsync();
    PyroState pyro = data.pyro.getRecentUnsync();
    Orientation orientation = data.orientation.getRecentUnsync();
    KalmanData kalman = data.kalman.getRecentUnsync();

    packet.lat = gps.latitude;
    packet.lon = gps.longitude;
    packet.gps_alt = (int16_t) gps.altitude;
    packet.baro_alt = (int16_t) barometer.altitude;
    packet.tilt = (int16_t) orientation.tilt;

    packet.highg_ax = (uint16_t) highg.ax;
    packet.highg_ay = (uint16_t) highg.ay;
    packet.highg_az = (uint16_t) highg.az;
    packet.batt_volt = (uint8_t) voltage.voltage;
    
    packet.pyro = 0;

    uint8_t sat_count = gps.fix_type;
    packet.fsm_callsign_satcount = ((uint8_t)fsm) | (sat_count << 4);
    float kf_vx_clamped = std::clamp(kalman.velocity.vx, -2000.f, 2000.f);

    #ifdef IS_SUSTAINER
    packet.fsm_callsign_satcount |= (1 << 7);
    #endif

    return packet;
}

void MIDASBLE::update_char_data(RocketData& rd) {

    BLEPacket p = make_packet(rd);
    uint8_t pbuf[ble_pkt_size];

    memcpy(pbuf, &p, ble_pkt_size);

    pchar->setValue(pbuf, ble_pkt_size);
}