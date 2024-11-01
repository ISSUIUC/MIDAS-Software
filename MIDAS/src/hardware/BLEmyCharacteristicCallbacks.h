#include <BLECharacteristic.h>
#include <Arduino.h>
#include <rocket_commands.h>





class BLEmyCharacteristicCallbacks : public BLECharacteristicCallbacks{

    void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) override
    {
        Serial.println("onRead function");
    }

    void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) override
    {
        // btRadioSettings settings;
        // memcpy(&settings, param->write.value, sizeof(settings));
        // Serial.print("freq: ");
        // Serial.println(settings.freq);
        
        // Serial.print("signalBandwidth: ");
        // Serial.println(settings.signalBandwidth);

        // Serial.print("codingRate4: ");
        // Serial.println(settings.codingRate4);

        // Serial.print("spreadingFactor: ");
        // Serial.println(settings.spreadingFactor);

        // Serial.print("payloadCrc: ");
        // Serial.println(settings.payloadCRC);

        secondStageThresholds thresholds;
        memcpy(&thresholds, param->write.value, sizeof(thresholds));

    }


};


