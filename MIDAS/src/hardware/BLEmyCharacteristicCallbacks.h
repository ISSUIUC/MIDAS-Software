#include <BLECharacteristic.h>
#include <Arduino.h>
#include <rocket_commands.h>

class BLEmyCharacteristicCallbacks : public BLECharacteristicCallbacks{

    BLEmyCharacteristicCallbacks(RocketSystems& systems) : refSystems(systems) {
        
    }

    void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) override
    {
       // Serial.println("onRead function");
        // Serial.flush();
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
        // memcpy(&thresholds, param->write.value, sizeof(thresholds));

        // Serial.print("thresholds variable 1:  sustainer_idle_to_first_boost_acceleration_threshold");
        // Serial.println(thresholds.sustainer_idle_to_first_boost_acceleration_threshold);
        // Serial.print("Raw BLE Write Value: ");
        // for (int i = 0; i < param->write.len; i++) {
        //     Serial.print(param->write.value[i], HEX);  // Print each byte in hex format
        //     Serial.print(" ");
        // }
        // Serial.println();


        // while(true){
            //ASK WHAT THRESHOLD
            // memcpy(&thresholds, param->write.value, sizeof(thresholds));
            int thresholdCode = param->write.value[0]; //first byte is threshold, ex) 0x01
            double newValue = 0;
            // Serial.flush();

            /*Notes from 11/9/24*/
            // NOTE: write in little endian (msb is at the end, not the front) (so 01 (for which case) then in backwards per two characters (in hex) we write the double value we want (IEEE floating point))

            //SEE systems.cpp to see how thresholds transferred
                //change switch statement, instead of updating every threshold here, we can do a switch statement inside fsm.cpp to update the certain threshold we want


            switch (thresholdCode) {
                case 0x01:
                    Serial.println("sustainer_idle_to_first_boost_acceleration_threshold: ");
                    memcpy(&newValue, param->write.value + 1, sizeof(double));
                    thresholds.sustainer_idle_to_first_boost_acceleration_threshold = newValue;
                    // Serial.println(param->write.len);
                    // for (unsigned i = 0; i < sizeof(double); ++i) {
                    //     Serial.println((int)param->write.value[i+1]);
                    // }
                    Serial.println(thresholds.sustainer_idle_to_first_boost_acceleration_threshold);
                    
                    rocketCommands rktStruct;
                    rktStruct.data.thresholds = thresholds;
                    refSystems.rocket_data.BluetoothCommands.send(rktStruct);
                    break;

                case 0x02:
                    Serial.print("sustainer_idle_to_first_boost_time_threshold: ");
                    Serial.println(thresholds.sustainer_idle_to_first_boost_time_threshold);
                    break;
                case 0x03:
                    Serial.print("sustainer_ignition_to_second_boost_acceleration_threshold: ");
                    Serial.println(thresholds.sustainer_ignition_to_second_boost_acceleration_threshold);
                    break;
                case 0x04:
                    Serial.print("sustainer_second_boost_to_coast_time_threshold: ");
                    Serial.println(thresholds.sustainer_second_boost_to_coast_time_threshold);
                    break;
                case 0x05:
                    Serial.print("sustainer_coast_detection_acceleration_threshold: ");
                    Serial.println(thresholds.sustainer_coast_detection_acceleration_threshold);
                    break;
                case 0x06:
                    Serial.print("sustainer_coast_to_apogee_vertical_speed_threshold: ");
                    Serial.println(thresholds.sustainer_coast_to_apogee_vertical_speed_threshold);
                    break;
                case 0x07:
                    Serial.print("sustainer_apogee_check_threshold: ");
                    Serial.println(thresholds.sustainer_apogee_check_threshold);
                    break;
                case 0x08:
                    Serial.print("sustainer_apogee_timer_threshold: ");
                    Serial.println(thresholds.sustainer_apogee_timer_threshold);
                    break;
                case 0x09:
                    Serial.print("sustainer_drogue_timer_threshold: ");
                    Serial.println(thresholds.sustainer_drogue_timer_threshold);
                    break;
                case 0x0A:
                    Serial.print("sustainer_main_to_main_deploy_timer_threshold: ");
                    Serial.println(thresholds.sustainer_main_to_main_deploy_timer_threshold);
                    break;
                case 0x0B:
                    Serial.print("sustainer_main_deploy_altitude_threshold: ");
                    Serial.println(thresholds.sustainer_main_deploy_altitude_threshold);
                    break;
                case 0x0C:
                    Serial.print("sustainer_ignition_to_second_boost_time_threshold: ");
                    Serial.println(thresholds.sustainer_ignition_to_second_boost_time_threshold);
                    break;
                case 0x0D:
                    Serial.print("sustainer_ignition_to_coast_timer_threshold: ");
                    Serial.println(thresholds.sustainer_ignition_to_coast_timer_threshold);
                    break;
                case 0x0E:
                    Serial.print("sustainer_landed_timer_threshold: ");
                    Serial.println(thresholds.sustainer_landed_timer_threshold);
                    break;
                case 0x0F:
                    Serial.print("sustainer_first_boost_to_burnout_time_threshold: ");
                    Serial.println(thresholds.sustainer_first_boost_to_burnout_time_threshold);
                    break;
                case 0x10:
                    Serial.print("sustainer_landed_vertical_speed_threshold: ");
                    Serial.println(thresholds.sustainer_landed_vertical_speed_threshold);
                    break;
                case 0x11:
                    Serial.print("sustainer_drogue_jerk_threshold: ");
                    Serial.println(thresholds.sustainer_drogue_jerk_threshold);
                    break;
                case 0x12:
                    Serial.print("sustainer_main_jerk_threshold: ");
                    Serial.println(thresholds.sustainer_main_jerk_threshold);
                    break;
                default:
                    Serial.println("Unknown threshold code");
                    break;
            // }

            //WHAT VALUE IN THRESHOLD



        }
        
    }

    RocketSystems &refSystems;


};


