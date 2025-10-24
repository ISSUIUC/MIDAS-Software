#include <Arduino.h>
#include <systems.h>
#include "kal.h"

MultipleLogSink<> sink;
RocketSystems systems{.log_sink = sink};

static bool read_exact(uint8_t* dst, size_t n, unsigned long timeout_ms) {
  unsigned long start = millis();
  size_t got = 0;
  while (got < n) {
    int avail = Serial.available();
    if (avail > 0) {
      int r = Serial.readBytes(dst + got, n - got);
      got += (r > 0 ? (size_t)r : 0);
      continue;
    }
    if ((millis() - start) > timeout_ms) return false;
    THREAD_SLEEP(1); // yield
  }
  return true;
}

DECLARE_THREAD(hilsim, void*arg) {
    int n = 0;
    // Debug kamaji output to verify if we're reading the correct packets
    while (Serial.read() != 33);
    char magic[] = {69, 110, 117, 109, 99, 108, 97, 119, 0};
    Serial.println(magic);
    Serial.println(__TIME__);
    Serial.println(__DATE__);
    Serial.flush();

    uint8_t header[5];
    uint8_t crc_buf[2];
    uint32_t timestamp;
    uint8_t discriminator;
    size_t data_size;
    uint8_t data_buf[128];
    uint16_t crc;
    
    while (true) {

        int delim;
        do {
            while (!Serial.available()) {
              k_tick();
            }
            delim = Serial.read();

        } while(delim != '$' && delim != '#');

        if(delim == '#') {
          if (!read_exact(data_buf, 16, 10)) continue;
          // Sys message is 14 byte data, 2 byte crc  
          memcpy(&crc, data_buf + 14, sizeof(crc_buf));
          k_handle_sys_msg(data_buf, crc);
          continue;
        }
            
        if(!read_exact(header, sizeof(header), 10)) continue;
        memcpy(&timestamp, &header[0], 4);
        discriminator = header[4];
        data_size = k_get_discriminant_size(discriminator);

        if (!read_exact(data_buf, (size_t)data_size, 10)) continue;
        if (!read_exact(crc_buf, sizeof(crc_buf), 5)) continue;

        memcpy(&crc, crc_buf, sizeof(crc_buf));

        k_handle_reading(timestamp, discriminator, data_buf, data_size, crc);

        k_tick();
        THREAD_SLEEP(1);
    }
}

void setup() {
    Serial.begin(9600);
    k_setup();
    hilsim_thread(nullptr);
}

void loop(){}
