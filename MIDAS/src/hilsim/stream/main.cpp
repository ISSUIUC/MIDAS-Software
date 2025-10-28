#include <iostream>
#include <serialib.h>
#include <chrono>
#include <fstream>
#include <cstdint>
#include "sensor_data.h"
#include "log_format.h"
#include "crc.h"

size_t struct_sizes[16];

#define ASSOCIATE(ty, id) struct_sizes[id] = sizeof(ty)
#define DEBUG

FILE* inptr;

// Packet: $ uint32_t:timestamp uint8_t:disc uint8_t*:data uint16_t:crc
typedef struct {
    uint32_t ts;
    uint8_t disc;
    uint8_t* data;
    uint16_t crc;
    size_t _data_size;
    
    uint8_t raw_data_stream[150];
    size_t raw_data_stream_size;
} entry_t;

void print_usage() {
    std::cerr << "Usage: kserial <serial_port>" << std::endl;
}

int wrap_err(int code) {
    std::cerr << "The operation failed with a generic status code (" << code << ")" << std::endl;
    return code;
}

void setup_ssizes() {
    ASSOCIATE(LowGData, ID_LOWG);
    ASSOCIATE(LowGLSM, ID_LOWGLSM);
    ASSOCIATE(HighGData, ID_HIGHG);
    ASSOCIATE(Barometer, ID_BAROMETER);
    ASSOCIATE(Continuity, ID_CONTINUITY);
    ASSOCIATE(Voltage, ID_VOLTAGE);
    ASSOCIATE(GPS, ID_GPS);
    ASSOCIATE(Magnetometer, ID_MAGNETOMETER);
    ASSOCIATE(Orientation, ID_ORIENTATION);
    ASSOCIATE(FSMState, ID_FSM);
    ASSOCIATE(KalmanData, ID_KALMAN);
    ASSOCIATE(PyroState, ID_PYRO);
}

size_t get_size(uint8_t discriminant) {
    return struct_sizes[discriminant];
}

void resolve_data(entry_t& dat) {
    memcpy(dat.raw_data_stream, &dat.ts, sizeof(uint32_t));
    memcpy(dat.raw_data_stream + 4, &dat.disc, sizeof(uint8_t));
    memcpy(dat.raw_data_stream + 4 + 1, dat.data, dat._data_size);

    size_t raw_size = sizeof(uint32_t) + sizeof(uint8_t) + dat._data_size;
    dat.crc = calculateCRC16(dat.raw_data_stream, raw_size);
    dat.raw_data_stream_size = raw_size;
}

// Packet: $ uint32_t:timestamp uint8_t:disc uint8_t*:data uint16_t:crc
bool read_entry(entry_t& entry) {
    uint8_t payload[128];

    uint32_t disc_i;

    size_t b_read = fread(&disc_i, 4, 1, inptr);

    if(!b_read) {
        if(feof(inptr)) {
            std::cerr << "(S) Reached EOF early" << std::endl;
            return false;
        }
        if(ferror(inptr)) {
            std::cerr << "(S) Error reading file" << std::endl;
            return false;
        }
    }

    entry.disc = static_cast<uint8_t>(disc_i);
    fread(&entry.ts, sizeof(uint32_t), 1, inptr);

    size_t entry_size = get_size(entry.disc);
    entry._data_size = entry_size;

    fread(&payload, entry_size, 1, inptr);

    entry.crc = 0;
    entry.data = payload;


    resolve_data(entry);
    return true;
}

void handle_return()


void send_data(serialib& s, entry_t& dat) {
    // $ 4 bytes ts, 1 byte disc, n bytes payload, 2 bytes crc
    uint8_t buf[150];
    buf[0] = (uint8_t)'$';
    
    memcpy(buf + 1, &dat.raw_data_stream, dat.raw_data_stream_size);
    memcpy(buf + 1 + dat.raw_data_stream_size, &dat.crc, sizeof(uint16_t));

    #ifdef DEBUG
    printf("$");
    for(int i = 1; i < dat.raw_data_stream_size + 2; i++) {
        printf("%x", buf[i]);
    }
    // printf("\n");
    #else
    s.writeBytes(buf, dat._data_size + num_overhead_bytes);
    #endif
}

int main(int argc, char** argv) {
    
    if(argc != 2) {
        print_usage();
        return 1;
    }

    setup_ssizes();

    serialib Serial;

    char* serial_port = argv[1];
    printf("Attempting connection to %s\n", serial_port);
    // char serial_open_err = Serial.openDevice(serial_port, 115200);

    // if (serial_open_err != 1) return wrap_err(serial_open_err);

    printf("Successful connection to %s\n", serial_port);
    printf("Awaiting commands from stdin...\n");

    fflush(stdout);

    entry_t entry;
    size_t num_read = 0;

    auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now(); 

    while (true) {
        char cmd;
        scanf("%c", &cmd);

        switch(cmd) {
            case 'd':
                // debug: discriminant <num>
                int discrim_int;
                scanf(" %i", &discrim_int);

                printf("DISCRIM  (%i) %uB\n", discrim_int, get_size(static_cast<uint8_t>(discrim_int)));
                fflush(stdout);
                break;
            case 'l':
                // LOAD(file) <filename>
                char inbuf[255];
                scanf(" %s", &inbuf);

                printf("LOAD  %s\n", inbuf);
                inptr = fopen(inbuf, "r");
                
                if (inptr == NULL) {
                    std::cerr << "Error opening file " << inbuf << std::endl;
                    continue;
                }

                // We'll read the header datas first.
                uint32_t checksum;

                fread(&checksum, sizeof(uint32_t), 1, inptr);
                // memcpy(&checksum, buf, sizeof(uint32_t));

                printf("Read checksum: 0x%x\n", checksum);

                fflush(stdout);
                break;
            case 'n':
                // debug: NEXT(gets next line)
                
                if (inptr) {
                    if(read_entry(entry)) {
                        printf("DEBUG ENTRY [%u]: (%u) <size: %uB> (CRC 0x%x)\n", entry.ts, entry.disc, entry._data_size, entry.crc);
                        fflush(stdin);
                    } 
                } else {
                    std::cerr << "No file specified" << std::endl;
                }
                break;
            case 'N':
                // debug: NEXT <n> gets n lines
                int n;
                scanf(" %i", &n);
                
                if (inptr) {
                    for(int a = 0; a < n; a++) {
                        if(read_entry(entry)) {
                            printf("DEBUG ENTRY [%u]: (%u) <size: %uB> (CRC 0x%x)\n", entry.ts, entry.disc, entry._data_size, entry.crc);
                            fflush(stdin);
                        } 
                    } 
                } else {
                    std::cerr << "No file specified" << std::endl;
                }
                break;
            case 's':
                // s(tream) -- stream all data to com port as fast as possible
                while(read_entry(entry)) {
                    num_read++;
                    if(num_read % 10000 == 0) {
                        printf("[%u] {time: %u}: (d%u) <size: %uB> (CRC 0x%x)\n", num_read, entry.ts, entry.disc, entry._data_size, entry.crc);
                    }
                }
                break;
            case 'r':
                // r(ealtime) -- streams data to com port in real time
                {
                    uint32_t cur_entry_time = 0;
                    uint32_t first_entry_time = 0;
                    long long millis = 0;

                    // Read first entry and set it up
                    if(read_entry(entry)) {
                        cur_entry_time = entry.ts;
                        first_entry_time = entry.ts;
                    }

                    start_time = std::chrono::high_resolution_clock::now();

                    while (true) {
                        current_time = std::chrono::high_resolution_clock::now(); 
                        auto duration = current_time - start_time;
                        millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
                        
                        while(millis > (cur_entry_time - first_entry_time)) {
                            if(read_entry(entry)) {
                                num_read++;
                                cur_entry_time = entry.ts;
                                // printf("[%u] {time: %u / %u}: (d%u) <size: %uB> (CRC 0x%x)\n", num_read, entry.ts, millis, entry.disc, entry._data_size, entry.crc);
                                // fflush(stdout);
                                send_data(Serial, entry);
                            } else {
                                break;
                            }
                        }

                    }
                    printf("Done streaming!\n");

                    // while (true) {
                    //     while(cur_entry_time == 0 || millis > (cur_entry_time - first_entry_time)) {
                    //         bool res = read_entry(entry);

                    //         num_read++;
                    //         cur_entry_time = entry.ts;

                    //         if(first_entry_time == 0) {
                    //             first_entry_time = cur_entry_time;
                    //         }

                    //         
                            
                    //         send_data(Serial, entry, millis);

                    //         if(!res) {
                    //             break;
                    //         }
                    //     }

                    // }
                }
                break;
            case '\n':
                // discard
                break;
            default:
                std::cerr << "Invalid command " << cmd << std::endl;
        }

        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    // Serial.closeDevice();
    return 0;
}