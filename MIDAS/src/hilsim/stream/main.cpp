#include <iostream>
#include <serialib.h>
#include <chrono>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <cstdlib>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <string>

#include "sensor_data.h"
#include "log_format.h"
#include "crc.h"

#define STR2(x) #x
#define STR(x)  STR2(x)
#pragma message "__cplusplus=" STR(__cplusplus)

size_t struct_sizes[READING_DISC_COUNT];
bool ignore_disc[READING_DISC_COUNT];


#define ASSOCIATE(ty, id) struct_sizes[id] = sizeof(ty)
// #define DEBUG

FILE* inptr;
FILE* outptr;

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
    std::cerr << "Usage: kserial    (NO ARGS)" << std::endl;
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
    ASSOCIATE(CameraData, ID_CAMERADATA);
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

class InputReader {

    private:

    std::queue<std::string> _q;
    std::istream& _in;
    std::thread _th;
    std::mutex _mut;
    std::condition_variable _cv;
    bool _stop = false;

    void run() {
        std::string line;

        while (std::getline(_in, line)) {
            { std::lock_guard<std::mutex> lk(_mut); _q.push(line); }
            _cv.notify_one();
        }
        // EOF or error: mark stop so mainside can finish if desired
        { std::lock_guard<std::mutex> lk(_mut); _stop = true; }
        _cv.notify_all();
    }

    public:
    explicit InputReader(std::istream& in) : _in(in), _th([this]{ run(); }) {}
    ~InputReader() {
        {
            std::lock_guard<std::mutex> lk(_mut); _stop = true;
        }
        _cv.notify_all();
        _th.join();
    }

    bool get(std::string& out) {
        std::lock_guard<std::mutex> lk(_mut);
        if (_q.empty()) { return false; }
        out = std::move(_q.front());
        _q.pop();
        return true;
    }
};



void send_data(serialib& s, entry_t& dat) {
    // $ 4 bytes ts, 1 byte disc, n bytes payload, 2 bytes crc
    size_t size_of_send_buf = 0;
    uint8_t buf[150];
    buf[0] = (uint8_t)'$'; size_of_send_buf += 1;
    
    memcpy(buf + 1, &dat.raw_data_stream, dat.raw_data_stream_size); size_of_send_buf += dat.raw_data_stream_size;
    memcpy(buf + 1 + dat.raw_data_stream_size, &dat.crc, sizeof(uint16_t)); size_of_send_buf += sizeof(uint16_t);

    if(outptr) {
        fputs(".D $", outptr);
        for(int i = 1; i < dat.raw_data_stream_size + 2; i++) {
            fprintf(outptr, "%x", buf[i]);
        }
        fputc('\n', outptr);
        fflush(outptr);
    }

    #ifdef DEBUG
    // printf("$");
    // for(int i = 1; i < dat.raw_data_stream_size + 2; i++) {
    //     printf("%x", buf[i]);
    // }
    // printf("\n");
    #else
    s.writeBytes(buf, size_of_send_buf);
    #endif
}

int main(int argc, char** argv) {
    
    if(argc != 1) {
        print_usage();
        return 1;
    }

    setup_ssizes();

    serialib Serial;

    printf("Awaiting commands from stdin...\n");

    fflush(stdout);

    entry_t entry;
    size_t num_read = 0;
    uint32_t _inf_checksum;

    float skip_threshold = 0;

    auto start_ts = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now(); 

    InputReader _ireader(std::cin);
    std::string line;

    while (true) {

        // handle serial output
        if(Serial.isDeviceOpen()) {
            // do something

            // TBD
        }
        
        // Handle inputs from stdin
        if(!_ireader.get(line)) {
            continue;
        } 

        char _inbuf[255];
        memcpy(_inbuf, line.c_str(), line.length() + 2);

        for(int i = 0; i < READING_DISC_COUNT; i++) {
            ignore_disc[i] = false;
        }

        // Command list
        // S <COMport:str> - Set serial output
        // o <Filepath:str> - Set output log file
        // d <disc_id:int> - [DEBUG] get size of discrim
        // l <Filepath:str> - load file for streaming
        // n - [DEBUG] gets next line and prints
        // N <num_lines:int> - Skips num_lines from the input
        // s - Stream data as fast as possible
        // r - Stream data in realtime

        // i <disc_id:int> - Ignore this disc_id when streaming
        // R <disc-id:int> <intv:float> - Enable this sensor report.
        // T <filter_thresh:float> - Entries will be probabilistically skipped if there is high latency. This sets the threshold (in seconds) at which 100% will be skipped.
        //    If unset / 0, then no entries are skipped


        switch(_inbuf[0]) {
            case 'i':
                    {
                    int discrim_int;
                    sscanf(_inbuf + 1, " %i", &discrim_int);

                    if(discrim_int >= 1 && discrim_int < READING_DISC_COUNT) {
                        ignore_disc[discrim_int] = true;
                        printf(".IGNORE %i\n", discrim_int);
                    }

                    fflush(stdout);
                }

                break;
            case 'T':
                {
                    float n;
                    sscanf(_inbuf + 1, " %f", &n);
                    if (n >= 0 && n < 100) {
                        skip_threshold = n;
                    }
                    printf(".SKIP_T %f\n", skip_threshold);
                    fflush(stdout);
                }
                break;

            case 'S':
                // (S)erial <COM>: Set serial port
                {
                    char sbuf[128];
                    sscanf(_inbuf + 1, " %s", &sbuf);
                    char serial_open_err = Serial.openDevice(sbuf, 921600);
                    if (serial_open_err != 1) {
                        printf(".SERIAL_BAD\n");
                    } else {
                        printf(".SERIAL_OK\n");
                    }
                    fflush(stdout);
                }
                break;
            case 'o':
                // (o)utfile <filepath>: Set data output path
                char sbuf[240];
                sscanf(_inbuf + 1, " %s", &sbuf);
                outptr = fopen(sbuf, "w+");
                
                {
                if (outptr == NULL) {
                    std::cerr << "Error opening file " << sbuf << std::endl;
                    continue;
                }

                printf(".OUTF %s\n", sbuf);

                fflush(stdout);
                }
                break;
            case 'd':
                // debug: discriminant <num>
                int discrim_int;
                sscanf(_inbuf + 1, " %i", &discrim_int);

                printf("DISCRIM  (%i) %uB\n", discrim_int, get_size(static_cast<uint8_t>(discrim_int)));
                fflush(stdout);
                break;
            case 'l':
                // LOAD(file) <filename>
                char inbuf[255];
                sscanf(_inbuf + 1, " %s", &inbuf);

                printf("LOAD  %s\n", inbuf);
                inptr = fopen(inbuf, "rb");
                
                if (inptr == NULL) {
                    std::cerr << "Error opening file " << inbuf << std::endl;
                    continue;
                }

                // We'll read the header datas first.
                uint32_t checksum;

                fread(&checksum, sizeof(uint32_t), 1, inptr);
                // memcpy(&checksum, buf, sizeof(uint32_t));

                printf(".CHECKSUM %x\n", checksum);
                _inf_checksum = checksum;

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
                // NEXT <n> skips n lines
                int n;
                sscanf(_inbuf + 1, " %i", &n);
                
                if (inptr) {
                    for(int a = 0; a < n; a++) {
                        if(read_entry(entry)) {
                            // printf("DEBUG ENTRY [%u]: (%u) <size: %uB> (CRC 0x%x)\n", entry.ts, entry.disc, entry._data_size, entry.crc);
                        }
                    } 
                } else {
                    std::cerr << "No file specified" << std::endl;
                }
                fflush(stdout);
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
                    fflush(stdout);
                    if (!inptr) {
                        std::cerr << "No file specified" << std::endl;
                        printf(".ERR no_ifs\n");
                        fflush(stdout);
                        break;
                    }

                    if (!Serial.isDeviceOpen()) {
                        std::cerr << "No serial open" << std::endl;
                        printf(".ERR no_ser\n");
                        fflush(stdout);
                        break;
                    }

                    printf(".BEGIN\n");
                    fflush(stdout);

                    if (outptr) {
                        fputs(".KAMAJI_STREAM\n", outptr);
                        fprintf(outptr, ".CHECKSUM %X\n", _inf_checksum);
                        fputs(".BEGIN_RUN:\n", outptr);
                        fflush(outptr);
                    }

                    int rows_read = 0;

                    uint32_t next_ts = 0;
                    uint32_t start_ts = 0;

                    // Read first entry and set it up
                    if(read_entry(entry)) {
                        start_ts = entry.ts;
                        next_ts = entry.ts;
                    }


                    long long millis = 0;
                    auto start_epoch = std::chrono::high_resolution_clock::now();

                    while (true) {

                        auto time_since_stream_start_ = std::chrono::high_resolution_clock::now() - start_epoch;
                        long long time_since_stream_start = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_stream_start_).count();
                        auto time_elapsed_in_log = next_ts - start_ts;

                        while (time_since_stream_start > time_elapsed_in_log) {
                            if(read_entry(entry)) {
                                rows_read++;

                                next_ts = entry.ts;
                                time_elapsed_in_log = next_ts - start_ts;

                                auto time_since_stream_start_ = std::chrono::high_resolution_clock::now() - start_epoch;
                                long long time_since_stream_start = std::chrono::duration_cast<std::chrono::milliseconds>(time_since_stream_start_).count();

                                auto latency = time_since_stream_start - time_elapsed_in_log;

                                if(!ignore_disc[entry.disc]) {
                                    float randm = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
                                    int threshold_ms = (int)(skip_threshold * 1000);

                                    if(latency < randm*threshold_ms) {
                                        send_data(Serial, entry);
                                    }
                                }

                                if (rows_read % 200 == 0) {
                                    printf("%i %i\n", time_since_stream_start, time_elapsed_in_log);
                                    fflush(stdout);
                                } 


                            } else {
                                exit(2);
                            }

                        }

                        // current_time = std::chrono::high_resolution_clock::now(); 
                        // auto duration = current_time - start_ts;
                        // millis = 
                        
                        // while(millis > (cur_entry_time - first_entry_time)) {
                        //     if(read_entry(entry)) {
                        //         num_read++;
                        //         cur_entry_time = entry.ts;

                        //         current_time = std::chrono::high_resolution_clock::now(); 
                        //         duration = current_time - start_ts;
                        //         millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

                        //         auto latency = millis - (cur_entry_time - first_entry_time);
                        //         printf(".r %i\n", latency);
                        //         fflush(stdout);

                        //         // check filter
                        //         if(!ignore_disc[entry.disc]) {
                        //             float randm = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
                        //             int threshold_ms = (int)(skip_threshold * 1000);

                        //             if(latency < randm*threshold_ms) {
                        //                 send_data(Serial, entry);
                        //             }
                        //         }

                        //     } else {
                        //         break;
                        //     }
                        // }

                    }
                    printf(".DONE\n");

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
            default:
                std::cerr << "Invalid command " << _inbuf[0] << std::endl;
        }

        
    }

    // Serial.closeDevice();
    return 0;
}