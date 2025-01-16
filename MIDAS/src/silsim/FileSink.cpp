#include "FileSink.h"
#include "iostream"

ErrorCode SDSink::init() {
    return ErrorCode::NoError;
}

void SDSink::write(const uint8_t* data, size_t size) {
    output_file->write(reinterpret_cast<const char*>(data), size);
}

SDSink::SDSink(const char* file_name) {
    // std::cerr << "Could not open file!\n" << std::flush;
    output_file = new std::ofstream();
    // std::cerr << "Could not open file!\n" << std::flush;
    output_file->open(file_name, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!output_file->is_open() || output_file->fail()) {
        std::cerr << "Could not open file!\n";
    }
}
