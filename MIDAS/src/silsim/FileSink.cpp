#include "FileSink.h"
#include "iostream"

ErrorCode SDSink::init() {
    return ErrorCode::NoError;
}

void SDSink::write(const uint8_t* data, size_t size) {
    output_file.write(reinterpret_cast<const char*>(data), size);
}

SDSink::SDSink(const char* file_name) {
    output_file.open(file_name, std::ios::out | std::ios::binary | std::ios::trunc);
}
