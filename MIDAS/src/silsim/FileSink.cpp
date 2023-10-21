#include "FileSink.h"
#include "iostream"

ErrorCode LogSink::init() {
    return ErrorCode::NoError;
}

void LogSink::write(const uint8_t* data, size_t size) {
    output_file.write(reinterpret_cast<const char*>(data), size);
}

LogSink::LogSink(const char* file_name) {
    output_file.open(file_name, std::ios::out | std::ios::binary | std::ios::trunc);
}
