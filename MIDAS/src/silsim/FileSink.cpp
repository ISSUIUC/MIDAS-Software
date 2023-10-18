#include "FileSink.h"
#include "iostream"

ErrorCode LogSink::init() {
    return ErrorCode::NoError;
}

void LogSink::write(const uint8_t* data, size_t size) {
    output_file.write(reinterpret_cast<const char*>(data), size);
}