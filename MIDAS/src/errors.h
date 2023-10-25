#pragma once


enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    GyroCouldNotBeInitialized,
    CanBusMessageSendFailed,
};