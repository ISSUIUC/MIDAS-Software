#pragma once


enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    HighGCouldNotBeInitialized,
    HighGCoulNotUpdateDataRate,
    CannotConnectMagnetometer,
    GyroCouldNotBeInitialized,
    GasCouldNotBeInitialized
};