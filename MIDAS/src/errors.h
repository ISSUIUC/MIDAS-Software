#pragma once

enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    SDBeginFailed,
    LowGRangeCouldNotBeSet,
    LowGODRLPFCouldNotBeSet,
    HighGCouldNotBeInitialized,
    HighGCouldNotUpdateDataRate,
    MagnetometerCouldNotBeInitialized,
    GyroCouldNotBeInitialized,
    PyroGPIOCouldNotBeInitialized,
    GPSCouldNotBeInitialized,
    ContinuityCouldNotBeInitialized,
    RadioInitFailed,
    RadioSetFrequencyFailed,
    CannotConnectBNO,
    CannotInitBNO
};

void update_error_LED(ErrorCode error);
