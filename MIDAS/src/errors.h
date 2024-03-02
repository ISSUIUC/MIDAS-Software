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
    CannotInitBNO,
    EmmcPinsAreWrong,
    EmmcCouldNotBegin,
    EmmcCouldNotOpenFile
};

void update_error_LED(ErrorCode error);
