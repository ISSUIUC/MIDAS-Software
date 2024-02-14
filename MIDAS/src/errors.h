#pragma once

enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    SDBeginFailed,
    LowGRangeCouldNotBeSet,
    LowGODRLPFCouldNotBeSet,
    HighGCouldNotBeInitialized,
    HighGCoulNotUpdateDataRate,
    MagnetometerCoultNotBeInitialized,
    GyroCouldNotBeInitialized,
    PyroGPIOCouldNotBeInitialized,
    GPSCouldNotBeInitialized,
    ContinuityCouldNotBeInitialized,
    CannotConnectBNO,
    CannotInitBNO,
    EmmcPinsAreWrong,
    EmmcCouldNotBegin,
    EmmcCouldNotOpenFile,
    PyroGPIOCouldNotBeInitialized
};

void update_error_LED(ErrorCode error);
