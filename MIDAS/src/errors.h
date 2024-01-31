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
    GasCouldNotBeInitialized,
    GPSCouldNotBeInitialized,
    ContinuityCouldNotBeInitialized,
    RadioInitFailed,
    RadioSetFrequencyFailed,
    CannotConnectBNO,
    CannotInitBNO
};

void update_error_LED(ErrorCode error);
