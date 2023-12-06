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
    GasCouldNotBeInitialized,
    GPSCouldNotBeInitialized,
    ContinuityCouldNotBeInitialized,
    RadioInitFailed,
    RadioSetFrequencyFailed,
    CannotConnectBNO,
    CannotInitBNO
};
