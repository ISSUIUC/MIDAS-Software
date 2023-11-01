#pragma once


enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    LowGRangeCouldNotBeSet,
    LowGODRLPFCouldNotBeSet,
    HighGCouldNotBeInitialized,
    HighGCoulNotUpdateDataRate,
    MagnetometerCoultNotBeInitialized,
    GyroCouldNotBeInitialized,
    GPSCouldNotBeInitialized
};