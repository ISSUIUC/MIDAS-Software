#pragma once


enum ErrorCode {
    NoError,
    LowGCouldNotBeInitialized,
    LowGRangeCouldNotBeSet,
    LowGODRLPFCouldNotBeSet,
    HighGCouldNotBeInitialized,
    HighGCoulNotUpdateDataRate,
    CannotConnectMagnetometer,
    GyroCouldNotBeInitialized,
    GasCouldNotBeInitialized
};