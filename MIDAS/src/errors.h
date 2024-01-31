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
    GPSCouldNotBeInitialized,
    ContinuityCouldNotBeInitialized,
    CannotConnectBNO,
    CannotInitBNO,
    CanBusMessageSendFailed
};

void update_error_LED(ErrorCode error);
