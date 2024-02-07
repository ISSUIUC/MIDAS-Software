#pragma once

#include "../lib/TCAL9539/TCAL9539.h"

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
    EmmcPinsAreWrong,
    EmmcCouldNotBegin,
    EmmcCouldNotOpenFile
};

void update_error_LED(ErrorCode error);
