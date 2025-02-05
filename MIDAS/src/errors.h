#pragma once

/**
 * @enum ErrorCode
 * 
 * @brief list of all error codes that can arise
*/
enum ErrorCode {
    NoError = 0,
    LowGCouldNotBeInitialized = 1,
    SDBeginFailed = 2,
    LowGRangeCouldNotBeSet = 3,
    LowGODRLPFCouldNotBeSet = 4,
    HighGCouldNotBeInitialized = 5,
    HighGCouldNotUpdateDataRate = 6,
    MagnetometerCouldNotBeInitialized = 7,
    GyroCouldNotBeInitialized = 8,
    PyroGPIOCouldNotBeInitialized = 9,
    GPSCouldNotBeInitialized = 10,
    ContinuityCouldNotBeInitialized = 11,
    RadioInitFailed = 12,
    RadioSetFrequencyFailed = 13,
    CannotConnectBNO = 14,
    CannotInitBNO = 15,
    EmmcPinsAreWrong = 16,
    EmmcCouldNotBegin = 17,
    EmmcCouldNotOpenFile = 19,
    SDCouldNotOpenFile = 20,
    LoraCouldNotBeInitialized = 21,
    LoraCommunicationFailed = 22,
};

void update_error_LED(ErrorCode error);
