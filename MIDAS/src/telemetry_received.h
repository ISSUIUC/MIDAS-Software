
/**
 * @struct TelemetryReceivedData
 * 
 * @brief format of the packet that telemetry receives
*/
struct TelemetryReceivedData {
    bool kalmanFilterReset;
    // TODO: Add frequency setting
    // TODO: Add changing call sign
};