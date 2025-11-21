/// @brief LSM6DSV320X classes for the modules it replaces

enum LSM6DSV320X_Regs {
    // Enable Embedded Functions Register
    // ---------------------------------------------
    LSM6DSV320X_FUNC_CFG_ACCESS = 0x01,
    // ---------------------------------------------

    // SDO, OCS_aux, SDO_aux Pins Pull UP Register
    // ---------------------------------------------
    LSM6DSV320X_PIN_CTRL = 0x02,
    // ---------------------------------------------

    // Interface Conifguration Register
    // ---------------------------------------------
    LSM6DSV320X_IF_CFG = 0x03,
    // ---------------------------------------------

    // RESERVED 0x04-0x05

    // ODR-triggered mode config register
    // ---------------------------------------------
    LSM6DSV320X_ODR_TRIG_CFG = 0x06,
    // ---------------------------------------------

    // FIFO Control Registers
    // ---------------------------------------------
    LSM6DSV320X_FIFO_CTRL1 = 0x07,
    LSM6DSV320X_FIFO_CTRL2 = 0x08,
    LSM6DSV320X_FIFO_CTRL3,
    LSM6DSV320X_FIFO_CTRL3,
    LSM6DSV320X_FIFO_CTRL4,
    // ---------------------------------------------

    // Counter Batch Data Register
    // ---------------------------------------------
    LSM6DSV320X_COUNTER_BDR_REG1,
    LSM6DSV320X_COUNTER_BDR_REG2,
    // ---------------------------------------------

    LSM6DSV320X_INT1_CTRL,  // Interrupt Pin Controls 1 & 2
    LSM6DSV320X_INT2_CTRL,

    LSM6DSV320X_WHO_AM_I,

    LSM6DSV320X_CTRL1, 
    LSM6DSV320X_CTRL2,
    LSM6DSV320X_CTRL3,
    LSM6DSV320X_CTRL4,
    LSM6DSV320X_CTRL5,
    LSM6DSV320X_CTRL6, 
    LSM6DSV320X_CTRL7,
    LSM6DSV320X_CTRL8,
    LSM6DSV320X_CTRL9,
    LSM6DSV320X_CTRL10,

    LSM6DSV320X__CTRL_STATUS,
    LSM6DSV320X_FIFO_STATUS1,
    LSM6DSV320X_FIFO_STATUS2,
    LSM6DSV320X_ALL_INT_SRC,
    LSM6DSV320X_STATUS_REG,

    // RESERVED 0x1f

 
    // Temperature Out Sensors
    // ---------------------------------------------
    LSM6DSV320X_OUT_TEMP_L = 0x20,
    LSM6DSV320X_OUT_TEMP_H,
    // ---------------------------------------------

 
    // Angular rate sensor pitches (pitch, roll, and yaw, respectively)
    // All of them are grouped in pairs
    // ---------------------------------------------
    LSM6DSV320X_OUTX_L_G = 0x22,
    LSM6DSV320X_OUTX_H_G,

    LSM6DSV320X_OUTY_L_G,
    LSM6DSV320X_OUTY_H_G,

    LSM6DSV320X_OUTZ_L_G,
    LSM6DSV320X_OUTZ_H_G,
    // ---------------------------------------------

    // Low-G Linear Acceleration Register Outputs
    // ---------------------------------------------
    LSM6DSV320X_OUTX_L_A = 0x28,
    LSM6DSV320X_OUTX_H_A,
    LSM6DSV320X_OUTY_L_A,
    LSM6DSV320X_OUTY_H_A,
    LSM6DSV320X_OUTZ_L_A,
    LSM6DSV320X_OUTZ_L_A,
    // ---------------------------------------------
    // i gave up on the spacers

    // Angular Rate Sensor Pitch, Roll, Yaw output registers
    LSM6DSV320X_UI_OUTX_L_G_OIS_EIS,
    LSM6DSV320X_UI_OUTX_H_G_OIS_EIS,
    LSM6DSV320X_UI_OUTY_L_G_OIS_EIS,
    LSM6DSV320X_UI_OUTY_H_G_OIS_EIS,
    LSM6DSV320X_UI_OUTZ_L_G_OIS_EIS,
    LSM6DSV320X_UI_OUTZ_H_G_OIS_EIS,

     // High-G Accelerometer Output
    LSM6DSV320X_UI_OUTX_L_A_OIS_HG = 0x34,
    LSM6DSV320X_UI_OUTX_H_A_OIS_HG,
    LSM6DSV320X_UI_OUTY_L_A_OIS_HG,
    LSM6DSV320X_UI_OUTY_H_A_OIS_HG,
    LSM6DSV320X_UI_OUTZ_L_A_OIS_HG,
    LSM6DSV320X_UI_OUTZ_H_A_OIS_HG,

    // RESERVED 0x3A-0x3F 

    // Timestamp Registers 
    LSM6DSV320X_TIMESTAMP0 = 0x40,
    LSM6DSV320X_TIMESTAMP1,
    LSM6DSV320X_TIMESTAMP2,
    LSM6DSV320X_TIMESTAMP3,

    // Status and Interrupt Sources 
    LSM6DSV320X_UI_STATUS_REG_OIS,
    LSM6DSV320X_WAKE_UP_SRC,
    LSM6DSV320X_TAP_SRC,
    LSM6DSV320X_D6D_SRC,
    LSM6DSV320X_STATUS_CONTROLLER_MAINPAGE,
    LSM6DSV320X_EMB_FUNC_STATUS_MAINPAGE,
    LSM6DSV320X_FSM_STATUS_MAINPAGE,
    LSM6DSV320X_MLC_STATUS_MAINPAGE,

    // High-G Control Block 
    LSM6DSV320X_HG_WAKE_UP_SRC,
    LSM6DSV320X_CTRL2_XL_HG,
    LSM6DSV320X_CTRL1_XL_HG,
    LSM6DSV320X_INTERNAL_FREQ_FINE,

    // Functions Enable
    LSM6DSV320X_FUNCTIONS_ENABLE,
    
    // RESERVED 0x51 

    // High-G Functions
    LSM6DSV320X_HG_FUNCTIONS_ENABLE = 0x52,
    LSM6DSV320X_HG_WAKE_UP_THS,

    // Activity, Tap, and Interrupt Configurations
    LSM6DSV320X_INACTIVITY_DUR,
    LSM6DSV320X_INACTIVITY_THS,
    LSM6DSV320X_TAP_CFG0,
    LSM6DSV320X_TAP_CFG1,
    LSM6DSV320X_TAP_CFG2,
    LSM6DSV320X_TAP_THS_6D,
    LSM6DSV320X_TAP_DUR,
    LSM6DSV320X_WAKE_UP_THS,
    LSM6DSV320X_WAKE_UP_DUR,
    LSM6DSV320X_FREE_FALL,
    LSM6DSV320X_MD1_CFG,
    LSM6DSV320X_MD2_CFG,

    // RESERVED 0x60-0x61

    // Embedded Functions & Handshake

    // Data Rate Configuration Register (Table 22 for ref) 
    // ---------------------------------------------
    LSM6DSV320X_HAODR_CFG = 0x62,
    // ---------------------------------------------
    LSM6DSV320X_EMB_FUNC_CFG,
    LSM6DSV320X_UI_HANDSHAKE_CTRL,
    LSM6DSV320X_UI_IF2_SHARED_0,
    LSM6DSV320X_UI_IF2_SHARED_1,
    LSM6DSV320X_UI_IF2_SHARED_2,
    LSM6DSV320X_UI_IF2_SHARED_3,
    LSM6DSV320X_UI_IF2_SHARED_4,
    LSM6DSV320X_UI_IF2_SHARED_5,
    LSM6DSV320X_CTRL_EIS,

    // High-G User Offsets 
    LSM6DSV320X_XL_HG_X_OFS_USR,
    LSM6DSV320X_XL_HG_Y_OFS_USR,
    LSM6DSV320X_XL_HG_Z_OFS_USR,

    // UI OIS Control
    LSM6DSV320X_UI_INT_OIS,
    LSM6DSV320X_UI_CTRL1_OIS,
    LSM6DSV320X_UI_CTRL2_OIS,
    LSM6DSV320X_UI_CTRL3_OIS,

    // User Offsets (Standard)
    LSM6DSV320X_X_OFS_USR,
    LSM6DSV320X_Y_OFS_USR,
    LSM6DSV320X_Z_OFS_USR,

    // RESERVED 0x76-0x77 

    // FIFO Data Out
    LSM6DSV320X_FIFO_DATA_OUT_TAG = 0x78,
    LSM6DSV320X_FIFO_DATA_OUT_X_L,
    LSM6DSV320X_FIFO_DATA_OUT_X_H,
    LSM6DSV320X_FIFO_DATA_OUT_Y_L,
    LSM6DSV320X_FIFO_DATA_OUT_Y_H,
    LSM6DSV320X_FIFO_DATA_OUT_Z_L,
    LSM6DSV320X_FIFO_DATA_OUT_Z_H
};


class LSM6DSV320X_HighG {
    public:
        /* @brief Default SPI frquency
        KX134 used 10MHz, so I will do the same for the LSM*/
        static constexpr uint32_t spiFrequency = 10000000;
};

class LSM6DSV320X_LowG {
    public:
        /* 
        ================================================
                    LOW-G SECTION 
        ================================================
        */

        /* @brief Default SPI frequency
        LSM supports up to 10MHz, so the values are the same for both the LSM and the ADXL355 */
        static constexpr uint32_t defaultSpiFrequency = 10000000;

        /* @brief Temperature intercept, LSB (at 26 deg C)
        ADXL was 1885, but the LSM is 0 LSB at 25 C (Datasheet 4.3)*/
        static constexpr int16_t temperatureInterceptLSB = 0;

        // @brief Temperature intercept, deg C 
        static constexpr float temperatureInterceptDegC = 25.0f;

        /* @brief Temperature slope, LSB/deg C
        ADXL was =9.06, while the LSM sensitivity is 256 LSB/deg C (Datasheet 4.3)*/
        static constexpr float temperatureSlope = 256.0f;

        /* Here would be the scaling factors for the sensitivity
        The only problem was that ADXL was 20-bit and the LSM is 16-bit.
        Table 3 (Mechanical Characteristics)*/

        /* @brief Acceleration Scale factor for +/- 2G range, g/LSB
        ADXL was 3.9e-6 and the LSM is 61e-6 g/LSB */
        static constexpr float accelerationScaleFactorRange2G = 61.0e-6;

        /* @brief Acceleration scale factor for +/- 4G range, g/LSB
        ADXL was 7.8e-6 and LSM is 122e-6 g/LSB*/
        static constexpr float accelerationScaleFactorRange4G = 122.0e-6;

        /* @brief Acceleration scale factor was +/- 8g range, g/LSB
        ADXL was 15.e-6 and the LSM is 244.0e-6 g/LSB*/
        static constexpr float accelerationScaleFactorRange8G = 244.0e-6;

        // LSM also has a 16G range if necessary (being 488.0e-6)

        /* @brief Maximum number of the FIFO samples
        ADXL was 96, LSM on the other hand has 4.5 KB FIFO
        4.5 KB / 7 bytes for the DATA_OUT = ~600 samples */
        static constexpr uint16_t fifoSamples = 600.0;

        // Constructor for the LSM6DSV
        LSM6DSV320X_LowG(uint8_t csPin, uint32_t defaultSpiFrequency);

        // LowG Instantiation
        void init() {

        }
};