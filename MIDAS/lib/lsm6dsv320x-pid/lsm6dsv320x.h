//Adapted from https://github.com/STMicroelectronics/lsm6dsv320x-pid/

#include <stdint.h>

#include <SPI.h>


#define DRV_BYTE_ORDER DRV_LITTLE_ENDIAN//??

#define PROPERTY_DISABLE                (0U)
#define PROPERTY_ENABLE                 (1U)


//Device ID
#define LSM6DSV320X_ID                             0x73U

//Registers
#define LSM6DSV320X_STATUS_REG                     0x1EU
#define LSM6DSV320X_OUTX_L_A                       0x28U
#define LSM6DSV320X_UI_OUTX_L_A_OIS_HG             0x34U
#define LSM6DSV320X_WHO_AM_I                       0x0FU
#define LSM6DSV320X_FUNC_CFG_ACCESS                0x1U
#define LSM6DSV320X_HAODR_CFG                      0x62U
#define LSM6DSV320X_CTRL1                          0x10U
#define LSM6DSV320X_CTRL2                          0x11U
#define LSM6DSV320X_CTRL4                          0x13U
#define LSM6DSV320X_CTRL6                          0x15U
#define LSM6DSV320X_CTRL7                          0x16U
#define LSM6DSV320X_CTRL8                          0x17U
#define LSM6DSV320X_CTRL9                          0x18U
#define LSM6DSV320X_CTRL1_XL_HG                    0x4EU
#define LSM6DSV320X_CTRL_EIS                       0x6BU
#define LSM6DSV320X_UI_CTRL1_OIS                   0x70U
#define LSM6DSV320X_OUTX_L_G                       0x22U
#define LSM6DSV320X_UI_INT_OIS                     0x6FU
#define LSM6DSV320X_EMB_FUNC_CFG                   0x63U
#define LSM6DSV320X_EMB_FUNC_EN_A                  0x4U
#define LSM6DSV320X_SFLP_GRAVX_L                   0x1EU
#define LSM6DSV320X_SFLP_GBIASX_L                  0x18U
#define LSM6DSV320X_SFLP_QUATW_L                   0x2AU

enum lsm6dsv320x_data_rate_t
{
  LSM6DSV320X_ODR_OFF              = 0x0,
  LSM6DSV320X_ODR_AT_1Hz875        = 0x1,
  LSM6DSV320X_ODR_AT_7Hz5          = 0x2,
  LSM6DSV320X_ODR_AT_15Hz          = 0x3,
  LSM6DSV320X_ODR_AT_30Hz          = 0x4,
  LSM6DSV320X_ODR_AT_60Hz          = 0x5,
  LSM6DSV320X_ODR_AT_120Hz         = 0x6,
  LSM6DSV320X_ODR_AT_240Hz         = 0x7,
  LSM6DSV320X_ODR_AT_480Hz         = 0x8,
  LSM6DSV320X_ODR_AT_960Hz         = 0x9,
  LSM6DSV320X_ODR_AT_1920Hz        = 0xA,
  LSM6DSV320X_ODR_AT_3840Hz        = 0xB,
  LSM6DSV320X_ODR_AT_7680Hz        = 0xC,
  LSM6DSV320X_ODR_HA01_AT_15Hz625  = 0x13,
  LSM6DSV320X_ODR_HA01_AT_31Hz25   = 0x14,
  LSM6DSV320X_ODR_HA01_AT_62Hz5    = 0x15,
  LSM6DSV320X_ODR_HA01_AT_125Hz    = 0x16,
  LSM6DSV320X_ODR_HA01_AT_250Hz    = 0x17,
  LSM6DSV320X_ODR_HA01_AT_500Hz    = 0x18,
  LSM6DSV320X_ODR_HA01_AT_1000Hz   = 0x19,
  LSM6DSV320X_ODR_HA01_AT_2000Hz   = 0x1A,
  LSM6DSV320X_ODR_HA01_AT_4000Hz   = 0x1B,
  LSM6DSV320X_ODR_HA01_AT_8000Hz   = 0x1C,
  LSM6DSV320X_ODR_HA02_AT_12Hz5    = 0x23,
  LSM6DSV320X_ODR_HA02_AT_25Hz     = 0x24,
  LSM6DSV320X_ODR_HA02_AT_50Hz     = 0x25,
  LSM6DSV320X_ODR_HA02_AT_100Hz    = 0x26,
  LSM6DSV320X_ODR_HA02_AT_200Hz    = 0x27,
  LSM6DSV320X_ODR_HA02_AT_400Hz    = 0x28,
  LSM6DSV320X_ODR_HA02_AT_800Hz    = 0x29,
  LSM6DSV320X_ODR_HA02_AT_1600Hz   = 0x2A,
  LSM6DSV320X_ODR_HA02_AT_3200Hz   = 0x2B,
  LSM6DSV320X_ODR_HA02_AT_6400Hz   = 0x2C,
  LSM6DSV320X_ODR_HA03_AT_13Hz     = 0x33,
  LSM6DSV320X_ODR_HA03_AT_26Hz     = 0x34,
  LSM6DSV320X_ODR_HA03_AT_52Hz     = 0x35,
  LSM6DSV320X_ODR_HA03_AT_104Hz    = 0x36,
  LSM6DSV320X_ODR_HA03_AT_208Hz    = 0x37,
  LSM6DSV320X_ODR_HA03_AT_417Hz    = 0x38,
  LSM6DSV320X_ODR_HA03_AT_833Hz    = 0x39,
  LSM6DSV320X_ODR_HA03_AT_1667Hz   = 0x3A,
  LSM6DSV320X_ODR_HA03_AT_3333Hz   = 0x3B,
  LSM6DSV320X_ODR_HA03_AT_6667Hz   = 0x3C,
};

enum lsm6dsv320x_xl_mode_t
{
  LSM6DSV320X_XL_HIGH_PERFORMANCE_MD   = 0x0,
  LSM6DSV320X_XL_HIGH_ACCURACY_ODR_MD  = 0x1,
  LSM6DSV320X_XL_ODR_TRIGGERED_MD      = 0x3,
  LSM6DSV320X_XL_LOW_POWER_2_AVG_MD    = 0x4,
  LSM6DSV320X_XL_LOW_POWER_4_AVG_MD    = 0x5,
  LSM6DSV320X_XL_LOW_POWER_8_AVG_MD    = 0x6,
  LSM6DSV320X_XL_NORMAL_MD             = 0x7,
};

enum lsm6dsv320x_gy_mode_t
{
  LSM6DSV320X_GY_HIGH_PERFORMANCE_MD   = 0x0,
  LSM6DSV320X_GY_HIGH_ACCURACY_ODR_MD  = 0x1,
  LSM6DSV320X_GY_ODR_TRIGGERED_MD      = 0x3,
  LSM6DSV320X_GY_SLEEP_MD              = 0x4,
  LSM6DSV320X_GY_LOW_POWER_MD          = 0x5,
};

enum lsm6dsv320x_hg_xl_data_rate_t
{
  LSM6DSV320X_HG_XL_ODR_OFF         = 0x0,
  LSM6DSV320X_HG_XL_ODR_AT_480Hz    = 0x3,
  LSM6DSV320X_HG_XL_ODR_AT_960Hz    = 0x4,
  LSM6DSV320X_HG_XL_ODR_AT_1920Hz   = 0x5,
  LSM6DSV320X_HG_XL_ODR_AT_3840Hz   = 0x6,
  LSM6DSV320X_HG_XL_ODR_AT_7680Hz   = 0x7,
};

enum lsm6dsv320x_gy_eis_data_rate_t
{
  LSM6DSV320X_EIS_ODR_OFF = 0x0,
  LSM6DSV320X_EIS_1920Hz  = 0x1,
  LSM6DSV320X_EIS_960Hz   = 0x2,
};

enum lsm6dsv320x_gy_full_scale_t
{
  LSM6DSV320X_250dps  = 0x1,
  LSM6DSV320X_500dps  = 0x2,
  LSM6DSV320X_1000dps = 0x3,
  LSM6DSV320X_2000dps = 0x4,
  LSM6DSV320X_4000dps = 0x5,
};

enum lsm6dsv320x_xl_full_scale_t
{
  LSM6DSV320X_2g  = 0x0,
  LSM6DSV320X_4g  = 0x1,
  LSM6DSV320X_8g  = 0x2,
  LSM6DSV320X_16g = 0x3,
};

enum lsm6dsv320x_hg_xl_full_scale_t
{
  LSM6DSV320X_32g  = 0x0,
  LSM6DSV320X_64g  = 0x1,
  LSM6DSV320X_128g  = 0x2,
  LSM6DSV320X_256g = 0x3,
  LSM6DSV320X_320g = 0x4,
};

enum lsm6dsv320x_filt_gy_lp1_bandwidth_t
{
  LSM6DSV320X_GY_ULTRA_LIGHT   = 0x0,
  LSM6DSV320X_GY_VERY_LIGHT    = 0x1,
  LSM6DSV320X_GY_LIGHT         = 0x2,
  LSM6DSV320X_GY_MEDIUM        = 0x3,
  LSM6DSV320X_GY_STRONG        = 0x4,
  LSM6DSV320X_GY_VERY_STRONG   = 0x5,
  LSM6DSV320X_GY_AGGRESSIVE    = 0x6,
  LSM6DSV320X_GY_XTREME        = 0x7,
};

enum lsm6dsv320x_filt_xl_lp2_bandwidth_t
{
  LSM6DSV320X_XL_ULTRA_LIGHT = 0x0,
  LSM6DSV320X_XL_VERY_LIGHT  = 0x1,
  LSM6DSV320X_XL_LIGHT       = 0x2,
  LSM6DSV320X_XL_MEDIUM      = 0x3,
  LSM6DSV320X_XL_STRONG      = 0x4,
  LSM6DSV320X_XL_VERY_STRONG = 0x5,
  LSM6DSV320X_XL_AGGRESSIVE  = 0x6,
  LSM6DSV320X_XL_XTREME      = 0x7,
};

typedef enum
{
  LSM6DSV320X_MAIN_MEM_BANK       = 0x0,
  LSM6DSV320X_EMBED_FUNC_MEM_BANK = 0x1,
  LSM6DSV320X_SENSOR_HUB_MEM_BANK = 0x2,
} lsm6dsv320x_mem_bank_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sflp_quatw                   : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sflp_quatw                   : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_sflp_quatw_l_t;

typedef struct
{
  uint8_t drdy                         : 1;
  uint8_t ois_drdy                     : 1;
  uint8_t irq_xl                       : 1;
  uint8_t irq_xl_hg                    : 1;
  uint8_t irq_g                        : 1;
} lsm6dsv320x_filt_settling_mask_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t xlda                         : 1;
  uint8_t gda                          : 1;
  uint8_t tda                          : 1;
  uint8_t xlhgda                       : 1;
  uint8_t gda_eis                      : 1;
  uint8_t ois_drdy                     : 1;
  uint8_t not_used0                    : 1;
  uint8_t timestamp_endcount           : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t timestamp_endcount           : 1;
  uint8_t not_used0                    : 1;
  uint8_t ois_drdy                     : 1;
  uint8_t gda_eis                      : 1;
  uint8_t xlhgda                       : 1;
  uint8_t tda                          : 1;
  uint8_t gda                          : 1;
  uint8_t xlda                         : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_status_reg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t ois_ctrl_from_ui             : 1;
  uint8_t if2_reset                    : 1;
  uint8_t sw_por                       : 1;
  uint8_t fsm_wr_ctrl_en               : 1;
  uint8_t not_used0                    : 2;
  uint8_t shub_reg_access              : 1;
  uint8_t emb_func_reg_access          : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t emb_func_reg_access          : 1;
  uint8_t shub_reg_access              : 1;
  uint8_t not_used0                    : 2;
  uint8_t fsm_wr_ctrl_en               : 1;
  uint8_t sw_por                       : 1;
  uint8_t if2_reset                    : 1;
  uint8_t ois_ctrl_from_ui             : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_func_cfg_access_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_xl                       : 4;
  uint8_t op_mode_xl                   : 3;
  uint8_t not_used0                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 1;
  uint8_t op_mode_xl                   : 3;
  uint8_t odr_xl                       : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl1_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t odr_g                        : 4;
  uint8_t op_mode_g                    : 3;
  uint8_t not_used0                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 1;
  uint8_t op_mode_g                    : 3;
  uint8_t odr_g                        : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl2_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t lpf1_g_en                    : 1;
  uint8_t not_used0                    : 5;
  uint8_t int2_drdy_xl_hg              : 1;
  uint8_t int1_drdy_xl_hg              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int1_drdy_xl_hg              : 1;
  uint8_t int2_drdy_xl_hg              : 1;
  uint8_t not_used0                    : 5;
  uint8_t lpf1_g_en                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl7_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl                        : 2;
  uint8_t not_used0                    : 3;
  uint8_t hp_lpf2_xl_bw                : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hp_lpf2_xl_bw                : 3;
  uint8_t not_used0                    : 3;
  uint8_t fs_xl                        : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl8_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t usr_off_on_out               : 1;
  uint8_t usr_off_w                    : 1;
  uint8_t not_used0                    : 1;
  uint8_t lpf2_xl_en                   : 1;
  uint8_t hp_slope_xl_en               : 1;
  uint8_t xl_fastsettl_mode            : 1;
  uint8_t hp_ref_mode_xl               : 1;
  uint8_t not_used1                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 1;
  uint8_t hp_ref_mode_xl               : 1;
  uint8_t xl_fastsettl_mode            : 1;
  uint8_t hp_slope_xl_en               : 1;
  uint8_t lpf2_xl_en                   : 1;
  uint8_t not_used0                    : 1;
  uint8_t usr_off_w                    : 1;
  uint8_t usr_off_on_out               : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl9_t;


typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t haodr_sel                    : 2;
  uint8_t not_used0                    : 6;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 6;
  uint8_t haodr_sel                    : 2;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_haodr_cfg_t;


typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 3;
  uint8_t emb_func_disable             : 1;
  uint8_t emb_func_irq_mask_xl_settl   : 1;
  uint8_t emb_func_irq_mask_g_settl    : 1;
  uint8_t emb_func_irq_mask_xl_hg_settl: 1;
  uint8_t hg_usr_off_on_emb_func       : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t hg_usr_off_on_emb_func       : 1;
  uint8_t emb_func_irq_mask_xl_hg_settl: 1;
  uint8_t emb_func_irq_mask_g_settl    : 1;
  uint8_t emb_func_irq_mask_xl_settl   : 1;
  uint8_t emb_func_disable             : 1;
  uint8_t not_used0                    : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_emb_func_cfg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_xl_hg                     : 3;
  uint8_t odr_xl_hg                    : 3;
  uint8_t hg_usr_off_on_out            : 1;
  uint8_t xl_hg_regout_en              : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t xl_hg_regout_en              : 1;
  uint8_t hg_usr_off_on_out            : 1;
  uint8_t odr_xl_hg                    : 3;
  uint8_t fs_xl_hg                     : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl1_xl_hg_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g_eis                     : 3;
  uint8_t g_eis_on_g_ois_out_reg       : 1;
  uint8_t lpf_g_eis_bw                 : 1;
  uint8_t not_used0                    : 1;
  uint8_t odr_g_eis                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t odr_g_eis                    : 2;
  uint8_t not_used0                    : 1;
  uint8_t lpf_g_eis_bw                 : 1;
  uint8_t g_eis_on_g_ois_out_reg       : 1;
  uint8_t fs_g_eis                     : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl_eis_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 4;
  uint8_t st_ois_clampdis              : 1;
  uint8_t not_used1                    : 1;
  uint8_t drdy_mask_ois                : 1;
  uint8_t int2_drdy_ois                : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t int2_drdy_ois                : 1;
  uint8_t drdy_mask_ois                : 1;
  uint8_t not_used1                    : 1;
  uint8_t st_ois_clampdis              : 1;
  uint8_t not_used0                    : 4;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ui_int_ois_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t int2_in_lh                   : 1;
  uint8_t drdy_pulsed                  : 1;
  uint8_t int2_drdy_temp               : 1;
  uint8_t drdy_mask                    : 1;
  uint8_t int2_on_int1                 : 1;
  uint8_t not_used0                    : 3;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 3;
  uint8_t int2_on_int1                 : 1;
  uint8_t drdy_mask                    : 1;
  uint8_t int2_drdy_temp               : 1;
  uint8_t drdy_pulsed                  : 1;
  uint8_t int2_in_lh                   : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl4_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t if2_spi_read_en              : 1;
  uint8_t ois_g_en                     : 1;
  uint8_t ois_xl_en                    : 1;
  uint8_t not_used0                    : 2;
  uint8_t sim_ois                      : 1;
  uint8_t not_used1                    : 2;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used1                    : 2;
  uint8_t sim_ois                      : 1;
  uint8_t not_used0                    : 2;
  uint8_t ois_xl_en                    : 1;
  uint8_t ois_g_en                     : 1;
  uint8_t if2_spi_read_en              : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ui_ctrl1_ois_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t fs_g                         : 3;
  uint8_t not_used1                    : 1;
  uint8_t lpf1_g_bw                    : 3;
  uint8_t not_used0                    : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t not_used0                    : 1;
  uint8_t lpf1_g_bw                    : 3;
  uint8_t not_used1                    : 1;
  uint8_t fs_g                         : 3;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_ctrl6_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t not_used0                    : 1;
  uint8_t sflp_game_en                 : 1;
  uint8_t not_used2                    : 1;
  uint8_t pedo_en                      : 1;
  uint8_t tilt_en                      : 1;
  uint8_t sign_motion_en               : 1;
  uint8_t not_used1                    : 1;
  uint8_t mlc_before_fsm_en            : 1;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t mlc_before_fsm_en            : 1;
  uint8_t not_used1                    : 1;
  uint8_t sign_motion_en               : 1;
  uint8_t tilt_en                      : 1;
  uint8_t pedo_en                      : 1;
  uint8_t not_used2                    : 1;
  uint8_t sflp_game_en                 : 1;
  uint8_t not_used0                    : 1;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_emb_func_en_a_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t outx_g                       : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t outx_g                       : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_outx_l_g_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sflp_gravx                   : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sflp_gravx                   : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_sflp_gravx_l_t;

typedef struct
{
#if DRV_BYTE_ORDER == DRV_LITTLE_ENDIAN
  uint8_t sflp_gbiasx                  : 8;
#elif DRV_BYTE_ORDER == DRV_BIG_ENDIAN
  uint8_t sflp_gbiasx                  : 8;
#endif /* DRV_BYTE_ORDER */
} lsm6dsv320x_sflp_gbiasx_l_t;


class LSM6DSV320XClass {
public:
    LSM6DSV320XClass(SPIClass& spi, int csPin, int irqPin);

    int32_t sw_por();

    lsm6dsv320x_status_reg_t get_status();
    int32_t device_id_get(uint8_t *val);
    int32_t mem_bank_set(lsm6dsv320x_mem_bank_t val);

    int32_t acceleration_raw_get(int16_t *val);
    int32_t hg_acceleration_raw_get(int16_t *val);
    int32_t angular_rate_raw_get(int16_t *val);

    float from_fs2_to_mg(int16_t lsb);
    float from_fs64_to_mg(int16_t lsb);
    float from_fs2000_to_mdps(int16_t lsb);
    float from_sflp_to_mg(int16_t lsb);

    int32_t xl_setup(lsm6dsv320x_data_rate_t xl_odr, lsm6dsv320x_xl_mode_t xl_mode);//Only sets up lowg
    int32_t gy_setup(lsm6dsv320x_data_rate_t gy_odr, lsm6dsv320x_gy_mode_t gy_mode);
    int32_t hg_xl_data_rate_set(lsm6dsv320x_hg_xl_data_rate_t val, uint8_t reg_out_en);

    int32_t haodr_set(lsm6dsv320x_data_rate_t xl_odr, lsm6dsv320x_xl_mode_t xl_mode, lsm6dsv320x_data_rate_t gy_odr, lsm6dsv320x_gy_mode_t gy_mode);

    int32_t hg_xl_full_scale_set(lsm6dsv320x_hg_xl_full_scale_t val); //set scale to 64gs
    int32_t hg_xl_full_scale_get(lsm6dsv320x_hg_xl_full_scale_t *val);
    int32_t gy_full_scale_set(lsm6dsv320x_gy_full_scale_t val);
    int32_t gy_full_scale_get(lsm6dsv320x_gy_full_scale_t *val);
    int32_t xl_full_scale_set(lsm6dsv320x_xl_full_scale_t val);
    int32_t xl_full_scale_get(lsm6dsv320x_xl_full_scale_t *val);

    int32_t filt_settling_mask_set(bool mask_drdy, bool max_irq_xl, bool mask_irq_g);

    int32_t filt_gy_lp1_set(uint8_t val);
    int32_t filt_gy_lp1_bandwidth_set(lsm6dsv320x_filt_gy_lp1_bandwidth_t val);
    int32_t filt_xl_lp2_set(uint8_t val);
    int32_t filt_xl_lp2_bandwidth_set(lsm6dsv320x_filt_xl_lp2_bandwidth_t val);

    int32_t sflp_enable_set(uint8_t val);
    int32_t lsm6dsv320x_sflp_quaternion_raw_get(uint16_t *val);
    int32_t sflp_gravity_raw_get(int16_t *val);
    int32_t sflp_gbias_raw_get(int16_t *val);
 
private:
    SPIClass * _spi;
    uint8_t _slaveAddress;
    int _csPin;
    int _irqPin;

    SPISettings _spiSettings;

    static void bytecpy(uint8_t *target, uint8_t *source);

    int32_t read_reg(uint8_t reg, uint8_t *data, uint16_t len);
    int32_t write_reg(uint8_t reg, uint8_t *data, uint16_t len);

    int32_t _set_filter_settling_mask(lsm6dsv320x_filt_settling_mask_t val);//Helper for public version
};

