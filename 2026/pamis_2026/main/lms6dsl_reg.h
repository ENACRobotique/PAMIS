#pragma once

#ifdef __cplusplus
extern "C" {
#endif


/************** Who am I  *******************/

#define LSM6DSL_WHO_AM_I           0x6A

/************** Device Register  *******************/

#define LSM6DSL_FUNC_CFG_ACCESS    0x01

#define LSM6DSL_SENSOR_SYNC_TIME   0x04
#define LSM6DSL_SENSOR_RES_RATIO   0x05

#define LSM6DSL_FIFO_CTRL1         0x06
#define LSM6DSL_FIFO_CTRL2         0x07
#define LSM6DSL_FIFO_CTRL3         0x08
#define LSM6DSL_FIFO_CTRL4         0x09
#define LSM6DSL_FIFO_CTRL5         0x0A

#define LSM6DSL_DRDY_PULSE_CFG_G   0x0B
#define LSM6DSL_INT1_CTRL          0x0D
#define LSM6DSL_INT2_CTRL          0x0E
#define LSM6DSL_WHO_AM_I_REG       0x0F
#define LSM6DSL_CTRL1_XL           0x10
#define LSM6DSL_CTRL2_G            0x11
#define LSM6DSL_CTRL3_C            0x12
#define LSM6DSL_CTRL4_C            0x13
#define LSM6DSL_CTRL5_C            0x14
#define LSM6DSL_CTRL6_C            0x15
#define LSM6DSL_CTRL7_G            0x16
#define LSM6DSL_CTRL8_XL           0x17
#define LSM6DSL_CTRL9_XL           0x18
#define LSM6DSL_CTRL10_C           0x19

#define LSM6DSL_MASTER_CONFIG      0x1A
#define LSM6DSL_WAKE_UP_SRC        0x1B
#define LSM6DSL_TAP_SRC            0x1C
#define LSM6DSL_D6D_SRC            0x1D
#define LSM6DSL_STATUS_REG         0x1E

#define LSM6DSL_OUT_TEMP_L         0x20
#define LSM6DSL_OUT_TEMP_H         0x21
#define LSM6DSL_OUTX_L_G           0x22
#define LSM6DSL_OUTX_H_G           0x23
#define LSM6DSL_OUTY_L_G           0x24
#define LSM6DSL_OUTY_H_G           0x25
#define LSM6DSL_OUTZ_L_G           0x26
#define LSM6DSL_OUTZ_H_G           0x27
#define LSM6DSL_OUTX_L_XL          0x28
#define LSM6DSL_OUTX_H_XL          0x29
#define LSM6DSL_OUTY_L_XL          0x2A
#define LSM6DSL_OUTY_H_XL          0x2B
#define LSM6DSL_OUTZ_L_XL          0x2C
#define LSM6DSL_OUTZ_H_XL          0x2D
#define LSM6DSL_SENSORHUB1_REG     0x2E
#define LSM6DSL_SENSORHUB2_REG     0x2F
#define LSM6DSL_SENSORHUB3_REG     0x30
#define LSM6DSL_SENSORHUB4_REG     0x31
#define LSM6DSL_SENSORHUB5_REG     0x32
#define LSM6DSL_SENSORHUB6_REG     0x33
#define LSM6DSL_SENSORHUB7_REG     0x34
#define LSM6DSL_SENSORHUB8_REG     0x35
#define LSM6DSL_SENSORHUB9_REG     0x36
#define LSM6DSL_SENSORHUB10_REG    0x37
#define LSM6DSL_SENSORHUB11_REG    0x38
#define LSM6DSL_SENSORHUB12_REG    0x39
#define LSM6DSL_FIFO_STATUS1       0x3A
#define LSM6DSL_FIFO_STATUS2       0x3B
#define LSM6DSL_FIFO_STATUS3       0x3C
#define LSM6DSL_FIFO_STATUS4       0x3D
#define LSM6DSL_FIFO_DATA_OUT_L    0x3E
#define LSM6DSL_FIFO_DATA_OUT_H    0x3F
#define LSM6DSL_TIMESTAMP0_REG     0x40
#define LSM6DSL_TIMESTAMP1_REG     0x41
#define LSM6DSL_TIMESTAMP2_REG     0x42

#define LSM6DSL_TIMESTAMP_L        0x49
#define LSM6DSL_TIMESTAMP_H        0x4A

#define LSM6DSL_STEP_COUNTER_L     0x4B
#define LSM6DSL_STEP_COUNTER_H     0x4C

#define LSM6DSL_SENSORHUB13_REG    0x4D
#define LSM6DSL_SENSORHUB14_REG    0x4E
#define LSM6DSL_SENSORHUB15_REG    0x4F
#define LSM6DSL_SENSORHUB16_REG    0x50
#define LSM6DSL_SENSORHUB17_REG    0x51
#define LSM6DSL_SENSORHUB18_REG    0x52

#define LSM6DSL_FUNC_SRC           0x53
#define LSM6DSL_TAP_CFG1           0x58
#define LSM6DSL_TAP_THS_6D         0x59
#define LSM6DSL_INT_DUR2           0x5A
#define LSM6DSL_WAKE_UP_THS        0x5B
#define LSM6DSL_WAKE_UP_DUR        0x5C
#define LSM6DSL_FREE_FALL          0x5D
#define LSM6DSL_MD1_CFG            0x5E
#define LSM6DSL_MD2_CFG            0x5F

#define LSM6DSL_OUT_MAG_RAW_X_L    0x66
#define LSM6DSL_OUT_MAG_RAW_X_H    0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L    0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H    0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L    0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H    0x6B

#define LSM6DSL_X_OFS_USR          0x73
#define LSM6DSL_Y_OFS_USR          0x74
#define LSM6DSL_Z_OFS_USR          0x75

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_SLV0_ADD                     0x02
#define LSM6DSL_SLV0_SUBADD                  0x03
#define LSM6DSL_SLAVE0_CONFIG                0x04
#define LSM6DSL_SLV1_ADD                     0x05
#define LSM6DSL_SLV1_SUBADD                  0x06
#define LSM6DSL_SLAVE1_CONFIG                0x07
#define LSM6DSL_SLV2_ADD                     0x08
#define LSM6DSL_SLV2_SUBADD                  0x09
#define LSM6DSL_SLAVE2_CONFIG                0x0A
#define LSM6DSL_SLV3_ADD                     0x0B
#define LSM6DSL_SLV3_SUBADD                  0x0C
#define LSM6DSL_SLAVE3_CONFIG                0x0D
#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define LSM6DSL_CONFIG_PEDO_THS_MIN          0x0F

#define LSM6DSL_SM_STEP_THS                  0x13
#define LSM6DSL_PEDO_DEB_REG                 0x14
#define LSM6DSL_STEP_COUNT_DELTA             0x15

#define LSM6DSL_MAG_SI_XX                    0x24
#define LSM6DSL_MAG_SI_XY                    0x25
#define LSM6DSL_MAG_SI_XZ                    0x26
#define LSM6DSL_MAG_SI_YX                    0x27
#define LSM6DSL_MAG_SI_YY                    0x28
#define LSM6DSL_MAG_SI_YZ                    0x29
#define LSM6DSL_MAG_SI_ZX                    0x2A
#define LSM6DSL_MAG_SI_ZY                    0x2B
#define LSM6DSL_MAG_SI_ZZ                    0x2C
#define LSM6DSL_MAG_OFFX_L                   0x2D
#define LSM6DSL_MAG_OFFX_H                   0x2E
#define LSM6DSL_MAG_OFFY_L                   0x2F
#define LSM6DSL_MAG_OFFY_H                   0x30
#define LSM6DSL_MAG_OFFZ_L                   0x31
#define LSM6DSL_MAG_OFFZ_H                   0x32

/* Accelero Full_ScaleSelection */
#define LSM6DSL_ACC_FULLSCALE_2G          ((uint8_t)0x00) /*!< +-2 g */
#define LSM6DSL_ACC_FULLSCALE_4G          ((uint8_t)0x08) /*!< +-4 g */
#define LSM6DSL_ACC_FULLSCALE_8G          ((uint8_t)0x0C) /*!< +-8 g */
#define LSM6DSL_ACC_FULLSCALE_16G         ((uint8_t)0x04) /*!< +-16 g */

/* Accelero Full Scale Sensitivity */
#define LSM6DSL_ACC_SENSITIVITY_2G     ((float)0.061f)  /*!< accelerometer sensitivity with 2 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_4G     ((float)0.122f)  /*!< accelerometer sensitivity with 4 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_8G     ((float)0.244f)  /*!< accelerometer sensitivity with 8 g full scale  [mgauss/LSB] */
#define LSM6DSL_ACC_SENSITIVITY_16G    ((float)0.488f)  /*!< accelerometer sensitivity with 12 g full scale [mgauss/LSB] */

/* Accelero Power Mode selection */
#define LSM6DSL_LP_XL_DISABLED     ((uint8_t)0x00) /* LP disabled*/
#define LSM6DSL_LP_XL_ENABLED      ((uint8_t)0x10) /* LP enabled*/

/* Output Data Rate */

#define LSM6DSL_ODR_POWER_DOWN       ((uint8_t)0x00) /* Power Down mode       */
#define LSM6DSL_ODR_13Hz             ((uint8_t)0x10) /* Low Power mode        */
#define LSM6DSL_ODR_26Hz             ((uint8_t)0x20) /* Low Power mode        */ 
#define LSM6DSL_ODR_52Hz             ((uint8_t)0x30) /* Low Power mode        */
#define LSM6DSL_ODR_104Hz            ((uint8_t)0x40) /* Normal mode           */
#define LSM6DSL_ODR_208Hz            ((uint8_t)0x50) /* Normal mode           */
#define LSM6DSL_ODR_416Hz            ((uint8_t)0x60) /* High Performance mode */
#define LSM6DSL_ODR_833Hz            ((uint8_t)0x70) /* High Performance mode */
#define LSM6DSL_ODR_1660Hz           ((uint8_t)0x80) /* High Performance mode */
#define LSM6DSL_ODR_3330Hz           ((uint8_t)0x90) /* High Performance mode */
#define LSM6DSL_ODR_6660Hz           ((uint8_t)0xA0) /* High Performance mode */ 

/* Gyro Full Scale Selection */
#define LSM6DSL_GYRO_FS_250            ((uint8_t)0x00)  
#define LSM6DSL_GYRO_FS_500            ((uint8_t)0x04)  
#define LSM6DSL_GYRO_FS_1000           ((uint8_t)0x08)  
#define LSM6DSL_GYRO_FS_2000           ((uint8_t)0x0C)

/* Gyro Full Scale Sensitivity */ 
#define LSM6DSL_GYRO_SENSITIVITY_245DPS            ((float)8.750f) /**< Sensitivity value for 245 dps full scale  [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_500DPS            ((float)17.50f) /**< Sensitivity value for 500 dps full scale  [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_1000DPS           ((float)35.00f) /**< Sensitivity value for 1000 dps full scale [mdps/LSB] */ 
#define LSM6DSL_GYRO_SENSITIVITY_2000DPS           ((float)70.00f) /**< Sensitivity value for 2000 dps full scale [mdps/LSB] */ 

/* Gyro Power Mode selection */
#define LSM6DSL_LP_G_DISABLED     ((uint8_t)0x00) /* LP disabled*/
#define LSM6DSL_LP_G_ENABLED      ((uint8_t)0x80) /* LP enabled*/

/* Block Data Update */  
#define LSM6DSL_BDU_CONTINUOS               ((uint8_t)0x00)
#define LSM6DSL_BDU_BLOCK_UPDATE            ((uint8_t)0x40)

/* Auto-increment */
#define LSM6DSL_IF_INC_DISABLED    ((uint8_t)0x00)
#define LSM6DSL_IF_INC_ENABLED     ((uint8_t)0x04)

/*Gyro FIFO decimation factor*/
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_0     ((uint8_t)0x0)      /**< Gyro sensor not in FIFO */ 
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_1     ((uint8_t)0x8)      /**< no decimation*/ 
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_2     ((uint8_t)0x10)     /**< decimation with factor 2*/ 
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_3     ((uint8_t)0x18)
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_4     ((uint8_t)0x20)
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_8     ((uint8_t)0x28)
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_16    ((uint8_t)0x30)
#define LSM6DSL_GYRO_FIFO_CTRL3_DEC_32    ((uint8_t)0x38)

/*Gyro FIFO decimation factor*/
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_0     ((uint8_t)0x0)       /**< Acc sensor not in FIFO */ 
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_1     ((uint8_t)0x1)       /**< no decimation*/ 
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_2     ((uint8_t)0x02)      /**< decimation with factor 2*/
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_3     ((uint8_t)0x03)
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_4     ((uint8_t)0x04)
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_8     ((uint8_t)0x05)
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_16    ((uint8_t)0x06)
#define LSM6DSL_ACC_FIFO_CTRL3_DEC_32    ((uint8_t)0x07)


/* FIFO ODR selection */
#define LSM6DSL_FIFO_CTRL5_ODR_0Hz        ((uint8_t) 0X00 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_12_5Hz     ((uint8_t) 0X01 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_26Hz       ((uint8_t) 0X02 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_52Hz       ((uint8_t) 0X03 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_104Hz      ((uint8_t) 0X04 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_208Hz      ((uint8_t) 0X05 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_416Hz      ((uint8_t) 0X06 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_833Hz      ((uint8_t) 0X07 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_1660Hz     ((uint8_t) 0X08 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_3330Hz     ((uint8_t) 0X09 << 3)
#define LSM6DSL_FIFO_CTRL5_ODR_6660Hz     ((uint8_t) 0X0A << 3)

/* FIFO mode selection */
#define LSM6DSL_FIFO_CTRL5_MODE_BYPASS              ((uint8_t) 0X00)
#define LSM6DSL_FIFO_CTRL5_MODE_FIFO                ((uint8_t) 0X01)
#define LSM6DSL_FIFO_CTRL5_MODE_CONTINUOUS_FIFO     ((uint8_t) 0X03)
#define LSM6DSL_FIFO_CTRL5_MODE_BYPASS_CONTINUOUS   ((uint8_t) 0X04)
#define LSM6DSL_FIFO_CTRL5_MODE_CONTINUOUS          ((uint8_t) 0X06)

#define LSM6DSL_INT1_CTRL_FTH           ((uint8_t)0x08)      /**< FIFO threshold interrupt*/
#define LSM6DSL_INT1_CTRL_FIFO_OVR      ((uint8_t)0x10)      /**< FIFO threshold interrupt*/
#define LSM6DSL_INT1_CTRL_FULL_FLAG     ((uint8_t)0x20)      /**< FIFO threshold interrupt*/


}
