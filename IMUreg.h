
#ifndef IMUREG
#define IMUREG
#include <stdint.h>

uint8_t ACT_THS              = 0B00000100;
uint8_t ACT_DUR              = 0B00000101;
uint8_t INT_GEN_CFG_XL       = 0B00000110;
uint8_t INT_GEN_THS_X_XL     = 0B00000111;
uint8_t INT_GEN_THS_Y_XL     = 0B00001000;
uint8_t INT_GEN_THS_Z_XL     = 0B00001001;
uint8_t INT_GEN_DUR_XL       = 0B00001010;
uint8_t REFERENCE_G          = 0B00001011;
uint8_t INT1_CTRL            = 0B00001100;
uint8_t INT2_CTRL            = 0B00001101;

uint8_t WHO_AM_I             = 0B00001111;
uint8_t CTRL_REG1_G          = 0B00010000;
uint8_t CTRL_REG2_G          = 0B00010001;
uint8_t CTRL_REG3_G          = 0B00010010;
uint8_t ORIENT_CFG_G         = 0B00010011;
uint8_t INT_GEN_SRC_G        = 0B00010100;
uint8_t OUT_TEMP_L           = 0B00010101;
uint8_t OUT_TEMP_H           = 0B00010110;
uint8_t STATUS_REG           = 0B00010111;
uint8_t OUT_X_L_G            = 0B00011000;
uint8_t OUT_X_H_G            = 0B00011001;
uint8_t OUT_Y_L_G            = 0B00011010;
uint8_t OUT_Y_H_G            = 0B00011011;
uint8_t OUT_Z_L_G            = 0B00011100;
uint8_t OUT_Z_H_G            = 0B00011101;
uint8_t CTRL_REG4            = 0B00011110;
uint8_t CTRL_REG5_XL         = 0B00011111;
uint8_t CTRL_REG6_XL         = 0B00100000;
uint8_t CTRL_REG7_XL         = 0B00100001;
uint8_t CTRL_REG8            = 0B00100010;
uint8_t CTRL_REG9            = 0B00100011;
uint8_t CTRL_REG10           = 0B00100100;

uint8_t INT_GEN_SRC_XL       = 0B00100110;
uint8_t STATUS_REG0          = 0B00100111;
uint8_t OUT_X_L_XL           = 0B00101000;
uint8_t OUT_X_H_XL           = 0B00101001;
uint8_t OUT_Y_L_XL           = 0B00101010;
uint8_t OUT_Y_H_XL           = 0B00101011;
uint8_t OUT_Z_L_XL           = 0B00101100;
uint8_t OUT_Z_H_XL           = 0B00101101;
uint8_t FIFO_CTRL            = 0B00101110;
uint8_t FIFO_SRC             = 0B00101111;
uint8_t INT_GEN_CFG_G        = 0B00110000;
uint8_t INT_GEN_THS_XH_G     = 0B00110001;
uint8_t INT_GEN_THS_XL_G     = 0B00110010;
uint8_t INT_GEN_THS_YH_G     = 0B00110011;
uint8_t INT_GEN_THS_YL_G     = 0B00110100;
uint8_t INT_GEN_THS_ZH_G     = 0B00110101;
uint8_t INT_GEN_THS_ZL_G     = 0B00110110;
uint8_t INT_GEN_DUR_G        = 0B00110111;


uint8_t OFFSET_X_REG_L_M     = 0B00000101;
uint8_t OFFSET_X_REG_H_M     = 0B00000110;
uint8_t OFFSET_Y_REG_L_M     = 0B00000111;
uint8_t OFFSET_Y_REG_H_M     = 0B00001000;
uint8_t OFFSET_Z_REG_L_M     = 0B00001001;
uint8_t OFFSET_Z_REG_H_M     = 0B00001010;

uint8_t WHO_AM_I_M           = 0B00001111;

uint8_t CTRL_REG1_M          = 0B00100000;
uint8_t CTRL_REG2_M          = 0B00100001;
uint8_t CTRL_REG3_M          = 0B00100010;
uint8_t CTRL_REG4_M          = 0B00100011;
uint8_t CTRL_REG5_M          = 0B00100100;

uint8_t STATUS_REG_M         = 0B00100111;
uint8_t OUT_X_L_M            = 0B00101000;
uint8_t OUT_X_H_M            = 0B00101001;
uint8_t OUT_Y_L_M            = 0B00101010;
uint8_t OUT_Y_H_M            = 0B00101011;
uint8_t OUT_Z_L_M            = 0B00101100;
uint8_t OUT_Z_H_M            = 0B00101101;

uint8_t INT_CFG_M            = 0B00110000;
uint8_t INT_SRC_M            = 0B00110001;
uint8_t INT_THS_L_M          = 0B00110010;
uint8_t INT_THS_H_M          = 0B00110011;

uint8_t Read    = 0B00000001;
uint8_t Write   = 0B00000000;
uint8_t Address_AG =   0B01101011;  //address of accelerometer/gyro with SDO_AG connected to Vdd
uint8_t Address_M   =   0B00011110;  //address of magnetometer with SDO_M connected to Vdd

#endif
