/*
 * L3G4200D.h
 *
 *  Created on: Mar 3, 2022
 *      Author: halako
 */

#ifndef INC_L3G4200D_H_
#define INC_L3G4200D_H_

/*************************
    L3G4200D Registers
*************************/
#define L3G4200D_ADDRESS           (0xD2 >> 1)

#define L3G4200D_REG_WHO_AM_I      (0x0F)

#define L3G4200D_REG_CTRL_REG1     (0x20)
#define L3G4200D_REG_CTRL_REG2     (0x21)
#define L3G4200D_REG_CTRL_REG3     (0x22)
#define L3G4200D_REG_CTRL_REG4     (0x23)
#define L3G4200D_REG_CTRL_REG5     (0x24)
#define L3G4200D_REG_REFERENCE     (0x25)
#define L3G4200D_REG_OUT_TEMP      (0x26)
#define L3G4200D_REG_STATUS_REG    (0x27)

#define L3G4200D_REG_OUT_X_L       (0x28)
#define L3G4200D_REG_OUT_X_H       (0x29)
#define L3G4200D_REG_OUT_Y_L       (0x2A)
#define L3G4200D_REG_OUT_Y_H       (0x2B)
#define L3G4200D_REG_OUT_Z_L       (0x2C)
#define L3G4200D_REG_OUT_Z_H       (0x2D)

#define L3G4200D_REG_FIFO_CTRL_REG (0x2E)
#define L3G4200D_REG_FIFO_SRC_REG  (0x2F)

#define L3G4200D_REG_INT1_CFG      (0x30)
#define L3G4200D_REG_INT1_SRC      (0x31)
#define L3G4200D_REG_INT1_THS_XH   (0x32)
#define L3G4200D_REG_INT1_THS_XL   (0x33)
#define L3G4200D_REG_INT1_THS_YH   (0x34)
#define L3G4200D_REG_INT1_THS_YL   (0x35)
#define L3G4200D_REG_INT1_THS_ZH   (0x36)
#define L3G4200D_REG_INT1_THS_ZL   (0x37)
#define L3G4200D_REG_INT1_DURATION (0x38)

#define L3G4200D_WHO_AM_I      0x0F

#define L3G4200D_ADDR (0xD2 >> 1)

#define L3G4200D_CTRL_REG1     0x20
#define L3G4200D_CTRL_REG4     0x23
#define L3G4200D_OUT_TEMP      0x26
#define L3G4200D_STATUS_REG    0x27

#define L3G4200D_CTRL_REG4_CONST_UPDATE 0b10000000
#define L3G4200D_CTRL_REG4_SCALE_250 0x00
#define L3G4200D_CTRL_REG4_SCALE_500 0b00010000
#define L3G4200D_CTRL_REG4_SCALE_2000 0b00100000

#define L3G4200D_CTRL_REG1_ON 0b00001111

#define L3G4200D_OUT_INCREMENT 0x28 | (1<<7)
#define L3G4200D_OUT_X_L       0x28
#define L3G4200D_OUT_X_H       0x29
#define L3G4200D_OUT_Y_L       0x2A
#define L3G4200D_OUT_Y_H       0x2B
#define L3G4200D_OUT_Z_L       0x2C
#define L3G4200D_OUT_Z_H       0x2D

//#ifndef VECTOR_STRUCT_H
//#define VECTOR_STRUCT_H
typedef struct Vector
{
	int x, y, z;
} L3G4200D_output;
//#endif

typedef enum
{
    L3G4200D_SCALE_2000DPS = 0b10,
    L3G4200D_SCALE_500DPS  = 0b01,
    L3G4200D_SCALE_250DPS  = 0b00
} l3g4200d_dps_t;

typedef enum
{
    L3G4200D_DATARATE_800HZ_110  = 0b1111,
    L3G4200D_DATARATE_800HZ_50   = 0b1110,
    L3G4200D_DATARATE_800HZ_35   = 0b1101,
    L3G4200D_DATARATE_800HZ_30   = 0b1100,
    L3G4200D_DATARATE_400HZ_110  = 0b1011,
    L3G4200D_DATARATE_400HZ_50   = 0b1010,
    L3G4200D_DATARATE_400HZ_25   = 0b1001,
    L3G4200D_DATARATE_400HZ_20   = 0b1000,
    L3G4200D_DATARATE_200HZ_70   = 0b0111,
    L3G4200D_DATARATE_200HZ_50   = 0b0110,
    L3G4200D_DATARATE_200HZ_25   = 0b0101,
    L3G4200D_DATARATE_200HZ_12_5 = 0b0100,
    L3G4200D_DATARATE_100HZ_25   = 0b0001,
    L3G4200D_DATARATE_100HZ_12_5 = 0b0000
} l3g4200d_odrbw_t;

//class L3G4200D{
//
//    public:
//		L3G4200D();
//		bool begin(l3g4200d_dps_t scale = L3G4200D_SCALE_2000DPS, l3g4200d_odrbw_t odrbw = L3G4200D_DATARATE_100HZ_12_5);
//		l3g4200d_dps_t getScale(void);
//		l3g4200d_odrbw_t getOdrBw(void);
//
//		void calibrate(uint8_t samples = 50);
//		void setThreshold(uint8_t multiple = 1);
//		uint8_t getThreshold(void);
//
//		Vector readRaw(void);
//		Vector readNormalize();
//		uint8_t readTemperature(void);
//
//    private:
//		Vector r;
//		Vector n;
//		Vector d;
//		Vector t;
//
//		bool useCalibrate;
//		float actualThreshold;
//		float dpsPerDigit;
//		float thresholdX;
//		float thresholdY;
//		float thresholdZ;
//
//		void writeRegister8(uint8_t reg, uint8_t value);
//		uint8_t readRegister8(uint8_t reg);
//		uint8_t fastRegister8(uint8_t reg);
//};

//class L3G4200D {
//
//public:
//	L3G4200D();
//	void init(I2C *i2c);
//	void calibrate();
//	void readRaw(int16_t &x, int16_t &y, int16_t &z);
//	//void readRaw(float &x, float &y, float &z);
//	void readRaw(vector &data);
//	void readAngle(int16_t &x, int16_t &y, int16_t &z);
//	void readAngle(vector &raw,vector &angle);
//	//void readAngle(vector *v);
//
//private:
//	I2C *I2Cx;
//
//	float zeroX, zeroY, zeroZ;
//
//	void getBits(uint8_t reg, int bytes, uint8_t *data);
//	void sendBit(uint8_t reg, uint8_t data);
//
//};


#endif /* INC_L3G4200D_H_ */
