#include "L3G4200D.h"

L3G4200D::L3G4200D() {
}

void L3G4200D::init(I2C *i2c) {
	I2Cx = i2c;

	sendBit(L3G4200D_REG_CTRL_REG1, 0b01001111);
	sendBit(L3G4200D_REG_CTRL_REG4, 0b10010000);

	zeroX = 0;
	zeroY = 0;
	zeroZ = 0;

	delayMs(250);
}

void L3G4200D::calibrate() {

	vector v;

	for (int i = 0; i < 1000; i++) {
		readRaw(v);

		zeroX += (float) v.x;
		zeroY += (float) v.y;
		zeroZ += (float) v.z;

		delayMs(5);
	}

	zeroX /= 1000;
	zeroY /= 1000;
	zeroZ /= 1000;

}

void L3G4200D::readAngle(vector &raw, vector &angle) {
	int16_t x, y, z;
	readRaw(x, y, z);

	raw.x = ((float)x) - zeroX;
	raw.y = ((float)y) - zeroY;
	raw.z = ((float)z) - zeroZ;
}

void L3G4200D::readRaw(int16_t &x, int16_t &y, int16_t &z) {
	uint8_t data[6];
	getBits(L3G4200D_OUT_INCREMENT, 6, data);
	x = (data[0]) | (data[1] << 8);
	y = (data[2]) | (data[3] << 8);
	z = (data[4]) | (data[5] << 8);
}

void L3G4200D::readRaw(vector &data) {
	int16_t x, y, z;
	readRaw(x, y, z);
	data.x = x;
	data.y = y;
	data.z = z;
}

void L3G4200D::getBits(uint8_t reg, int bytes, uint8_t *data) {
	I2Cx->readBytes(L3G4200D_ADDR, reg, bytes, data);
}

void L3G4200D::sendBit(uint8_t reg, uint8_t data) {
	I2Cx->sendByte(L3G4200D_ADDR, reg, data);
}
