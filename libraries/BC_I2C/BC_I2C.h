#ifndef __BC_I2C_H__
#define __BC_I2C_H__

#include <BC_Common.h>
#include <Arduino.h>

class BC_I2C
{
public:
	BC_I2C():
		i2c_errors_count(0),
		neutralizeTime(0)
	{}
	
	void init(void);
	void getSixRawADC(uint8_t add, uint8_t reg, uint8_t *rawADC);
	void writeReg(uint8_t add, uint8_t reg, uint8_t val);
	
private:
	int16_t i2c_errors_count;
	uint32_t neutralizeTime;
	
	void rep_start(uint8_t address);
	void stop(void);
	void write(uint8_t data );
	uint8_t read(uint8_t ack);
	uint8_t readAck();
	uint8_t readNak(void);
	void waitTransmission();
	void read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);
	void swap_endianness(void *buf, size_t size);
	uint8_t readReg(uint8_t add, uint8_t reg);
	
};


#endif // __BC_I2C_H__
