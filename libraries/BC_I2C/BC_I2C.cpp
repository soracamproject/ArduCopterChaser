/** charset=UTF-8 **/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "BC_I2C.h"
#include <Arduino.h>


// ***********************************************************************************
// defines
// ***********************************************************************************
#define I2C_PULLUPS_ENABLE         PORTD |= 1<<0; PORTD |= 1<<1;       // PIN 20&21 (SDA&SCL)
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1);


// ***********************************************************************************
// functions
// ***********************************************************************************
void BC_I2C::init(void) {
  #if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
  #else
    I2C_PULLUPS_DISABLE
  #endif
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / 400000) - 16) / 2;          // set the I2C clock rate to 400kHz
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void BC_I2C::rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmission();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmission();                       // wail until transmission completed
}

void BC_I2C::stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void BC_I2C::write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmission();
}

uint8_t BC_I2C::read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmission();
  uint8_t r = TWDR;
  if (!ack) stop();
  return r;
}

uint8_t BC_I2C::readAck() {
  return read(1);
}

uint8_t BC_I2C::readNak(void) {
  return read(0);
}

void BC_I2C::waitTransmission() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

void BC_I2C::read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
  rep_start(add<<1); // I2C write direction
  write(reg);        // register selection
  rep_start((add<<1) | 1);  // I2C read direction
  uint8_t *b = buf;
  while (size--) {
    /* acknowledge all but the final byte */
    *b++ = read(size > 0);
  }
}

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void BC_I2C::swap_endianness(void *buf, size_t size) {
  /* we swap in-place, so we only have to
  * place _one_ element on a temporary tray
  */
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  /* keep swapping until the pointers have assed each other */
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

// MPU6050用？
void BC_I2C::getSixRawADC(uint8_t add, uint8_t reg, uint8_t *rawADC) {
  read_reg_to_buf(add, reg, rawADC, 6);
}

void BC_I2C::writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  rep_start(add<<1); // I2C write direction
  write(reg);        // register selection
  write(val);        // value to write in register
  stop();
}

uint8_t BC_I2C::readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  read_reg_to_buf(add, reg, &val, 1);
  return val;
}