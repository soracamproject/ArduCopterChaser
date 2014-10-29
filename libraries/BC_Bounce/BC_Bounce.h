
/*
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *      
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *      
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */



/*  * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 Main code by Thomas O Fredericks (tof@t-o-f.info)
 Previous contributions by Eric Lowry, Jim Schimpf and Tom Harkaway
* * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Uncomment the following line for "LOCK-OUT" debounce method
//#define BOUNCE_LOCK-OUT


#ifndef BC_Bounce_h
#define BC_Bounce_h

#include <inttypes.h>

class Bounce
{

public:
	// Create an instance of the bounce library
	Bounce():
		click_flag(false),
		push_step(0),
		interval_millis(10)
	{
	}
	
	// Attach to a pin (and also sets initial state)
	void attach(int pin);
	
	// Sets the debounce interval
	void interval(unsigned long interval_millis); 
	
	// Updates the pin
	// Returns 1 if the state changed
	// Returns 0 if the state did not change
	bool update();
	
	// Returns the updated pin state
	uint8_t read();
	
	// ボタンが押されたかをチェックする
	bool push_check();
	
	bool click();
	
	uint8_t long_push();

private:
	bool click_flag;
	uint32_t last_push;
	uint8_t push_step;

protected:
  int debounce();
  unsigned long  previous_millis, interval_millis;
  uint8_t debouncedState;
  uint8_t unstableState;
  uint8_t pin;
  uint8_t stateChanged;
};

#endif


