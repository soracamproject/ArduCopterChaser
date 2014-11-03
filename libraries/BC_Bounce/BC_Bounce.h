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

#define BUTTON_NONE         0
#define BUTTON_CLICK        1
#define BUTTON_LONG_PRESS   2

#include <inttypes.h>

class Bounce
{

public:
	// Create an instance of the bounce library
	Bounce():
		click_flag(false),
		press_step(0),
		interval_millis(50),
		state(BUTTON_NONE)
	{
	}
	
	// Attach to a pin (and also sets initial state)
	void attach(int pin);
	
	// Sets the debounce interval
	void interval(unsigned long interval_millis); 
	
	// Updates the pin
	// Returns 1 if the state changed
	// Returns 0 if the state did not change
	bool detect_update();
	
	// 状態(state)を読み取る
	uint8_t const read(){ return state; }
	
	// 状態を更新しstateに以下のいずれかを代入する
	// クリック時、長押し判定時に1回だけ更新されるのでプログラムのloopのはじめに1回実行し状態はreadする
	// クリック時: BUTTON CLICK
	// 長押し時  : BUTTON_LONG_PRESS
	// 上記以外  : BUTTON_NONE
	void update();
	
private:
	bool     click_flag;
	uint32_t last_press;
	uint8_t  press_step;
	uint8_t  state;

protected:
  int debounce();
  unsigned long  previous_millis, interval_millis;
  uint8_t debouncedState;
  uint8_t unstableState;
  uint8_t pin;
  uint8_t stateChanged;
};

#endif


