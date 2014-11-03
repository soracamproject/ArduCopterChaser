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

#include "BC_LED.h"
#include <Arduino.h>


// ***********************************************************************************
// functions
// ***********************************************************************************
void BC_LED::init() {
	pinMode(_pin, OUTPUT);
}

void BC_LED::on() {
	_status = HIGH;
	_change = true;
	_blink = false;
}

void BC_LED::off() {
	_status = LOW;
	_change = true;
	_blink =false;
}

void BC_LED::blink() {
	_blink = true;
}

void BC_LED::update(bool blink_update, bool blink_state) {
	if(_blink){
		if(blink_update){
			digitalWrite(_pin, blink_state);
			return;
		}
	} else if(_change){
		digitalWrite(_pin, _status);
		_change = false;
		return;
	}
	
	return;
}





