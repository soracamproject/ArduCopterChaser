/** charset=UTF-8 **/

#ifndef __BC_LED_H__
#define __BC_LED_H__

#include <BC_Common.h>

// ***********************************************************************************
// class
// ***********************************************************************************
class BC_LED
{
public:
	BC_LED(uint8_t pin):
		_pin(pin),
		_status(0),
		_blink(false),
		_change(false)
	{}
	
	void init();
	void on();
	void off();
	void blink();
	void update(bool blink_update, bool blink_state);
	//uint8_t const read() { return _status; }
	//bool const change() { return _change; }
	//bool const blink() { return _blink; }
	
	
private:
	uint8_t _pin;
	uint8_t _status;
	bool    _blink;
	bool    _change;
};

#endif // __BC_LED_H__
