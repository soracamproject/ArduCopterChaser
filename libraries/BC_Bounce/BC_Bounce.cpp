
// Please read Bounce.h for information about the liscence and authors

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "BC_Bounce.h"

// 長押し判定時間
#define LONG_PRESS_MS 2000

void Bounce::attach(int pin)
{
 this->pin = pin;
 debouncedState = unstableState = digitalRead(pin);
 #ifdef BOUNCE_LOCK-OUT
 previous_millis = 0;
 #else
 previous_millis = millis();
 #endif
}

void Bounce::interval(unsigned long interval_millis)
{
  this->interval_millis = interval_millis;
  
}

bool Bounce::detect_update()
{
#ifdef BOUNCE_LOCK-OUT
    stateChanged = false;
	// Ignore everything if we are locked out
	if (millis() - previous_millis >= interval_millis) {
		uint8_t currentState = digitalRead(pin);
		if (debouncedState != currentState ) {
			previous_millis = millis();
			debouncedState = currentState;
			stateChanged = true;
		}
	}
	return stateChanged;
#else
	// Lire l'etat de l'interrupteur dans une variable temporaire.
	uint8_t currentState = digitalRead(pin);
	stateChanged = false;

	// Redemarrer le compteur timeStamp tant et aussi longtemps que
	// la lecture ne se stabilise pas.
	if ( currentState != unstableState ) {
			previous_millis = millis();
	} else 	if ( millis() - previous_millis >= interval_millis ) {
				// Rendu ici, la lecture est stable

				// Est-ce que la lecture est diffÃ©rente de l'etat emmagasine de l'interrupteur?
				if ( currentState != debouncedState ) {
						debouncedState = currentState;
						stateChanged = true;
						
				}

	}
	 
	unstableState = currentState;
	return stateChanged;
#endif
}

void Bounce::update(){
	uint32_t now = millis();
	uint32_t dt = now - last_press;
	
	switch(press_step){
		case 0:
			if(detect_update() && debouncedState==HIGH){
				press_step = 1;
				last_press = now;
			}
			state = BUTTON_NONE;
			break;
		
		case 1:
			if(detect_update() && debouncedState==LOW){
				press_step = 0;
				state = BUTTON_CLICK;
			} else {
				if(dt > LONG_PRESS_MS){
					press_step = 2;
					state = BUTTON_LONG_PRESS;
					break;
				}
				state = BUTTON_NONE;
			}
			break;
		
		case 2:
			if(detect_update() && debouncedState==LOW){
				press_step = 0;
			}
			state = BUTTON_NONE;
			break;
	}
}

