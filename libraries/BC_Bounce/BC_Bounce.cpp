
// Please read Bounce.h for information about the liscence and authors

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "BC_Bounce.h"




#define LONG_PUSH_MS 3000


void Bounce::attach(int pin) {
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


bool Bounce::update()
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

uint8_t Bounce::read()
{
	return debouncedState;
}

bool Bounce::push_check(){
	return (update()==1) && (read() == HIGH);
}

bool Bounce::click(){
	if(push_step>0){
		return false;
	}
	
	if(click_flag){
		if(update() && read() == LOW){
			click_flag = false;
			return true;
		}
	} else {
		if(update() && read() == HIGH){
			click_flag = true;
		}
	}
	
	return false;
}

uint8_t Bounce::long_push(){
	uint32_t now = millis();
	uint32_t dt = now - last_push;
	
	switch(push_step){
		case 0:
			if(update() && read()==HIGH){
				push_step = 1;
				last_push = now;
			}
			break;
		
		case 1:
			if(update() && read()==LOW){
				push_step = 0;
				return 1;
			} else {
				if(dt > LONG_PUSH_MS){
					push_step = 2;
					return 2;
				}
			}
			break;
		
		case 2:
			if(update() && read()==LOW){
				push_step = 0;
			}
			break;
	}
	
	return 0;
}

