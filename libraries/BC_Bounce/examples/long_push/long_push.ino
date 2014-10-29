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

#include <BC_Bounce.h>




// ***********************************************************************************
// LED関連変数および宣言
// ***********************************************************************************
	// pheasant用
	#define LED1   2
	#define LED2   3
	#define LED3   5
	#define LED4   6


// ***********************************************************************************
// ボタン関連変数および宣言
// ***********************************************************************************
	// pheasant用
	#define BUTTON1      36
	#define BUTTON2      37
	#define BUTTON_UP    33
	#define BUTTON_DOWN  34
	#define BUTTON_RIGHT 35
	#define BUTTON_LEFT  32
	Bounce button1  = Bounce();
	Bounce button2  = Bounce();
	Bounce button_u = Bounce();
	Bounce button_d = Bounce();
	Bounce button_r = Bounce();
	Bounce button_l = Bounce();





void setup(){
	// LED初期化と全点灯
	pinMode(LED1, OUTPUT);	// R
	pinMode(LED2, OUTPUT);	// Y
	pinMode(LED3, OUTPUT);	// G
	pinMode(LED4, OUTPUT);	// B
	control_led(1,1,1,1);
	
	
	// BUTTON初期化
	pinMode(BUTTON1, INPUT);
	pinMode(BUTTON2, INPUT);
	button1.attach(BUTTON1);
	button2.attach(BUTTON2);
	button1.interval(50);			//たぶんチャタ防止間隔5ms
	button2.interval(50);			//たぶんチャタ防止間隔5ms
	pinMode(BUTTON_UP, INPUT);
	pinMode(BUTTON_DOWN, INPUT);
	pinMode(BUTTON_RIGHT, INPUT);
	pinMode(BUTTON_LEFT, INPUT);
	button_u.attach(BUTTON_UP);
	button_d.attach(BUTTON_DOWN);
	button_r.attach(BUTTON_RIGHT);
	button_l.attach(BUTTON_LEFT);
	button_u.interval(50);			//たぶんチャタ防止間隔5ms
	button_d.interval(50);			//たぶんチャタ防止間隔5ms
	button_r.interval(50);			//たぶんチャタ防止間隔5ms
	button_l.interval(50);			//たぶんチャタ防止間隔5ms
	
	
	// LED全消灯
	control_led(-1,-1,-1,-1);
}


void loop(){
	static bool flag;
	static bool flag_l;
	static uint32_t last;
	
	uint32_t now = millis();
	uint32_t dt = now - last;
	
	if(dt > 50){
		
		// 読み込んでから判定しないとおかしな挙動になる
		uint8_t bt = button1.long_push();
		if(bt==1){
			if(!flag){
				control_led(1,0,0,0);
				flag = true;
			} else {
				control_led(-1,0,0,0);
				flag = false;
			}
		} else if(bt==2){
			if(!flag_l){
				control_led(0,1,0,0);
				flag_l = true;
			} else {
				control_led(0,-1,0,0);
				flag_l = false;
			}
		}
		last = now;
	}
}

// LEDの点灯用関数
// -1:消灯、0:そのまま、1:点灯
// （本当はマクロとか組めばいいのだけど書きやすいようにリッチにやってます）
static void control_led(int8_t one, int8_t two, int8_t three, int8_t four){
	if (one==-1){
		digitalWrite(LED1, LOW);
	} else if(one==1){
		digitalWrite(LED1, HIGH);
	}
	
	if (two==-1){
		digitalWrite(LED2, LOW);
	} else if(two==1) {
		digitalWrite(LED2, HIGH);
	}
	
	if (three==-1){
		digitalWrite(LED3, LOW);
	} else if(three==1){
		digitalWrite(LED3, HIGH);
	}
	
	if (four==-1){
		digitalWrite(LED4, LOW);
	} else if(four==1){
		digitalWrite(LED4, HIGH);
	}
}



