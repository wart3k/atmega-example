/*
 * example-project.cpp
 *
 * Created: 19.10.2025 18:31:03
 * Author : WART3K
 */ 


#include <util/delay.h>

#include "hal-library/GpioClass.h"

using LED1 = GPIO<PortA, 2>;
using LED2 = GPIO<PortB, 2>;
using Button1 = GPIO<PortC, 2>;
using Button2 = GPIO<PortD, 2>;

void onButton1Pressed() {
	LED1::toggle();
}

void onButton2Pressed() {
	LED2::toggle();
}

int main(void)
{
	LED1::setMode(PinMode::OUTPUT);
	LED1::write(PinState::LOW);
	
	LED2::setMode(PinMode::OUTPUT);
	LED2::write(PinState::LOW);
	
	Button1::setMode(PinMode::INPUT);
	Button1::enablePullup();
	Button1::attachInterrupt(InterruptMode::FALLING_EDGE, onButton1Pressed);
	
	Button2::setMode(PinMode::INPUT);
	Button2::enablePullup();
	Button2::attachInterrupt(InterruptMode::FALLING_EDGE, onButton2Pressed);
	
	sei();
	
    while (1) 
    {
		LED1::toggle();
		_delay_ms(1000);
    }
	
	return 0;
}

