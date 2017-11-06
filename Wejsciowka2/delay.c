#include "delay.h"

#include "stm32f10x.h"

void DelayTick(void){
	// dekrementacja licznika (o ile jest wiekszy od 0)
	if(msc != 0){
		msc--;
	}
	
}

void Delay(unsigned int ms){
	msc = ms;
	while(msc != 0){};
	// zmiana wartosci licznika
	// testowanie czy jest on wiekszy od 0
}	

