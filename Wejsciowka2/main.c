/****************************************
 * wejsciowka 2: Implementacja Delay    *
 ****************************************/
#include "stm32f10x.h"
#include "delay.h"
#include <stdbool.h>

void RCC_Config(void);

bool flag = true; // flaga

int main(void){
	RCC_Config();
	
	SysTick_Config(1000);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	// Tutaj skonfigurowac zegar SysTick
		
	while(1){
		flag = !flag; // przelacz flage (warto ja obserwowac w panelu Watch)
		Delay(5000); // odczekaj 5000ms = 5s
	}
}

void RCC_Config(void) {
  // konfigurowanie sygnalow taktujacych
  RCC_DeInit();                           // reset ustawien RCC
	FLASH_SetLatency(FLASH_Latency_0);      // zwloka Flasha: 0 taktow
	RCC_HCLKConfig(RCC_SYSCLK_Div1);        // HCLK=SYSCLK/1
	RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); // SYSCLK=HSI
	while(RCC_GetSYSCLKSource() != 0x00);   // czekaj az HSI bedzie 
																					// sygnalem zegarowym systemu   
}
