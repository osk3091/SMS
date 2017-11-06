/****************************************
 * projekt01: konfiguracja zegarow      *
 ****************************************/
#include "stm32f10x.h"
#include "lcd_hd44780.h"
#include <stdbool.h>   // true, false
#include <stdio.h>

#define DELAY_TIME 8000000

bool RCC_Config(void);
void GPIO_Config(void);
void GPIO_PWM_Config(void);
void LED8On(void);
void LED8Off(void);
void LED9On(void);
void LED9Off(void);
void Delay(unsigned int);
unsigned int readADC(void);
void ADC_Config(void);

int main(void) {
    RCC_Config();          // konfiguracja RCC
    GPIO_Config();         // konfiguracja GPIO
		GPIO_PWM_Config();
		ADC_Config();
	
		LCD_Initialize(); 
	
		unsigned int val = 1024;
		bool button = false;
	
	//LCD_WriteCommand ( HD44780_DISPLAY_CURSOR_SHIFT | HD44780_SHIFT_CURSOR | HD44780_SHIFT_RIGHT ) ;

	
	//LCD_WriteCommand(unsigned char commandToWrite);
	char sym = 's';
	
	char *wsk = &sym;
	
	
	
	//LCD_GoTo(unsigned char x, unsigned char y);
	
	char buffer[32];
		
    while(1) {             // petla glowna programu
        //LED8On();           // wlaczenie diody
				//TIM4->CCR3 = val/2;
        //Delay(DELAY_TIME*2); // odczekanie 1s
        //LED8Off();   
				//TIM4->CCR3 = 0;			// wylaczenie diody
        //Delay(DELAY_TIME*2/10); // odczekanie 1s
				
				 
			 Delay(DELAY_TIME*2/10);
				if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
					LED9On();
					sprintf(buffer, "ADC: %4.1fV", readADC()*3.3/4096);
					LCD_GoTo(0, 0);
					
					LCD_WriteText(buffer);
					//LCD_WriteCommand ( HD44780_DISPLAY_CURSOR_SHIFT | HD44780_SHIFT_DISPLAY | HD44780_SHIFT_RIGHT ) ;

				}else{
					LED9Off();
				}

				TIM4->CCR3 = readADC();
    }
}

bool RCC_Config(void) {
    ErrorStatus HSEStartUpStatus;                          // zmienna opisujaca rezultat 
    // uruchomienia HSE
    // konfigurowanie sygnalow taktujacych
    RCC_DeInit();                                          // reset ustawien RCC
    RCC_HSEConfig(RCC_HSE_ON);                             // wlacz HSE
    HSEStartUpStatus = RCC_WaitForHSEStartUp();            // czekaj na gotowosc HSE
    if(HSEStartUpStatus == SUCCESS) {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//
        FLASH_SetLatency(FLASH_Latency_2);                   // zwloka Flasha: 2 takty 

        RCC_HCLKConfig(RCC_SYSCLK_Div1);                     // HCLK=SYSCLK/1
        RCC_PCLK2Config(RCC_HCLK_Div1);                      // PCLK2=HCLK/1
        RCC_PCLK1Config(RCC_HCLK_Div2);                      // PCLK1=HCLK/2
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // PLLCLK = (HSE/1)*9 
        // czyli 8MHz * 9 = 72 MHz
        RCC_PLLCmd(ENABLE);                                  // wlacz PLL
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // czekaj na uruchomienie PLL
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);           // ustaw PLL jako zrodlo 
        // sygnalu zegarowego
        while(RCC_GetSYSCLKSource() != 0x08);                // czekaj az PLL bedzie 
        // sygnalem zegarowym systemu    
        // konfiguracja sygnalow taktujacych uzywanych peryferii 
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// wlacz taktowanie portu GPIO B
			
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
			
				RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
			
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

				RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);// wlacz taktowanie ADC1

				RCC_ADCCLKConfig(RCC_PCLK2_Div6);
				
        return true;
    }
    return false;
}


unsigned int readADC(void){
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                  // start pomiaru
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);   // czekaj na koniec pomiaru
    return ADC_GetConversionValue(ADC1);                     // odczyt pomiaru (12 bit)
} 

void GPIO_Config(void) {
    // konfigurowanie portow GPIO
    GPIO_InitTypeDef  GPIO_InitStructure; 
		
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;        // pin 8
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // czestotliwosc zmiany 2MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // wyjscie w trybie push-pull
    GPIO_Init(GPIOB, &GPIO_InitStructure);           // inicjaliacja portu B
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // wejscie w trybie pull-up
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

void GPIO_PWM_Config(void) {
    //konfigurowanie portow GPIO
    GPIO_InitTypeDef  GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef timerInitStructure;
    TIM_OCInitTypeDef outputChannelInit;

    // konfiguracja pinu
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;         // pin 8
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // szybkosc 50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;   // wyjscie w trybie alt. push-pull
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // konfiguracja timera
    timerInitStructure.TIM_Prescaler = 0;             // prescaler = 0
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // zliczanie w gore
    timerInitStructure.TIM_Period = 4095;             // okres dlugosci 4095+1
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; // dzielnik czestotliwosci = 1
    timerInitStructure.TIM_RepetitionCounter = 0;     // brak powtorzen
    TIM_TimeBaseInit(TIM4, &timerInitStructure);      // inicjalizacja timera TIM4
    TIM_Cmd(TIM4, ENABLE);                            // aktywacja timera TIM4

    // konfiguracja kanalu timera
    outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;   // tryb PWM1
    outputChannelInit.TIM_Pulse = 1024*2;               // wypelnienie 1024/4095*100% = 25%
    outputChannelInit.TIM_OutputState = TIM_OutputState_Enable; // stan Enable
    outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High; // polaryzacja Active High
    TIM_OC3Init(TIM4, &outputChannelInit);            // inicjalizacja kanalu 3 timera TIM4
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); // konfiguracja preload register
}

void ADC_Config(void) {
    ADC_InitTypeDef  ADC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure; 

    ADC_DeInit(ADC1);                                   // reset ustawien ADC1

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;           // pin 0
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // szybkosc 50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;// wyjscie w floating
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // niezalezne dzialanie ADC 1 i 2
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // pomiar pojedynczego kanalu
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // pomiar na zadanie
    ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None; // programowy start
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // pomiar wyrownany do prawej
    ADC_InitStructure.ADC_NbrOfChannel = 1;             // jeden kanal
    ADC_Init(ADC1, &ADC_InitStructure);                 // inicjalizacja ADC1
    ADC_RegularChannelConfig(ADC1, 8, 1, ADC_SampleTime_1Cycles5); // ADC1, kanal 8, 
    // 1.5 cyklu 
    ADC_Cmd(ADC1, ENABLE);                              // aktywacja ADC1

    ADC_ResetCalibration(ADC1);                         // reset rejestru kalibracji ADC1
    while(ADC_GetResetCalibrationStatus(ADC1));         // oczekiwanie na koniec resetu
    ADC_StartCalibration(ADC1);                         // start kalibracji ADC1
    while(ADC_GetCalibrationStatus(ADC1));              // czekaj na koniec kalibracji
}

void LED8On(void) {
    // wlaczenie diody LED podlaczonej do pinu 8 portu B
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
}

void LED9On(void) {
    // wlaczenie diody LED podlaczonej do pinu 8 portu B
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_SET);
}

void LED8Off(void) {
    // wylaczenie diody LED podlaczonej do pinu 8 portu B
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
}
void LED9Off(void) {
    // wylaczenie diody LED podlaczonej do pinu 8 portu B
    GPIO_WriteBit(GPIOB, GPIO_Pin_9, Bit_RESET);
}

void Delay(unsigned int counter){
    // opoznienie programowe
    while (counter--){   // sprawdzenie warunku
        __NOP();         // No Operation
        __NOP();         // No Operation
    }
}

