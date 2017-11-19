#include "stm32f10x.h"
#include "lcd_hd44780.h"

#include "main.h"
#include "delay.h"
#include "stdio.h"
#include <stdbool.h>   // true, false



bool RCC_Config(void);
void GPIO_Config(void);
void GPIO_SW0_Config(void); 
void GPIO_TIM4_Config(void); 
void GPIO_ADC_Config(void);
unsigned int readADC(void);
void GPIO_KB_Config(void);
void NVIC_Config(void);
void refreshLCD(void);

char KB2char(void);

unsigned char key_buff[] ;
volatile unsigned int key_pt;

const unsigned char KBkody[16] = {'1','2','3','A',\
							  '4','5','6','B',\
							  '7','8','9','C',\
							  '*','0','#','D'};
char buffer[32];
								int i;
int main(void) {	
  RCC_Config();	      // konfiguracja RCC
  NVIC_Config();      // konfiguracja NVIC
  GPIO_Config();      // konfiguracja GPIO
  GPIO_TIM4_Config(); // konfiguracja TIM4
  GPIO_SW0_Config();  // konfiguracja SW0
  GPIO_ADC_Config();  // konfiguracja ADC
  GPIO_KB_Config();   // konfiguracja KB
	
  LCD_Initialize();   // inicjalizacja LCD
	
	SysTick_Config(9000); // (72MHz/8) / 9000 = 1KHz (1/1KHz = 1ms)
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	//LED(LEDALL, LED_OFF);
	
	//sprintf(buffer, "ADC: %4.1fV", 3.3/4096);
	//LCD_GoTo(0, 0);
	
	//LCD_WriteText(buffer);
	for(i=0; i<32; i++)
		key_buff[i]='\0';
	
	while(1) {
		LCD_GoTo(0,0);
		for(i=0; i<16; i++){
			LCD_WriteData(key_buff[i]);
		}
		LCD_GoTo(0,1);
		for(i=16; i<32; i++){
			LCD_WriteData(key_buff[i]);
		}
		
		LED(LED1, LED_TOGGLE);

		Delay(4000);
		
		

		// TODO
	}
}

void refreshLCD(void){
	// TODO
}

bool RCC_Config(void) {
  ErrorStatus HSEStartUpStatus;                          // zmienna opisujaca rezultat 
																												 // uruchomienia HSE
  // konfigurowanie sygnalow taktujacych
  RCC_DeInit();                                          // reset ustawień RCC
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
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);
		
    // konfiguracja sygnalow taktujacych uzywanych peryferii 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// wlacz taktowanie portu GPIO A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// wlacz taktowanie portu GPIO B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// wlacz taktowanie portu GPIO C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);// wlacz taktowanie portu GPIO D
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);// wlacz taktowanie timera TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);// wlacz taktowanie timera TIM3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);// wlacz taktowanie timera TIM4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);// wlacz taktowanie AFIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);// wlacz taktowanie przetwornika ADC1
    return true;
  }
  return false;
}

void GPIO_ADC_Config(void){
	ADC_InitTypeDef  ADC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	
	ADC_DeInit(ADC1);                                   // reset ustawien ADC1  
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // niezalezne dzialanie ADC 1 i 2
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // pomiar pojedynczego kanalu
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  // pomiar automatyczny
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2; // T2CC2->ADC
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // pomiar wyrownany do prawej
	ADC_InitStructure.ADC_NbrOfChannel = 1;             // jeden kanal
	ADC_Init(ADC1, &ADC_InitStructure);                 // inicjalizacja ADC1

	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_41Cycles5); // konf.
	ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	ADC_Cmd(ADC1, ENABLE);                              // aktywacja ADC1

	ADC_ResetCalibration(ADC1);                         // reset rejestru kalibracji ADC1
	while(ADC_GetResetCalibrationStatus(ADC1));         // oczekiwanie na koniec resetu
	ADC_StartCalibration(ADC1);                         // start kalibracji ADC1
	while(ADC_GetCalibrationStatus(ADC1));              // czekaj na koniec kalibracji

	ADC_TempSensorVrefintCmd(ENABLE);                   // wlaczenie czujnika temperatury

  // konfiguracja timera TIM2
	// TODO
  
  // konfiguracja kanalu 2 timera TIM2
	// TODO
	
	// wlaczenie przerwan timera TIM2
	// TODO	
	
	// konfiguracja ADC1
	// TODO
	
	// konfiguracja przerwań ADC1
	// TODO
	
	// kalibracja ADC1
	// TODO
	
	// wlaczyc czujnik temperatury
	// TODO
}

unsigned int readADC(void){
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);                  // start pomiaru
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);   // czekaj na koniec pomiaru
    return ADC_GetConversionValue(ADC1);                     // odczyt pomiaru (12 bit)
} 





void NVIC_Config(void) {
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	
	NVIC_ClearPendingIRQ(TIM4_IRQn);                    // wyczyszczenie bitu przerwania
	NVIC_EnableIRQ(TIM4_IRQn);                          // wlaczenie obslugi przerwania
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;     // nazwa przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // priorytet wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     // wlaczenie
	NVIC_Init(&NVIC_InitStructure);  
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0); // PA0 -> EXTI0_IRQn 
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;                  // linia : 0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;         // tryb  : przerwanie
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     // zbocze: opadajace
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;                   // aktywowanie konfig.
	EXTI_Init(&EXTI_InitStructure);                             // inicjalizacja

	NVIC_ClearPendingIRQ(EXTI0_IRQn);                           // czyszcz. bitu przerw.
	NVIC_EnableIRQ(EXTI0_IRQn);                                 // wlaczenie przerwania
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;            // przerwanie EXTI0_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   // prior. wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;          // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // aktywowanie konfig.
	NVIC_Init(&NVIC_InitStructure);                             // inicjalizacja
	

	NVIC_ClearPendingIRQ(TIM3_IRQn);                            // czyszcz. bitu przerw.
	NVIC_EnableIRQ(TIM3_IRQn);                                  // wlaczenie przerwania
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;             // nazwa przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   // prior. wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // aktywowanie konfig.
	NVIC_Init(&NVIC_InitStructure);                             // inicjalizacja
	
	NVIC_ClearPendingIRQ(TIM2_IRQn);                            // czyszcz. bitu przerw.
	NVIC_EnableIRQ(TIM2_IRQn);                                  // wlaczenie przerwania
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;             // nazwa przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // prior. wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;          // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // aktywowanie konfig.
	NVIC_Init(&NVIC_InitStructure);                             // inicjalizacja
	
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);                            // czyszcz. bitu przerw.
	NVIC_EnableIRQ(ADC1_2_IRQn);                                  // wlaczenie przerwania
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;             // nazwa przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // prior. wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;          // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // aktywowanie konfig.
	NVIC_Init(&NVIC_InitStructure);                             // inicjalizacja
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource6); // PA0 -> EXTI0_IRQn 
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;                  // linia : 0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;         // tryb  : przerwanie
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     // zbocze: opadajace
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;                   // aktywowanie konfig.
	EXTI_Init(&EXTI_InitStructure);                             // inicjalizacja

	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);                           // czyszcz. bitu przerw.
	NVIC_EnableIRQ(EXTI9_5_IRQn);                                 // wlaczenie przerwania
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;            // przerwanie EXTI0_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;   // prior. wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;          // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // aktywowanie konfig.
	NVIC_Init(&NVIC_InitStructure);                             // inicjalizacja
	
}

void GPIO_Config(void) {
  // konfigurowanie portow GPIO
  GPIO_InitTypeDef  GPIO_InitStructure; 
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;        // pin 8
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // czestotliwosc zmiany 2MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // wyjscie w trybie push-pull
	GPIO_Init(GPIOB, &GPIO_InitStructure);           // inicjaliacja portu B
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // wejscie w trybie pull-up
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;        // pin 8
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // czestotliwosc zmiany 2MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; // wyjscie w trybie push-pull
	GPIO_Init(GPIOD, &GPIO_InitStructure);           // inicjaliacja portu D
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;        // pin 8
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // czestotliwosc zmiany 2MHz
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // wyjscie w trybie push-pull
	GPIO_Init(GPIOD, &GPIO_InitStructure);           // inicjaliacja portu D
	

	
	// PB6 -- przycisk zewnetrzny bez podciagniecia
	// TODO
	
	// PD10-13 -- klawiatura
	// TODO
	
	// PD6-9 -- klawiatura
	// TODO
	
	// LEDALL -- wszystkie diody swiecace (port B)
	// TODO
	
	// PA0 -- przycisk z podciagnieciem (SW0)
	// TODO
}

void GPIO_TIM4_Config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;               // 72MHz/7200=10kHz
	TIM_TimeBaseStructure.TIM_Period = 10000;                   // 10kHz/10000=1Hz (1s)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // zliczanie w gore
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            // brak powtorzen
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);             // inicjalizacja TIM4
	TIM_ITConfig ( TIM4, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_Update, ENABLE );  // wlaczenie przerwan
	TIM_Cmd(TIM4, ENABLE);                                      // aktywacja timera TIM4

	// konfiguracja kanalu 1 timera
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;         // brak zmian OCxREF
	TIM_OCInitStructure.TIM_Pulse = 2000;                       // wartosc do porownania
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // wlaczenie kanalu
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);   
	
	// konfiguracja kanalu 2 timera
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;         // brak zmian OCxREF
	TIM_OCInitStructure.TIM_Pulse = 7000;                       // wartosc do porownania
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // wlaczenie kanalu
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);     
	
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;               // 72MHz/7200=10kHz
	TIM_TimeBaseStructure.TIM_Period = 350;                     // 10kHz/350~=29Hz(35ms)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // zliczanie w gore
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;            // brak powtorzen
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);             // inicjalizacja TIM3
	TIM_ITConfig ( TIM3, TIM_IT_Update, DISABLE );              // wylaczenie przerwan
	TIM_Cmd(TIM3, DISABLE);                                     // wylaczenie timera
	




	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_Period = 5000; // 2Hz -> 0.5s
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// konfiguracja kanalu timera
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 1; // minimalne przesuniecie
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_ITConfig ( TIM2, TIM_IT_CC2 , ENABLE );
	TIM_Cmd(TIM2, ENABLE);
	
		// konfiguracja timera TIM4
	// TODO
  
  // konfiguracja kanalu 1 timera TIM4
	// TODO
	
  // konfiguracja kanalu 2 timera TIM4
	// TODO
	
	// wlaczenie przerwan timera TIM4
	// TODO	
}

void GPIO_SW0_Config(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	// konfiguracja EXTI dla PA0
	// TODO
	
  // konfiguracja timera TIM3 do realizacji opoznienia
	// TODO	
}

void GPIO_KB_Config(void) {
	EXTI_InitTypeDef EXTI_InitStructure;
	
	// konfiguracja EXTI dla PD6
	// TODO
	
	// konfiguracja EXTI dla PD7
	// TODO
	
	// konfiguracja EXTI dla PD8
	// TODO
	
	// konfiguracja EXTI dla PD9
	// TODO
}

char KB2char(void){
	unsigned int GPIO_Pin_row, GPIO_Pin_col, i, j;
	const unsigned char KBkody[16] = {'1','2','3','A',\
							  '4','5','6','B',\
							  '7','8','9','C',\
							  '*','0','#','D'};
	GPIO_SetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
	GPIO_Pin_row = GPIO_Pin_10;
	for(i=0;i<4;++i){
		GPIO_ResetBits(GPIOD, GPIO_Pin_row);
		Delay(5); // opoznienie 5 ms na ustalenie sygnalow
		GPIO_Pin_col = GPIO_Pin_6;
		for(j=0;j<4;++j){
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_col) == 0){
				GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|
				  GPIO_Pin_12|GPIO_Pin_13);
				return KBkody[4*i+j];
			}
			GPIO_Pin_col = GPIO_Pin_col << 1;
		}
		GPIO_SetBits(GPIOD, GPIO_Pin_row);
		GPIO_Pin_row = GPIO_Pin_row << 1;
	}
	GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
	return 0;
}

void LED(uint16_t led, enum LED_ACTION act) {
	switch(act){
		case LED_ON: GPIO_SetBits(GPIOB, led); break;
		case LED_OFF: GPIO_ResetBits(GPIOB, led); break;
		case LED_TOGGLE: GPIO_WriteBit(GPIOB, led, 
			(GPIO_ReadOutputDataBit(GPIOB, led) == Bit_SET?Bit_RESET:Bit_SET));
	}
}
