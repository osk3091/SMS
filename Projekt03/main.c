/****************************************
 * projekt01: RCC configuration         *
 ****************************************/
#include "modbus_conf.h"
#include "modbus.h"
#include "stm32f10x.h"
#include "lcd_hd44780.h"
#include "delay.h"
#include "misc.h"
#include "main.h"
#include <stdbool.h> 		// true, false
#include <stdio.h>

void RCC_Config(void);	
void GPIO_Config(void); 
void NVIC_Config(void);	
void TIM2_Config(void);	
void TIM4_Config(void);	
void ADC1_Config(void);	
void USART1_Config(void);

//char KB2char(void);
unsigned char key_buffer[32] = {0}; // bufor na znaki z obs=ugi przerwania
volatile unsigned int key_pointer = 0; // wskaünik do bufora

uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t write_single_coil_3[] = {0x00, 0x03, 0xFF, 0x00};
int main(void) {
  RCC_Config();		  // konfiguracja RCC
  GPIO_Config();    // konfiguracja GPIO
	NVIC_Config();		// konfiguracja NVIC
	TIM2_Config();		// konfigruacja TIM2
	TIM4_Config();		// konfiguracja TIM4
	ADC1_Config();		// konfiguracja ADC1
	USART1_Config();	// konfiguracja USART1
	//MB_Config( TODO );// konfiguracja MB
	
	//SysTick_Config( TODO ); // 10us // (72MHz/8) / 9000 = 1KHz (1/1KHz = 1ms)
	//NVIC_SetPriority(SysTick_IRQn, 0);
	//SysTick_CLKSourceConfig( TODO );
	
	NVIC_SetPriority(SysTick_IRQn, 0);
	
	LCD_Initialize(); // inicjalizacja LCD
	
	//unsigned int i;
	//for (i=0; i<32; i++)
	//	key_buffer[i] = ' ';
	
	while(1) {
//		LCD_GoTo(0,0);
//		for (i=0; i<16; i++){
//			LCD_WriteCharacter(key_buffer[i]);
//		}
//		LCD_GoTo(0,1);
//		for (i=16; i<32; i++){
//			LCD_WriteCharacter(key_buffer[i]);
//		}
//		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET);
//		delay_us(4000000);
//		GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET);
//		delay_us(4000000);

		// ,,Regulator''
		
		// MB_SendRequest( ... );
		MB_SendRequest(103, FUN_WRITE_SINGLE_COIL, write_single_coil_3, 4);
		// respstate = MB_GetResponse( ... );
		respstate = MB_GetResponse(103, FUN_WRITE_SINGLE_COIL, &resp, &resplen, 1000);
		if(respstate != RESPONSE_OK){
			// reakcja na blad komunikacji
		}
		
		// TODO
		
		Delay_ms(500);
	}
}


void USART1_Config(){
	USART_InitTypeDef USART_InitStruct;
		
	// Konfiguracja USART (przerwania wylaczone)
	// TODO
	USART_InitStruct.USART_BaudRate = 19200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_WordLength = USART_WordLength_9b;
	USART_InitStruct.USART_Parity = USART_Parity_Even;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStruct);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
}

void Communication_Mode(bool rx, bool tx){ // TODO
	// jesli rx == true, wlacz przerwanie USART_IT_RXNE, w przeciwnym razie wylacz
	// jesli tx == true, wlacz przerwanie USART_IT_TXE, w przeciwnym razie wylacz
	USART_ITConfig(USART1, USART_IT_RXNE, rx?ENABLE:DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE , tx?ENABLE:DISABLE);
}

void Communication_Put(uint8_t ch){ // TODO
	// wyslij znak na USART1
	USART_SendData(USART1, ch);
}

uint8_t Communication_Get(void){ // TODO
	uint8_t tmp = 0;
	// odczytaj znak z USART1
	// ustaw flage, ze znak juz nie jest do odczytu
	tmp = USART_ReceiveData(USART1);
	SetCharacterReceived(false);
	return tmp;
}

void Enable50usTimer(void){ // TODO
	// wlacz przerwanie TIM_IT_Update
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void Disable50usTimer(void){ // TODO
	// wylacz przerwanie TIM_IT_Update
	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);
}

void NVIC_Config(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	// NVIC_PriorityGroupConfig( TODO );
	
	// Klawiatura PD6 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource6);
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Klawiatura PD7 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource7);
	EXTI_InitStructure.EXTI_Line = EXTI_Line7;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Klawiatura PD8 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Klawiatura PD9 -> EXTI
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	// Klawiatura -- EXTI9_5_IRQn
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // Konfiguracja priorytetow TODO
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // Konfiguracja priorytetow TODO
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// Przetwornik ADC -- ADC1_2_IRQn
	NVIC_ClearPendingIRQ(ADC1_2_IRQn);
	NVIC_EnableIRQ(ADC1_2_IRQn);
	NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// USART1 -- USART1_IRQn
	// TODO
	// USART1 -- USART1_IRQn
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART1, ENABLE);
	
	// TIM4 (timer 50us) -- TIM4_IRQn
	// TODO
	NVIC_ClearPendingIRQ(TIM4_IRQn); // wyczyszczenie bitu przerwania
	NVIC_EnableIRQ(TIM4_IRQn); // wlaczenie obslugi przerwania
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; // nazwa przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // priorytet wywlaszczania
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // podpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // wlaczenie
	NVIC_Init(&NVIC_InitStructure); // inicjalizacja struktury
}

void RCC_Config(void) {
  ErrorStatus HSEStartUpStatus;                          // zmienna opisujaca rezultat 
  							                                         // uruchomienia HSE
  // konfigurowanie sygnalow taktujacych
  RCC_DeInit();                                          // reset ustawien RCC
  RCC_HSEConfig(RCC_HSE_ON);                             // wlacz HSE
  HSEStartUpStatus = RCC_WaitForHSEStartUp();            // czekaj na gotowosc HSE
  if(HSEStartUpStatus == SUCCESS) {
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//
    FLASH_SetLatency(FLASH_Latency_2);                   // zwloka Flasha: 2 takty 
																												 // dla 72MHz
    RCC_HCLKConfig(RCC_SYSCLK_Div1);                     // HCLK = SYSCLK
    RCC_PCLK2Config(RCC_HCLK_Div1);                      // PCLK2 = HCLK
    RCC_PCLK1Config(RCC_HCLK_Div2);                      // PCLK1 = HCLK/2
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // PLLCLK = HSE*9 
																												 // czyli 8MHz * 9 = 72 MHz
    RCC_PLLCmd(ENABLE);                                  // wlacz PLL
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);  // czekaj na uruchomienie PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);           // ustaw PLL jako zrodlo 
																							  				 // sygnalu zegarowego
    while(RCC_GetSYSCLKSource() != 0x08);                // czekaj az PLL bedzie 
                                                         // sygnalem zegarowym systemu    
		RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    // ADCCLK = PCLK2/6 = 12 MHz
	
    // konfiguracja sygnalow taktujacych uzywanych peryferii 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);// wlacz taktowanie timera TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);// wlacz taktowanie timera TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);// wlacz taktowanie timera TIM4
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);// wlacz taktowanie portu AFIO
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);// wlacz taktowanie ADC1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);// wlacz taktowanie portu GPIO A
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// wlacz taktowanie portu GPIO B
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);// wlacz taktowanie portu GPIO C
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);// wlacz taktowanie portu GPIO D
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);// wlacz taktowanie portu GPIO E
		// wlacz taktowanie USART1
		// TODO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE); // wlacz taktowanie AFIO
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // wlacz taktowanie GPIOA
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // wlacz taktowanie USART1
  }
}

void GPIO_Config(void) {
  // konfigurowanie portow GPIO
  GPIO_InitTypeDef  GPIO_InitStructure; 
	
	// Konfiguracja Tx USART1 (PA9) -- funkcja alternatywna, push-pull
	// TODO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Konfiguracja Tx USART2 (PA10) -- wejscie plywajace
	// TODO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
			
	// Konfiguracja wejscia analogowego (PB0) -- wejscie analogowe
	// TODO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // wejscie w trybie pull-up
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// Konfiguracja LEDALL (PB8-PB15) -- wyjscie, push-pull
	GPIO_InitStructure.GPIO_Pin = LEDALL;             		// pin 0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      // wejscie w trybie analogous
	GPIO_Init(GPIOB, &GPIO_InitStructure);                // inicjalizacja portu B
	
	// Konfiguracja klawiatury (PD10-PD13) -- wyjscie, otwarty dren
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	// Konfiguracja klawiatury (PD6-PD9) -- wejscie plywajace
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
}

void LCD_Refresh(void){
	LCD_GoTo(0,0);
	LCD_WriteText((unsigned char*)"Wake up!"); // wyswietl zawartosc bufora na LCD
	// Wyswietlenie wszystkich informacji jednoczesnie
	// TODO
}

void TIM2_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
		
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1;
	TIM_TimeBaseStructure.TIM_Period = 5000;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	// konfiguracja kanalu timera
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_ITConfig ( TIM2, TIM_IT_CC2 , ENABLE );
	TIM_Cmd(TIM2, ENABLE);
}
	
void TIM4_Config(void){
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	// Konfiguracja timera TIM4 do odliczania kwantow 50us
	// TODO
	TIM_TimeBaseStructure.TIM_Prescaler = 7200-1; // 72MHz/7200=10kHz
	TIM_TimeBaseStructure.TIM_Period = 10000; // 10kHz/10000=1Hz (1s)
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // zliczanie w gore
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; // brak powtorzen
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // inicjalizacja TIM4
	TIM_ITConfig ( TIM4, TIM_IT_CC2 | TIM_IT_Update, ENABLE ); // wlaczenie przerwan
	TIM_Cmd(TIM4, ENABLE); // aktywacja timera TIM4
}

void ADC1_Config(void) {
	ADC_InitTypeDef  ADC_InitStructure;
	
	ADC_DeInit(ADC1);                                   // reset ustawien ADC1
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;  // niezalezne dzialanie ADC 1 i 2
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // pomiar pojedynczego kanalu
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // pomiar automatyczny
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2; // start na przerwanie
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // pomiar wyrownany do prawej
	ADC_InitStructure.ADC_NbrOfChannel = 1;             // jeden kanal
	ADC_Init(ADC1, &ADC_InitStructure);                 // inicjalizacja ADC1

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_41Cycles5); // ADC1, kanal 16, 
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);                              // aktywacja ADC1
		
	ADC_ResetCalibration(ADC1);                         // reset rejestru kalibracji ADC1
	while(ADC_GetResetCalibrationStatus(ADC1));         // oczekiwanie na koniec resetu
	ADC_StartCalibration(ADC1);                         // start kalibracji ADC1
	while(ADC_GetCalibrationStatus(ADC1));              // czekaj na koniec kalibracji
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
		Delay_ms(5);
		GPIO_Pin_col = GPIO_Pin_6;
		for(j=0;j<4;++j){
			if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_col) == 0){
				GPIO_ResetBits(GPIOD, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13);
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
		case LED_TOGGLE: GPIO_WriteBit(GPIOB, led, (GPIO_ReadOutputDataBit(GPIOB, led) == Bit_SET?Bit_RESET:Bit_SET));
	}
}
