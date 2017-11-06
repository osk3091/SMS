/****************************************
* 	projekt00 : symulacja komputerowa 	*
****************************************/

#include "stm32f10x.h"

char strDst[32] = "\0";
int copyStr(char* , char* );

int main(void){
	copyStr(strDst, "Source String : 0123456789");
	while(1);
}

int copyStr(char * dst, char * src){
	int counter = 0;
	while (src[counter] != '\0') {
		dst[counter] = src[counter];
		++counter;
	}
	return counter;
}
