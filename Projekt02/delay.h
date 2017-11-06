#ifndef __DELAY_H
#define __DELAY_H

static unsigned int msc = 0; // zmienna przechowujaca czas pozostaly czas opoznienia

void DelayTick(void);        // aktualizacja czasu opoznienia
void Delay(unsigned int ms); // ustawienie czasu opoznienia i zawieszenie dzialania programu

#endif