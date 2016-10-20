/*
 * main.c
 *
 *  Created on: 28 kwi 2014
 *  Autor: £ukasz £uczko
 *
 */

#ifndef BMP180_H_
#define BMP180_H_

#define BMP180_ADDR 0xEE  	// Adres I2C
#define BMP180_MODE 0		// oversampling setting (0-3)

void BMP180_init( void );	// Inicjalizacja czujnika
void BMP180_getut(void);	// Odczyt temperatury bez kompensacji
void BMP180_getup(void);	// odczyt ciœnienia bez kompensacji
long BMP180_gett(void);		// Wylicza temperaturê
long BMP180_getp(void);		// Wylicza ciœnienie

#endif /* BMP180_H_ */
