#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "uart.c"
#include "ds18b20.h"
#include "BMP180.h"
#include "dht.h"
#include "I2C_TWI/i2c_twi.h"

//#define DEBUG 1 //1 or 2 lvl or comment for disable
#define SECONDS 600
#define ADC_BAT PA2
//SW
#define BAUD 57600
#define BIT_TIME_US 8 // (1000ms/1200baudeRate)
#define SUART_PORT PORTA
#define SUART_DDR  DDRA
#define SUART_PIN  PA1
#define SUART_PIN_m _BV(SUART_PIN)
//HW
#define UART_BAUD_RATE 115200

//r=7cm
//2.40114km/h 66m/s
const float WIND_FACTOR = 66.69;
volatile unsigned short wnd_rotates = 0;

volatile unsigned short timer1_cnt = 0;
char printbuff[100];

//bmp180
volatile unsigned short pressure;

//dht22
#if DHT_FLOAT == 0
	int8_t temperature = 0;
	int8_t humidity = 0;
#elif DHT_FLOAT == 1
	float temperature = 0;
	float humidity = 0;
#endif
float bat_v;
char *body_packet_tmpl = "GET /ws/save.php?ws=%.1f&hm=%d&te=%.1f&tb=%.1f&p=%d&bv=%.2f HTTP/1.1\r\nHost: 192.168.0.33\r\nConnection: close\r\n\r\n";
char *send_packet = "AT+CIPSEND=4,%d\r\n";
#ifdef DEBUG
/**
 * software uart init
 */
void suart_init(void) {
	SUART_DDR |= SUART_PIN_m;
}

/**
 * software uart
 */
void suart_putc(char c) {
	uint8_t i;
	// Start bit
	SUART_PORT &= ~SUART_PIN_m;
	_delay_us(BIT_TIME_US);

	// Shift out. LSB first.
	for (i = 0; i < 8; i++) {
		if (c & 1)
			SUART_PORT |= SUART_PIN_m;
		else
			SUART_PORT &= ~SUART_PIN_m;
		c >>= 1;
		_delay_us(BIT_TIME_US);
	}
	// Stop bit
	SUART_PORT |= SUART_PIN_m;
	_delay_us(BIT_TIME_US);
}
/**
 * sftware uart
 */
void suart_puts(char *s) {
	while (*s)
		suart_putc(*s++);
}
#endif
/**
 * recive from hw uart
 */
int uart_recive(){
	/*
	 * Get received character from ringbuffer
	 * uart_getc() returns in the lower byte the received character and
	 * in the higher byte (bitmask) the last receive error
	 * UART_NO_DATA is returned when no data is available.
	 *
	 */
	unsigned int c = uart_getc();
	if (c & UART_NO_DATA) {
		#if DEBUG
			suart_puts("ESP: no data\n\r");
		#endif
		return 0;
	} else {
		/*
		 * new data available from UART
		 * check for Frame or Overrun error
		 */
		if (c & UART_FRAME_ERROR) {
			/* Framing Error detected, i.e no stop bit detected */
			#if DEBUG
				suart_puts("ESP Frame Error:\n\r");
			#endif
			return 0;
		}
		if (c & UART_OVERRUN_ERROR) {
			/*
			 * Overrun, a character already present in the UART UDR register was
			 * not read by the interrupt handler before the next character arrived,
			 * one or more received characters have been dropped
			 */
			#if DEBUG
				suart_puts("ESP Overrun Error:\n\r");
			#endif
			return 0;
		}
		if (c & UART_BUFFER_OVERFLOW) {
			/*
			 * We are not reading the receive buffer fast enough,
			 * one or more received character have been dropped
			 */
			#if DEBUG
				suart_puts("ESP Buffer overflow error:\n\r");
			#endif
			return 0;
		}
		/*
		 * send received character
		 */
		#if DEBUG
			suart_puts( &c );
			suart_puts("\r\n");
		#endif
		return 1;


	}
}
/**
 * send to esp
 */
void esp_send(char *send){
	uart_puts(send);
	#if DEBUG
		suart_puts("ESP UART: ");
	#endif
		while(uart_recive());
	#if DEBUG
		suart_puts("\n\r");
	#endif
	_delay_ms(500);
}
/**
 * read dht22
 */
void dht22Read(){
	if(dht_gettemperaturehumidity(&temperature, &humidity) != -1) {

		#if DHT_FLOAT == 0
		itoa(temperature, printbuff, 10);
		#elif DHT_FLOAT == 1
		dtostrf(temperature, 3, 3, printbuff);
		#endif
		#if DEBUG
		suart_puts("dht22 temperature: "); suart_puts(printbuff); suart_puts("C");suart_puts("\n\r");
		#endif
		#if	 DHT_FLOAT == 0
		itoa(humidity, printbuff, 10);
		#elif DHT_FLOAT == 1
		dtostrf(humidity, 3, 3, printbuff);
		#endif
		#if DEBUG
		suart_puts("dht22 humidity: "); suart_puts(printbuff); suart_puts(" %RH");suart_puts("\n\r");
		#endif

	} else {
		#if DEBUG
		suart_puts("dht22 error"); suart_puts("\n\r");
		#endif
	}
}
void bmp180Read(){
	BMP180_gett();
	pressure = (BMP180_getp() / 100);

	#if DEBUG
		ltoa(pressure, printbuff, 10);
		suart_puts("bmp180 pressure: ");
		suart_puts(printbuff);
		suart_puts(".");
	#endif
}
/**
 * sensor measure timer
 */
void initMeasureTimer(){
	TIMSK |= _BV(OCIE1A);
	TCCR1B |= _BV(WGM12);
	//http://fabacademy.org/archives/content/tutorials/09_Embedded_Programming/1sec/index.htm
	OCR1A = 46876 ; //16000000 / 1024 = r ; 65535/r = x ; 65535/x = 1second increment //every 3 sec
	TCCR1B |= _BV(CS10) | _BV(CS12);//1024
}
void initInt1(){
	//pd3 wejscie
	//wiatromierz przerwanie
	DDRD  &= ~ _BV(PD3);    // Configure PD3 as input
	PORTD |= _BV(PD3);		// Set 1
	// MCUCR default triggers on LOW level. Changing to edge trigger requires different sleep mode to wake up.
	MCUCR |= _BV(ISC11)|_BV(ISC10); //Tabel 34
	GICR  |=   _BV(INT1);   // Enable INT1 PD3
}
void esp_on(){
	PORTC |= _BV(PINC7);
}
void esp_off(){
	PORTC &= ~_BV(PINC7);
}
void battery_voltage(){
	//PA3 ADC3
}
void enableADC(){
	ADCSRA = _BV(ADEN); //enable adc
	ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
	ADMUX = _BV(REFS0) ;//| _BV(REFS1) ; // use AREFs
	ADMUX |= _BV(MUX1); //ADC2
	DDRA &=~ _BV(ADC_BAT); //ADC2 as input
	PORTA &=~ _BV(ADC_BAT); //ADC2 set low as +
}
void disableADC(){
	ADCSRA &= ~_BV(ADEN);
	PORTA |= _BV(ADC_BAT);
}
void readADC(){
	enableADC();
	ADCSRA |= _BV(ADSC);//conversion start
	while(ADCSRA & (1<<ADSC)); //wait for end
#ifdef DEBUG
	suart_puts("ADC:");
	itoa(ADC,printbuff,10);
	suart_puts(printbuff);
	suart_puts("\r\n");
#endif
	bat_v = (ADC * (3.3 / 1024)) * 2; // ref 3.3 *2 dzielnik;
#ifdef DEBUG
	dtostrf(bat_v, 1, 2, printbuff);
	suart_puts(printbuff);
	suart_puts("\r\n");
#endif
	disableADC();
}
void goSleep(){
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_mode();
}
void read_sensors(){
	float windSpeed = ((wnd_rotates * WIND_FACTOR) / 100) / SECONDS;
	wnd_rotates = 0;
	float batTemp = (float) ds18b20_gettemp();

#if DEBUG
	dtostrf(windSpeed, 6, 2, printbuff);
	suart_puts("windSpeed:");
	suart_puts(printbuff);
	suart_puts(" m/s \r\n");
	dtostrf(batTemp, 6, 2, printbuff);
	suart_puts("batTemp:");
	suart_puts(printbuff);
	suart_puts("\r\n");
#endif

	dht22Read();
	bmp180Read();
	readADC();
	_delay_ms(5000);
	esp_send("AT+CIFSR\r\n");
	esp_send("AT+CWMODE=1\r\n");
	esp_send("AT+CIPMUX=1\r\n");
	esp_send("AT+CIPSTART=4,\"TCP\",\"192.168.0.33\",80\r\n");
	char body_packet[120];
	sprintf(body_packet, body_packet_tmpl, windSpeed, (char) humidity, temperature, batTemp, pressure, bat_v);
#ifdef DEBUG
	suart_puts(body_packet);
	suart_puts("\r\n");
#endif
	unsigned char length = strlen(body_packet);
	sprintf(printbuff, send_packet, length);
#ifdef DEBUG
	suart_puts(printbuff);
	suart_puts("\r\n");
#endif
	esp_send(printbuff);
	esp_send(body_packet);
	esp_send("AT+CIPCLOSE=4\r\n");
#ifdef DEBUG
	suart_puts("packet send \r\n");
#endif
}
int main(void) {

	DDRA = 0xFF;
	PORTA = 0xFF;
	DDRB = 0xFF;
	PORTB = 0xFF;
	DDRC = 0xFF;
	PORTC = 0xFF;
	DDRD = 0xFF;
	PORTD = 0xFF;

	DS18B20_DDR &= ~_BV(DS18B20_DQ);
	DS18B20_PORT |= _BV(DS18B20_DQ);

	DDRA |= _BV(PINA0); //led as out
	PORTA |= _BV(PINA0);//led disable

	initInt1();

	initMeasureTimer();

	//init uart
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	// DHT22 set bitrate
	i2cSetBitrate(100);
	// inicjalizacja BMP180 - pobranie danych calibracyjnych
	BMP180_init();

	//init global interrupts
	sei();
	//soft uart init
#ifdef DEBUG
	suart_init();
	suart_puts("started.\r\n");
#endif
	esp_on();
	read_sensors();
	esp_off();

	for (;;) {
#ifdef DEBUG
		suart_puts("go sleep \r\n");
#endif
		goSleep();
	}
	
	return 0;
}

ISR(INT1_vect) {

	wnd_rotates++;
	#if DEBUG==2
		itoa(wnd_rotates, printbuff, 10 );
		suart_puts(printbuff);
		suart_puts("\r\n");
	#endif
}
ISR(TIMER1_COMPA_vect) {
	timer1_cnt+=3;//interrupt every 3 seconds
	if (timer1_cnt == SECONDS) {
		sleep_disable();
		esp_on();
		timer1_cnt = 0;
		read_sensors();
		esp_off();
		goSleep();
	}

}
