/*
 * PROJEKT: Zdalnie sterowany samochodzik (ATmega2560)
 * MODUŁY: HC-05, 2xL298N, LCD I2C, HC-SR04, Buzzer, PCA9685, Fotorezystor
 *
 * PINY:
 * - UART1 (Bluetooth): TX=Pin18(PD3), RX=Pin19(PD2)
 * - I2C (LCD, PCA9685): SDA=Pin20, SCL=Pin21 (Adresy: LCD=0x27, PCA=0x40)
 * - L298N #1 (Przód): ENA=Pin5, IN1=22, IN2=23 | ENB=Pin6, IN3=24, IN4=25
 * - L298N #2 (Tył): ENC=Pin7, IN5=26, IN6=27 | END=Pin8, IN7=28, IN8=29
 * - HC-SR04 (Dystans): TRIG=Pin38(PD7), ECHO=Pin48(PL1/ICP5)
 * - INNE: Buzzer=Pin11(PB5/OC1A), Foto=A0(PF0/ADC0)
 *
 * TIMERY I PERYFERIA:
 * - T0 (CTC): Zegar systemowy (millis)
 * - T1 (CTC Toggle): Generowanie fali dla Buzzera
 * - T2 (Polling): Aktywne opóźnienia (czekaj_us / czekaj_ms)
 * - T3 & T4 (Fast PWM): Sterowanie prędkością silników (ENA-END)
 * - T5 (Input Capture): Pomiar echa z HC-SR04
 * - ADC: Przerwania, preskaler 128 (odczyt fotorezystora)
 *
 * KOMENDY (UART):
 * - RUCH: F(Przód), B(Tył), L/R(Pivot), X/Y(Bok), I/K(Skos), S(Stop)
 * - NAPĘD: + (Szybciej), - (Wolniej)
 * - ŚWIATŁA: D(DRL Manual), A(DRL Auto), Q/E(Kierunkowskazy L/P), H(Awaryjne)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define F_CPU 16000000UL

/* Timer2 - polling, prescaler 8, 0.5us/tik - czekaj_us() i czekaj_ms() */

/*!
 *  @brief Blokujące opóźnienie w mikrosekundach oparte na Timer2.
 *  @param us liczba mikrosekund do odczekania.
 *  @returns nic.
 *  @side effects: blokuje CPU na czas opóźnienia, modyfikuje TCCR2A, TCCR2B, TCNT2.
 */
static void czekaj_us(uint32_t us) {
  uint32_t cnt = us;
  TCCR2A = 0U;
  TCCR2B = (uint8_t)(1U << CS21); /* prescaler 8 -> 0.5us/tik */
  while (cnt != 0U) {
    uint8_t sreg = SREG;
    cli();
    TCNT2 = 0U;
    while (TCNT2 < 2U) { ; }
    SREG = sreg;
    cnt--;
  }
  TCCR2B = 0U;
}

/*!
 *  @brief Blokujące opóźnienie w milisekundach oparte na Timer2.
 *  @param ms liczba milisekund do odczekania.
 *  @returns nic.
 *  @side effects: blokuje CPU na czas opóźnienia, wywołuje czekaj_us() 1000 razy na ms.
 */
static void czekaj_ms(uint32_t ms) {
  uint32_t cnt = ms;
  while (cnt != 0U) {
    czekaj_us(1000UL);
    cnt--;
  }
}

/* Timer0 - CTC, prescaler 64, OCR0A=249 -> przerwanie co 1ms -> millis() */

static volatile uint32_t ms_count = 0U;

ISR(TIMER0_COMPA_vect) {
  ms_count++;
}

/*!
 *  @brief Inicjalizacja Timera0 w trybie CTC, przerwanie co 1ms.
 *  @returns nic.
 *  @side effects: ustawia TCCR0A, TCCR0B, OCR0A, TIMSK0.
 */
static void timer0_init(void) {
  TCCR0A = (uint8_t)(1U << WGM01);
  TCCR0B = (uint8_t)((1U << CS01) | (1U << CS00));
  OCR0A = 249U;
  TIMSK0 = (uint8_t)(1U << OCIE0A);
}

/*!
 *  @brief Zwraca czas od startu w milisekundach.
 *  @returns liczba milisekund od uruchomienia programu (uint32_t).
 *  @side effects: chwilowo wyłącza przerwania podczas odczytu ms_count.
 */
static uint32_t millis(void) {
  uint32_t t;
  uint8_t sreg = SREG;
  cli();
  t = ms_count;
  SREG = sreg;
  return t;
}

/* UART0 i UART1 - 9600 baud @ 16MHz, UBRR=103 */

#define BAUD 9600UL
/* UBRR = F_CPU / (16 * BAUD) - 1 = 16000000 / 153600 - 1 = 103; high byte = 0 */
#define UBRR_VAL_H ((uint8_t)0U)
#define UBRR_VAL_L ((uint8_t)103U)

/*!
 *  @brief Inicjalizacja UART0 (USB/debug), 9600 8N1.
 *  @returns nic.
 *  @side effects: ustawia UBRR0, UCSR0B, UCSR0C.
 */
static void uart0_init(void) {
  UBRR0H = UBRR_VAL_H;
  UBRR0L = UBRR_VAL_L;
  UCSR0B = (uint8_t)((1U << RXEN0) | (1U << TXEN0));
  UCSR0C = (uint8_t)((1U << UCSZ01) | (1U << UCSZ00));
}

/*!
 *  @brief Sprawdza czy w buforze UART0 czeka bajt.
 *  @returns true jeśli jest bajt do odczytania, false otherwise.
 */
static bool uart0_dostepny(void) {
  return ((UCSR0A & (uint8_t)(1U << RXC0)) != 0U);
}

/*!
 *  @brief Odczytuje jeden bajt z UART0.
 *  @returns odebrany bajt (uint8_t).
 *  @side effects: blokuje jeśli bufor pusty - sprawdź uart0_dostepny() wcześniej.
 */
static uint8_t uart0_czytaj(void) {
  return UDR0;
}

/*!
 *  @brief Wysyła jeden znak przez UART0.
 *  @param c znak do wysłania.
 *  @returns nic.
 *  @side effects: blokuje CPU do momentu zwolnienia bufora nadawczego.
 */
static void uart0_putc(char c) {
  while ((UCSR0A & (uint8_t)(1U << UDRE0)) == 0U) { ; }
  UDR0 = c;
}

/*!
 *  @brief Wysyła napis przez UART0.
 *  @param s wskaźnik na napis zakończony '\0'.
 *  @returns nic.
 *  @side effects: blokuje CPU na czas wysyłania każdego znaku.
 */
static void uart0_puts(const char *s) {
  uint8_t i = 0U;
  while (s[i] != '\0') {
    uart0_putc(s[i]);
    i++;
  }
}

/*!
 *  @brief Inicjalizacja UART1 (HC-05 Bluetooth), 9600 8N1.
 *  @returns nic.
 *  @side effects: ustawia UBRR1, UCSR1B, UCSR1C.
 */
static void uart1_init(void) {
  UBRR1H = UBRR_VAL_H;
  UBRR1L = UBRR_VAL_L;
  UCSR1B = (uint8_t)((1U << RXEN1) | (1U << TXEN1));
  UCSR1C = (uint8_t)((1U << UCSZ11) | (1U << UCSZ10));
}

/*!
 *  @brief Sprawdza czy w buforze UART1 czeka bajt.
 *  @returns true jeśli jest bajt do odczytania, false otherwise.
 */
static bool uart1_dostepny(void) {
  return ((UCSR1A & (uint8_t)(1U << RXC1)) != 0U);
}

/*!
 *  @brief Odczytuje jeden bajt z UART1.
 *  @returns odebrany bajt (uint8_t).
 *  @side effects: blokuje jeśli bufor pusty - sprawdź uart1_dostepny() wcześniej.
 */
static uint8_t uart1_czytaj(void) {
  return UDR1;
}

/*!
 *  @brief Wysyła jeden znak przez UART1.
 *  @param c znak do wysłania.
 *  @returns nic.
 *  @side effects: blokuje CPU do momentu zwolnienia bufora nadawczego.
 */
static void uart1_putc(char c) {
  while ((UCSR1A & (uint8_t)(1U << UDRE1)) == 0U) { ; }
  UDR1 = c;
}

/*!
 *  @brief Wysyła napis przez UART1.
 *  @param s wskaźnik na napis zakończony '\0'.
 *  @returns nic.
 *  @side effects: blokuje CPU na czas wysyłania każdego znaku.
 */
static void uart1_puts(const char *s) {
  uint8_t i = 0U;
  while (s[i] != '\0') {
    uart1_putc(s[i]);
    i++;
  }
}

/*!
 *  @brief Wysyła liczbę całkowitą przez wybrany UART.
 *  @param port 0 = UART0, 1 = UART1.
 *  @param n liczba do wysłania (int32_t).
 *  @returns nic.
 *  @side effects: blokuje CPU na czas wysyłania.
 */
static void uart_wyslij_liczbe(uint8_t port, int32_t n) {
  char buf[12];
  uint8_t i = 0U;
  int32_t val = n;
  void (*wyslij_znak)(char) = ((port == 0U) ? uart0_putc : uart1_putc);

  if (val < 0) {
    wyslij_znak('-');
    val = -val;
  }
  if (val == 0) {
    wyslij_znak('0');
  } else {
    while (val > 0) {
      int32_t d = (int32_t)'0' + (val % 10);
      buf[i] = (char)d;
      i++;
      val /= 10;
    }
    while (i > 0U) {
      i--;
      wyslij_znak(buf[i]);
    }
  }
}

/*!
 *  @brief Wysyła napis przez oba UARTy z CRLF na końcu.
 *  @param s wskaźnik na napis zakończony '\0'.
 *  @returns nic.
 *  @side effects: blokuje CPU na czas wysyłania.
 */
static void info(const char *s) {
  uart0_puts(s);
  uart0_puts("\r\n");
  uart1_puts(s);
  uart1_puts("\r\n");
}

/* TWI (I2C) - 100 kHz, TWBR=72, prescaler=1 */

#define I2C_TIMEOUT 10000U

/*!
 *  @brief Inicjalizacja magistrali TWI w trybie master, 100 kHz.
 *  @returns nic.
 *  @side effects: ustawia TWSR, TWBR, TWCR.
 */
static void twi_init(void) {
  TWSR = 0x00U;
  TWBR = (uint8_t)72U;
  TWCR = (uint8_t)(1U << TWEN);
}

/*!
 *  @brief Wysyła warunek START na magistralę TWI.
 *  @returns true jeśli START wysłany poprawnie, false przy przekroczeniu czasu.
 *  @side effects: modyfikuje TWCR, blokuje CPU do potwierdzenia lub timeoutu.
 */
static bool twi_start(void) {
  bool ok;
  uint16_t t = 0U;
  TWCR = (uint8_t)((1U << TWINT) | (1U << TWSTA) | (1U << TWEN));
  while ((TWCR & (uint8_t)(1U << TWINT)) == 0U) {
    t++;
    if (t > (uint16_t)I2C_TIMEOUT) { break; }
  }
  ok = (t <= (uint16_t)I2C_TIMEOUT);
  return ok;
}

/*!
 *  @brief Wysyła warunek STOP na magistralę TWI.
 *  @returns nic.
 *  @side effects: modyfikuje TWCR, zwalnia magistralę.
 */
static void twi_stop(void) {
  TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN) | (1U << TWSTO));
}

/*!
 *  @brief Wysyła jeden bajt przez magistralę TWI.
 *  @param bajt bajt do wysłania.
 *  @returns true jeśli bajt wysłany poprawnie, false przy przekroczeniu czasu.
 *  @side effects: modyfikuje TWDR, TWCR, blokuje CPU do potwierdzenia lub timeoutu.
 */
static bool twi_wyslij_bajt(uint8_t bajt) {
  bool ok;
  uint16_t t = 0U;
  TWDR = bajt;
  TWCR = (uint8_t)((1U << TWINT) | (1U << TWEN));
  while ((TWCR & (uint8_t)(1U << TWINT)) == 0U) {
    t++;
    if (t > (uint16_t)I2C_TIMEOUT) { break; }
  }
  ok = (t <= (uint16_t)I2C_TIMEOUT);
  return ok;
}

/* LCD HD44780 w trybie 4-bit przez PCF8574T (addr 0x27) */
/* Mapowanie PCF: P0=RS P1=RW P2=EN P3=BL P4-P7=D4-D7 */

#define LCD_ADDR_W ((uint8_t)(0x27U << 1U))
#define LCD_RS ((uint8_t)(1U << 0U))
#define LCD_EN ((uint8_t)(1U << 2U))
#define LCD_BL ((uint8_t)(1U << 3U))

/*!
 *  @brief Wysyła bajt do ekspandera PCF8574T przez TWI.
 *  @param b bajt do wysłania (stan pinów P0-P7).
 *  @returns nic.
 *  @side effects: wykonuje transakcję TWI, modyfikuje stan pinów LCD.
 */
static void pcf_wyslij(uint8_t b) {
  if (twi_start()) {
    (void)twi_wyslij_bajt(LCD_ADDR_W);
    (void)twi_wyslij_bajt(b);
    twi_stop();
  }
}

/*!
 *  @brief Generuje impuls EN (strobe) dla LCD - zatrzaskuje dane w sterowniku.
 *  @param b bajt z danymi i flagami (RS, BL) bez EN.
 *  @returns nic.
 *  @side effects: wykonuje dwie transakcje TWI, czeka 1us i 50us.
 */
static void lcd_strobe(uint8_t b) {
  uint8_t mask = (uint8_t)(~LCD_EN);
  pcf_wyslij((uint8_t)(b | LCD_EN));
  czekaj_us(1UL);
  pcf_wyslij((uint8_t)(b & mask));
  czekaj_us(50UL);
}

/*!
 *  @brief Wysyła bajt do LCD w trybie 4-bit (dwa strobowania: high nibble, low nibble).
 *  @param bajt bajt do wysłania (komenda lub znak).
 *  @param rs 0 = komenda, LCD_RS = dane (znak).
 *  @returns nic.
 *  @side effects: wykonuje cztery transakcje TWI.
 */
static void lcd_wyslij(uint8_t bajt, uint8_t rs) {
  uint8_t hi = (uint8_t)((uint8_t)(bajt  & 0xF0U) | (uint8_t)(LCD_BL | rs));
  uint8_t lo = (uint8_t)((uint8_t)((bajt << 4U) & 0xF0U) | (uint8_t)(LCD_BL | rs));
  lcd_strobe(hi);
  lcd_strobe(lo);
}

#define lcd_cmd(c) lcd_wyslij((c), (uint8_t)0U)
#define lcd_chr(c) lcd_wyslij((c), LCD_RS)

/*!
 *  @brief Inicjalizacja LCD w trybie 4-bit zgodnie z sekwencją startu HD44780.
 *  @returns nic.
 *  @side effects: wysyła sekwencję inicjalizacyjną przez TWI, czeka łącznie ~60ms.
 */
static void lcd_init(void) {
  czekaj_ms(50UL);
  pcf_wyslij((uint8_t)(0x30U | LCD_BL));
  lcd_strobe((uint8_t)(0x30U | LCD_BL));
  czekaj_ms(5UL);
  pcf_wyslij((uint8_t)(0x30U | LCD_BL));
  lcd_strobe((uint8_t)(0x30U | LCD_BL));
  czekaj_us(150UL);
  pcf_wyslij((uint8_t)(0x30U | LCD_BL));
  lcd_strobe((uint8_t)(0x30U | LCD_BL));
  czekaj_us(150UL);
  pcf_wyslij((uint8_t)(0x20U | LCD_BL));
  lcd_strobe((uint8_t)(0x20U | LCD_BL));
  czekaj_us(150UL);
  lcd_cmd((uint8_t)0x28U);
  czekaj_us(50UL); /* 4-bit, 2 linie, 5x8 */
  lcd_cmd((uint8_t)0x08U);
  czekaj_us(50UL); /* display off */
  lcd_cmd((uint8_t)0x01U);
  czekaj_ms(2UL);  /* clear */
  lcd_cmd((uint8_t)0x06U);
  czekaj_us(50UL); /* entry mode: cursor right */
  lcd_cmd((uint8_t)0x0CU);
  czekaj_us(50UL); /* display on, cursor off */
}

/*!
 *  @brief Czyści ekran LCD.
 *  @returns nic.
 *  @side effects: wysyła komendę 0x01, czeka 2ms.
 */
static void lcd_clear(void) {
  lcd_cmd((uint8_t)0x01U);
  czekaj_ms(2UL);
}

/*!
 *  @brief Ustawia kursor LCD na podaną pozycję.
 *  @param col kolumna (0-15).
 *  @param row wiersz (0 lub 1).
 *  @returns nic.
 *  @side effects: wysyła komendę ustawienia adresu DDRAM, czeka 50us.
 */
static void lcd_goto(uint8_t col, uint8_t row) {
  uint8_t addr = (row != 0U) ? (uint8_t)(0x40U + col) : col;
  lcd_cmd((uint8_t)(0x80U | addr));
  czekaj_us(50UL);
}

/*!
 *  @brief Wypisuje napis na LCD od bieżącej pozycji kursora.
 *  @param s wskaźnik na napis zakończony '\0'.
 *  @returns nic.
 *  @side effects: przesuwa kursor, czeka 50us po każdym znaku.
 */
static void lcd_puts(const char *s) {
  uint8_t i = 0U;
  while (s[i] != '\0') {
    lcd_chr((uint8_t)s[i]);
    czekaj_us(50UL);
    i++;
  }
}

/*!
 *  @brief Wypisuje napis na LCD i dopełnia spacjami do podanej szerokości.
 *  @param s wskaźnik na napis zakończony '\0'.
 *  @param szerokosc docelowa liczba znaków do wypełnienia.
 *  @returns nic.
 *  @side effects: przesuwa kursor, zapobiega smugom po krótszych napisach.
 */
static void lcd_puts_w(const char *s, uint8_t szerokosc) {
  uint8_t n = 0U;
  uint8_t i = 0U;
  while ((s[i] != '\0') && (n < szerokosc)) {
    lcd_chr((uint8_t)s[i]);
    czekaj_us(50UL);
    n++;
    i++;
  }
  while (n < szerokosc) {
    lcd_chr((uint8_t)' ');
    czekaj_us(50UL);
    n++;
  }
}

/* PCA9685 - ekspander PWM dla LED (kanaly 0-3) */
/* addr=0x40, tryb auto-increment, czestotliwosc domyslna ~200Hz */

#define PCA_ADDR_W ((uint8_t)(0x40U << 1U))
#define PCA_MODE1 ((uint8_t)0x00U)
#define PCA_LED0 ((uint8_t)0x06U)

/*!
 *  @brief Inicjalizacja PCA9685 - wybudza układ i włącza auto-increment rejestrów.
 *  @returns nic.
 *  @side effects: wysyła dwa bajty przez TWI, czeka 1ms na stabilizację.
 */
static void pca9685_init(void) {
  if (twi_start()) {
    (void)twi_wyslij_bajt(PCA_ADDR_W);
    (void)twi_wyslij_bajt(PCA_MODE1);
    (void)twi_wyslij_bajt((uint8_t)0x21U); /* SLEEP=0, AI=1 */
    twi_stop();
    czekaj_ms(1UL);
  }
}

/*!
 *  @brief Ustawia wartości ON i OFF dla wybranego kanału PWM PCA9685.
 *  @param kanal numer kanału (0-15).
 *  @param on moment włączenia w cyklu PWM (0-4095).
 *  @param off moment wyłączenia w cyklu PWM (0-4095).
 *  @returns nic.
 *  @side effects: wysyła 5 bajtów przez TWI.
 */
static void pca_ustaw(uint8_t kanal, uint16_t on, uint16_t off) {
  if (twi_start()) {
    (void)twi_wyslij_bajt(PCA_ADDR_W);
    (void)twi_wyslij_bajt((uint8_t)((uint16_t)PCA_LED0 + (uint16_t)(4U * (uint16_t)kanal)));
    (void)twi_wyslij_bajt((uint8_t)(on  & (uint16_t)0xFFU));
    (void)twi_wyslij_bajt((uint8_t)(on  >> 8U));
    (void)twi_wyslij_bajt((uint8_t)(off & (uint16_t)0xFFU));
    (void)twi_wyslij_bajt((uint8_t)(off >> 8U));
    twi_stop();
  }
}

/* PWM silników - Fast PWM 8-bit, prescaler 8, ~7.8 kHz */
/* Timer3: OC3A=ENA=Pin5=PE3 */
/* Timer4: OC4A=ENB=Pin6=PH3, OC4B=ENC=Pin7=PH4, OC4C=END=Pin8=PH5 */

/*!
 *  @brief Inicjalizacja Timera3 i Timera4 w trybie Fast PWM 8-bit dla silników.
 *  @returns nic.
 *  @side effects: ustawia TCCR3A, TCCR3B, TCCR4A, TCCR4B, DDRE, DDRH.
 */
static void pwm_init(void) {
  DDRE |= (uint8_t)(1U << PE3);
  TCCR3A = (uint8_t)((1U << COM3A1) | (1U << WGM30));
  TCCR3B = (uint8_t)((1U << WGM32) | (1U << CS31));
  OCR3A = 0U;

  DDRH |= (uint8_t)((1U << PH3) | (1U << PH4) | (1U << PH5));
  TCCR4A = (uint8_t)((1U << COM4A1) | (1U << COM4B1) | (1U << COM4C1) | (1U << WGM40));
  TCCR4B = (uint8_t)((1U << WGM42) | (1U << CS41));
  OCR4A = 0U;
  OCR4B = 0U;
  OCR4C = 0U;
}

/* GPIO - kierunki i stany początkowe pinów */

/*!
 *  @brief Inicjalizacja pinów GPIO - kierunki i stany początkowe.
 *  @returns nic.
 *  @side effects: ustawia DDRA, PORTA, DDRB, PORTB, DDRL, PORTL, DDRD, PORTD.
 */
static void gpio_init(void) {
  DDRA = 0xFFU;
  PORTA = 0x00U;

  DDRB |= (uint8_t)(1U << PB5);
  PORTB &= (uint8_t)(~(uint8_t)(1U << PB5));

  DDRD |= (uint8_t)(1U << PD7);
  PORTD &= (uint8_t)(~(uint8_t)(1U << PD7));

  DDRL &= (uint8_t)(~(uint8_t)(1U << PL1));
  PORTL &= (uint8_t)(~(uint8_t)(1U << PL1));
}

/* Buzzer - Timer1 CTC toggle OC1A=PB5=Pin11 */
/* OCR1A = F_CPU / (2 * prescaler * freq) - 1 */

/*!
 *  @brief Uruchamia buzzer na podanej częstotliwości.
 *  @param freq częstotliwość dźwięku w Hz.
 *  @returns nic.
 *  @side effects: ustawia TCCR1A, TCCR1B, OCR1A, DDRB - zajmuje Timer1.
 */
static void buzzer_start(uint16_t freq) {
  uint32_t ocr = (F_CPU / (2UL * 8UL * (uint32_t)freq)) - 1UL;
  if (ocr > 65535UL) { ocr = 65535UL; }
  TCCR1A = (uint8_t)(1U << COM1A0);
  TCCR1B = (uint8_t)((1U << WGM12) | (1U << CS11));
  OCR1A = (uint16_t)ocr;
  DDRB |= (uint8_t)(1U << PB5);
}

/*!
 *  @brief Zatrzymuje buzzer i wycisza pin.
 *  @returns nic.
 *  @side effects: zeruje TCCR1A, TCCR1B, ściąga PB5 do zera.
 */
static void buzzer_stop(void) {
  TCCR1A = 0U;
  TCCR1B = 0U;
  PORTB &= (uint8_t)(~(uint8_t)(1U << PB5));
}

/* HC-SR04 - asynchroniczny pomiar przez Input Capture Timer5 */
/* ICP5=PL1=Pin48, prescaler 8 -> 0.5us/tik */
/* Odleglosc [cm] = tiki / 116 */

static volatile int32_t sr04_odleglosc = 999;
static volatile bool sr04_w_trakcie = false;

ISR(TIMER5_CAPT_vect) {
  static volatile uint16_t sr04_start = 0U;
  uint16_t teraz = ICR5;
  if ((TCCR5B & (uint8_t)(1U << ICES5)) != 0U) {
    /* zbocze narastające - zapisz moment startu echa */
    sr04_start = teraz;
    TCCR5B &= (uint8_t)(~(uint8_t)(1U << ICES5));
    TIFR5 = (uint8_t)(1U << ICF5);
  } else {
    /* zbocze opadające - oblicz dystans */
    uint16_t dt = (uint16_t)(teraz - sr04_start);
    int32_t d = (int32_t)dt / (int32_t)116;
    sr04_odleglosc = (d > (int32_t)400) ? (int32_t)999 : d;
    TCCR5B = 0U;
    TIMSK5 &= (uint8_t)(~(uint8_t)(1U << ICIE5));
    sr04_w_trakcie = false;
  }
}

/*!
 *  @brief Wyzwala pomiar HC-SR04 i zwraca ostatni zmierzony dystans.
 *  @returns ostatni zmierzony dystans w cm, 999 jeśli brak echa lub poza zasięgiem.
 *  @side effects: wysyła impuls TRIG 10us, uruchamia Timer5 Input Capture,
 *  wynik dostępny po ~150ms w kolejnym wywołaniu.
 */
static int32_t sr04_zmierz(void) {
  if (sr04_w_trakcie) {
    TCCR5B = 0U;
    TIMSK5 &= (uint8_t)(~(uint8_t)(1U << ICIE5));
    sr04_w_trakcie = false;
    sr04_odleglosc = (int32_t)999;
  }
  sr04_w_trakcie = true;
  PORTD |= (uint8_t)(1U << PD7);
  czekaj_us(10UL);
  PORTD &= (uint8_t)(~(uint8_t)(1U << PD7));
  TCNT5 = 0U;
  TIFR5 = (uint8_t)(1U << ICF5);
  TIMSK5 |= (uint8_t)(1U << ICIE5);
  TCCR5B = (uint8_t)((1U << ICNC5) | (1U << ICES5) | (1U << CS51));
  return sr04_odleglosc;
}

/* ADC - fotorezystor (PF0/A0) do auto-DRL */
/* AVCC jako referencja, preskaler 128, przerwanie po konwersji */

static volatile uint16_t adc_wynik = 0U;

ISR(ADC_vect) {
  adc_wynik = ADC;
}

/*!
 *  @brief Inicjalizacja ADC - kanał ADC0, referencja AVCC, preskaler 128.
 *  @returns nic.
 *  @side effects: ustawia DIDR0, ADMUX, ADCSRA, uruchamia pierwszą konwersję.
 */
static void adc_init(void) {
  DIDR0 |= (uint8_t)(1U << ADC0D);
  ADMUX = (uint8_t)(1U << REFS0);
  ADCSRA = (uint8_t)((1U << ADEN) | (1U << ADIE) |
           (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0));
  ADCSRA |= (uint8_t)(1U << ADSC);
}

/* Silniki - koła mecanum X-layout */
/* k > 0 = "przód" w sensie okablowania  k < 0 = "tył"  k = 0 = stop */
/* Uwaga: k=-1 odpowiada fizycznej jeździe DO PRZODU */

/*!
 *  @brief Steruje silnikiem A (FL przód-lewy).
 *  @param k kierunek: 1 = "przód okablowania", -1 = "tył okablowania", 0 = stop.
 *  @param spd wypełnienie PWM (0-255).
 *  @returns nic.
 *  @side effects: ustawia PA0, PA1, OCR3A.
 */
static void silnikA(int8_t k, uint8_t spd) {
  if (k > 0) {
    PORTA |= (uint8_t)(1U << PA0);
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA1));
  } else if (k < 0) {
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA0));
    PORTA |= (uint8_t)(1U << PA1);
  } else {
    PORTA &= (uint8_t)(~(uint8_t)((1U << PA0) | (1U << PA1)));
  }
  OCR3A = (k != 0) ? (uint16_t)spd : (uint16_t)0U;
}

/*!
 *  @brief Steruje silnikiem B (FR przód-prawy).
 *  @param k kierunek: 1 = "przód okablowania", -1 = "tył okablowania", 0 = stop.
 *  @param spd wypełnienie PWM (0-255).
 *  @returns nic.
 *  @side effects: ustawia PA2, PA3, OCR4A.
 */
static void silnikB(int8_t k, uint8_t spd) {
  if (k > 0) {
    PORTA |=  (uint8_t)(1U << PA2);
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA3));
  } else if (k < 0) {
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA2));
    PORTA |=  (uint8_t)(1U << PA3);
  } else {
    PORTA &= (uint8_t)(~(uint8_t)((1U << PA2) | (1U << PA3)));
  }
  OCR4A = (k != 0) ? (uint16_t)spd : (uint16_t)0U;
}

/*!
 *  @brief Steruje silnikiem C (RR tył-prawy).
 *  @param k kierunek: 1 = "przód okablowania", -1 = "tył okablowania", 0 = stop.
 *  @param spd wypełnienie PWM (0-255).
 *  @returns nic.
 *  @side effects: ustawia PA4, PA5, OCR4B.
 */
static void silnikC(int8_t k, uint8_t spd) {
  if (k > 0) {
    PORTA |=  (uint8_t)(1U << PA4);
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA5));
  } else if (k < 0) {
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA4));
    PORTA |=  (uint8_t)(1U << PA5);
  } else {
    PORTA &= (uint8_t)(~(uint8_t)((1U << PA4) | (1U << PA5)));
  }
  OCR4B = (k != 0) ? (uint16_t)spd : (uint16_t)0U;
}

/*!
 *  @brief Steruje silnikiem D (RL tył-lewy).
 *  @param k kierunek: 1 = "przód okablowania", -1 = "tył okablowania", 0 = stop.
 *  @param spd wypełnienie PWM (0-255).
 *  @returns nic.
 *  @side effects: ustawia PA6, PA7, OCR4C.
 */
static void silnikD(int8_t k, uint8_t spd) {
  if (k > 0) {
    PORTA |=  (uint8_t)(1U << PA6);
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA7));
  } else if (k < 0) {
    PORTA &= (uint8_t)(~(uint8_t)(1U << PA6));
    PORTA |=  (uint8_t)(1U << PA7);
  } else {
    PORTA &= (uint8_t)(~(uint8_t)((1U << PA6) | (1U << PA7)));
  }
  OCR4C = (k != 0) ? (uint16_t)spd : (uint16_t)0U;
}

/*!
 *  @brief Zatrzymuje wszystkie cztery silniki.
 *  @returns nic.
 *  @side effects: wywołuje silnikA-D z k=0, zeruje wszystkie OCR.
 */
static void stop_all(void) {
  silnikA((int8_t)0, (uint8_t)0U);
  silnikB((int8_t)0, (uint8_t)0U);
  silnikC((int8_t)0, (uint8_t)0U);
  silnikD((int8_t)0, (uint8_t)0U);
}

/* Oświetlenie — PCA9685 kanały 0-3 */

#define KANAL_DRL_LEWY ((uint8_t)0U)
#define KANAL_DRL_PRAWY ((uint8_t)1U)
#define KANAL_KIERUNK_LEWY ((uint8_t)2U)
#define KANAL_KIERUNK_PRAWY ((uint8_t)3U)
#define JASNOSC_PELNA ((uint16_t)4095U)
#define JASNOSC_ZERO ((uint16_t)0U)
#define OKRES_PULSOWANIA_MS ((uint32_t)1000UL)

static bool drl_wlaczone = false;
static bool kierunk_lewy_wlaczony = false;
static bool kierunk_prawy_wlaczony = false;
static bool awaryjne_wlaczone = false;

/*!
 *  @brief Ustawia kanał PCA9685 w stan włączony lub wyłączony (pełna jasność / zero).
 *  @param kanal numer kanału PCA9685 (0-15).
 *  @param wlaczony true = pełna jasność, false = zgaszony.
 *  @returns nic.
 *  @side effects: wywołuje pca_ustaw(), wykonuje transakcję TWI.
 */
static void ustaw_kanal(uint8_t kanal, bool wlaczony) {
  pca_ustaw(kanal, (uint16_t)0U, wlaczony ? JASNOSC_PELNA : JASNOSC_ZERO);
}

/*!
 *  @brief Włącza światła DRL (oba kanały).
 *  @returns nic.
 *  @side effects: ustawia drl_wlaczone=true, wysyła stan do PCA9685, loguje przez UART.
 */
static void drl_wlacz(void) {
  drl_wlaczone = true;
  ustaw_kanal(KANAL_DRL_LEWY,  true);
  ustaw_kanal(KANAL_DRL_PRAWY, true);
  info("DRL: WLACZONE");
}

/*!
 *  @brief Wyłącza światła DRL (oba kanały).
 *  @returns nic.
 *  @side effects: ustawia drl_wlaczone=false, wysyła stan do PCA9685, loguje przez UART.
 */
static void drl_wylacz(void) {
  drl_wlaczone = false;
  ustaw_kanal(KANAL_DRL_LEWY,  false);
  ustaw_kanal(KANAL_DRL_PRAWY, false);
  info("DRL: WYLACZONE");
}

/*!
 *  @brief Włącza lewy kierunkowskaz; wyłącza prawy i awaryjne jeśli były aktywne.
 *  @returns nic.
 *  @side effects: modyfikuje flagi kierunk_*, awaryjne_wlaczone, wysyła stan do PCA9685.
 */
static void kierunk_lewy_wlacz(void) {
  if (kierunk_prawy_wlaczony) {
    kierunk_prawy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  }
  if (awaryjne_wlaczone) {
    awaryjne_wlaczone = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY,  false);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  }
  kierunk_lewy_wlaczony = true;
  ustaw_kanal(KANAL_KIERUNK_LEWY, true);
  info("KIERUNK: LEWY");
}

/*!
 *  @brief Wyłącza lewy kierunkowskaz.
 *  @returns nic.
 *  @side effects: ustawia kierunk_lewy_wlaczony=false, wysyła stan do PCA9685.
 */
static void kierunk_lewy_wylacz(void) {
  kierunk_lewy_wlaczony = false;
  ustaw_kanal(KANAL_KIERUNK_LEWY, false);
  info("KIERUNK: LEWY WYLACZONY");
}

/*!
 *  @brief Włącza prawy kierunkowskaz; wyłącza lewy i awaryjne jeśli były aktywne.
 *  @returns nic.
 *  @side effects: modyfikuje flagi kierunk_*, awaryjne_wlaczone, wysyła stan do PCA9685.
 */
static void kierunk_prawy_wlacz(void) {
  if (kierunk_lewy_wlaczony) {
    kierunk_lewy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false);
  }
  if (awaryjne_wlaczone) {
    awaryjne_wlaczone = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY,  false);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  }
  kierunk_prawy_wlaczony = true;
  ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
  info("KIERUNK: PRAWY");
}

/*!
 *  @brief Wyłącza prawy kierunkowskaz.
 *  @returns nic.
 *  @side effects: ustawia kierunk_prawy_wlaczony=false, wysyła stan do PCA9685.
 */
static void kierunk_prawy_wylacz(void) {
  kierunk_prawy_wlaczony = false;
  ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  info("KIERUNK: PRAWY WYLACZONY");
}

/*!
 *  @brief Włącza światła awaryjne (oba kierunkowskazy jednocześnie).
 *  @returns nic.
 *  @side effects: wyłącza oba kierunkowskazy jeśli były aktywne,
 *  ustawia awaryjne_wlaczone=true, wysyła stan do PCA9685.
 */
static void awaryjne_wlacz(void) {
  if (kierunk_lewy_wlaczony) {
    kierunk_lewy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false);
  }
  if (kierunk_prawy_wlaczony) {
    kierunk_prawy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  }
  awaryjne_wlaczone = true;
  ustaw_kanal(KANAL_KIERUNK_LEWY,  true);
  ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
  info("AWARYJNE: WLACZONE");
}

/*!
 *  @brief Wyłącza światła awaryjne.
 *  @returns nic.
 *  @side effects: ustawia awaryjne_wlaczone=false, wysyła stan do PCA9685.
 */
static void awaryjne_wylacz(void) {
  awaryjne_wlaczone = false;
  ustaw_kanal(KANAL_KIERUNK_LEWY,  false);
  ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
  info("AWARYJNE: WYLACZONE");
}

/*!
 *  @brief Obsługuje płynne pulsowanie kierunkowskazów (przebieg trójkątny, cykl 1s).
 *  Ogranicza częstość zapisu do PCA9685 do max 50 Hz.
 *  @returns nic.
 *  @side effects: co ~20ms wysyła nową jasność do PCA9685 przez TWI.
 */
static void obsluz_miganie(void) {
  static uint32_t czas_ostatniego_mig = 0UL;
  bool aktywne = (kierunk_lewy_wlaczony || kierunk_prawy_wlaczony) || awaryjne_wlaczone;

  if (aktywne) {
    uint32_t teraz = millis();
    if ((teraz - czas_ostatniego_mig) >= 20UL) {
      uint32_t t;
      uint16_t jasnosc;
      czas_ostatniego_mig = teraz;
      t = teraz % OKRES_PULSOWANIA_MS;
      if (t < (OKRES_PULSOWANIA_MS / 2UL)) {
        jasnosc = (uint16_t)(t * 4095UL / (OKRES_PULSOWANIA_MS / 2UL));
      } else {
        jasnosc = (uint16_t)((OKRES_PULSOWANIA_MS - t) * 4095UL / (OKRES_PULSOWANIA_MS / 2UL));
      }
      if (awaryjne_wlaczone) {
        pca_ustaw(KANAL_KIERUNK_LEWY, (uint16_t)0U, jasnosc);
        pca_ustaw(KANAL_KIERUNK_PRAWY, (uint16_t)0U, jasnosc);
      } else {
        if (kierunk_lewy_wlaczony)  { pca_ustaw(KANAL_KIERUNK_LEWY,  (uint16_t)0U, jasnosc); }
        if (kierunk_prawy_wlaczony) { pca_ustaw(KANAL_KIERUNK_PRAWY, (uint16_t)0U, jasnosc); }
      }
    }
  }
}

/* Stan globalny */

static bool tryb_auto_drl = false;
static int32_t odleglosc_cm = (int32_t)999;
static char aktualny_kierunek[17] = "STOP";

/* lcd_update */

/*!
 *  @brief Odświeża wyświetlacz LCD - tylko zmienione linie są przepisywane.
 *  @returns nic.
 *  @side effects: wykonuje transakcje TWI jeśli wartości uległy zmianie.
 */
static void lcd_update(void) {
  static char prev_kierunek[17]  = "";
  static int32_t prev_odleglosc_lcd = (int32_t)-1;

  if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
    lcd_goto(0U, 0U);
    lcd_puts("Kier: ");
    lcd_puts_w(aktualny_kierunek, 10U);
    (void)strcpy(prev_kierunek, aktualny_kierunek);
  }

  if (odleglosc_cm != prev_odleglosc_lcd) {
    char buf[11];
    if (odleglosc_cm == (int32_t)999) {
      (void)strcpy(buf, "Brak/Max");
    } else {
      uint8_t idx = 0U;
      int32_t v = odleglosc_cm;
      if (v == 0) {
        buf[idx] = '0';
        idx++;
      } else {
        while (v > 0) {
          int32_t d = (int32_t)'0' + (v % (int32_t)10);
          buf[idx] = (char)d;
          idx++;
          v /= (int32_t)10;
        }
        {
          uint8_t a = 0U;
          uint8_t b_i = (uint8_t)(idx - 1U);
          while (a < b_i) {
            char tmp = buf[a];
            buf[a] = buf[b_i];
            buf[b_i] = tmp;
            a++;
            b_i--;
          }
        }
      }
      buf[idx] = ' '; idx++;
      buf[idx] = 'c'; idx++;
      buf[idx] = 'm'; idx++;
      buf[idx] = '\0';
    }
    lcd_goto(0U, 1U);
    lcd_puts("Dyst: ");
    lcd_puts_w(buf, 10U);
    prev_odleglosc_lcd = odleglosc_cm;
  }
}

/*!
 *  @brief Obsługuje pojedynczy znak komendy odebrany przez UART.
 *  @param c odebrany znak komendy (wielkość liter nieistotna).
 *  @returns nic.
 *  @side effects: steruje silnikami, oświetleniem, zmienia predkosc,
 *  aktualizuje aktualny_kierunek, loguje przez UART.
 */
static void handle_cmd(char c) {
  static uint8_t predkosc = 200U;
  static char ostatni_ruch = 'S';
  char cmd = c;
  bool ponow = true;

  while (ponow) {
    ponow = false;

    switch (cmd) {

      /* jazda prosto */
      case 'F':
      case 'f':
        silnikA((int8_t)-1, predkosc);
        silnikB((int8_t)-1, predkosc);
        silnikC((int8_t)-1, predkosc);
        silnikD((int8_t)-1, predkosc);
        ostatni_ruch = 'F';
        (void)strcpy(aktualny_kierunek, "PRZOD");
        info("PRZOD");
        break;

      case 'B':
      case 'b':
        silnikA((int8_t)1, predkosc);
        silnikB((int8_t)1, predkosc);
        silnikC((int8_t)1, predkosc);
        silnikD((int8_t)1, predkosc);
        ostatni_ruch = 'B';
        (void)strcpy(aktualny_kierunek, "TYL");
        info("TYL");
        break;

      /* obrót pivot */
      case 'L':
      case 'l':
        silnikA((int8_t)-1, predkosc);
        silnikD((int8_t)-1, predkosc);
        silnikB((int8_t)1, predkosc);
        silnikC((int8_t)1, predkosc);
        ostatni_ruch = 'L';
        (void)strcpy(aktualny_kierunek, "LEWO");
        info("LEWO");
        break;

      case 'R':
      case 'r':
        silnikA((int8_t)1, predkosc);
        silnikD((int8_t)1, predkosc);
        silnikB((int8_t)-1, predkosc);
        silnikC((int8_t)-1, predkosc);
        ostatni_ruch = 'R';
        (void)strcpy(aktualny_kierunek, "PRAWO");
        info("PRAWO");
        break;

      /* jazda bokiem, strafe mecanum X-layout
       * lewo (X): FL=tył, FR=przód, RL=przód, RR=tył
       * prawo (Y): FL=przód, FR=tył, RL=tył, RR=przód */
      case 'X':
      case 'x':
        silnikA((int8_t)1, predkosc);
        silnikB((int8_t)-1, predkosc);
        silnikC((int8_t)1, predkosc);
        silnikD((int8_t)-1, predkosc);
        ostatni_ruch = 'X';
        (void)strcpy(aktualny_kierunek, "BOK-LEWO");
        info("BOK-LEWO");
        break;

      case 'Y':
      case 'y':
        silnikA((int8_t)-1, predkosc);
        silnikB((int8_t)1, predkosc);
        silnikC((int8_t)-1, predkosc);
        silnikD((int8_t)1, predkosc);
        ostatni_ruch = 'Y';
        (void)strcpy(aktualny_kierunek, "BOK-PRAWO");
        info("BOK-PRAWO");
        break;

      /* skos do przodu, diagonal mecanum X-layout
       * przód-lewo (I): FR=przód, RL=przód; FL i RR stop
       * przód-prawo (K): FL=przód, RR=przód; FR i RL stop */
      case 'I':
      case 'i':
        silnikA((int8_t)0, (uint8_t)0U);
        silnikB((int8_t)-1, predkosc);
        silnikC((int8_t)0, (uint8_t)0U);
        silnikD((int8_t)-1, predkosc);
        ostatni_ruch = 'I';
        (void)strcpy(aktualny_kierunek, "SKOS-PL");
        info("SKOS PRZOD-LEWO");
        break;

      case 'K':
      case 'k':
        silnikA((int8_t)-1, predkosc);
        silnikB((int8_t)0, (uint8_t)0U);
        silnikC((int8_t)-1, predkosc);
        silnikD((int8_t)0, (uint8_t)0U);
        ostatni_ruch = 'K';
        (void)strcpy(aktualny_kierunek, "SKOS-PP");
        info("SKOS PRZOD-PRAWO");
        break;

      case 'S':
      case 's':
        stop_all();
        ostatni_ruch = 'S';
        (void)strcpy(aktualny_kierunek, "STOP");
        info("STOP");
        break;

      /* prędkość */
      case '+':
        if (predkosc <= 235U) { predkosc = (uint8_t)(predkosc + 20U); }
        else { predkosc = 255U; }
        uart0_puts("Predkosc: ");
        uart_wyslij_liczbe(0U, (int32_t)predkosc);
        uart0_puts("\r\n");
        uart1_puts("Predkosc: ");
        uart_wyslij_liczbe(1U, (int32_t)predkosc);
        uart1_puts("\r\n");
        cmd = ostatni_ruch;
        ponow = true;
        break;

      case '-':
        if (predkosc >= 80U) { predkosc = (uint8_t)(predkosc - 20U); }
        else { predkosc = 60U; }
        uart0_puts("Predkosc: ");
        uart_wyslij_liczbe(0U, (int32_t)predkosc);
        uart0_puts("\r\n");
        uart1_puts("Predkosc: ");
        uart_wyslij_liczbe(1U, (int32_t)predkosc);
        uart1_puts("\r\n");
        cmd = ostatni_ruch;
        ponow = true;
        break;

      /* oświetlenie */
      case 'D':
      case 'd':
        tryb_auto_drl = false;
        if (drl_wlaczone) { drl_wylacz(); }
        else { drl_wlacz(); }
        break;

      case 'A':
      case 'a':
        tryb_auto_drl = true;
        info("DRL: TRYB AUTO");
        break;

      case 'Q':
      case 'q':
        if (kierunk_lewy_wlaczony) { kierunk_lewy_wylacz(); }
        else { kierunk_lewy_wlacz(); }
        break;

      case 'E':
      case 'e':
        if (kierunk_prawy_wlaczony) { kierunk_prawy_wylacz(); }
        else { kierunk_prawy_wlacz(); }
        break;

      case 'H':
      case 'h':
        if (awaryjne_wlaczone) { awaryjne_wylacz(); }
        else { awaryjne_wlacz(); }
        break;

      default: break;
    }
  }
}

/* main */

int main(void) {
  static bool pikniecie_trwa = false;
  static uint32_t czas_pomiaru = 0UL;
  static uint32_t czas_ostatniego_pikn = 0UL;

  timer0_init();
  sei();

  uart0_init();
  uart1_init();
  twi_init();
  pwm_init();
  gpio_init();
  adc_init();

  lcd_init();
  pca9685_init();
  {
    uint8_t k;
    for (k = 0U; k < 4U; k++) { pca_ustaw(k, (uint16_t)0U, (uint16_t)0U); }
  }

  lcd_goto(0U, 0U);
  lcd_puts("  SAMOCHODZIK   ");
  lcd_goto(0U, 1U);
  lcd_puts("   GOTOWY :)    ");
  czekaj_ms(1500UL);
  lcd_clear();

  stop_all();
  lcd_update();
  info("=== GOTOWY ===");
  info("F/B/L/R/X/Y/I/K/S/+/-/D/A/Q/E/H");

  while (1) {
    uint32_t teraz = millis();

    /* pomiar dystansu HC-SR04 co 150ms */
    if ((teraz - czas_pomiaru) >= 150UL) {
      czas_pomiaru = teraz;
      odleglosc_cm = sr04_zmierz();
    }

    /* Asystent parkowania, buzzer proporcjonalny do dystansu */
    if (odleglosc_cm > (int32_t)100) {
      if (pikniecie_trwa) {
        buzzer_stop();
        pikniecie_trwa = false;
      }
    } else if (odleglosc_cm <= (int32_t)15) {
      if (!pikniecie_trwa) {
        buzzer_start(1000U);
        pikniecie_trwa = true;
      }
      czas_ostatniego_pikn = teraz;
    } else {
      uint16_t interwal = (uint16_t)((uint32_t)odleglosc_cm * 10UL);
      if (!pikniecie_trwa) {
        if ((teraz - czas_ostatniego_pikn) >= (uint32_t)interwal) {
          czas_ostatniego_pikn = teraz;
          buzzer_start(1000U);
          pikniecie_trwa = true;
        }
      } else {
        if ((teraz - czas_ostatniego_pikn) >= 80UL) {
          buzzer_stop();
          pikniecie_trwa = false;
        }
      }
    }

    /* ADC (fotorezystor) co 100ms */
    {
      static uint32_t czas_adc = 0UL;
      static bool jest_ciemno = false;

      if ((teraz - czas_adc) >= 100UL) {
        uint16_t odczyt;
        uint8_t sreg = SREG;
        czas_adc = teraz;
        cli();
        odczyt = adc_wynik;
        SREG = sreg;
        ADCSRA |= (uint8_t)(1U << ADSC);

        if (odczyt > 700U) { jest_ciemno = false; }
        else if (odczyt < 500U) { jest_ciemno = true;  }
        else { ; }

        if (tryb_auto_drl) {
          if (jest_ciemno  && !drl_wlaczone) { drl_wlacz();  }
          else if (!jest_ciemno &&  drl_wlaczone) { drl_wylacz(); }
          else { ; }
        }
      }
    }

    /* odświeżanie LCD tylko przy zmianach wartości */
    lcd_update();

    /* pulsowanie kierunkowskazów */
    obsluz_miganie();

    /* komendy Bluetooth */
    if (uart0_dostepny()) { handle_cmd((char)uart0_czytaj()); }
    if (uart1_dostepny()) { handle_cmd((char)uart1_czytaj()); }
  }

  return 0;
}