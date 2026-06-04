/*
 * SAMOCHODZIK — Bare Metal ATmega2560
 * Bluetooth HC-05 + 2x L298N + LCD I2C + HC-SR04 + Buzzer + PCA9685 + ADC
 *
 * Piny:
 *   HC-05:    UART1 TX=Pin19(PD3) RX=Pin18(PD2)
 *   L298N #1: ENA=Pin5(PE3/OC3A)  IN1=22(PA0) IN2=23(PA1)
 *             ENB=Pin6(PH3/OC4A)  IN3=24(PA2) IN4=25(PA3)
 *   L298N #2: ENC=Pin7(PH4/OC4B)  IN5=26(PA4) IN6=27(PA5)
 *             END=Pin8(PH5/OC4C)  IN7=28(PA6) IN8=29(PA7)
 *   LCD I2C:  SDA=Pin20  SCL=Pin21  PCF8574T addr=0x27
 *   PCA9685:  ten sam I2C, addr=0x40
 *             kanal 0=DRL lewy (bialy)   kanal 1=DRL prawy (bialy)
 *             kanal 2=kier. lewy (zolty) kanal 3=kier. prawy (zolty)
 *   HC-SR04:  TRIG=Pin38(PD7)  ECHO=Pin48(PL1/ICP5)
 *   Buzzer:   Pin13(PB5/OC1A)
 *   LED:      Pin46(PL3)
 *   Foto:     Pin A0 (PF0/ADC0) — fotorezystor do auto-DRL
 *
 * Timery:
 *   Timer0: CTC 1ms  -> millis()
 *   Timer1: CTC toggle OC1A -> buzzer (Pin13/PB5)
 *   Timer3: Fast PWM 8-bit  -> ENA (Pin5/PE3)
 *   Timer4: Fast PWM 8-bit  -> ENB(6) ENC(7) END(8)
 *   Timer5: Input Capture   -> pomiar HC-SR04 async (ICP5=PL1=Pin48)
 *
 * Osie silnikow (kola mecanum X-layout):
 *   silnikA = FL przod-lewy   (ENA=OC3A, IN1=PA0, IN2=PA1)
 *   silnikB = FR przod-prawy  (ENB=OC4A, IN3=PA2, IN4=PA3)
 *   silnikC = RR tyl-prawy    (ENC=OC4B, IN5=PA4, IN6=PA5)
 *   silnikD = RL tyl-lewy     (END=OC4C, IN7=PA6, IN8=PA7)
 *
 * LCD:
 *   Linia 0: kierunek jazdy
 *   Linia 1: dystans HC-SR04 [cm]
 *
 * Komendy Bluetooth:
 *   F/B     = przod / tyl
 *   L/R     = obrot pivot lewo / prawo
 *   X/Y     = jazda bokiem lewo / prawo  (strafe mecanum)
 *   I/K     = skos przod-lewo / przod-prawo (diagonal mecanum)
 *   S       = stop
 *   +/-     = szybciej / wolniej
 *   D       = DRL wlacz/wylacz (tryb reczny)
 *   A       = DRL tryb auto (fotorezystor)
 *   Q/E     = kierunkowskaz lewy / prawy
 *   H       = swiatla awaryjne
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define F_CPU 16000000UL

/* ================================================================
   MILLIS — Timer0 CTC, prescaler 64, OCR0A=249 -> 1 ms/przerwanie
   ================================================================ */
volatile uint32_t ms_count = 0u;

ISR(TIMER0_COMPA_vect) { ms_count++; }

static void timer0_init(void) {
    TCCR0A = (1u << WGM01);
    TCCR0B = (1u << CS01) | (1u << CS00);
    OCR0A  = 249u;
    TIMSK0 = (1u << OCIE0A);
}

static uint32_t millis(void) {
    uint32_t t;
    uint8_t sreg = SREG;
    cli();
    t = ms_count;
    SREG = sreg;
    return t;
}

/* ================================================================
   UART0 (USB debug) + UART1 (HC-05 Bluetooth)
   9600 baud @ 16 MHz  ->  UBRR = 103
   ================================================================ */
#define BAUD     9600UL
#define UBRR_VAL (F_CPU / (16UL * BAUD) - 1u)

static void uart0_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8u);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1u << RXEN0) | (1u << TXEN0);
    UCSR0C = (1u << UCSZ01) | (1u << UCSZ00);
}

static bool    uart0_available(void) { return (bool)(UCSR0A & (1u << RXC0)); }
static uint8_t uart0_read(void)      { return UDR0; }
static void    uart0_putc(char c)    { while (!(UCSR0A & (1u << UDRE0))); UDR0 = c; }
static void    uart0_puts(const char *s) { while (*s) uart0_putc(*s++); }

static void uart1_init(void) {
    UBRR1H = (uint8_t)(UBRR_VAL >> 8u);
    UBRR1L = (uint8_t)(UBRR_VAL);
    UCSR1B = (1u << RXEN1) | (1u << TXEN1);
    UCSR1C = (1u << UCSZ11) | (1u << UCSZ10);
}

static bool    uart1_available(void) { return (bool)(UCSR1A & (1u << RXC1)); }
static uint8_t uart1_read(void)      { return UDR1; }
static void    uart1_putc(char c)    { while (!(UCSR1A & (1u << UDRE1))); UDR1 = c; }
static void    uart1_puts(const char *s) { while (*s) uart1_putc(*s++); }

static void uart_putn(uint8_t port, int32_t n) {
    char buf[12]; int8_t i = 0;
    if (n < 0) { (port == 0u ? uart0_putc : uart1_putc)('-'); n = -n; }
    if (n == 0) { (port == 0u ? uart0_putc : uart1_putc)('0'); return; }
    while (n > 0) { buf[i++] = (char)('0' + (n % 10)); n /= 10; }
    while (i > 0) { i--; (port == 0u ? uart0_putc : uart1_putc)(buf[i]); }
}

static void info(const char *s) {
    uart0_puts(s); uart0_puts("\r\n");
    uart1_puts(s); uart1_puts("\r\n");
}

/* ================================================================
   TWI (I2C) — 100 kHz
   TWBR=72, TWSR=0x00 (prescaler=1)
   ================================================================ */
#define I2C_TIMEOUT 10000u

static void twi_init(void) {
    TWSR = 0x00u;
    TWBR = 72u;
    TWCR = (1u << TWEN);
}

static bool twi_start(void) {
    uint16_t t = 0u;
    TWCR = (1u << TWINT) | (1u << TWSTA) | (1u << TWEN);
    while (!(TWCR & (1u << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

static void twi_stop(void) {
    TWCR = (1u << TWINT) | (1u << TWEN) | (1u << TWSTO);
}

static bool twi_write_byte(uint8_t data) {
    uint16_t t = 0u;
    TWDR = data;
    TWCR = (1u << TWINT) | (1u << TWEN);
    while (!(TWCR & (1u << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

/* ================================================================
   LCD HD44780 w trybie 4-bit przez PCF8574T (addr 0x27)
   Mapowanie PCF: P0=RS P1=RW P2=EN P3=BL P4-P7=D4-D7
   ================================================================ */
#define LCD_ADDR_W  (0x27u << 1u)
#define LCD_RS      (1u << 0u)
#define LCD_EN      (1u << 2u)
#define LCD_BL      (1u << 3u)

static void pcf_write(uint8_t b) {
    if (!twi_start()) return;
    (void)twi_write_byte(LCD_ADDR_W);
    (void)twi_write_byte(b);
    twi_stop();
}

static void lcd_strobe(uint8_t b) {
    pcf_write(b | LCD_EN);              _delay_us(1);
    pcf_write(b & (uint8_t)~LCD_EN);   _delay_us(50);
}

static void lcd_send(uint8_t byte, uint8_t rs) {
    uint8_t hi = (byte & 0xF0u)         | LCD_BL | rs;
    uint8_t lo = ((byte << 4u) & 0xF0u) | LCD_BL | rs;
    lcd_strobe(hi);
    lcd_strobe(lo);
}

#define lcd_cmd(c) lcd_send((c), 0u)
#define lcd_chr(c) lcd_send((c), LCD_RS)

static void lcd_init(void) {
    _delay_ms(50);
    pcf_write(0x30u | LCD_BL); lcd_strobe(0x30u | LCD_BL); _delay_ms(5);
    pcf_write(0x30u | LCD_BL); lcd_strobe(0x30u | LCD_BL); _delay_us(150);
    pcf_write(0x30u | LCD_BL); lcd_strobe(0x30u | LCD_BL); _delay_us(150);
    pcf_write(0x20u | LCD_BL); lcd_strobe(0x20u | LCD_BL); _delay_us(150);
    lcd_cmd(0x28u); _delay_us(50);   /* 4-bit, 2 linie, 5x8 */
    lcd_cmd(0x08u); _delay_us(50);   /* display off */
    lcd_cmd(0x01u); _delay_ms(2);    /* clear */
    lcd_cmd(0x06u); _delay_us(50);   /* entry mode: cursor right */
    lcd_cmd(0x0Cu); _delay_us(50);   /* display on, cursor off */
}

static void lcd_clear(void) { lcd_cmd(0x01u); _delay_ms(2); }

static void lcd_goto(uint8_t col, uint8_t row) {
    lcd_cmd(0x80u | ((row ? 0x40u : 0x00u) + col));
    _delay_us(50);
}

static void lcd_puts(const char *s) {
    while (*s) { lcd_chr((uint8_t)*s++); _delay_us(50); }
}

/* Wypisuje string i dopelnia spacjami do zadanej szerokosci.
   Zapobiega „smugom" po krotszych napisach bez lcd_clear(). */
static void lcd_puts_w(const char *s, uint8_t szerokosc) {
    uint8_t n = 0u;
    while (*s && n < szerokosc) { lcd_chr((uint8_t)*s++); _delay_us(50); n++; }
    while (n < szerokosc)       { lcd_chr(' ');            _delay_us(50); n++; }
}

/* ================================================================
   PCA9685 — ekspander PWM dla LED (kanaly 0-3)
   addr=0x40, tryb AI (auto-increment), czestotliwosc domyslna ~200Hz
   ================================================================ */
#define PCA_ADDR_W  (0x40u << 1u)
#define PCA_MODE1   0x00u
#define PCA_LED0    0x06u

static void pca9685_init(void) {
    if (!twi_start()) return;
    (void)twi_write_byte(PCA_ADDR_W);
    (void)twi_write_byte(PCA_MODE1);
    (void)twi_write_byte(0x21u);   /* SLEEP=0, AI=1 */
    twi_stop();
    _delay_ms(1);
}

static void pca_set(uint8_t ch, uint16_t on, uint16_t off) {
    if (!twi_start()) return;
    (void)twi_write_byte(PCA_ADDR_W);
    (void)twi_write_byte((uint8_t)(PCA_LED0 + 4u * ch));
    (void)twi_write_byte((uint8_t)(on  & 0xFFu));
    (void)twi_write_byte((uint8_t)(on  >> 8u));
    (void)twi_write_byte((uint8_t)(off & 0xFFu));
    (void)twi_write_byte((uint8_t)(off >> 8u));
    twi_stop();
}

/* ================================================================
   PWM silnikow
   Timer3: OC3A=ENA=Pin5=PE3
   Timer4: OC4A=ENB=Pin6=PH3, OC4B=ENC=Pin7=PH4, OC4C=END=Pin8=PH5
   Fast PWM 8-bit, prescaler 8 -> ~7.8 kHz
   ================================================================ */
static void pwm_init(void) {
    TCCR3A = (1u << COM3A1) | (1u << WGM30);
    TCCR3B = (1u << WGM32)  | (1u << CS31);
    OCR3A  = 0u;
    DDRE  |= (1u << PE3);

    TCCR4A = (1u << COM4A1) | (1u << COM4B1) | (1u << COM4C1) | (1u << WGM40);
    TCCR4B = (1u << WGM42)  | (1u << CS41);
    OCR4A  = 0u; OCR4B = 0u; OCR4C = 0u;
    DDRH  |= (1u << PH3) | (1u << PH4) | (1u << PH5);
}

/* ================================================================
   GPIO
   Kierunki silnikow: PORTA PA0-PA7 = Piny 22-29
   Buzzer:  PB5=Pin13   LED: PL3=Pin46
   SR04:    TRIG=PD7=Pin38  ECHO=PL1=Pin48
   ================================================================ */
static void gpio_init(void) {
    DDRA  = 0xFFu; PORTA = 0x00u;
    DDRB  |=  (1u << PB5); PORTB &= ~(1u << PB5);
    DDRL  |=  (1u << PL3); PORTL &= ~(1u << PL3);
    DDRD  |=  (1u << PD7); PORTD &= ~(1u << PD7);
    DDRL  &= ~(1u << PL1); PORTL &= ~(1u << PL1);
}

/* ================================================================
   BUZZER — Timer1 CTC toggle OC1A=PB5=Pin13
   OCR1A = F_CPU / (2 * prescaler * freq) - 1
   ================================================================ */
static void tone_start(uint16_t freq) {
    uint32_t ocr = F_CPU / (2UL * 8UL * (uint32_t)freq) - 1UL;
    if (ocr > 65535UL) ocr = 65535UL;
    TCCR1A = (1u << COM1A0);
    TCCR1B = (1u << WGM12) | (1u << CS11);
    OCR1A  = (uint16_t)ocr;
    DDRB  |= (1u << PB5);
}

static void tone_stop(void) {
    TCCR1A = 0u; TCCR1B = 0u;
    PORTB &= ~(1u << PB5);
}

/* ================================================================
   HC-SR04 — asynchroniczny pomiar Input Capture Timer5
   ICP5=PL1=Pin48, prescaler 8 -> 0.5 us/tick
   Odleglosc [cm] = tiki / 116
   ================================================================ */
static volatile uint16_t sr04_start     = 0u;
static volatile long     sr04_odleglosc = 999L;
static volatile bool     sr04_w_trakcie = false;

ISR(TIMER5_CAPT_vect) {
    uint16_t teraz = ICR5;
    if (TCCR5B & (1u << ICES5)) {
        /* zbocze narastajace: zapisz czas startu echa */
        sr04_start = teraz;
        TCCR5B &= ~(1u << ICES5);
        TIFR5   = (1u << ICF5);
    } else {
        /* zbocze opadajace: oblicz dystans */
        uint16_t dt = teraz - sr04_start;
        long d = (long)dt / 116L;
        sr04_odleglosc = (d > 400L) ? 999L : d;
        TCCR5B  = 0u;
        TIMSK5 &= ~(1u << ICIE5);
        sr04_w_trakcie = false;
    }
}

static long sr04_measure(void) {
    if (sr04_w_trakcie) {
        /* poprzedni pomiar nie skonczony — resetuj */
        TCCR5B  = 0u;
        TIMSK5 &= ~(1u << ICIE5);
        sr04_w_trakcie = false;
        sr04_odleglosc = 999L;
    }
    sr04_w_trakcie = true;
    PORTD |=  (1u << PD7); _delay_us(10);
    PORTD &= ~(1u << PD7);
    TCNT5  = 0u;
    TIFR5  = (1u << ICF5);
    TIMSK5 |= (1u << ICIE5);
    TCCR5B  = (1u << ICNC5) | (1u << ICES5) | (1u << CS51);
    return sr04_odleglosc;   /* zwraca ostatni wynik; nowy bedzie po ~150ms */
}

/* ================================================================
   ADC — fotorezystor (PF0/A0) do auto-DRL
   AVCC jako referencja, preskaler 128, przerwanie po konwersji
   ================================================================ */
static volatile uint16_t adc_wynik = 0u;

ISR(ADC_vect) {
    adc_wynik = ADC;   /* makro ADC czyta ADCL+ADCH w kolejnosci */
}

static void adc_init(void) {
    DIDR0  |= (1u << ADC0D);                              /* wylacz bufor cyfrowy PF0 */
    ADMUX   = (1u << REFS0);                              /* AVCC, kanal ADC0 */
    ADCSRA  = (1u << ADEN) | (1u << ADIE)                /* wlacz ADC + przerwanie */
            | (1u << ADPS2) | (1u << ADPS1) | (1u << ADPS0); /* preskaler 128 */
    ADCSRA |= (1u << ADSC);                               /* start pierwszej konwersji */
}

/* ================================================================
   SILNIKI — kola mecanum X-layout
   k > 0 = "przod" w sensie okablowania  k < 0 = "tyl"  k = 0 = stop
   Uwaga: w tym projekcie k=-1 odpowiada fizycznej jeździe DO PRZODU.
   ================================================================ */
static uint8_t predkosc = 200u;

static void silnikA(int8_t k, uint8_t spd) {   /* FL przod-lewy */
    if      (k > 0) { PORTA |=  (1u<<PA0); PORTA &= ~(1u<<PA1); }
    else if (k < 0) { PORTA &= ~(1u<<PA0); PORTA |=  (1u<<PA1); }
    else            { PORTA &= ~((1u<<PA0)|(1u<<PA1)); }
    OCR3A = (k != 0) ? spd : 0u;
}

static void silnikB(int8_t k, uint8_t spd) {   /* FR przod-prawy */
    if      (k > 0) { PORTA |=  (1u<<PA2); PORTA &= ~(1u<<PA3); }
    else if (k < 0) { PORTA &= ~(1u<<PA2); PORTA |=  (1u<<PA3); }
    else            { PORTA &= ~((1u<<PA2)|(1u<<PA3)); }
    OCR4A = (k != 0) ? spd : 0u;
}

static void silnikC(int8_t k, uint8_t spd) {   /* RR tyl-prawy */
    if      (k > 0) { PORTA |=  (1u<<PA4); PORTA &= ~(1u<<PA5); }
    else if (k < 0) { PORTA &= ~(1u<<PA4); PORTA |=  (1u<<PA5); }
    else            { PORTA &= ~((1u<<PA4)|(1u<<PA5)); }
    OCR4B = (k != 0) ? spd : 0u;
}

static void silnikD(int8_t k, uint8_t spd) {   /* RL tyl-lewy */
    if      (k > 0) { PORTA |=  (1u<<PA6); PORTA &= ~(1u<<PA7); }
    else if (k < 0) { PORTA &= ~(1u<<PA6); PORTA |=  (1u<<PA7); }
    else            { PORTA &= ~((1u<<PA6)|(1u<<PA7)); }
    OCR4C = (k != 0) ? spd : 0u;
}

static void stop_all(void) {
    silnikA(0,0u); silnikB(0,0u); silnikC(0,0u); silnikD(0,0u);
}

/* ================================================================
   OSWIETLENIE — PCA9685 kanaly 0-3
   0=DRL lewy  1=DRL prawy  2=kier. lewy  3=kier. prawy
   ================================================================ */
#define KANAL_DRL_LEWY      0u
#define KANAL_DRL_PRAWY     1u
#define KANAL_KIERUNK_LEWY  2u
#define KANAL_KIERUNK_PRAWY 3u
#define JASNOSC_PELNA       4095u
#define JASNOSC_ZERO        0u
#define OKRES_PULSOWANIA_MS 1000u

static bool     drl_wlaczone            = false;
static bool     kierunk_lewy_wlaczony   = false;
static bool     kierunk_prawy_wlaczony  = false;
static bool     awaryjne_wlaczone       = false;
static uint32_t czas_ostatniego_i2c_mig = 0u;

static void ustaw_kanal(uint8_t kanal, bool wlaczony) {
    pca_set(kanal, 0u, wlaczony ? JASNOSC_PELNA : JASNOSC_ZERO);
}

static void drl_wlacz(void) {
    drl_wlaczone = true;
    ustaw_kanal(KANAL_DRL_LEWY, true); ustaw_kanal(KANAL_DRL_PRAWY, true);
    info("DRL: WLACZONE");
}
static void drl_wylacz(void) {
    drl_wlaczone = false;
    ustaw_kanal(KANAL_DRL_LEWY, false); ustaw_kanal(KANAL_DRL_PRAWY, false);
    info("DRL: WYLACZONE");
}

static void kierunk_lewy_wlacz(void) {
    if (kierunk_prawy_wlaczony) { kierunk_prawy_wlaczony = false; ustaw_kanal(KANAL_KIERUNK_PRAWY, false); }
    awaryjne_wlaczone = false;
    kierunk_lewy_wlaczony = true;
    ustaw_kanal(KANAL_KIERUNK_LEWY, true);
    info("KIERUNK: LEWY");
}
static void kierunk_lewy_wylacz(void) {
    kierunk_lewy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false);
    info("KIERUNK: LEWY WYLACZONY");
}

static void kierunk_prawy_wlacz(void) {
    if (kierunk_lewy_wlaczony) { kierunk_lewy_wlaczony = false; ustaw_kanal(KANAL_KIERUNK_LEWY, false); }
    awaryjne_wlaczone = false;
    kierunk_prawy_wlaczony = true;
    ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
    info("KIERUNK: PRAWY");
}
static void kierunk_prawy_wylacz(void) {
    kierunk_prawy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    info("KIERUNK: PRAWY WYLACZONY");
}

static void awaryjne_wlacz(void) {
    if (kierunk_lewy_wlaczony)  { kierunk_lewy_wlaczony  = false; ustaw_kanal(KANAL_KIERUNK_LEWY,  false); }
    if (kierunk_prawy_wlaczony) { kierunk_prawy_wlaczony = false; ustaw_kanal(KANAL_KIERUNK_PRAWY, false); }
    awaryjne_wlaczone = true;
    ustaw_kanal(KANAL_KIERUNK_LEWY, true); ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
    info("AWARYJNE: WLACZONE");
}
static void awaryjne_wylacz(void) {
    awaryjne_wlaczone = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false); ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    info("AWARYJNE: WYLACZONE");
}

/* Plynne pulsowanie kierunkowskazow (trójkatne, cykl 1s, max 50 Hz I2C). */
static void obsluz_miganie(void) {
    if (!kierunk_lewy_wlaczony && !kierunk_prawy_wlaczony && !awaryjne_wlaczone) return;
    uint32_t teraz = millis();
    if ((teraz - czas_ostatniego_i2c_mig) < 20u) return;
    czas_ostatniego_i2c_mig = teraz;

    uint32_t t = teraz % OKRES_PULSOWANIA_MS;
    uint16_t jasnosc = (t < (OKRES_PULSOWANIA_MS / 2u))
        ? (uint16_t)((uint32_t)t * 4095UL / (OKRES_PULSOWANIA_MS / 2u))
        : (uint16_t)((OKRES_PULSOWANIA_MS - t) * 4095UL / (OKRES_PULSOWANIA_MS / 2u));

    if (awaryjne_wlaczone) {
        pca_set(KANAL_KIERUNK_LEWY,  0u, jasnosc);
        pca_set(KANAL_KIERUNK_PRAWY, 0u, jasnosc);
    } else {
        if (kierunk_lewy_wlaczony)  pca_set(KANAL_KIERUNK_LEWY,  0u, jasnosc);
        if (kierunk_prawy_wlaczony) pca_set(KANAL_KIERUNK_PRAWY, 0u, jasnosc);
    }
}

/* ================================================================
   STAN GLOBALNY
   ================================================================ */
static bool     pikniecie_trwa            = false;
static bool     tryb_auto_drl             = false;

static uint32_t czas_pomiaru              = 0u;
static uint32_t czas_ostatniego_pikniecia = 0u;

static long     odleglosc_cm              = 999L;

static char     aktualny_kierunek[17]     = "STOP";
static char     prev_kierunek[17]         = "";
static char     ostatni_ruch              = 'S';

static long     prev_odleglosc_lcd        = -1L;

/* ================================================================
   LCD — aktualizacja przy zmianie wartosci
   Linia 0: "Kier: XXXXXXXXXX" (6 + 10 = 16)
   Linia 1: "Dyst: XXXXXXXXXX" (6 + 10 = 16)
   ================================================================ */
static void lcd_update(void) {
    /* linia 0 — kierunek jazdy */
    if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
        lcd_goto(0u, 0u);
        lcd_puts("Kier: ");
        lcd_puts_w(aktualny_kierunek, 10u);
        strcpy(prev_kierunek, aktualny_kierunek);
    }

    /* linia 1 — dystans od przeszkody */
    if (odleglosc_cm != prev_odleglosc_lcd) {
        char buf[11];
        if (odleglosc_cm == 999L) {
            strcpy(buf, "Brak/Max");
        } else {
            uint8_t i = 0u;
            long v = odleglosc_cm;
            if (v == 0L) {
                buf[i++] = '0';
            } else {
                while (v > 0L) { buf[i++] = (char)('0' + (v % 10L)); v /= 10L; }
                for (uint8_t a = 0u, b = (uint8_t)(i - 1u); a < b; a++, b--) {
                    char tmp = buf[a]; buf[a] = buf[b]; buf[b] = tmp;
                }
            }
            buf[i++] = ' '; buf[i++] = 'c'; buf[i++] = 'm';
            buf[i]   = '\0';
        }
        lcd_goto(0u, 1u);
        lcd_puts("Dyst: ");
        lcd_puts_w(buf, 10u);
        prev_odleglosc_lcd = odleglosc_cm;
    }
}

/* ================================================================
   OBSLUGA KOMEND BLUETOOTH
   ================================================================ */
static void handle_cmd(char c) {
    switch (c) {

        /* --- jazda prosto --- */
        case 'F': case 'f':
            silnikA(-1, predkosc); silnikB(-1, predkosc);
            silnikC(-1, predkosc); silnikD(-1, predkosc);
            ostatni_ruch = 'F';
            strcpy(aktualny_kierunek, "PRZOD"); info("PRZOD");
            break;

        case 'B': case 'b':
            silnikA(1, predkosc); silnikB(1, predkosc);
            silnikC(1, predkosc); silnikD(1, predkosc);
            ostatni_ruch = 'B';
            strcpy(aktualny_kierunek, "TYL"); info("TYL");
            break;

        /* --- obrot pivot (lewa os vs prawa os) --- */
        case 'L': case 'l':
            silnikA(-1, predkosc); silnikD(-1, predkosc);
            silnikB( 1, predkosc); silnikC( 1, predkosc);
            ostatni_ruch = 'L';
            strcpy(aktualny_kierunek, "LEWO"); info("LEWO");
            break;

        case 'R': case 'r':
            silnikA( 1, predkosc); silnikD( 1, predkosc);
            silnikB(-1, predkosc); silnikC(-1, predkosc);
            ostatni_ruch = 'R';
            strcpy(aktualny_kierunek, "PRAWO"); info("PRAWO");
            break;

        /* --- jazda bokiem — strafe mecanum X-layout ---
         * Strafe prawo (Y): FL=przod, FR=tyl, RL=tyl, RR=przod
         * Strafe lewo  (X): FL=tyl,  FR=przod, RL=przod, RR=tyl  */
        case 'X': case 'x':
            silnikA( 1, predkosc); silnikB(-1, predkosc);
            silnikC( 1, predkosc); silnikD(-1, predkosc);
            ostatni_ruch = 'X';
            strcpy(aktualny_kierunek, "BOK-LEWO"); info("BOK-LEWO");
            break;

        case 'Y': case 'y':
            silnikA(-1, predkosc); silnikB( 1, predkosc);
            silnikC(-1, predkosc); silnikD( 1, predkosc);
            ostatni_ruch = 'Y';
            strcpy(aktualny_kierunek, "BOK-PRAWO"); info("BOK-PRAWO");
            break;

        /* --- skos do przodu — diagonal mecanum X-layout ---
         * Skos przod-lewo  (I): FR=przod, RL=przod; FL i RR stop
         * Skos przod-prawo (K): FL=przod, RR=przod; FR i RL stop */
        case 'I': case 'i':
            silnikA(0, 0u);          silnikB(-1, predkosc);
            silnikC(0, 0u);          silnikD(-1, predkosc);
            ostatni_ruch = 'I';
            strcpy(aktualny_kierunek, "SKOS-PL"); info("SKOS PRZOD-LEWO");
            break;

        case 'K': case 'k':
            silnikA(-1, predkosc);   silnikB(0, 0u);
            silnikC(-1, predkosc);   silnikD(0, 0u);
            ostatni_ruch = 'K';
            strcpy(aktualny_kierunek, "SKOS-PP"); info("SKOS PRZOD-PRAWO");
            break;

        case 'S': case 's':
            stop_all();
            ostatni_ruch = 'S';
            strcpy(aktualny_kierunek, "STOP"); info("STOP");
            break;

        /* --- predkosc --- */
        case '+':
            if (predkosc <= 235u) predkosc += 20u; else predkosc = 255u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            handle_cmd(ostatni_ruch);
            break;

        case '-':
            if (predkosc >= 80u) predkosc -= 20u; else predkosc = 60u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            handle_cmd(ostatni_ruch);
            break;

        /* --- oswietlenie --- */
        case 'D': case 'd':
            tryb_auto_drl = false;   /* reczne D wylacza automat */
            if (drl_wlaczone) drl_wylacz(); else drl_wlacz();
            break;

        case 'A': case 'a':
            tryb_auto_drl = true;
            info("DRL: TRYB AUTO");
            break;

        case 'Q': case 'q':
            if (kierunk_lewy_wlaczony)  kierunk_lewy_wylacz();  else kierunk_lewy_wlacz();
            break;

        case 'E': case 'e':
            if (kierunk_prawy_wlaczony) kierunk_prawy_wylacz(); else kierunk_prawy_wlacz();
            break;

        case 'H': case 'h':
            if (awaryjne_wlaczone) awaryjne_wylacz(); else awaryjne_wlacz();
            break;

        default: break;
    }
}

/* ================================================================
   MAIN
   ================================================================ */
int main(void) {
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
    for (uint8_t i = 0u; i < 4u; i++) { pca_set(i, 0u, 0u); }

    lcd_goto(0u, 0u); lcd_puts("  SAMOCHODZIK   ");
    lcd_goto(0u, 1u); lcd_puts("   GOTOWY :)    ");
    _delay_ms(1500);
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
            odleglosc_cm = sr04_measure();
        }

        /* PDC — asystent parkowania (buzzer proporcjonalny do dystansu) */
        if (odleglosc_cm > 100L || odleglosc_cm == 999L) {
            if (pikniecie_trwa) { tone_stop(); pikniecie_trwa = false; }
        } else if (odleglosc_cm <= 15L) {
            if (!pikniecie_trwa) { tone_start(1000u); pikniecie_trwa = true; }
            czas_ostatniego_pikniecia = teraz;
        } else {
            uint16_t interwal = (uint16_t)(odleglosc_cm * 10L);
            if (!pikniecie_trwa) {
                if ((teraz - czas_ostatniego_pikniecia) >= (uint32_t)interwal) {
                    czas_ostatniego_pikniecia = teraz;
                    tone_start(1000u);
                    pikniecie_trwa = true;
                }
            } else {
                if ((teraz - czas_ostatniego_pikniecia) >= 80UL) {
                    tone_stop();
                    pikniecie_trwa = false;
                }
            }
        }

        /* ADC (fotorezystor) co 100ms — auto-DRL z histereza */
        static uint32_t czas_adc = 0u;
        static bool     jest_ciemno = false;

        if ((teraz - czas_adc) >= 100UL) {
            czas_adc = teraz;

            uint16_t odczyt;
            uint8_t sreg = SREG;
            cli();
            odczyt = adc_wynik;
            SREG = sreg;
            ADCSRA |= (1u << ADSC);   /* start kolejnej konwersji */

            /* histereza: wlacza przy >700, wylacza przy <500 */
            if      (odczyt > 700u) jest_ciemno = true;
            else if (odczyt < 500u) jest_ciemno = false;

            if (tryb_auto_drl) {
                if  (jest_ciemno && !drl_wlaczone) drl_wlacz();
                else if (!jest_ciemno && drl_wlaczone) drl_wylacz();
            }
        }

        /* odswiezanie LCD tylko przy zmianach wartosci */
        lcd_update();

        /* pulsowanie kierunkowskazow */
        obsluz_miganie();

        /* komendy z USB i Bluetooth */
        if (uart0_available()) { handle_cmd((char)uart0_read()); }
        if (uart1_available()) { handle_cmd((char)uart1_read()); }
    }

    return 0;
}