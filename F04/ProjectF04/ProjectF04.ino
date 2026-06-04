/*
 * SAMOCHODZIK — Bare Metal ATmega2560
 * Bluetooth HC-05 + 2x L298N + LCD I2C + HC-SR04 + Buzzer + LED + PCA9685
 *
 * Piny:
 *   HC-05:    UART1 TX=Pin19(PD3) RX=Pin18(PD2)
 *   L298N #1: ENA=Pin5(PE3/OC3A)  IN1=22(PA0) IN2=23(PA1)
 *             ENB=Pin6(PH3/OC4A)  IN3=24(PA2) IN4=25(PA3)
 *   L298N #2: ENC=Pin7(PH4/OC4B)  IN5=26(PA4) IN6=27(PA5)
 *             END=Pin8(PH5/OC4C)  IN7=28(PA6) IN8=29(PA7)
 *   LCD I2C:  SDA=Pin20 SCL=Pin21, PCF8574T addr=0x27
 *   PCA9685:  ten sam I2C, addr=0x40
 *             kanal 0 = DRL lewy (bialy)
 *             kanal 1 = DRL prawy (bialy)
 *             kanal 2 = kierunkowskaz lewy (zolty)
 *             kanal 3 = kierunkowskaz prawy (zolty)
 *   HC-SR04:  TRIG=Pin38(PD7)  ECHO=Pin48(PL1/ICP5)
 *   Buzzer:   Pin13(PB5/OC1A)
 *   LED:      Pin46(PL3)
 *   Hall:     PE4 = Pin2 (INT4)
 *
 * Timery:
 *   Timer0: CTC 1ms -> millis()
 *   Timer1: CTC toggle OC1A -> buzzer (Pin13/PB5)
 *   Timer3: Fast PWM 8-bit -> ENA (Pin5/PE3)
 *   Timer4: Fast PWM 8-bit -> ENB(6) ENC(7) END(8)
 *   Timer5: Input Capture -> pomiar HC-SR04 asynchroniczny (ICP5=PL1=Pin48)
 *
 * Skret - skid-steer pivot:
 *   Lewa os:  silnikA + silnikD
 *   Prawa os: silnikB + silnikC
 *   LEWO:  lewa os do tylu, prawa do przodu
 *   PRAWO: lewa os do przodu, prawa do tylu
 *   PREDKOSC_SKRET=255 (max) - silniki TT potrzebuja pelnego
 *   momentu do pokonania tarcia bocznego przy rozstawie 16x21cm.
 *
 * LCD:
 *   Linia 0: kierunek jazdy
 *   Linia 1: RPM z czujnika Halla
 *
 * Komendy Bluetooth:
 *   F/B/L/R/S = jazda przod/tyl/lewo/prawo/stop
 *   +/-       = szybciej/wolniej (tylko jazda prosto/wstecz)
 *   D         = DRL wlacz/wylacz
 *   Q         = lewy kierunkowskaz wlacz/wylacz
 *   E         = prawy kierunkowskaz wlacz/wylacz
 *   H         = awaryjne wlacz/wylacz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define F_CPU 16000000UL

/* Predkosc obrotu zawsze maksymalna — silniki TT przy rozstawie
 * 16x21 cm potrzebuja pelnego momentu do pivot-turn. */
#define PREDKOSC_SKRET 255u

/* ================================================================
   MILLIS - Timer0 CTC
   Prescaler 64, OCR0A=249 -> 1ms na przerwanie
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
   UART0 (USB) + UART1 (HC-05)
   UBRR = F_CPU/(16*BAUD) - 1 = 103 dla 9600 baud @ 16MHz
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
   TWI (I2C) - 100 kHz
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
   PCF8574T + HD44780 LCD (4-bit przez I2C)
   Adres 0x27 -> SLA+W=0x4E
   P0=RS P1=RW P2=EN P3=BL P4-P7=D4-D7
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
    pcf_write(b | LCD_EN);   _delay_us(1);
    pcf_write(b & (uint8_t)~LCD_EN); _delay_us(50);
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
    lcd_cmd(0x28u); _delay_us(50);
    lcd_cmd(0x08u); _delay_us(50);
    lcd_cmd(0x01u); _delay_ms(2);
    lcd_cmd(0x06u); _delay_us(50);
    lcd_cmd(0x0Cu); _delay_us(50);
}

static void lcd_clear(void) { lcd_cmd(0x01u); _delay_ms(2); }

static void lcd_goto(uint8_t col, uint8_t row) {
    lcd_cmd(0x80u | ((row ? 0x40u : 0x00u) + col));
    _delay_us(50);
}

static void lcd_puts(const char *s) {
    while (*s) { lcd_chr((uint8_t)*s++); _delay_us(50); }
}

static void lcd_putn(int32_t n) {
    char buf[12]; int8_t i = 0;
    if (n < 0) { lcd_chr('-'); n = -n; }
    if (n == 0) { lcd_chr('0'); return; }
    while (n > 0) { buf[i++] = (char)('0' + (n % 10)); n /= 10; }
    while (i > 0) { lcd_chr((uint8_t)buf[--i]); }
}

static void lcd_puts_pad(const char *s) {
    uint8_t len = (uint8_t)strlen(s);
    if (len > 16u) len = 16u;
    lcd_puts(s);
    for (uint8_t i = len; i < 16u; i++) { lcd_chr(' '); }
}

/* ================================================================
   PCA9685 - ekspander PWM (LEDy kanaly 0-3)
   Adres 0x40, MODE1=0x21 (AI=1), LED0_ON_L=0x06
   ================================================================ */
#define PCA_ADDR_W  (0x40u << 1u)
#define PCA_MODE1   0x00u
#define PCA_LED0    0x06u

static void pca9685_init(void) {
    if (!twi_start()) return;
    (void)twi_write_byte(PCA_ADDR_W);
    (void)twi_write_byte(PCA_MODE1);
    (void)twi_write_byte(0x21u);
    twi_stop();
    _delay_ms(1);
}

static void pca_set(uint8_t ch, uint16_t on, uint16_t off) {
    if (!twi_start()) return;
    (void)twi_write_byte(PCA_ADDR_W);
    (void)twi_write_byte(PCA_LED0 + 4u * ch);
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
   Silniki: PORTA PA0-PA7 = Piny 22-29
   Buzzer:  PB5 = Pin13
   LED:     PL3 = Pin46
   SR04:    TRIG=PD7=Pin38, ECHO=PL1=Pin48 (ICP5)
   Hall:    PE4 = Pin2 (INT4, wejscie z pull-up)
   ================================================================ */
static void gpio_init(void) {
    DDRA  = 0xFFu; PORTA = 0x00u;
    DDRB  |=  (1u << PB5); PORTB &= ~(1u << PB5);
    DDRL  |=  (1u << PL3); PORTL &= ~(1u << PL3);
    DDRD  |=  (1u << PD7); PORTD &= ~(1u << PD7);
    DDRL  &= ~(1u << PL1); PORTL &= ~(1u << PL1);
}

/* ================================================================
   BUZZER - Timer1 CTC, toggle OC1A=PB5=Pin13
   OCR1A = F_CPU/(2*8*freq) - 1
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
   HC-SR04 - asynchroniczny pomiar przez Timer5 Input Capture
   ICP5=PL1=Pin48, prescaler 8 -> 0.5us/tick
   Odleglosc [cm] = tiki / 116
   ================================================================ */
static volatile uint16_t sr04_start     = 0u;
static volatile long     sr04_odleglosc = 999L;
static volatile bool     sr04_w_trakcie = false;

/**
 * @brief  Przerwanie Input Capture Timer5.
 *         Zbocze narastajace: zapis czasu startu echa.
 *         Zbocze opadajace:   obliczenie odleglosci.
 */
ISR(TIMER5_CAPT_vect) {
    uint16_t teraz = ICR5;
    if (TCCR5B & (1u << ICES5)) {
        sr04_start = teraz;
        TCCR5B &= ~(1u << ICES5);
        TIFR5   = (1u << ICF5);
    } else {
        uint16_t dt = teraz - sr04_start;
        long d = (long)dt / 116L;
        sr04_odleglosc = (d > 400L) ? 999L : d;
        TCCR5B  = 0u;
        TIMSK5 &= ~(1u << ICIE5);
        sr04_w_trakcie = false;
    }
}

/**
 * @brief  Wyzwala impuls TRIG i startuje asynchroniczny pomiar.
 * @return Ostatnio zmierzona odleglosc [cm] lub 999.
 */
static long sr04_measure(void) {
    if (sr04_w_trakcie) {
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
    return sr04_odleglosc;
}

/* ================================================================
   SILNIKI
   Lewa os:  silnikA (PA0/PA1, OCR3A) + silnikD (PA6/PA7, OCR4C)
   Prawa os: silnikB (PA2/PA3, OCR4A) + silnikC (PA4/PA5, OCR4B)
   k>0=przod  k<0=tyl  k=0=stop   spd=PWM 0-255
   ================================================================ */
static uint8_t predkosc = 200u;

/**
 * @brief  Silnik A - przod-lewy (ENA=OCR3A, IN1=PA0, IN2=PA1).
 * @param  k    Kierunek: >0 przod, <0 tyl, 0 stop.
 * @param  spd  Wartosc PWM 0-255.
 */
static void silnikA(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u<<PA0); PORTA &= ~(1u<<PA1); }
    else if (k < 0) { PORTA &= ~(1u<<PA0); PORTA |=  (1u<<PA1); }
    else            { PORTA &= ~((1u<<PA0)|(1u<<PA1)); }
    OCR3A = (k != 0) ? spd : 0u;
}

/**
 * @brief  Silnik B - przod-prawy (ENB=OCR4A, IN3=PA2, IN4=PA3).
 */
static void silnikB(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u<<PA2); PORTA &= ~(1u<<PA3); }
    else if (k < 0) { PORTA &= ~(1u<<PA2); PORTA |=  (1u<<PA3); }
    else            { PORTA &= ~((1u<<PA2)|(1u<<PA3)); }
    OCR4A = (k != 0) ? spd : 0u;
}

/**
 * @brief  Silnik C - tyl-prawy (ENC=OCR4B, IN5=PA4, IN6=PA5).
 */
static void silnikC(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u<<PA4); PORTA &= ~(1u<<PA5); }
    else if (k < 0) { PORTA &= ~(1u<<PA4); PORTA |=  (1u<<PA5); }
    else            { PORTA &= ~((1u<<PA4)|(1u<<PA5)); }
    OCR4B = (k != 0) ? spd : 0u;
}

/**
 * @brief  Silnik D - tyl-lewy (END=OCR4C, IN7=PA6, IN8=PA7).
 */
static void silnikD(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u<<PA6); PORTA &= ~(1u<<PA7); }
    else if (k < 0) { PORTA &= ~(1u<<PA6); PORTA |=  (1u<<PA7); }
    else            { PORTA &= ~((1u<<PA6)|(1u<<PA7)); }
    OCR4C = (k != 0) ? spd : 0u;
}

static void stop_all(void) {
    silnikA(0,0u); silnikB(0,0u); silnikC(0,0u); silnikD(0,0u);
}

/* ================================================================
   LEDY PCA9685
   kanal 0=DRL lewy  1=DRL prawy  2=kier.lewy  3=kier.prawy
   ================================================================ */
#define KANAL_DRL_LEWY      0u
#define KANAL_DRL_PRAWY     1u
#define KANAL_KIERUNK_LEWY  2u
#define KANAL_KIERUNK_PRAWY 3u
#define JASNOSC_PELNA       4095u
#define JASNOSC_ZERO        0u
#define OKRES_PULSOWANIA_MS 1000u

static bool     drl_wlaczone           = false;
static bool     kierunk_lewy_wlaczony  = false;
static bool     kierunk_prawy_wlaczony = false;
static bool     awaryjne_wlaczone      = false;
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

/**
 * @brief  Plynne pulsowanie kierunkowskazow (50 Hz, cykl 1s).
 *         Ogranicza zapisy I2C do co 20ms.
 */
static void obsluz_miganie(void) {
    if (!kierunk_lewy_wlaczony && !kierunk_prawy_wlaczony && !awaryjne_wlaczone) return;
    uint32_t teraz = millis();
    if ((teraz - czas_ostatniego_i2c_mig) < 20u) return;
    czas_ostatniego_i2c_mig = teraz;

    uint32_t t = teraz % OKRES_PULSOWANIA_MS;
    uint16_t jasnosc;
    if (t < (OKRES_PULSOWANIA_MS / 2u)) {
        jasnosc = (uint16_t)((uint32_t)t * 4095UL / (OKRES_PULSOWANIA_MS / 2u));
    } else {
        jasnosc = (uint16_t)((OKRES_PULSOWANIA_MS - t) * 4095UL / (OKRES_PULSOWANIA_MS / 2u));
    }

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
static bool     jedzie_przod              = false;
static bool     pikniecie_trwa            = false;

static uint32_t czas_pomiaru              = 0u;
static uint32_t czas_ostatniego_pikniecia = 0u;

static long     odleglosc_cm = 999L;

static char aktualny_kierunek[17] = "STOP";
static char prev_kierunek[17]     = "";
static char ostatni_ruch           = 'S';

/* poprzednie wartosci do detekcji zmian na LCD */
static long prev_odleglosc_lcd = -1L;

/* ================================================================
   AKTUALIZACJA LCD
   Linia 0: "Kier: XXXX      " (16 znakow lacznie)
   Linia 1: "RPM:  XXXXX     " (16 znakow lacznie)
   Wywolywana przy kazdej zmianie wartosci ORAZ co 150ms
   (razem z pomiarem SR04) zeby miec pewnosc ze ekran dziala.
   ================================================================ */

/**
 * @brief  Wypisuje string na LCD i dopelnia spacjami do zadanej
 *         szerokosci. Uzyj po lcd_goto(), podaj ile znakow zostalo
 *         w linii (nie dlugos stringa).
 * @param  s       String do wypisania.
 * @param  szerokosc  Ile znakow ma zająć cała kolumna.
 */
static void lcd_puts_w(const char *s, uint8_t szerokosc) {
    uint8_t n = 0u;
    while (*s && n < szerokosc) { lcd_chr((uint8_t)*s++); _delay_us(50); n++; }
    while (n < szerokosc)       { lcd_chr(' ');            _delay_us(50); n++; }
}

static void lcd_update(void) {
    /* linia 0 - "Kier: " (6) + kierunek dopelniany spacjami do 10 = 16 */
    if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
        lcd_goto(0u, 0u);
        lcd_puts("Kier: ");
        lcd_puts_w(aktualny_kierunek, 10u);   /* 10 = 16 - 6 */
        strcpy(prev_kierunek, aktualny_kierunek);
    }
/* linia 1 - "Dyst: " (6) + liczba dopelniana spacjami do 10 = 16 */
    if (odleglosc_cm != prev_odleglosc_lcd) {
        char buf[11];
        // Zabezpieczenie przed brakiem echa (999 oznacza brak przeszkody)
        if (odleglosc_cm == 999L) {
            strcpy(buf, "Brak/Max");
        } else {
            uint8_t i = 0u;
            long v = odleglosc_cm;
            // Konwersja liczby na string (itoa)
            if (v == 0L) { 
                buf[i++] = '0'; 
            } else { 
                while (v > 0L) { 
                    buf[i++] = (char)('0' + (v % 10L)); 
                    v /= 10L; 
                } 
            }
            // Odwrócenie cyfr
            for (uint8_t a = 0u, b = (uint8_t)(i - 1u); a < b; a++, b--) {
                char tmp = buf[a]; buf[a] = buf[b]; buf[b] = tmp;
            }
            // Dopisywanie jednostki " cm"
            buf[i++] = ' ';
            buf[i++] = 'c';
            buf[i++] = 'm';
            buf[i] = '\0';
        }
        lcd_goto(0u, 1u);
        lcd_puts("Dyst: ");
        lcd_puts_w(buf, 10u);
        prev_odleglosc_lcd = odleglosc_cm;
    }
}

/* ================================================================
   OBSLUGA KOMENDY
   ================================================================ */
static void handle_cmd(char c) {
    switch (c) {
        case 'F': case 'f':
            silnikA(-1, predkosc); silnikB(-1, predkosc);
            silnikC(-1, predkosc); silnikD(-1, predkosc);
            jedzie_przod = true;  ostatni_ruch = 'F';
            strcpy(aktualny_kierunek, "PRZOD"); info("PRZOD");
            break;

        case 'B': case 'b':
            silnikA(1, predkosc); silnikB(1, predkosc);
            silnikC(1, predkosc); silnikD(1, predkosc);
            jedzie_przod = false; ostatni_ruch = 'B';
            strcpy(aktualny_kierunek, "TYL"); info("TYL");
            break;

        /*
         * LEWO: lewa os (A+D) do tylu, prawa os (B+C) do przodu.
         * Predkosc=255 — pelny moment do pivot-turn.
         */
        case 'L': case 'l':
            silnikA(-1, PREDKOSC_SKRET); silnikD(-1, PREDKOSC_SKRET);
            silnikB( 1, PREDKOSC_SKRET); silnikC( 1, PREDKOSC_SKRET);
            jedzie_przod = false; ostatni_ruch = 'L';
            strcpy(aktualny_kierunek, "LEWO"); info("LEWO");
            break;

        /*
         * PRAWO: lewa os (A+D) do przodu, prawa os (B+C) do tylu.
         */
        case 'R': case 'r':
            silnikA( 1, PREDKOSC_SKRET); silnikD( 1, PREDKOSC_SKRET);
            silnikB(-1, PREDKOSC_SKRET); silnikC(-1, PREDKOSC_SKRET);
            jedzie_przod = false; ostatni_ruch = 'R';
            strcpy(aktualny_kierunek, "PRAWO"); info("PRAWO");
            break;

        case 'S': case 's':
            stop_all();
            jedzie_przod = false; ostatni_ruch = 'S';
            strcpy(aktualny_kierunek, "STOP"); info("STOP");
            break;

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

        case 'D': case 'd':
            if (drl_wlaczone)           drl_wylacz();           else drl_wlacz();           break;
        case 'Q': case 'q':
            if (kierunk_lewy_wlaczony)  kierunk_lewy_wylacz();  else kierunk_lewy_wlacz();  break;
        case 'E': case 'e':
            if (kierunk_prawy_wlaczony) kierunk_prawy_wylacz(); else kierunk_prawy_wlacz(); break;
        case 'H': case 'h':
            if (awaryjne_wlaczone)      awaryjne_wylacz();      else awaryjne_wlacz();      break;

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
    info("F/B/L/R/S/+/-/D/Q/E/H");

    while (1) {
        uint32_t teraz = millis();

        /* pomiar odleglosci co 150ms */
        if ((teraz - czas_pomiaru) >= 150UL) {
            czas_pomiaru = teraz;
            odleglosc_cm = sr04_measure();
        }

        /* PDC - asystent parkowania */
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

        /* aktualizacja LCD przy kazdej zmianie (wewnetrzne porownanie
         * chroni przed zbednymi zapisami I2C gdy nic sie nie zmienilo) */
        lcd_update();

        /* miganie / pulsowanie kierunkowskazow */
        obsluz_miganie();

        /* komendy Bluetooth / USB */
        if (uart0_available()) { handle_cmd((char)uart0_read()); }
        if (uart1_available()) { handle_cmd((char)uart1_read()); }
    }

    return 0;
}