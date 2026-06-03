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
 *
 * Timery:
 *   Timer0: CTC 1ms -> millis()
 *   Timer1: CTC toggle OC1A -> buzzer (Pin13/PB5)
 *   Timer3: Fast PWM 8-bit -> ENA (Pin5/PE3)
 *   Timer4: Fast PWM 8-bit -> ENB(6) ENC(7) END(8)
 *   Timer5: Input Capture -> pomiar HC-SR04 asynchroniczny (ICP5=PL1=Pin48)
 *
 * Skret — skid-steer pivot:
 *   Lewa os:  silnikA + silnikC
 *   Prawa os: silnikB + silnikD
 *   LEWO:  lewa os do tylu, prawa do przodu — obrót w miejscu
 *   PRAWO: lewa os do przodu, prawa do tylu — obrót w miejscu
 *   predkosc_skret = 255 (max) niezaleznie od predkosc jazdy,
 *   bo silniki TT 3-8V maja wysoki opór statyczny przy 16x21cm
 *   rozstawie i potrzebuja pelnego momentu do pivot-turn.
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

/*
 * Predkosc skrętu zawsze maksymalna (255).
 * Silniki TT przy rozstawie 16x21 cm musza pokonac tarcie
 * boczne wszystkich 4 kol — nizsze PWM powoduje blokowanie.
 * Predkosc jazdy (predkosc) reguluje tylko F i B.
 */
#define PREDKOSC_SKRET 255u

/* ================================================================
   MILLIS — Timer0 CTC
   Prescaler 64, OCR0A=249 -> 250 tickow x 4us = 1ms
   ================================================================ */
volatile uint32_t ms_count = 0;

ISR(TIMER0_COMPA_vect) {
    ms_count++;
}

static void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249u;
    TIMSK0 = (1 << OCIE0A);
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
   UART0 (USB monitor) + UART1 (HC-05 Bluetooth)
   UBRR = F_CPU / (16 x BAUD) - 1 = 103 dla 9600 baud @ 16MHz
   ================================================================ */
#define BAUD     9600UL
#define UBRR_VAL (F_CPU / (16UL * BAUD) - 1u)

static void uart0_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8u);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static bool    uart0_available(void) { return (bool)(UCSR0A & (1 << RXC0)); }
static uint8_t uart0_read(void)      { return UDR0; }
static void    uart0_putc(char c)    { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
static void    uart0_puts(const char *s) { while (*s) uart0_putc(*s++); }

static void uart1_init(void) {
    UBRR1H = (uint8_t)(UBRR_VAL >> 8u);
    UBRR1L = (uint8_t)(UBRR_VAL);
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

static bool    uart1_available(void) { return (bool)(UCSR1A & (1 << RXC1)); }
static uint8_t uart1_read(void)      { return UDR1; }
static void    uart1_putc(char c)    { while (!(UCSR1A & (1 << UDRE1))); UDR1 = c; }
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
    TWCR = (1 << TWEN);
}

static bool twi_start(void) {
    uint16_t t = 0u;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

static void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static bool twi_write_byte(uint8_t data) {
    uint16_t t = 0u;
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

/* ================================================================
   PCF8574T + HD44780 LCD (tryb 4-bit przez I2C)
   Adres: 0x27 -> SLA+W = 0x4E
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
    pcf_write(b | LCD_EN);
    _delay_us(1);
    pcf_write(b & (uint8_t)~LCD_EN);
    _delay_us(50);
}

static void lcd_send(uint8_t byte, uint8_t rs) {
    uint8_t hi = (byte & 0xF0u)          | LCD_BL | rs;
    uint8_t lo = ((byte << 4u) & 0xF0u)  | LCD_BL | rs;
    lcd_strobe(hi);
    lcd_strobe(lo);
}

#define lcd_cmd(c)  lcd_send((c), 0u)
#define lcd_chr(c)  lcd_send((c), LCD_RS)

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
   PCA9685 — ekspander PWM (LEDy kanaly 0-3)
   Adres: 0x40 -> SLA+W = 0x80
   MODE1=0x21 (AI=1), LED0_ON_L=0x06, +4 na kanal
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
   Timer3: OC3A = ENA = Pin5  = PE3
   Timer4: OC4A = ENB = Pin6  = PH3
           OC4B = ENC = Pin7  = PH4
           OC4C = END = Pin8  = PH5
   Fast PWM 8-bit, prescaler 8 -> ~7.8 kHz
   ================================================================ */
static void pwm_init(void) {
    TCCR3A = (1 << COM3A1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31);
    OCR3A  = 0u;
    DDRE  |= (1 << PE3);

    TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM40);
    TCCR4B = (1 << WGM42)  | (1 << CS41);
    OCR4A  = 0u; OCR4B = 0u; OCR4C = 0u;
    DDRH  |= (1 << PH3) | (1 << PH4) | (1 << PH5);
}

/* ================================================================
   GPIO
   Silniki: PORTA PA0-PA7 = Piny 22-29
   Buzzer:  PB5 = Pin13
   LED:     PL3 = Pin46
   SR04:    TRIG=PD7=Pin38  ECHO=PL1=Pin48 (ICP5, wejscie)
   ================================================================ */
static void gpio_init(void) {
    /* silniki */
    DDRA  = 0xFFu;
    PORTA = 0x00u;

    /* buzzer Pin13 = PB5 */
    DDRB  |=  (1u << PB5);
    PORTB &= ~(1u << PB5);

    /* LED Pin46 = PL3 */
    DDRL  |=  (1u << PL3);
    PORTL &= ~(1u << PL3);

    /* SR04 TRIG = PD7 (wyjscie) */
    DDRD  |=  (1u << PD7);
    PORTD &= ~(1u << PD7);

    /* SR04 ECHO = PL1/ICP5 = Pin48 (wejscie, bez pull-up) */
    DDRL  &= ~(1u << PL1);
    PORTL &= ~(1u << PL1);
}

/* ================================================================
   BUZZER TONE — Timer1 CTC, toggle OC1A = PB5 = Pin13
   WGM12=1 (CTC, TOP=OCR1A), COM1A0=1 (toggle)
   Prescaler 8: OCR1A = F_CPU/(2*8*freq) - 1
   1000 Hz -> OCR1A = 999
   ================================================================ */
static void tone_start(uint16_t freq) {
    uint32_t ocr = F_CPU / (2UL * 8UL * (uint32_t)freq) - 1UL;
    if (ocr > 65535UL) ocr = 65535UL;
    TCCR1A = (1u << COM1A0);
    TCCR1B = (1u << WGM12) | (1u << CS11);
    OCR1A  = (uint16_t)ocr;   /* POPRAWKA: bylo OCR5A */
    DDRB  |= (1u << PB5);
}

static void tone_stop(void) {
    TCCR1A = 0u;
    TCCR1B = 0u;
    PORTB &= ~(1u << PB5);
}

/* ================================================================
   HC-SR04 — asynchroniczny pomiar przez Timer5 Input Capture
   ICP5 = PL1 = Pin48
   Timer5 normal mode, prescaler 8 -> 0.5 us/tick
   Odleglosc [cm] = czas_trwania_tiki / 116
   ================================================================ */
static volatile uint16_t sr04_start       = 0u;
static volatile long     sr04_odleglosc   = 999L;
static volatile bool     sr04_w_trakcie   = false;

/**
 * @brief  Przerwanie Input Capture Timer5.
 *         Zbocze narastajace: zapisuje czas startu echa.
 *         Zbocze opadajace:   oblicza odleglosc i zatrzymuje timer.
 */
ISR(TIMER5_CAPT_vect) {
    uint16_t teraz = ICR5;

    if (TCCR5B & (1u << ICES5)) {
        /* zbocze narastajace — poczatek echa */
        sr04_start = teraz;
        TCCR5B &= ~(1u << ICES5);  /* przestaw na opadajace */
        TIFR5   = (1u << ICF5);    /* skasuj falszywa flage */
    } else {
        /* zbocze opadajace — koniec echa */
        uint16_t dt = teraz - sr04_start;
        long d = (long)dt / 116L;
        sr04_odleglosc = (d > 400L) ? 999L : d;

        TCCR5B  = 0u;
        TIMSK5 &= ~(1u << ICIE5);
        sr04_w_trakcie = false;
    }
}

/**
 * @brief  Wyzwala impuls TRIG i startuje Timer5 do asynchronicznego
 *         pomiaru. Zwraca wynik z poprzedniego cyklu (nieblokujace).
 * @return Ostatnio zmierzona odleglosc [cm] lub 999 gdy poza zasiegiem.
 */
static long sr04_measure(void) {
    if (sr04_w_trakcie) {
        /* timeout — poprzedni pomiar bez echa */
        TCCR5B  = 0u;
        TIMSK5 &= ~(1u << ICIE5);
        sr04_w_trakcie = false;
        sr04_odleglosc = 999L;
    }

    sr04_w_trakcie = true;

    PORTD |=  (1u << PD7);   /* TRIG HIGH */
    _delay_us(10);
    PORTD &= ~(1u << PD7);   /* TRIG LOW  */

    TCNT5  = 0u;
    TIFR5  = (1u << ICF5);
    TIMSK5 |= (1u << ICIE5);
    TCCR5B  = (1u << ICNC5) | (1u << ICES5) | (1u << CS51);

    return sr04_odleglosc;
}

/* ================================================================
   CZUJNIK HALLA — Pomiar RPM (Przerwanie INT4)
   Czujnik: Tesla MH3SS2. Sygnal podłączony do PE4 = Pin 2
   ================================================================ */
static volatile uint16_t impulsy_halla = 0;
static uint32_t czas_rpm = 0;
static uint16_t aktualne_rpm = 0;

/**
 * @brief Przerwanie zewnętrzne INT4 (Pin 2 / PE4).
 * Wywoływane sprzętowo przy każdym zboczu opadającym (wykrycie magnesu).
 */
ISR(INT4_vect) {
    impulsy_halla++;
}

/**
 * @brief Inicjalizacja pinu i rejestrów przerwania zewnętrznego INT4.
 */
static void hall_init(void) {
    // 1. Konfiguracja pinu PE4 (Pin 2) jako wejście
    DDRE &= ~(1 << PE4);
    
    // 2. Włączenie wewnętrznego rezystora Pull-up
    // (Wymagane dla czujników typu Open-Collector jak MH3SS2)
    PORTE |= (1 << PE4);
    
    // 3. Konfiguracja wyzwalania przerwania (Rejestr EICRB dla INT7:4)
    // ISC41 = 1, ISC40 = 0 -> wyzwalanie na zboczu opadającym (Falling Edge)
    EICRB |= (1 << ISC41);
    EICRB &= ~(1 << ISC40);
    
    // 4. Wyczyszczenie ewentualnych starych flag przerwań
    EIFR = (1 << INTF4);
    
    // 5. Włączenie maski przerwania dla INT4 w rejestrze EIMSK
    EIMSK |= (1 << INT4);
}

/* ================================================================
   SILNIKI
   PA0=IN1 PA1=IN2  (silnik A, ENA=OCR3A)
   PA2=IN3 PA3=IN4  (silnik B, ENB=OCR4A)
   PA4=IN5 PA5=IN6  (silnik C, ENC=OCR4B)
   PA6=IN7 PA7=IN8  (silnik D, END=OCR4C)

   Lewa os:  A (przod-lewy) + C (tyl-lewy)
   Prawa os: B (przod-prawy) + D (tyl-prawy)

   k > 0 = przod, k < 0 = tyl, k = 0 = stop
   spd = wartosc PWM 0-255
   ================================================================ */
static uint8_t predkosc = 200u;

static void silnikA(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u << PA0); PORTA &= ~(1u << PA1); }
    else if (k < 0) { PORTA &= ~(1u << PA0); PORTA |=  (1u << PA1); }
    else            { PORTA &= ~((1u << PA0) | (1u << PA1)); }
    OCR3A = (k != 0) ? spd : 0u;
}

static void silnikB(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u << PA2); PORTA &= ~(1u << PA3); }
    else if (k < 0) { PORTA &= ~(1u << PA2); PORTA |=  (1u << PA3); }
    else            { PORTA &= ~((1u << PA2) | (1u << PA3)); }
    OCR4A = (k != 0) ? spd : 0u;
}

static void silnikC(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u << PA4); PORTA &= ~(1u << PA5); }
    else if (k < 0) { PORTA &= ~(1u << PA4); PORTA |=  (1u << PA5); }
    else            { PORTA &= ~((1u << PA4) | (1u << PA5)); }
    OCR4B = (k != 0) ? spd : 0u;
}

static void silnikD(int8_t k, uint8_t spd) {
    if      (k > 0) { PORTA |=  (1u << PA6); PORTA &= ~(1u << PA7); }
    else if (k < 0) { PORTA &= ~(1u << PA6); PORTA |=  (1u << PA7); }
    else            { PORTA &= ~((1u << PA6) | (1u << PA7)); }
    OCR4C = (k != 0) ? spd : 0u;
}

static void stop_all(void) {
    silnikA(0, 0u); silnikB(0, 0u); silnikC(0, 0u); silnikD(0, 0u);
}

/* ================================================================
   LEDY PCA9685
   kanal 0 = DRL lewy (bialy)
   kanal 1 = DRL prawy (bialy)
   kanal 2 = kierunkowskaz lewy (zolty)
   kanal 3 = kierunkowskaz prawy (zolty)
   ================================================================ */
#define KANAL_DRL_LEWY      0u
#define KANAL_DRL_PRAWY     1u
#define KANAL_KIERUNK_LEWY  2u
#define KANAL_KIERUNK_PRAWY 3u
#define JASNOSC_PELNA       4095u
#define JASNOSC_ZERO        0u
#define CZAS_MIG_MS         500u
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
    ustaw_kanal(KANAL_DRL_LEWY,  true);
    ustaw_kanal(KANAL_DRL_PRAWY, true);
    info("DRL: WLACZONE");
}

static void drl_wylacz(void) {
    drl_wlaczone = false;
    ustaw_kanal(KANAL_DRL_LEWY,  false);
    ustaw_kanal(KANAL_DRL_PRAWY, false);
    info("DRL: WYLACZONE");
}

static void kierunk_lewy_wlacz(void) {
    if (kierunk_prawy_wlaczony) { kierunk_prawy_wlaczony = false; ustaw_kanal(KANAL_KIERUNK_PRAWY, false); }
    awaryjne_wlaczone     = false;
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
    awaryjne_wlaczone      = false;
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
    ustaw_kanal(KANAL_KIERUNK_LEWY,  true);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
    info("AWARYJNE: WLACZONE");
}

static void awaryjne_wylacz(void) {
    awaryjne_wlaczone = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY,  false);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    info("AWARYJNE: WYLACZONE");
}

/**
 * @brief  Plynne pulsowanie kierunkowskazow i awaryjnych przez PWM PCA9685.
 *         Wywoływane co iteracje petli glownej; wewnetrznie ogranicza
 *         czestotliwosc zapisu I2C do co 20 ms (50 Hz).
 *         Jasnosc zmienia sie liniowo w gore i w dol w cyklu 1 s.
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

static long     odleglosc_cm   = 999L;
static long     prev_rpm       = 0xFFFF;

static char aktualny_kierunek[17] = "STOP";
static char prev_kierunek[17]     = "";
static char ostatni_ruch           = 'S';

/* ================================================================
   AKTUALIZACJA LCD (tylko przy zmianie wartosci)
   ================================================================ */
static void lcd_update(void) {
    if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
        lcd_goto(0u, 0u);
        lcd_puts("Kier: ");
        lcd_puts_pad(aktualny_kierunek);
        strcpy(prev_kierunek, aktualny_kierunek);
    }
    if (aktualne_rpm != prev_rpm) {
        lcd_goto(0, 1);
        lcd_puts("RPM: ");
        lcd_putn((int32_t)aktualne_rpm);
        lcd_puts("       "); // Puste spacje, aby nadpisać ewentualne stare, dłuższe liczby
        prev_rpm = aktualne_rpm;
}

/* ================================================================
   OBSLUGA KOMENDY
   ================================================================ */
static void handle_cmd(char c) {
    switch (c) {
        /* --- JAZDA --- */
        case 'F': case 'f':
            silnikA(-1, predkosc); silnikB(-1, predkosc);
            silnikC(-1, predkosc); silnikD(-1, predkosc);
            jedzie_przod = true;
            ostatni_ruch = 'F';
            strcpy(aktualny_kierunek, "PRZOD");
            info("PRZOD");
            break;

        case 'B': case 'b':
            silnikA(1, predkosc); silnikB(1, predkosc);
            silnikC(1, predkosc); silnikD(1, predkosc);
            jedzie_przod = false;
            ostatni_ruch = 'B';
            strcpy(aktualny_kierunek, "TYL");
            info("TYL");
            break;

        /*
         * SKRET LEWO — skid-steer pivot, predkosc maksymalna.
         * Lewa os (A+C) do tylu, prawa os (B+D) do przodu.
         * PREDKOSC_SKRET=255 zapewnia wystarczajacy moment
         * obrotowy do pokonania tarcia bocznego 4 kol TT
         * przy rozstawie 16x21 cm.
         */
        case 'L': case 'l':
            silnikA(-1, PREDKOSC_SKRET); silnikC(-1, PREDKOSC_SKRET);  /* lewa  do tylu  */
            silnikB( 1, PREDKOSC_SKRET); silnikD( 1, PREDKOSC_SKRET);  /* prawa do przodu */
            jedzie_przod = false;
            ostatni_ruch = 'L';
            strcpy(aktualny_kierunek, "LEWO");
            info("LEWO");
            break;

        /*
         * SKRET PRAWO — skid-steer pivot, predkosc maksymalna.
         * Lewa os (A+C) do przodu, prawa os (B+D) do tylu.
         */
        case 'R': case 'r':
            silnikA( 1, PREDKOSC_SKRET); silnikD( 1, PREDKOSC_SKRET);  /* lewa  do przodu */
            silnikB(-1, PREDKOSC_SKRET); silnikC(-1, PREDKOSC_SKRET);  /* prawa do tylu   */
            jedzie_przod = false;
            ostatni_ruch = 'R';
            strcpy(aktualny_kierunek, "PRAWO");
            info("PRAWO");
            break;

        case 'S': case 's':
            stop_all();
            jedzie_przod = false;
            ostatni_ruch = 'S';
            strcpy(aktualny_kierunek, "STOP");
            info("STOP");
            break;

        case '+':
            if (predkosc <= 235u) predkosc += 20u; else predkosc = 255u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            handle_cmd(ostatni_ruch);   /* zastosuj nowa predkosc natychmiast */
            break;

        case '-':
            if (predkosc >= 80u) predkosc -= 20u; else predkosc = 60u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            handle_cmd(ostatni_ruch);
            break;

        /* --- SWIATLA --- */
        case 'D': case 'd':
            if (drl_wlaczone)           drl_wylacz();           else drl_wlacz();           break;
        case 'Q': case 'q':
            if (kierunk_lewy_wlaczony)  kierunk_lewy_wylacz();  else kierunk_lewy_wlacz();  break;
        case 'E': case 'e':
            if (kierunk_prawy_wlaczony) kierunk_prawy_wylacz(); else kierunk_prawy_wlacz(); break;
        case 'H': case 'h':
            if (awaryjne_wlaczone)      awaryjne_wylacz();      else awaryjne_wlacz();      break;

        default:
            break;
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

        /* pomiar odleglosci co 150 ms */
        if ((teraz - czas_pomiaru) >= 150UL) {
            czas_pomiaru = teraz;
            odleglosc_cm = sr04_measure();
            lcd_update();
        }

        /* PDC — asystent parkowania */
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

        /* 5. Obliczanie RPM z czujnika Halla (co 1000 ms) */
        if (teraz - czas_rpm >= 1000) {
            czas_rpm = teraz;
            
            uint16_t kopia_impulsow;
            
            // BLOK OPERACJI ATOMOWEJ
            // Zabezpieczenie przed wywołaniem przerwania sprzętowego 
            // w trakcie kopiowania 16-bitowej zmiennej do rejestrów roboczych.
            uint8_t stary_sreg = SREG;
            cli();
            kopia_impulsow = impulsy_halla;
            impulsy_halla = 0; // zerujemy sprzętowy licznik na kolejną sekundę
            SREG = stary_sreg;
            
            // Fizyka zjawiska: Zakładając 1 magnes na osi koła, 1 impuls to 1 obrót.
            // W ciągu 1 sekundy wykonano 'kopia_impulsow' obrotów.
            // Mnożymy to razy 60, aby uzyskać RPM (obroty na minutę).
            aktualne_rpm = kopia_impulsow * 60;
            
            lcd_update();
        }

        /* miganie kierunkowskazow i awaryjnych */

        obsluz_miganie();

        /* komendy Bluetooth / USB */
        if (uart0_available()) { handle_cmd((char)uart0_read()); }
        if (uart1_available()) { handle_cmd((char)uart1_read()); }
    }

    return 0;
}