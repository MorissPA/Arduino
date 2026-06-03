/*
 * SAMOCHODZIK — Bare Metal ATmega2560
 * Bluetooth HC-05 + 2x L298N + LCD I2C + HC-SR04 + Buzzer + LED + PCA9685
 *
 * Piny:
 *   HC-05:    UART1 TX=Pin19(PD3) RX=Pin18(PD2)
 *   L298N #1: ENA=Pin5(PE3/OC3A) IN1=22(PA0) IN2=23(PA1)
 *             ENB=Pin6(PH3/OC4A) IN3=24(PA2) IN4=25(PA3)
 *   L298N #2: ENC=Pin7(PH4/OC4B) IN5=26(PA4) IN6=27(PA5)
 *             END=Pin8(PH5/OC4C) IN7=28(PA6) IN8=29(PA7)
 *   LCD I2C:  SDA=Pin20 SCL=Pin21, PCF8574T addr=0x27
 *   PCA9685:  ten sam I2C, addr=0x40
 *             kanal 0 = DRL lewy (bialy)
 *             kanal 1 = DRL prawy (bialy)
 *             kanal 2 = kierunkowskaz lewy (zolty)
 *             kanal 3 = kierunkowskaz prawy (zolty)
 *   HC-SR04:  TRIG=Pin38(PD7) ECHO=Pin40(PG1)
 *   Buzzer:   Pin44(PL5/OC5C)
 *   LED:      Pin46(PL3)
 *
 * Timery:
 *   Timer0: CTC 1ms -> millis()
 *   Timer1: pomiar HC-SR04 (chwilowy)
 *   Timer3: Fast PWM 8-bit -> ENA (Pin5)
 *   Timer4: Fast PWM 8-bit -> ENB(6) ENC(7) END(8)
 *   Timer5: CTC toggle -> ton buzzera (Pin44)
 *
 * Komendy Bluetooth:
 *   F/B/L/R/S = jazda przod/tyl/lewo/prawo/stop
 *   +/-       = szybciej/wolniej
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

/* ================================================================
   MILLIS — Timer0 CTC
   Prescaler 64, OCR0A=249 -> 250 tickow x 4us = 1ms
   TCCR0A: WGM01=1 (CTC)
   TCCR0B: CS01|CS00 (prescaler 64)
   ================================================================ */
volatile uint32_t ms_count = 0;

ISR(TIMER0_COMPA_vect) {
    ms_count++;
}

static void timer0_init(void) {
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
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
   UCSR1C: UCSZ11|UCSZ10 = format 8N1
   ================================================================ */
#define BAUD     9600UL
#define UBRR_VAL (F_CPU / (16UL * BAUD) - 1)

static void uart0_init(void) {
    UBRR0H = (uint8_t)(UBRR_VAL >> 8);
    UBRR0L = (uint8_t)(UBRR_VAL);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static bool    uart0_available(void) { return (bool)(UCSR0A & (1 << RXC0)); }
static uint8_t uart0_read(void)      { return UDR0; }
static void    uart0_putc(char c)    { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
static void    uart0_puts(const char *s) { while (*s) uart0_putc(*s++); }

static void uart1_init(void) {
    UBRR1H = (uint8_t)(UBRR_VAL >> 8);
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
    if (n < 0) { (port==0u ? uart0_putc : uart1_putc)('-'); n = -n; }
    if (n == 0) { (port==0u ? uart0_putc : uart1_putc)('0'); return; }
    while (n > 0) { buf[i++] = '0' + (n % 10); n /= 10; }
    while (i > 0) { i--; (port==0u ? uart0_putc : uart1_putc)(buf[i]); }
}

static void info(const char *s) {
    uart0_puts(s); uart0_puts("\r\n");
    uart1_puts(s); uart1_puts("\r\n");
}

/* ================================================================
   TWI (I2C) — 100 kHz
   TWBR = (F_CPU/SCL - 16) / (2 x prescaler) = 72
   TWSR = 0x00 (prescaler=1)
   Sekwencja: START -> SLA+W -> dane -> STOP
   ================================================================ */
#define I2C_TIMEOUT 10000u

static void twi_init(void) {
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

static bool twi_start(void) {
    uint16_t t = 0;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

static void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static bool twi_write_byte(uint8_t data) {
    uint16_t t = 0;
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) { if (++t > I2C_TIMEOUT) return false; }
    return true;
}

/* ================================================================
   PCF8574T + HD44780 LCD (tryb 4-bit przez I2C)
   Adres PCF8574T: 0x27 -> SLA+W = 0x4E
   Mapowanie: P0=RS P1=RW P2=EN P3=BL P4-P7=D4-D7
   HD44780: dane odbierane dwoma nibblami, zatrzask na zboczu
   opadajacym EN (datasheet HD44780 str.45-46)
   ================================================================ */
#define LCD_ADDR_W  (0x27u << 1)
#define LCD_RS      (1u << 0)
#define LCD_EN      (1u << 2)
#define LCD_BL      (1u << 3)

static void pcf_write(uint8_t b) {
    if (!twi_start()) return;
    twi_write_byte(LCD_ADDR_W);
    twi_write_byte(b);
    twi_stop();
}

static void lcd_strobe(uint8_t b) {
    pcf_write(b | LCD_EN);
    _delay_us(1);
    pcf_write(b & ~LCD_EN);
    _delay_us(50);
}

static void lcd_send(uint8_t byte, uint8_t rs) {
    uint8_t hi = (byte & 0xF0u)         | LCD_BL | rs;
    uint8_t lo = ((byte << 4u) & 0xF0u) | LCD_BL | rs;
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
    lcd_cmd(0x28u); _delay_us(50);  /* 4-bit, 2 linie, 5x8 */
    lcd_cmd(0x08u); _delay_us(50);  /* display OFF */
    lcd_cmd(0x01u); _delay_ms(2);   /* clear display */
    lcd_cmd(0x06u); _delay_us(50);  /* entry mode: kursor w prawo */
    lcd_cmd(0x0Cu); _delay_us(50);  /* display ON, kursor OFF */
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
    while (n > 0) { buf[i++] = '0' + (n % 10); n /= 10; }
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
   Adres I2C: 0x40 -> SLA+W = 0x80
   MODE1 (0x00): AI=1 (auto-increment), brak snu -> 0x21
   LED0_ON_L (0x06): rejestr bazowy, +4 na kanal
   Zapis 4 bajtow na kanal: ON_L ON_H OFF_L OFF_H
   ================================================================ */
#define PCA_ADDR_W  (0x40u << 1)
#define PCA_MODE1   0x00u
#define PCA_LED0    0x06u

static void pca9685_init(void) {
    if (!twi_start()) return;
    twi_write_byte(PCA_ADDR_W);
    twi_write_byte(PCA_MODE1);
    twi_write_byte(0x21u);  /* AI=1, wyjscie ze snu */
    twi_stop();
    _delay_ms(1);
}

static void pca_set(uint8_t ch, uint16_t on, uint16_t off) {
    if (!twi_start()) return;
    twi_write_byte(PCA_ADDR_W);
    twi_write_byte(PCA_LED0 + 4u * ch);
    twi_write_byte((uint8_t)(on  & 0xFFu));
    twi_write_byte((uint8_t)(on  >> 8u));
    twi_write_byte((uint8_t)(off & 0xFFu));
    twi_write_byte((uint8_t)(off >> 8u));
    twi_stop();
}

/* ================================================================
   PWM silnikow
   Timer3 Fast PWM 8-bit -> OC3A = ENA = Pin5 = PE3
   Timer4 Fast PWM 8-bit -> OC4A = ENB = Pin6 = PH3
                             OC4B = ENC = Pin7 = PH4
                             OC4C = END = Pin8 = PH5
   WGM = 0101 (Fast PWM 8-bit, TOP=0xFF)
   COM = 10 (non-inverting)
   Prescaler 8 -> f_PWM = 16MHz / (8x256) ~ 7.8kHz
   ================================================================ */
static void pwm_init(void) {
    /* Timer3: OC3A (ENA, Pin5, PE3) */
    TCCR3A = (1 << COM3A1) | (1 << WGM30);
    TCCR3B = (1 << WGM32)  | (1 << CS31);
    OCR3A  = 0;
    DDRE  |= (1 << PE3);

    /* Timer4: OC4A OC4B OC4C (ENB ENC END, Pins 6 7 8, PH3 PH4 PH5) */
    TCCR4A = (1 << COM4A1) | (1 << COM4B1) | (1 << COM4C1) | (1 << WGM40);
    TCCR4B = (1 << WGM42)  | (1 << CS41);
    OCR4A  = 0; OCR4B = 0; OCR4C = 0;
    DDRH  |= (1 << PH3) | (1 << PH4) | (1 << PH5);
}

/* ================================================================
   GPIO
   Silniki: PORTA (PA0-PA7) = Piny 22-29
   Buzzer:  PL5 = Pin44
   LED:     PL3 = Pin46
   SR04:    TRIG=PD7=Pin38, ECHO=PG1=Pin40
   ================================================================ */
static void gpio_init(void) {
    DDRA  = 0xFFu; PORTA = 0x00u;
    DDRL |= (1 << PL5) | (1 << PL3);
    PORTL &= ~((1 << PL5) | (1 << PL3));
    DDRD  |= (1 << PD7); PORTD &= ~(1 << PD7);
    DDRG  &= ~(1 << PG1);
}

/* ================================================================
   BUZZER TONE — Timer5 CTC, toggle OC5C = PL5 = Pin44
   WGM52=1 (CTC, TOP=OCR5A), COM5C0=1 (toggle)
   Prescaler 8: OCR5A = F_CPU/(2x8xfreq) - 1
   1000Hz -> OCR5A=999, 1500Hz -> OCR5A=666
   ================================================================ */
static void tone_start(uint16_t freq) {
    uint32_t ocr = F_CPU / (2UL * 8UL * (uint32_t)freq) - 1UL;
    if (ocr > 65535UL) ocr = 65535UL;
    TCCR5A = (1 << COM5C0);
    TCCR5B = (1 << WGM52) | (1 << CS51);
    OCR5A  = (uint16_t)ocr;
    DDRL  |= (1 << PL5);
}

static void tone_stop(void) {
    TCCR5A = 0; TCCR5B = 0;
    PORTL &= ~(1 << PL5);
}

/* ================================================================
   HC-SR04 — pomiar przez Timer1
   Timer1 normal mode, prescaler 8 -> 0.5 us/tick
   Odleglosc [cm] = TCNT1 * 0.5 / 58 = TCNT1 / 116
   ================================================================ */
static long sr04_measure(void) {
    PORTD |= (1 << PD7);
    _delay_us(10);
    PORTD &= ~(1 << PD7);

    uint32_t t = millis();
    while (!(PING & (1 << PG1))) {
        if ((millis() - t) > 30UL) return 999L;
    }

    TCCR1A = 0;
    TCCR1B = (1 << CS11);
    TCNT1  = 0;

    while (PING & (1 << PG1)) {
        if (TCNT1 > 50000u) { TCCR1B = 0; return 999L; }
    }

    uint16_t ticks = TCNT1;
    TCCR1B = 0;
    return (long)ticks / 116;
}

/* ================================================================
   SILNIKI
   PA0=IN1 PA1=IN2 PA2=IN3 PA3=IN4 PA4=IN5 PA5=IN6 PA6=IN7 PA7=IN8
   ================================================================ */
static uint8_t predkosc = 200u;

static void silnikA(int8_t k) {
    if      (k > 0) { PORTA |=  (1u<<PA0); PORTA &= ~(1u<<PA1); }
    else if (k < 0) { PORTA &= ~(1u<<PA0); PORTA |=  (1u<<PA1); }
    else            { PORTA &= ~((1u<<PA0)|(1u<<PA1)); }
    OCR3A = (k != 0) ? predkosc : 0u;
}
static void silnikB(int8_t k) {
    if      (k > 0) { PORTA |=  (1u<<PA2); PORTA &= ~(1u<<PA3); }
    else if (k < 0) { PORTA &= ~(1u<<PA2); PORTA |=  (1u<<PA3); }
    else            { PORTA &= ~((1u<<PA2)|(1u<<PA3)); }
    OCR4A = (k != 0) ? predkosc : 0u;
}
static void silnikC(int8_t k) {
    if      (k > 0) { PORTA |=  (1u<<PA4); PORTA &= ~(1u<<PA5); }
    else if (k < 0) { PORTA &= ~(1u<<PA4); PORTA |=  (1u<<PA5); }
    else            { PORTA &= ~((1u<<PA4)|(1u<<PA5)); }
    OCR4B = (k != 0) ? predkosc : 0u;
}
static void silnikD(int8_t k) {
    if      (k > 0) { PORTA |=  (1u<<PA6); PORTA &= ~(1u<<PA7); }
    else if (k < 0) { PORTA &= ~(1u<<PA6); PORTA |=  (1u<<PA7); }
    else            { PORTA &= ~((1u<<PA6)|(1u<<PA7)); }
    OCR4C = (k != 0) ? predkosc : 0u;
}
static void stop_all(void) { silnikA(0); silnikB(0); silnikC(0); silnikD(0); }

/* ================================================================
   LEDY PCA9685
   kanal 0 = DRL lewy (bialy)
   kanal 1 = DRL prawy (bialy)
   kanal 2 = kierunkowskaz lewy (zolty)
   kanal 3 = kierunkowskaz prawy (zolty)
   ================================================================ */
#define KANAL_DRL_LEWY     0u
#define KANAL_DRL_PRAWY    1u
#define KANAL_KIERUNK_LEWY  2u
#define KANAL_KIERUNK_PRAWY 3u
#define JASNOSC_PELNA      4095u
#define JASNOSC_ZERO       0u
#define CZAS_MIG_MS        500u

static bool     drl_wlaczone          = false;
static bool     kierunk_lewy_wlaczony = false;
static bool     kierunk_prawy_wlaczony = false;
static bool     awaryjne_wlaczone     = false;
static bool     mig_stan              = false;
static uint32_t czas_ostatniego_mig   = 0;

static void ustaw_kanal(uint8_t kanal, bool wlaczony) {
    if (wlaczony) {
        pca_set(kanal, 0u, JASNOSC_PELNA);
    } else {
        pca_set(kanal, 0u, JASNOSC_ZERO);
    }
}

static void drl_wlacz(void) {
    drl_wlaczone = true;
    ustaw_kanal(KANAL_DRL_LEWY, true);
    ustaw_kanal(KANAL_DRL_PRAWY, true);
    info("DRL: WLACZONE");
}

static void drl_wylacz(void) {
    drl_wlaczone = false;
    ustaw_kanal(KANAL_DRL_LEWY, false);
    ustaw_kanal(KANAL_DRL_PRAWY, false);
    info("DRL: WYLACZONE");
}

static void kierunk_lewy_wlacz(void) {
    if (kierunk_prawy_wlaczony) {
        kierunk_prawy_wlaczony = false;
        ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    }
    if (awaryjne_wlaczone) {
        awaryjne_wlaczone = false;
    }
    kierunk_lewy_wlaczony = true;
    mig_stan = true;
    czas_ostatniego_mig = millis();
    ustaw_kanal(KANAL_KIERUNK_LEWY, true);
    info("KIERUNK: LEWY");
}

static void kierunk_lewy_wylacz(void) {
    kierunk_lewy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false);
    info("KIERUNK: LEWY WYLACZONY");
}

static void kierunk_prawy_wlacz(void) {
    if (kierunk_lewy_wlaczony) {
        kierunk_lewy_wlaczony = false;
        ustaw_kanal(KANAL_KIERUNK_LEWY, false);
    }
    if (awaryjne_wlaczone) {
        awaryjne_wlaczone = false;
    }
    kierunk_prawy_wlaczony = true;
    mig_stan = true;
    czas_ostatniego_mig = millis();
    ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
    info("KIERUNK: PRAWY");
}

static void kierunk_prawy_wylacz(void) {
    kierunk_prawy_wlaczony = false;
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    info("KIERUNK: PRAWY WYLACZONY");
}

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
    mig_stan = true;
    czas_ostatniego_mig = millis();
    ustaw_kanal(KANAL_KIERUNK_LEWY, true);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, true);
    info("AWARYJNE: WLACZONE");
}

static void awaryjne_wylacz(void) {
    awaryjne_wlaczone = false;
    ustaw_kanal(KANAL_KIERUNK_LEWY, false);
    ustaw_kanal(KANAL_KIERUNK_PRAWY, false);
    info("AWARYJNE: WYLACZONE");
}

static void obsluz_miganie(void) {
    if (!kierunk_lewy_wlaczony && !kierunk_prawy_wlaczony && !awaryjne_wlaczone) {
        return;
    }
    uint32_t teraz = millis();
    if ((teraz - czas_ostatniego_mig) < CZAS_MIG_MS) {
        return;
    }
    czas_ostatniego_mig = teraz;
    mig_stan = !mig_stan;
    if (awaryjne_wlaczone) {
        ustaw_kanal(KANAL_KIERUNK_LEWY,  mig_stan);
        ustaw_kanal(KANAL_KIERUNK_PRAWY, mig_stan);
    } else {
        if (kierunk_lewy_wlaczony) {
            ustaw_kanal(KANAL_KIERUNK_LEWY, mig_stan);
        }
        if (kierunk_prawy_wlaczony) {
            ustaw_kanal(KANAL_KIERUNK_PRAWY, mig_stan);
        }
    }
}

/* ================================================================
   STAN GLOBALNY — jazda + czujnik
   ================================================================ */
static bool     jedzie_przod   = false;
static bool     za_blisko      = false;
static bool     led_stan       = false;
static bool     buzzer_aktywny = false;

static uint32_t czas_pomiaru   = 0;
static uint32_t czas_led       = 0;
static uint32_t czas_buzzer    = 0;

static long     odleglosc_cm   = 999L;
static long     prev_odleglosc = -1L;

static char aktualny_kierunek[17] = "STOP";
static char prev_kierunek[17]     = "";

/* ================================================================
   AKTUALIZACJA LCD (tylko przy zmianie)
   ================================================================ */
static void lcd_update(void) {
    if (strcmp(aktualny_kierunek, prev_kierunek) != 0) {
        lcd_goto(0, 0);
        lcd_puts("Kier: ");
        lcd_puts_pad(aktualny_kierunek);
        strcpy(prev_kierunek, aktualny_kierunek);
    }
    if (odleglosc_cm != prev_odleglosc) {
        lcd_goto(0, 1);
        if (odleglosc_cm >= 999L) {
            lcd_puts("Odl: poza zasieg");
        } else {
            lcd_puts("Odl: ");
            lcd_putn(odleglosc_cm);
            lcd_puts(" cm         ");
        }
        prev_odleglosc = odleglosc_cm;
    }
}

/* ================================================================
   OBSLUGA KOMENDY
   ================================================================ */
static void handle_cmd(char c) {
    switch (c) {
        /* --- JAZDA --- */
        case 'F': case 'f':
            if (!za_blisko) {
                silnikA(-1); silnikB(-1); silnikC(-1); silnikD(-1);
                jedzie_przod = true;
                strcpy(aktualny_kierunek, "PRZOD");
                info("PRZOD");
            } else { info("BLOKADA"); }
            break;
        case 'B': case 'b':
            silnikA(1); silnikB(1); silnikC(1); silnikD(1);
            jedzie_przod = false;
            strcpy(aktualny_kierunek, "TYL");
            info("TYL");
            break;
        case 'L': case 'l':
            silnikA(-1); silnikC(-1); silnikB(1); silnikD(1);
            jedzie_przod = false;
            strcpy(aktualny_kierunek, "LEWO");
            info("LEWO");
            break;
        case 'R': case 'r':
            silnikA(1); silnikC(1); silnikB(-1); silnikD(-1);
            jedzie_przod = false;
            strcpy(aktualny_kierunek, "PRAWO");
            info("PRAWO");
            break;
        case 'S': case 's':
            stop_all();
            jedzie_przod = false;
            strcpy(aktualny_kierunek, "STOP");
            info("STOP");
            break;
        case '+':
            if (predkosc <= 235u) predkosc += 20u; else predkosc = 255u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            break;
        case '-':
            if (predkosc >= 80u) predkosc -= 20u; else predkosc = 60u;
            uart0_puts("Predkosc: "); uart_putn(0, (int32_t)predkosc); uart0_puts("\r\n");
            uart1_puts("Predkosc: "); uart_putn(1, (int32_t)predkosc); uart1_puts("\r\n");
            break;

        /* --- SWIATLA --- */
        case 'D': case 'd':
            if (drl_wlaczone) { drl_wylacz(); } else { drl_wlacz(); }
            break;
        case 'Q': case 'q':
            if (kierunk_lewy_wlaczony) { kierunk_lewy_wylacz(); } else { kierunk_lewy_wlacz(); }
            break;
        case 'E': case 'e':
            if (kierunk_prawy_wlaczony) { kierunk_prawy_wylacz(); } else { kierunk_prawy_wlacz(); }
            break;
        case 'H': case 'h':
            if (awaryjne_wlaczone) { awaryjne_wylacz(); } else { awaryjne_wlacz(); }
            break;

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

    /* wyzeruj wszystkie kanaly PCA9685 */
    for (uint8_t i = 0u; i < 4u; i++) {
        pca_set(i, 0u, 0u);
    }

    /* ekran powitalny */
    lcd_goto(0, 0); lcd_puts("  SAMOCHODZIK   ");
    lcd_goto(0, 1); lcd_puts("   GOTOWY :)    ");
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
            za_blisko    = (odleglosc_cm < 20L);

            if (jedzie_przod && za_blisko) {
                stop_all();
                jedzie_przod = false;
                strcpy(aktualny_kierunek, "AUTO-STOP");
                tone_start(1000u);
                buzzer_aktywny = true;
                czas_buzzer    = teraz;
                info("AUTO-STOP!");
            }
            lcd_update();
        }

        /* wylacz buzzer po 300ms */
        if (buzzer_aktywny && ((teraz - czas_buzzer) >= 300UL)) {
            tone_stop();
            buzzer_aktywny = false;
        }

        /* mruganie LED (pin 46) + buzzer co 200ms gdy za blisko */
        if ((teraz - czas_led) >= 200UL) {
            czas_led = teraz;
            if (za_blisko) {
                led_stan = !led_stan;
                if (led_stan) { PORTL |=  (1u << PL3); }
                else          { PORTL &= ~(1u << PL3); }
                if (led_stan && !buzzer_aktywny) {
                    tone_start(1500u);
                    buzzer_aktywny = true;
                    czas_buzzer    = teraz;
                }
            } else {
                PORTL &= ~(1u << PL3);
                led_stan = false;
            }
        }

        /* miganie kierunkowskazow i awaryjnych */
        obsluz_miganie();

        /* komendy */
        if (uart0_available()) { handle_cmd((char)uart0_read()); }
        if (uart1_available()) { handle_cmd((char)uart1_read()); }
    }

    return 0;
}