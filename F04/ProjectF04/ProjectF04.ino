#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>

/* --- Definicje dla UART --- */
#define F_CPU 16000000UL
#define BAUD 9600
#define MY_UBRR (F_CPU/16/BAUD-1)

/* --- Definicje dla PCA9685 (I2C) --- */
#define PCA9685_ADDR      0x40  // Domyślny 7-bitowy adres I2C modułu PCA9685
#define PCA9685_MODE1     0x00  // Rejestr konfiguracyjny 1
#define PCA9685_LED0_ON_L 0x06  // Rejestr bazowy dla pierwszego kanału (Kanał 0)

/* Zabezpieczenie FMEA - limit pętli, aby uniknąć nieskończonego zawieszenia I2C */
#define I2C_TIMEOUT       10000 

// =====================================================================
//                       OBSŁUGA I2C (TWI)
// =====================================================================

/*!
 * @brief    Inicjalizuje sprzętowy interfejs TWI (I2C).
 * @side effects: Ustawia prędkość zegara SCL na 100 kHz. Włącza moduł TWI.
 */
void twi_init(void) {
    /* Ustawienie preskalera na 1 (TWPS1=0, TWPS0=0) */
    TWSR = 0x00; 
    
    /* Obliczenie wartości dla 100 kHz: SCL = F_CPU / (16 + 2*TWBR*Prescaler) 
       16000000 / (16 + 2*72*1) = 100000 Hz */
    TWBR = 72;   
    
    /* Włączenie interfejsu TWI */
    TWCR = (1 << TWEN); 
}

/*!
 * @brief    Wysyła warunek START na magistralę I2C. Zwraca false w razie błędu.
 */
bool twi_start(void) {
    uint16_t timeout = 0;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    /* Czekaj na ustawienie flagi TWINT (zakończenie operacji) lub przerwij po czasie */
    while (!(TWCR & (1 << TWINT))) {
        if (++timeout > I2C_TIMEOUT) return false;
    }
    return true;
}

/*!
 * @brief    Wysyła warunek STOP na magistralę I2C.
 */
void twi_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

/*!
 * @brief    Wysyła jeden bajt danych przez interfejs TWI. Zwraca false w razie błędu.
 * @param data Bajt do wysłania.
 */
bool twi_write(uint8_t data) {
    uint16_t timeout = 0;
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    /* Czekaj na potwierdzenie wysłania lub przerwij po czasie */
    while (!(TWCR & (1 << TWINT))) {
        if (++timeout > I2C_TIMEOUT) return false;
    }
    return true;
}


// =====================================================================
//                       OBSŁUGA PCA9685
// =====================================================================

/*!
 * @brief    Wybudza i inicjalizuje kontroler PCA9685.
 */
void pca9685_init(void) {
    if (!twi_start()) return; // Zabezpieczenie FMEA
    
    /* Wysłanie adresu z bitem zapisu (SLA+W). Adres przesunięty w lewo o 1 bit. */
    twi_write((PCA9685_ADDR << 1) | 0); 
    
    twi_write(PCA9685_MODE1); // Wybór rejestru MODE1
    twi_write(0x21);          // AI=1 (Auto-Increment włączony), wybudzenie ze snu
    twi_stop();
}

/*!
 * @brief    Ustawia sygnał PWM dla konkretnego kanału.
 * @param channel Numer kanału (0 do 15)
 * @param on Czas włączenia w cyklu 0-4095
 * @param off Czas wyłączenia w cyklu 0-4095
 */
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off) {
    if (!twi_start()) return; // Zabezpieczenie FMEA
    
    twi_write((PCA9685_ADDR << 1) | 0);
    
    /* Dzięki Auto-Increment, zapisujemy 4 kolejne rejestry za jednym zamachem */
    twi_write(PCA9685_LED0_ON_L + 4 * channel); 
    twi_write(on & 0xFF);        // ON_L
    twi_write(on >> 8);          // ON_H
    twi_write(off & 0xFF);       // OFF_L
    twi_write(off >> 8);         // OFF_H
    
    twi_stop();
}


// =====================================================================
//                       OBSŁUGA UART (Bluetooth)
// =====================================================================

void init_uart_bluetooth(uint16_t baudrate_register_value) {
    UBRR1H = (uint8_t)(baudrate_register_value >> 8);
    UBRR1L = (uint8_t)baudrate_register_value;
    UCSR1B = (1 << RXEN1) | (1 << TXEN1);
    UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
}

bool uart_data_available(void) {
    return (UCSR1A & (1 << RXC1));
}

uint8_t uart_receive_char(void) {
    return UDR1;
}

void uart_transmit_char(uint8_t data) {
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = data;
}

void uart_transmit_string(const char* str) {
    uint16_t i = 0;
    while (str[i] != '\0') {
        uart_transmit_char((uint8_t)str[i]);
        i++;
    }
}


// =====================================================================
//                       GŁÓWNY PROGRAM
// =====================================================================

int main(void) {
    /* Inicjalizacja sprzętowa na poziomie rejestrów (dawne setup) */
    init_uart_bluetooth(MY_UBRR);
    twi_init();       // Inicjalizacja I2C (TWI)
    pca9685_init();   // Inicjalizacja modułu LED

    uint8_t command_received = 0;

    /* Pętla nieskończona programu */
    while (1) {
        
        /* Metoda odpytywania (Polling) */
        if (uart_data_available()) {
            
            command_received = uart_receive_char();

            /* Analiza komendy i sterowanie modułem PCA9685 (Kanał 0) */
            if (command_received == '1') {
                /* Włącz diodę na maxa: ON na takcie 0, OFF na takcie 4095 */
                pca9685_set_pwm(0, 0, 4095); 
                uart_transmit_string("Dioda PCA9685: WLACZONA\r\n");
            } 
            else if (command_received == '0') {
                /* Wyłącz diodę: ON na takcie 0, OFF na takcie 0 */
                pca9685_set_pwm(0, 0, 0); 
                uart_transmit_string("Dioda PCA9685: WYLACZONA\r\n");
            }
            else {
                uart_transmit_string("Nierozpoznana komenda!\r\n");
            }
        }
        
    }

    return 0; 
}