#include <avr/io.h>

int main(void) {
    // 1. Ustawienie Pinu 11 (PB5) jako wyjście
    DDRB |= (1 << PB5);

    // 2. Podanie stałego stanu wysokiego (5V)
    PORTB |= (1 << PB5);

    // 3. Zatrzymanie procesora w nieskończonej pętli
    while (1) {
        // Nic tu nie robimy, pin caly czas ma 5V
    }

    return 0;
}