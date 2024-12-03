#include <avr/io.h>
#include <util/delay.h>
#define MYDELAY 3000

int main(void){
    DDRB = 1<<PB5;
    
    while(1){
        PORTB = 1<<PB5;
        _delay_ms(MYDELAY);
        PORTB &= ~ (1<<PB5); //Added & on Midterm.
        _delay_ms(MYDELAY);
    }

    return 0;
}