/* 

This Project is the very first for ECE 231 in Spring 2023. 

Basically, if I press P3, then P6 is high. If I press P4, then P7 is high. 
Finally, if I press P5, then both P6 and P7 are high. 

The multiple AND statements in my IF statements are there to prevent 
any lights from shining if more than one button is pressed. 

*/

#include <avr/io.h>
#include <util/delay.h>

#define MYDELAY 200

int main(void){
    DDRD = 1<<PD2|1<<PD6|1<<PD7; // Set D6, D7 as the outputs #### Try D2 as well
    
    PORTD = 0x23; // 0x means hexidecimal number. 2 means 0010 for PD2. C means 0011 for PD6 and PD7

    //PORTD = 1<<PORTD5|1<<PORTD3|1<<PORTD4; // Set Pullup on D3 and D4. #### Try D5 as well

    PORTD = 0x1C; // 0x means hexidecimal number. 1 means 0001 for PORTD3. C means 1100 for PORTD4 and PORTD5
    
    while(1){
       
        if ( ((PIND & (1<<PIND3)) == 0) & 
        !((PIND & (1<<PIND4)) == 0) & 
        !((PIND & (1<<PIND5)) == 0) ) {

            PORTD |= (1<<PORTD6); // P6 is high when P3 is pressed
            _delay_ms(MYDELAY);
            PORTD &= ~ (1<<PORTD6); // P6 becomes low. 
        }
        else {
            PORTD &= ~(1<<PORTD6); // P6 becomes low indefinitely
        }


        if ( ((PIND & (1<<PIND4)) == 0) & 
        !((PIND & (1<<PIND3)) == 0) & 
        !((PIND & (1<<PIND5)) == 0)   ) {

            PORTD |= (1<<PORTD7); // P7 is high when P4 is pressed
            _delay_ms(MYDELAY);
            PORTD &= ~ (1<<PORTD7); // P7 becomes low.
        }
        else {
            PORTD &= ~(1<<PORTD7); // P7 becomes low indefinitely
        }


        if ( ((PIND & (1<<PIND5)) == 0) & 
        !((PIND & (1<<PIND3)) == 0) 
        & !((PIND & (1<<PIND4)) == 0)  )  {

            PORTD |= (1<<PORTD7); // P7 is high when P4 is pressed
            PORTD |= (1<<PORTD6); // P6 is high when P3 is pressed
            _delay_ms(MYDELAY);
            PORTD &= ~ (1<<PORTD7); // P7 becomes low
            PORTD &= ~ (1<<PORTD6); // P6 becomes low

        }
        else {
            PORTD &= ~(1<<PORTD7); // P7 becomes low indefinitely
            PORTD &= ~ (1<<PORTD6); // P6 becomes low indefinitely
        }
    }
    return 0;
}


