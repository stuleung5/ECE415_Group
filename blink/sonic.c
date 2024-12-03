/*ULTRASONIC DISTANCE SENSOR: 
Bryan Leung
4/7/23
ECE 231

In this lab, for hardware, I looked at Slides L12 and L13. TRIG originates
in PB1(Port B) while ECHO should be in PB0(Port B). 

For SCL, this should go to PC0(Port C) while SDA should go to PC1(Port C).
For sound sensor, use 5 V while for OLED display, use 3.3 V. 

For software, I copied and pasted the OLED libraries. To get pulse width, I 
caclulated the time TCNT0 was 0. Then, I calculated the time TCNTO was 1. Then, 
I used the OLED to calculate difference falling and rising. Finally, I divided
by 2.54 to get inch results.  

*/



#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>  

#define TRIG PB1 //PB1 is in PORTB: A0
#define ECHO PB0 //PB1 is in PORTB: A1
uint8_t OledLineNum, OledCursorPos;

void timer0_init(void);
//void uart_init(void);
//void uart_send(char letter);
void send_string(char *stringAddress);
void send_to_monitor(unsigned char, unsigned char, float);

void oledSendCommand(uint8_t cmd);
void oledSendStart(uint8_t address);
void oledSendStop(void);
void oledWaitForAck(void);
void oledSendByte(uint8_t ch);
#ifndef i2c_h
#define i2c_h

#define F_I2C 25000L
#define TRANSMISSION_SUCCESS -1
#define TRANSMISSION_ERROR -2
#define BUS_CONNECTED -3
#define BUS_DISCONNECTED -4
#define MASTER_TRANSMITTER 0
#define MASTER_RECEIVER 1
#define ACK 0
#define TIMEOUT 50

typedef uint8_t bool;
void i2c_init(void);
uint8_t i2c_tx_start(bool mode);
uint8_t i2c_tx_address(uint8_t address);
bool i2c_timeout(void);
uint8_t i2c_rx_byte(bool acknack);
void i2c_tx_stop(void);

#define FONT_SIZE 5
const unsigned char OledFontTable[][FONT_SIZE]={
    
    {0x00, 0x00, 0x00, 0x00, 0x00},   // space
    {0x00, 0x00, 0x2f, 0x00, 0x00},   // !
    {0x00, 0x07, 0x00, 0x07, 0x00},   // "
    {0x14, 0x7f, 0x14, 0x7f, 0x14},   // #
    {0x24, 0x2a, 0x7f, 0x2a, 0x12},   // $
    {0x23, 0x13, 0x08, 0x64, 0x62},   // %
    {0x36, 0x49, 0x55, 0x22, 0x50},   // &
    {0x00, 0x05, 0x03, 0x00, 0x00},   // '
    {0x00, 0x1c, 0x22, 0x41, 0x00},   // (
    {0x00, 0x41, 0x22, 0x1c, 0x00},   // )
    {0x14, 0x08, 0x3E, 0x08, 0x14},   // *
    {0x08, 0x08, 0x3E, 0x08, 0x08},   // +
    {0x00, 0x00, 0xA0, 0x60, 0x00},   // ,
    {0x08, 0x08, 0x08, 0x08, 0x08},   // -
    {0x00, 0x60, 0x60, 0x00, 0x00},   // .
    {0x20, 0x10, 0x08, 0x04, 0x02},   // /
    
    {0x3E, 0x51, 0x49, 0x45, 0x3E},   // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},   // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},   // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},   // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},   // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},   // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},   // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},   // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},   // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},   // 9
    
    {0x00, 0x36, 0x36, 0x00, 0x00},   // :
    {0x00, 0x56, 0x36, 0x00, 0x00},   // ;
    {0x08, 0x14, 0x22, 0x41, 0x00},   // <
    {0x14, 0x14, 0x14, 0x14, 0x14},   // =
    {0x00, 0x41, 0x22, 0x14, 0x08},   // >
    {0x02, 0x01, 0x51, 0x09, 0x06},   // ?
    {0x32, 0x49, 0x59, 0x51, 0x3E},   // @
    
    {0x7C, 0x12, 0x11, 0x12, 0x7C},   // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},   // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},   // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},   // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},   // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},   // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A},   // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},   // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},   // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},   // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},   // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},   // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F},   // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},   // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},   // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},   // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},   // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},   // R
    {0x46, 0x49, 0x49, 0x49, 0x31},   // S
    {0x01, 0x01, 0x7F, 0x01, 0x01},   // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},   // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},   // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F},   // W
    {0x63, 0x14, 0x08, 0x14, 0x63},   // X
    {0x07, 0x08, 0x70, 0x08, 0x07},   // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},   // Z
    
    {0x00, 0x7F, 0x41, 0x41, 0x00},   // [
    {0x55, 0xAA, 0x55, 0xAA, 0x55},   // Backslash (Checker pattern)
    {0x00, 0x41, 0x41, 0x7F, 0x00},   // ]
    {0x04, 0x02, 0x01, 0x02, 0x04},   // ^
    {0x40, 0x40, 0x40, 0x40, 0x40},   // _
    {0x00, 0x03, 0x05, 0x00, 0x00},   // `
    
    {0x20, 0x54, 0x54, 0x54, 0x78},   // a
    {0x7F, 0x48, 0x44, 0x44, 0x38},   // b
    {0x38, 0x44, 0x44, 0x44, 0x20},   // c
    {0x38, 0x44, 0x44, 0x48, 0x7F},   // d
    {0x38, 0x54, 0x54, 0x54, 0x18},   // e
    {0x08, 0x7E, 0x09, 0x01, 0x02},   // f
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C},   // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},   // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},   // i
    {0x40, 0x80, 0x84, 0x7D, 0x00},   // j
    {0x7F, 0x10, 0x28, 0x44, 0x00},   // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},   // l
    {0x7C, 0x04, 0x18, 0x04, 0x78},   // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},   // n
    {0x38, 0x44, 0x44, 0x44, 0x38},   // o
    {0xFC, 0x24, 0x24, 0x24, 0x18},   // p
    {0x18, 0x24, 0x24, 0x18, 0xFC},   // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},   // r
    {0x48, 0x54, 0x54, 0x54, 0x20},   // s
    {0x04, 0x3F, 0x44, 0x40, 0x20},   // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},   // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},   // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},   // w
    {0x44, 0x28, 0x10, 0x28, 0x44},   // x
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C},   // y
    {0x44, 0x64, 0x54, 0x4C, 0x44},   // z
    
    {0x00, 0x10, 0x7C, 0x82, 0x00},   // {
    {0x00, 0x00, 0xFF, 0x00, 0x00},   // |
    {0x00, 0x82, 0x7C, 0x10, 0x00},   // }
    {0x00, 0x06, 0x09, 0x09, 0x06}    // ~ (Degrees)
};

#ifndef SSD1306_h
#define SSD1306_h

/***************************************************************************************************
 Macros to find the mod of a number
 ****************************************************************************************************/
#define util_GetMod8(dividend, divisor) (uint8_t)(dividend - (divisor * (uint8_t)(dividend / divisor)))
#define util_GetMod16(dividend, divisor) (uint16_t)(dividend - (divisor * (uint16_t)(dividend / divisor)))
#define util_GetMod32(dividend, divisor) (uint32_t)(dividend - (divisor * (uint32_t)(dividend / divisor)))
/***************************************************************************************************/

/***************************************************************************************************
 Macros for Dec2Ascii,Hec2Ascii and Acsii2Hex conversion
 *****************************************************************************************************/
#define util_Dec2Ascii(Dec) ((Dec) + 0x30)
#define util_Ascii2Dec(Asc) ((Asc)-0x30)
#define util_Hex2Ascii(Hex) (((Hex) > 0x09) ? ((Hex) + 0x37) : ((Hex) + 0x30))
#define util_Ascii2Hex(Asc) (((Asc) > 0x39) ? ((Asc)-0x37) : ((Asc)-0x30))
#define util_GetBitStatus(x, bit) (((x) & (util_GetBitMask(bit))) != 0u)
#define util_GetBitMask(bit) (1 << (bit))
/***************************************************************************************************/

/******************************************************************************
 standard defs
 ******************************************************************************/

typedef signed char sint8_t;
typedef unsigned char uint8_t;
typedef signed int sint16_t;
typedef signed long int sint32_t;

#define C_BINARY_U8 2u
#define C_DECIMAL_U8 10u
#define C_HEX_U8 16u

#define C_SINT8_MAX 0x7F
#define C_SINT8_MIN -128

#define C_UINT8_MAX 0xFFu
#define C_UINT8_MIN 0x00u

#define C_SINT16_MAX 32767
#define C_SINT16_MIN -32768

#define C_UINT16_MAX 0xFFFFu
#define C_UINT16_MIN 0x00u

#define C_SINT32_MAX 2147483647
#define C_SINT32_MIN -2147483648

#define C_UINT32_MAX 0xFFFFFFFFu
#define C_UINT32_MIN 0x00u

typedef enum
{
    E_BINARY = 2,
    E_DECIMAL = 10,
    E_HEX = 16
} NumericSystem_et;

/******************************************************************************
 SSD1306 ID and Command List
 ******************************************************************************/
#define SSD1306_ADDRESS 0x3C

#define SSD1306_COMMAND 0x00
#define SSD1306_DATA 0xC0
#define SSD1306_DATA_CONTINUE 0x40

#define SSD1306_SET_CONTRAST_CONTROL 0x81
#define SSD1306_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_DISPLAY_ALL_ON 0xA5
#define SSD1306_NORMAL_DISPLAY 0xA6
#define SSD1306_INVERT_DISPLAY 0xA7
#define SSD1306_DISPLAY_OFF 0xAE
#define SSD1306_DISPLAY_ON 0xAF
#define SSD1306_NOP 0xE3

#define SSD1306_HORIZONTAL_SCROLL_RIGHT 0x26
#define SSD1306_HORIZONTAL_SCROLL_LEFT 0x27
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_RIGHT 0x29
#define SSD1306_HORIZONTAL_SCROLL_VERTICAL_AND_LEFT 0x2A
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3

#define SSD1306_SET_LOWER_COLUMN 0x00
#define SSD1306_SET_HIGHER_COLUMN 0x10
#define SSD1306_MEMORY_ADDR_MODE 0x20
#define SSD1306_SET_COLUMN_ADDR 0x21
#define SSD1306_SET_PAGE_ADDR 0x22

#define SSD1306_SET_START_LINE 0x40
#define SSD1306_SET_SEGMENT_REMAP 0xA0
#define SSD1306_SET_MULTIPLEX_RATIO 0xA8
#define SSD1306_COM_SCAN_DIR_INC 0xC0
#define SSD1306_COM_SCAN_DIR_DEC 0xC8
#define SSD1306_SET_DISPLAY_OFFSET 0xD3
#define SSD1306_SET_COM_PINS 0xDA
#define SSD1306_CHARGE_PUMP 0x8D

#define SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO 0xD5
#define SSD1306_SET_PRECHARGE_PERIOD 0xD9
#define SSD1306_SET_VCOM_DESELECT 0xDB
/****************************************************************************/

/***************************************************************************************************
 Commonly used LCD macros/Constants
 ***************************************************************************************************/
#define C_DefaultDigits 10

#define C_OledFirstLine_U8 0x00u
#define C_OledLastLine_U8 0x07u

#define C_FirstLineAddress_U8 0xB8
#define C_LastLineAddress_U8 0xBF

#define C_DisplayDefaultDigits_U8 0xffu                              // Will display the exact digits in the number
#define C_MaxDigitsToDisplay_U8 10u                                  // Max decimal/hexadecimal digits to be displayed
#define C_NumOfBinDigitsToDisplay_U8 16u                             // Max bits of a binary number to be displayed
#define C_MaxDigitsToDisplayUsingPrintf_U8 C_DisplayDefaultDigits_U8 /* Max dec/hexadecimal digits to be displayed using printf */

#define C_MaxBarGraphs_U8 5
/**************************************************************************************************/

/***************************************************************************************************
 PreCompile configurations to enable/disable the functions
 ****************************************************************************************************
 PreCompile configuration to enable or disable the API's.
 1.Required interfaces can be enabled/disabled by configuring its respective macros to 1/0.
 2. By default all the API's are disabled.
 3. Displaying of floating number takes huge controller resources and need to be enabled only
 if required. This implies for other interfaces as well.
 ****************************************************************************************************/
#define Enable_OLED_DisplayString 1
#define Enable_OLED_ScrollMessage 1
#define Enable_OLED_DisplayNumber 1
#define Enable_OLED_DisplayFloatNumber 1
#define Enable_OLED_Printf 1
#define Enable_OLED_DisplayLogo 1
#define Enable_OLED_EnableInversion 1
#define Enable_OLED_DisableInversion 1
#define ENABLE_OLED_VerticalGraph 1
#define ENABLE_OLED_HorizontalGraph 1
#define Enable_OLED_SetBrightness 1
#define Enable_OLED_GoToLine 1
/**************************************************************************************************/

/***************************************************************************************************
 Function Prototypes
 ***************************************************************************************************/
void OLED_Init(void);
void OLED_DisplayChar(uint8_t ch);
void OLED_DisplayString(char *);   // changed from uint8_t to char to clear warning DM 4/4/22
void OLED_ScrollMessage(uint8_t lineNum, char *strptr);
void OLED_DisplayNumber(uint8_t v_numericSystem_u8, uint32_t v_number_u32, uint8_t v_numOfDigitsToDisplay_u8);
void OLED_DisplayFloatNumber(double v_floatNum_f32);
void OLED_Printf(const char *argList, ...);
void OLED_Clear(void);
void OLED_GoToPage(uint8_t);
void OLED_GoToLine(uint8_t);
void OLED_GoToNextLine(void);
void OLED_SetCursor(uint8_t lineNumber, uint8_t CursorPosition);
void OLED_DisplayLogo(char *ptr_Logo);
void OLED_EnableInversion(void);
void OLED_DisableInversion(void);
void OLED_VerticalGraph(uint8_t var_barGraphNumber_u8, uint8_t var_percentageValue_u8);
void OLED_HorizontalGraph(uint8_t var_barGraphNumber_u8, uint8_t var_percentageValue_u8);
/**************************************************************************************************/

#endif /* SSD1306_h */

int main(void) {
    
    unsigned char Rising;
    unsigned char Falling;
    unsigned char pulseWidth;
    float range;

    DDRB = 1<<TRIG;
    PORTB &= ~(1<<TRIG);
    timer0_init();
    OLED_Init();

    while (1) {
        
        TCNT0 = 0;
        PORTB |= 1<<TRIG;
        _delay_us(10);
        PORTB &= ~(1<<TRIG);

        while ((PINB & (1<<ECHO)) == 0) {
            Rising = TCNT0;
        }
        
        while (!(PINB & (1<<ECHO)) == 0) {
            Falling = TCNT0;
        }
        
        if (Falling > Rising) {
            pulseWidth = Falling - Rising; 
            range = pulseWidth * 1.098; //one way distance to target in cm
            send_to_monitor(Rising, Falling, range);
        }

        OLED_GoToLine(3);
        OLED_DisplayString("Dist. (in cm): ");

        OLED_DisplayNumber(10,Falling-Rising,3);
        OLED_DisplayString(" cm");

        OLED_GoToLine(4);
        OLED_DisplayString("Dist. (in in): ");

        OLED_DisplayNumber(10,Falling/2.54 - Rising/2.54,2);
        OLED_DisplayString(" in");

        
        
        _delay_ms(5);

    }
    return 0;
}
/// Written by David McLaughlin. 

void timer0_init(void){
	TCCR0A = 0; //timer mode - normal 
	TCCR0B = 5; //1024 prescaler
    TCNT0 = 0;
}

void send_to_monitor(unsigned char t1, unsigned char t2, float range) {
    char buffer[10];
    send_string("Rising Edge: ");
    utoa(t1, buffer, 10);

    send_string("Falling Edge: ");
    utoa(t2, buffer, 10);
    send_string(buffer);

    send_string("Echo Pulse Width: ");
    utoa(t2-t1, buffer, 10);
    send_string(buffer);

    send_string(" Echo pulse width: ");
    utoa(t2-t1, buffer, 10);
    send_string(buffer);
    send_string(" clock pulses.    Target Range= ");
    
    dtostrf(range/2.54, 3,0,buffer);
    send_string(buffer);
    send_string(" inch");
    uart_send(13);
    uart_send(10);
}

void send_string(char *stringAddress){
    unsigned char i;
    for (i = 0; i < strlen(stringAddress); i++) {
        uart_send(stringAddress[i]);
    }
        
}

/*void uart_init(void){
    UCSR0B = (1 << TXEN0); //enable the UART transmitter
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); //set 8 bit character size
    UBRR0L = 103; //set baud rate to 9600 for 16 MHz crystal
}

void uart_send(char letter){
    while (!(UCSR0A & (1 << UDRE0))) {
        UDR0 = letter; //write the character to the USART data register
    } //wait til tx data buffer empty
    
}*/

static bool masterMode;

// MODS TO THIS CODE BY DM FOR 16MHz CLOCK 4/4/22
void i2c_init(void)
{
    //TWSR &= ~((1 << TWPS1) | (1 << TWPS0)); // pre-scalar 1
    TWSR |=  (1<< TWPS1);  // This code changed by DM 4/4/22
    TWSR &=  ~(1<< TWPS0); // pre-scaler = 16 
   // TWBR = ((F_CPU / F_I2C) - 16) / 2; // baud rate factor 12
     TWBR = ((F_CPU / F_I2C) - 16) /32; // baud rate factor 19.5 DM 4/4/22
}

uint8_t i2c_tx_start(bool mode)
{
    int8_t status = 0;
    masterMode = mode; // set global state of R/W bit

    TWCR |=  (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT))) {
        switch (TWSR & 0xF8) {
            /* start condition sent from master */
            case 0x08:
                status = TRANSMISSION_SUCCESS;
                break;

            /* repeat start condition sent from master */
            case 0x10:
                status = TRANSMISSION_SUCCESS;
                break;

            default:
                status = TRANSMISSION_ERROR;
                break;
        }
    }

    return status;
}

uint8_t i2c_tx_address(uint8_t address)
{
    int8_t status = 0;

    TWDR = (address << 1) | masterMode;
    /* clear start command to release bus as master */
    TWCR &= ~(1 << TWSTA);
    /* clear interrupt flag */
    TWCR |=  (1 << TWINT);

    /* wait until address transmitted */
    while (!(TWCR & (1 << TWINT)));

    if (masterMode == MASTER_TRANSMITTER) {
        switch (TWSR & 0xF8) {
            /* address|write sent and ACK returned */
            case 0x18:
                status = TRANSMISSION_SUCCESS;
                break;

           /* address|write sent and NACK returned slave */
           case 0x20:
                status = TRANSMISSION_ERROR;
                break;

            /* address|write sent and bus failure detected */
            case 0x38:
                status = TRANSMISSION_ERROR;
                break;

            default:
                status = TRANSMISSION_ERROR;
                break;
        }
    } else if (masterMode == MASTER_RECEIVER) {
        switch (TWSR & 0xF8) {
            /* address|read sent and ACK returned */
            case 0x40:
                status = TRANSMISSION_SUCCESS;
                break;

            /* address|read sent and NACK returned */
            case 0x48:
                status = TRANSMISSION_ERROR;
                break;

            case 0x38:
                status = TRANSMISSION_ERROR;
                break;

            default:
                status = TRANSMISSION_ERROR;
                break;
        }
    }
    return status;
}

uint8_t i2c_tx_byte(uint8_t byteData)
{
    int8_t status = 0;
    TWDR  = byteData; // load data buffer with data to be transmitted
    TWCR |= (1 << TWINT); // clear interrupt flag

    /* wait until data transmitted */
    while (!(TWCR & (1 << TWINT)));

    /* retrieve transmission status codes */
    switch (TWSR & 0xF8) {
        /* byte sent and ACK returned */
        case 0x28:
            status = TRANSMISSION_SUCCESS;
            break;

        /* byte sent and NACK returned */
        case 0x30:
            status = TRANSMISSION_ERROR;
            break;

        /* byte sent and bus failure detected */
        case 0x38:
            status = TRANSMISSION_ERROR;
            break;

        default:
            status = TRANSMISSION_ERROR;
            break;
    }

    return status;
}

bool i2c_timeout(void)
{
    uint8_t time = TIMEOUT;
    int8_t status = BUS_DISCONNECTED;

    while (time-- > 0) {
        /* check to see if bus is ready */
        if ((TWCR & (1 << TWINT))) {
            status = BUS_CONNECTED;
            break;
        }
    }
    return status;
}

uint8_t i2c_rx_byte(bool response)
{
    int8_t status;

    if (response == ACK) {
        TWCR |= (1 << TWEA); // generate ACK
    } else {
        TWCR &= ~(1 << TWEA); // generate NACK
    }

    /* clear interrupt flag */
    TWCR |= (1 << TWINT);

    /* detect bus time-out */
    if (i2c_timeout() != BUS_DISCONNECTED) {
        /* retrieve transmission status codes or received data */
        switch (TWSR & 0xF8) {
            /* data byte read and ACK returned by master */
            case 0x50:
                status = TWDR;
                break;

            /* data byte read and NACK returned by master */
            case 0x58:
                status = TWDR;
                break;

            /* bus failure detected */
            case 0x38:
                status = TRANSMISSION_ERROR;
                break;

            default:
                status = TRANSMISSION_ERROR;
                break;
        }
    } else {
        status = TRANSMISSION_ERROR;
    }

    return status;
}

void i2c_tx_stop(void)
{
    /* clear interrupt flag, issue stop command (cleared automatically) */
    TWCR |= (1 << TWINT) | (1 << TWSTO);

    while (!(TWCR & (1 << TWSTO))); // wait until stop transmitted
}
#endif

void OLED_Init(void)
{
    i2c_init();
    
    oledSendCommand(SSD1306_DISPLAY_OFF);
    oledSendCommand(SSD1306_SET_DISPLAY_CLOCK_DIV_RATIO);
    oledSendCommand(0x80);
    oledSendCommand(SSD1306_SET_MULTIPLEX_RATIO);
    oledSendCommand(0x3F);
    oledSendCommand(SSD1306_SET_DISPLAY_OFFSET);
    oledSendCommand(0x0);
    oledSendCommand(SSD1306_SET_START_LINE | 0x0);
    oledSendCommand(SSD1306_CHARGE_PUMP);
    oledSendCommand(0x14);
    oledSendCommand(SSD1306_MEMORY_ADDR_MODE);
    oledSendCommand(0x00);
    oledSendCommand(SSD1306_SET_SEGMENT_REMAP | 0x1);
    oledSendCommand(SSD1306_COM_SCAN_DIR_DEC);
    oledSendCommand(SSD1306_SET_COM_PINS);
    oledSendCommand(0x12);
    oledSendCommand(SSD1306_SET_CONTRAST_CONTROL);
    oledSendCommand(0xCF);
    oledSendCommand(SSD1306_SET_PRECHARGE_PERIOD);
    oledSendCommand(0xF1);
    oledSendCommand(SSD1306_SET_VCOM_DESELECT);
    oledSendCommand(0x40);
    oledSendCommand(SSD1306_DISPLAY_ALL_ON_RESUME);
    oledSendCommand(SSD1306_NORMAL_DISPLAY);
    oledSendCommand(SSD1306_DISPLAY_ON);
    
    OLED_Clear();  /* Clear the complete LCD during init */
}

void OLED_DisplayChar(uint8_t ch)
{
    uint8_t dat,i=0;
    
    if(((OledCursorPos+FONT_SIZE)>=128) || (ch=='\n'))
    {
        /* If the cursor has reached to end of line on page1
         OR NewLine command is issued Then Move the cursor to next line */
        OLED_GoToNextLine();
    }
    if(ch!='\n') /* TODO */
    {
        ch = ch-0x20; // As the lookup table starts from Space(0x20)
        
        while(1)
        {
            dat= OledFontTable[ch][i]; /* Get the data to be displayed for LookUptable*/
            
            
            oledSendByte(dat); /* Display the data and keep track of cursor */
            OledCursorPos++;
            
            i++;
            
            if(i==FONT_SIZE) /* Exit the loop if End of char is encountered */
            {
                oledSendByte(0x00); /* Display the data and keep track of cursor */
                OledCursorPos++;
                break;
            }
        }
    }
}

void OLED_DisplayString(char *ptr) // Changed from uint8_t to char to address compiler warning DM 4/4/22
{
    while(*ptr)
        OLED_DisplayChar(*ptr++);
}

void OLED_ScrollMessage(uint8_t lineNum, char *strptr)
{
    unsigned char i,j,k,l,cursor,ch;
    
    if(lineNum > 7)
        lineNum = 0; // Select first line if the lineNumberToStartDisplay is out of range
    
    for(i=0;strptr[i];i++)
    {
        /* Loop to display the complete string,    each time 16 chars are displayed and
         pointer is incremented to point to next char */
        
        for(k=0;k<6;k++)
        {
            OLED_SetCursor(lineNum,6-k);     //Move the Cursor to first line
            cursor = 6-k;
            
            for(j=0;(strptr[i+j] && (cursor<128));j++)
            {
                ch = strptr[i+j]-0x20;
                for(l=0;(l<5) && (cursor<128);l++)//Display first 16 Chars or till Null char is reached
                {
                    oledSendByte(OledFontTable[ch][l]);
                    cursor++;
                }
                
                oledSendByte(0);
                _delay_us(10);
                cursor++;
            }
            _delay_ms(20);
        }
    }
}

void OLED_DisplayNumber(uint8_t v_numericSystem_u8, uint32_t v_number_u32, uint8_t v_numOfDigitsToDisplay_u8)
{
    uint8_t i=0,a[10];
    
    if(C_BINARY_U8 == v_numericSystem_u8)
    {
        while(v_numOfDigitsToDisplay_u8!=0)
        {
            /* Start Extracting the bits from the specified bit positions.
             Get the Acsii values of the bits and display */
            i = util_GetBitStatus(v_number_u32,(v_numOfDigitsToDisplay_u8-1));
            OLED_DisplayChar(util_Dec2Ascii(i));
            v_numOfDigitsToDisplay_u8--;
        }
    }
    else if(v_number_u32==0)
    {
        /* If the number is zero then update the array with the same for displaying */
        for(i=0;((i<v_numOfDigitsToDisplay_u8) && (i<C_MaxDigitsToDisplay_U8));i++)
            OLED_DisplayChar('0');
    }
    else
    {
        for(i=0;i<v_numOfDigitsToDisplay_u8;i++)
        {
            /* Continue extracting the digits from right side
             till the Specified v_numOfDigitsToDisplay_u8 */
            if(v_number_u32!=0)
            {
                /* Extract the digits from the number till it becomes zero.
                 First get the remainder and divide the number by TypeOfNum(10-Dec, 16-Hex) each time.
                 example for Decimal number:
                 If v_number_u32 = 123 then extracted remainder will be 3 and number will be 12.
                 The process continues till it becomes zero or max digits reached*/
                a[i]=util_GetMod32(v_number_u32,v_numericSystem_u8);
                v_number_u32=v_number_u32/v_numericSystem_u8;
            }
            else if( (v_numOfDigitsToDisplay_u8 == C_DisplayDefaultDigits_U8) ||
                    (v_numOfDigitsToDisplay_u8 > C_MaxDigitsToDisplay_U8))
            {
                /* Stop the iteration if the Max number of digits are reached or
                 the user expects exact(Default) digits in the number to be displayed */
                break;
            }
            else
            {
                /* In case user expects more digits to be displayed than the actual digits in number,
                 then update the remaining digits with zero.
                 Ex: v_num_u32 is 123 and user wants five digits then 00123 has to be displayed */
                a[i]=0;
            }
        }
        
        while(i!=0)
        {
            /* Finally get the ascii values of the digits and display*/
            OLED_DisplayChar(util_Hex2Ascii(a[i-1]));
            i--;
        }
    }
}

void OLED_DisplayFloatNumber(double v_floatNum_f32)
{
    uint32_t v_temp_u32;
    /* Dirty hack to support the floating point by extracting the integer and fractional part.
     1.Type cast the number to int to get the integer part.
     2.Display the extracted integer part followed by a decimal point(.)
     3.Later the integer part is made zero by subtracting with the extracted integer value.
     4.Finally the fractional part is multiplied by 100000 to support 6-digit precision */
    
    v_temp_u32 = (uint32_t) v_floatNum_f32;
    OLED_DisplayNumber(C_DECIMAL_U8,v_temp_u32,C_DisplayDefaultDigits_U8);
    
    OLED_DisplayChar('.');
    
    v_floatNum_f32 = v_floatNum_f32 - v_temp_u32;
    v_temp_u32 = v_floatNum_f32 * 1000000;
    OLED_DisplayNumber(C_DECIMAL_U8,v_temp_u32,C_DisplayDefaultDigits_U8);
}

void OLED_Clear()
{
    int i;
    
    oledSendCommand(SSD1306_SET_COLUMN_ADDR);
    oledSendCommand(0);
    oledSendCommand(127);
    
    oledSendCommand(SSD1306_SET_PAGE_ADDR);
    oledSendCommand(0);
    oledSendCommand(7);
    
    
    oledSendStart(SSD1306_ADDRESS);
    oledSendByte(SSD1306_DATA_CONTINUE);
    
    for (i=0; i<1024; i++)      // Write Zeros to clear the display
    {
        oledSendByte(0);
    }
    
    oledSendCommand(SSD1306_SET_COLUMN_ADDR);
    oledSendCommand(0);
    oledSendCommand(127);
    
    oledSendCommand(SSD1306_SET_PAGE_ADDR);
    oledSendCommand(0);
    oledSendCommand(7);
    
    oledSendStart(SSD1306_ADDRESS);
    oledSendByte(SSD1306_DATA_CONTINUE);
}

void  OLED_GoToLine(uint8_t lineNumber)
{
    if(lineNumber<8)
    {   /* If the line number is within range
         then move it to specified line and keep track*/
        OledLineNum = lineNumber;
        OLED_SetCursor(OledLineNum,0);
    }
}

void  OLED_GoToNextLine()
{
    /*Increment the current line number.
     In case it exceeds the limit, rool it back to first line */
    OledLineNum++;
    OledLineNum = OledLineNum&0x07;
    OLED_SetCursor(OledLineNum,0); /* Finally move it to next line */
}

void OLED_SetCursor(uint8_t lineNumber,uint8_t cursorPosition)
{
    /* Move the Cursor to specified position only if it is in range */
    if((lineNumber <= C_OledLastLine_U8) && (cursorPosition <= 127))
    {
        OledLineNum=lineNumber;   /* Save the specified line number */
        OledCursorPos=cursorPosition; /* Save the specified cursor position */
        
        oledSendCommand(SSD1306_SET_COLUMN_ADDR);
        oledSendCommand(cursorPosition);
        oledSendCommand(127);
        
        oledSendCommand(SSD1306_SET_PAGE_ADDR);
        oledSendCommand(lineNumber);
        oledSendCommand(7);
        
        oledSendStart(SSD1306_ADDRESS);
        oledSendByte(SSD1306_DATA_CONTINUE);
    }
}

void OLED_DisplayLogo(char *ptr_Logo)
{
    int i;
    
    OLED_SetCursor(0,0);
    
    oledSendStart(SSD1306_ADDRESS);
    oledSendByte(SSD1306_DATA_CONTINUE);
    
    for ( i=0; i<1024; i++)      // Send data
    {
        oledSendByte(ptr_Logo[i]);
    }
}

void OLED_VerticalGraph(uint8_t barGraphNumber, uint8_t percentageValue)
{
    uint8_t lineNumberToStartDisplay,i,j,barGraphPosition;
    uint8_t lineNumber,valueToDisplay;
    
    
    if((barGraphNumber < C_MaxBarGraphs_U8) && (percentageValue<=100))
    {
        barGraphPosition = barGraphNumber * 32;
        
        OLED_SetCursor(0,barGraphPosition+8);
        OLED_DisplayNumber(E_DECIMAL,percentageValue,3);
        
        
        /* Divide the value by 8, as we have 8-pixels for each line */
        percentageValue = percentageValue/2;
        lineNumberToStartDisplay = (percentageValue>>3);
        lineNumber = 7-lineNumberToStartDisplay;
        
        
        for(i=1;i<8;i++)
        {
            OLED_SetCursor(i,(barGraphPosition+12));
            if(i<lineNumber)
            {
                valueToDisplay = 0x00;
            }
            else if(i== lineNumber)
            {
                valueToDisplay = util_GetMod8(percentageValue,8);
                valueToDisplay = (0xff<<(8-valueToDisplay));
            }
            else
            {
                valueToDisplay = 0xff;
            }
            
            for(j=0;j<12;j++)
            {
                oledSendByte(valueToDisplay);
            }
        }
    }
}

void OLED_HorizontalGraph(uint8_t barGraphNumber, uint8_t percentageValue)
{
    uint8_t lineNumberToStartDisplay,i;
    lineNumberToStartDisplay = (barGraphNumber * 2)+1;
    
    if((barGraphNumber < C_MaxBarGraphs_U8) && (percentageValue<=100))
    {
        OLED_SetCursor(lineNumberToStartDisplay,0);
        
        for(i=0;i<percentageValue;i++)
        {
            oledSendByte(0xff);
        }
        
        for(i=percentageValue;i<100;i++)
        {
            oledSendByte(0x00);
        }
        
        OLED_SetCursor(lineNumberToStartDisplay,105);
        
        OLED_DisplayNumber(E_DECIMAL,percentageValue,3);
    }
}

void OLED_EnableInversion(void)
{
    oledSendCommand(SSD1306_INVERT_DISPLAY);
}

void OLED_DisableInversion(void)
{
    oledSendCommand(SSD1306_NORMAL_DISPLAY);
}

void OLED_SetBrightness(uint8_t brightnessValue)
{
    oledSendCommand(SSD1306_SET_CONTRAST_CONTROL);
    oledSendCommand(brightnessValue);
}

void oledSendStart(uint8_t address){

    
    i2c_tx_start(MASTER_TRANSMITTER);
    i2c_tx_address(address);
}

void oledSendStop(void){

    i2c_tx_stop();
}

void oledSendByte(uint8_t ch){

    
    i2c_tx_byte(ch);
    
}

void oledSendCommand(uint8_t cmd){
    //oledSendStart(SSD1306_ADDRESS<<1);
    oledSendStart(SSD1306_ADDRESS);
    oledSendByte(SSD1306_COMMAND);
    oledSendByte(cmd);
    oledSendStop();
}




