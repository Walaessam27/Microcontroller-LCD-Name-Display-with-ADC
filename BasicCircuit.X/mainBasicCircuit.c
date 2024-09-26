// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#include <xc.h>
#include<stdio.h>
#include "my_adc.h"
#include "lcd_x8.h"

void initPorts(void);
void setupSerial(void) ;
void send_byte_no_lib(unsigned char c);
unsigned char read_byte_no_lib(void);
void send_string_no_lib(unsigned char *p);
float read_adc_voltage(unsigned char channel);
unsigned char is_byte_available(void);


unsigned char channelRead;
float AN[3];  
float voltage;
char buffer[32];
char x;
int Column=1;
int Line=2;
void main(void) {
    INTCON = 0;
    initPorts();
    setupSerial();
    init_adc_no_lib();
    lcd_init(); 

     delay_ms(3000);   
     lcd_putc('\f'); //clears the display
     sprintf(buffer, "DOAA - WALAA");
     lcd_gotoxy(1,1);
     lcd_puts(buffer);
     delay_ms(2000); // 20 secs delay
     lcd_putc('\f'); // clear after that
    
    delay_ms(1000); 
  
    sprintf(buffer, "DOAA - WALAA \r\n");
     send_string_no_lib(buffer);
    delay_ms(1000); // 20secs dealy to display it
    
    while(1){ 
        CLRWDT();

        for (channelRead = 0; channelRead < 3; channelRead++) {
            if(channelRead==2){
              AN[channelRead] = read_adc_temp((unsigned char) channelRead);
            }
            else { 
                
                voltage = read_adc_voltage((unsigned char) channelRead);
                AN[channelRead] = voltage; // store in array AN0--AN2
            
            }
        }
        
        lcd_gotoxy(1, 1);
        sprintf(buffer, "V0=%4.2f--T=%4.2f", AN[0], AN[2]);
        lcd_puts(buffer);
               
        if (is_byte_available()) {
             x = read_byte_no_lib();
              PORTDbits.RD7 = !PORTDbits.RD7;
             sprintf(buffer, "%c", x);
             lcd_gotoxy(Column, Line);
             lcd_puts(buffer);
             send_string_no_lib(buffer);
            
             if(Column==16 && Line==2)
             {
                 Column=1;
                 Line=2;
             }
             else Column++;

             if (x=='?'){
                   lcd_putc('\f'); //clears the display
                    Column=1;
                    Line=2;
                    lcd_gotoxy(Column, Line);
             }
                 

            delay_ms(300);
        }
         
    }
    
    return;
}

void initPorts(void)
{
    ADCON1 = 0xC; // or ox0C means 3 analog inputs
    LATA = LATB = LATC = LATD = LATE =0;
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISD = TRISE =0;
    TRISC = 0b10000000;//0x80
    
}
void setupSerial(void) {
    unsigned char dummy;
    BAUDCONbits.BRG16 = 0;
    TXSTA = 0;
    SPBRG = 25;//25;
    SPBRGH = 0;
    TXSTAbits.BRGH = 1; //baud rate high speed option
    TXSTAbits.TXEN = 1; //	;enable transmission


    RCSTA = 0; // ;SERIAL RECEPTION W/ 8 BITS,
    RCSTAbits.CREN = 1; //;enable reception
    RCSTAbits.SPEN = 1;
    ; //enable serial port
    dummy = RCREG; //, W        ; clear the receiver buffer      
    dummy = RCREG; //,W         ; clear the receiver buffer
    return;
}

unsigned char read_byte_no_lib(void) {
    unsigned char c;

    if (RCSTAbits.FERR || RCSTAbits.OERR)//check for error
    {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    
    // wait until byte available
    while (!(PIR1bits.RCIF)) {
        CLRWDT(); //if enabled
    }
    c = RCREG;
    return c;
}

void send_byte_no_lib(unsigned char c) {
    while (!TXSTAbits.TRMT)//fixed 
    {
        CLRWDT(); //if enabled  
    }
    TXREG = c;
}

void send_string_no_lib(unsigned char *p) {
    while (*p) {
        send_byte_no_lib(*p); //or use the send_byte_no_lib()
        p++;
    }
}
unsigned char is_byte_available(void) {
    if (RCSTAbits.FERR || RCSTAbits.OERR) {
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }
    if (PIR1bits.RCIF) return 1;
    else return 0;
}


void delay_ms(unsigned int n) // int in XC* is 16 bit same as short n
{
    int i;
    for (i=0; i < n; i++){
         __delaywdt_ms(1) ; 
    }
}

float read_adc_voltage(unsigned char channel) {
    int raw_value;
    float voltage;
    raw_value = read_adc_raw_no_lib(channel);
    voltage = (raw_value * 5) / 1023.0;
    return voltage;

}