#include <xc.h>
#define _XTAL_FREQ 16000000

#define RS                  RB4                        /* Register select pin definition                         */
#define RW                  RB7                        /* Read/Write pin definition                              */
#define EN                  RB5                        /* Enable  pin definition                                 */
#define Data_PORT           PORTB                                 /* DATA PORT definition                                   */
#define CTRL_PORT_DIR       TRISB
#define DATA_PORT_DIR       TRISB
void lcd_cmd (char cmd);
void lcd_data(char data);
void CreateCustomCharacter (unsigned char *Pattern, const char Location);
void _lcd_write_data(unsigned char);

void CreateCustomCharacter (unsigned char *Pattern, const char Location)
{ 
    int i=0; 
    //lcd_cmd (0x40+(Location*8));  //Send the Address of CGRAM
    _lcd_write_data(0x40+(Location*8));
    for (i=0; i<8; i++)
    //lcd_data (Pattern [ i ] );  //Pass the bytes of pattern on LCD 
    _lcd_write_data(Pattern[i]);
}

void lcd_cmd (char cmd)
{
  RS=0;RW=0;
  Data_PORT = cmd;
  EN=1;
  __delay_ms(5);
  EN=0;
  Data_PORT =((cmd<<4) & 0xF0);
  EN=1;
  __delay_ms(5);
  EN=0;
}

void lcd_data(char data)
{
  RS=1;RW=0;
  Data_PORT =data;
  EN=1;
  __delay_ms(5);
  EN=0;
  Data_PORT=((data<<4)&0xF0);
  EN=1;
  __delay_ms(5);
  EN=0;
}