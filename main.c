/*
 * File:   main.c
 * Author: witek
 *
 * Created on 29 września 2019, 10:34
 * 20191215 v.1.01
 *  - nowe ustawienia początkowe
 *  - poprawienie prawdopodobnego błędu band--
 * 20230410 v.1.0.3
 *  - zamiast 3,5 CW jest 5MHz
 * ---------
 * Z1: RA4 ICOM port; nóżka 6
 * Z2: RD4 DATA port; nóżka 27
 *   
 * Z3: RD5 "5MHz"; nóżka 28
 * Z4: RD7 włączanie dziesiątego pasma; nóżka 30
 *
 * 
 * 
 */

//#define PASMA

#include <xc.h>
#define _XTAL_FREQ 16000000
#include <string.h>
#include <stdio.h>
#include "eeprom.h"

// BEGIN CONFIG
#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT enabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
//END CONFIG

#define PGM_P const char *

#define _2004

//Определения препроцессора
#define setbit(var,bitnomber) ((var) |= 1 << (bitnomber))
#define clrbit(var,bitnomber) ((var) &= ~(1 << (bitnomber)))
#define getbit(var,bitnomber) (((var)>>(bitnomber))&1)
#define BitIsClear(var,bitnomber)  (!(var & (1<<bitnomber)))
#define BitIsSet(var,bitnomber)    (var & (1<<bitnomber))


#define ButtonPort      PORTC
#define ButtonPortDDR   TRISC
#define Button1         RC7
#define Button2         RC6
//#define Button3         PIND6
#define ButtonPinPort   PORTC


#define FanPort         PORTC
#define FanPortDDR      DDRC
#define FanPin          PINC2


#define AnodPort        PORTC
#define AnodPortDDR     DDRC
#define AnodPin         PINC7


#define SWR3DDR         DDRD
#define SWR3Port        PORTD
#define SWR3Pin         PIND7

// kanały analogowe:
#define FEW             0
#define REW             1
#define PlateCurrent    2
#define IcomPin         4
#define PlateVoltage    5
#define GridCurrent     6   // AN6 nóżka 9

//определение выводов,управляющих 561ИД1

#define BandPort        PORTC
#define BandPortDDR     TRISC

//#define _A   RC0
//#define _B   RC1
//#define _C   RC2
//#define _D   RC3
#define _A 0b00000001
#define _B 0b00000010
#define _C 0b00000100
#define _D 0b00001000

/*
#define BandData_A RD0
#define BandData_B RD1
#define BandData_C RD2
#define BandData_D RD3
*/
enum
{
_160metr,
_80metr,
_40metr,
_30metr,
_20metr,
_17metr,
_15metr,
_12metr,
_10metr,
_6metr      // pasmo 50MHz lub 5 MHz w zależności od portu RD5 (Z3)(28))
};

//unsigned char BandPins[]={_B,_C,_A,_A+_B,~(_A+_B+_C+_D),_A+_C,_B+_C};
//unsigned char BandPins[]={0x00, 0x01, 0x02, 0x03,0x04};
//#define EnableBand(x)   BandPort |= BandPins[x]
#define EnableBand(x)   BandPort |= x;

//.............................................................

unsigned int U_forward;
unsigned int U_reflect;
unsigned int U_plate;
unsigned int I_plate;





unsigned long x;
unsigned long PWRin;
unsigned long PWRout;
unsigned long xW1;

unsigned int swr;
unsigned int u_input;
unsigned int u_delta;
unsigned int delta;
unsigned int u_output;
unsigned int u_output_old;
unsigned char OldBand;

unsigned char config_dirty = 0;
unsigned long brudny_czas = 0;		// czas ostatniego żądania zapisu do EEPROM

void KeyPadControl();
void DataPortControl();

// prototypy dla custom characters
void lcd_cmd (char cmd);
void lcd_data(char data);
void CreateCustomCharacter (unsigned char *Pattern, const char Location);

// co nieco do obsługi pamięci pasma:
#define ZAPIS_CO	2000 		// co jaki czas zapis do EEPROM - minimum
void check_for_dirty_configuration();
unsigned long czas = 0;
#define SBIT_PS2  2
char zapis = 0;
void __interrupt(high_priority) tcInt(void)
{  
    if(TMR0IF == 1)
    {
        czas++;   // czas upływa co 1ms od startu systemu
        TMR0 = 132;     /*Load the timer Value, (Note: Timervalue is 101 instaed of 100 as the
                          TImer0 needs two instruction Cycles to start incrementing TMR0 */
        TMR0IF=0;       // Clear timer interrupt flag
    } 
}
// koniec co nieco do obsługi pamięci pasma

//прототипы функций 
//LCD:
void enable_half_byte(void);
void enable (void);
void PrintLcd(unsigned char value);
void writeCGRAM(unsigned char addr, unsigned char data);
//=======================================================
//-------------------------------------------------------
//Высокоуровневые функции, не используемые CV Avr
//-------------------------------------------------------
void Lcd_Chr(unsigned char line,unsigned char column,char ch);


//=======================================================
//-------------------------------------------------------
//низкоуровневые функции, используемые CV Avr
//-------------------------------------------------------
void _lcd_ready(void);
//void _lcd_write_data(char);
void _lcd_write_data(unsigned char);
void lcd_write_byte(unsigned char addr, unsigned char data);
unsigned char lcd_read_byte(unsigned char addr);
//-------------------------------------------------------
//Высокоуровневые функции, используемые CV Avr
//-------------------------------------------------------
void lcd_init(void);
void ClrScr(void);
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_puts(char *str);
// LCD end
void PrintSwr(void);
void InitADC(void);
unsigned int ADC_READ (unsigned char channel);
void PrintPower(void);
void LongThermometr(void);
void RightTermometr(unsigned int);
void LeftTermometr(unsigned int);
void read_array_from_EEMEM(unsigned char* adress);
void PrintPlateVoltage(unsigned int u_dat);
void PrintPlateCurrent(unsigned int i_dat);

void PrintGridCurrent(unsigned int is_dat);

void PrintResults(void);
void icom(void);
void SelectBand();
void PrintValue4(unsigned int value,unsigned char pos,unsigned char line);
void ChangeBand(void);
void screen1(void);
void PowerScale(void);
void mask(unsigned char UpLine,unsigned char LowLine);
void SynthesChar(void);
void ShowThermometr(void);
void privetstvie(void);
void Temperature(void);

void ADC_Init();

//================================================================
//Параметры, хранящиеся в ЕЕПРОМ
//================================================================
//__eeprom unsigned int aaa=0x1111; //предохранительная ячейка на возможную изнашиваемость __eeprom
//__eeprom unsigned char EEMEM_magic_number = 0x0c; //предохранительная ячейка на возможную изнашиваемость __eeprom
//__eeprom unsigned char EEMEM_band = _80metr;    // pasmo w eepromie
//__eeprom unsigned int EEMEM_MaxPower=1500; //максимальная проходящая мощность
//__eeprom unsigned int EEMEM_MaxI=1500;     //максимальный ток анода
//__eeprom unsigned int EEMEM_MaxU=2700;     //максимальное напряжение анода
//__eeprom unsigned int EEMEM_FanT_ON=90;    //температура включения вентилятора
//__eeprom unsigned int EEMEM_FanT_OFF=60;   //температура отключения вентилятора
//__eeprom unsigned int EEMEM_Z_line=50;     //волновое сопротивление кабеля
//__eeprom unsigned int EEMEM_koeff=10;      //масштабирующий коэффициент по мощности
                                            //для 1500 вт равен примерно 10

unsigned int MaxPower = 3000;
unsigned long MaxU = 3800;
unsigned long MaxI = 2000;
unsigned long MaxIs = 1000;
unsigned int FanT_ON;
unsigned int FanT_OFF;
unsigned int Z_line = 50;
unsigned int koeff = 10;

//===================================================================
const char Mes0[] = "      Welcome!      ";
const char Mes1[] = "POWER=    W SWR= .  ";
const char Mes2[] = "BAND    MHz Ua=    V";
const char Mes3[] = "Is=    mA  Ia=    mA";
const char Mes4[] = "   PA  controller";  
const char Mes5[] = "    Beta Ver 1.03";
const char Mes6[] = "    Warming tube  ";
const char Mes7[] = "  Switching on  Ua  ";
const char Mes8[] = "    in       sec    ";
const char Mes9[] = " Anod tension is ON ";



PGM_P string_table[] = 
{
Mes0,
Mes1,
Mes2,
Mes3,
Mes4,
Mes5,
Mes6,
Mes7,
Mes8,
Mes9
};

const char freq1_8[] = "1,8MHz";
const char freq3_5[] = "  5MHz";
const char freq3_8[] = "3,5MHz";
const char freq7[]   = "  7MHz";
const char freq10[]  = " 10MHz";
const char freq14[]  = " 14MHz";
const char freq18[]  = " 18MHz";
const char freq21[]  = " 21MHz";
const char freq24[]  = " 24MHz";
const char freq28[]  = " 28MHz";
const char freq50[]  = " 50MHz";

PGM_P freq_table[] = 
{
freq1_8,
freq3_8,
freq7,
freq10,
freq14,
freq18,
freq21,
freq24,
freq28,
freq50,
freq3_5
};

unsigned char letter_I[]   = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};   //  0/III
unsigned char letter_II[]   = {0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c};   //  III/0
unsigned char letter_III[] = {0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f};  //  III/III

//PGM_P znak_table[] =
//{
//letter_III_0,    //III
//letter_III_III,    //II
//letter_0_III,     //I
//};

char buffer[21];
char Upbuf[21];
char Lowbuf[21];
char letter[8];
/*
#define kilowatt   //прошивка для 1500 вт проходящей мощности
                   //чтобы скомпилировать на 250 вт следует закоментарить эту строку

#ifdef kilowatt
#define koeff 15
#define MaxPower    2500
#else
#define koeff 1 
//#define MaxPower    250
#endif
*/


#define _2004    //для компиляции для 20х4 символов
//#define _1604  //для компиляции для 16х4 символов

//#define  Z_line  50
#define  a  128

unsigned char band;

void main(void) 
{
    unsigned char i;
    // ustawienie przerwania dla zegara sytemowego
    OPTION_REG = (1<<SBIT_PS2);  // Timer0 with external freq and 32 as prescalar
    TMR0=100;       // Load the time value for 1ms delay
    TMR0IE=1;       //Enable timer interrupt bit in PIE1 register
    GIE=1;          //Enable Global Interrupt
    PEIE=1;         //Enable the Peripheral Interrupt
    // IRQ end
    ADC_Init();                   //Initialize ADC
    lcd_init();
    TRISE2 = 0;     // RE2 output
    RE2 = 0;        // port sygnalizacji przekroczenia SWR (powyżej 3))
    for(i=0;i<8;i++)
    {
    //загрузка в CG память ЖКИ символа из 3 полосок сверху
    writeCGRAM(0b01011000+i, letter_I[i]);//смещение 3 в CGMEM
    //загрузка в CG память ЖКИ символа из 3 полосок сверху/снизу
    writeCGRAM(0b01001000+i, letter_III[i]);//смещение 1 в CGMEM
    //загрузка в CG память ЖКИ символа из 3 полосок снизу
    writeCGRAM(0b01010000+i, letter_II[i]);//смещение 2 в CGMEM
    //по смещению 3 будет синтезироваться символ конца мощностного градусника
    //по смещению 4 будет синтезироваться символ левого начала компаратора при 
    //заполненном градуснике мощности
    //по смещению 5 будет синтезироваться символ левого конца компаратора при 
    //пустом градуснике мощности
    //по смещению 6 будет синтезироваться символ правого конца компаратора при 
    //пустом градуснике мощности
    }

    //CreateCustomCharacter(letter_III_0, 0);
    //CreateCustomCharacter(letter_III_III, 1);
    //CreateCustomCharacter(letter_0_III, 2);
    ClrScr();
    //================================================
    //инициализация индикации и переключателя диапазонов
    //=================================================

    //BandPortDDR |= _A+_B+_C+_D;
    TRISC = 0b11000000;
    
    TRISD = 0b11111111;     // port D cały jako input
    // wymagany jest pull up dla dataportu D: RD0, RD1, RD2, RD3 - cztery rezystory (np.4,7k) do +5V 
    // RD0-RD3 wejście na sterowanie digital DATAPORT z transceivera
    //    DataPort Codes 
    //      kody dla data portu:
    //Band 	Code
    //Line 	dcba
    //Pin 	7654
    //
    //160m 	0001
    //80m 	0010
    //40m 	0011
    //30m 	0100
    //20m 	0101
    //17m 	0110
    //15m 	0111
    //12m 	1000
    //10m 	1001
    //6m 	1010
    band = EEPROM_ReadByte(0);
    if (band > 9)
    {
        band = _160metr;        
        config_dirty = 1;
    }
    else
    {
            config_dirty = 0;
    }
    OldBand = band;
    EnableBand(OldBand);
    privetstvie();
    screen1();
    SelectBand();
    //================================================
    //инициализация кнопок переключателя диапазонов
    //===============================================
    //ButtonPortDDR &= ~(_BV(Button1)+_BV(Button2)+_BV(Button3));
    //ButtonPort &=~(_BV(Button1)+_BV(Button2)+_BV(Button3));
    //ButtonPort &=~(_BV(Button1)+_BV(Button2)+_BV(Button3));
    //заменено на
    //clrbit(ButtonPortDDR,Button1);
    //clrbit(ButtonPortDDR,Button2);
    //setbit(ButtonPort,Button1);
    RC7 = 1;
    //setbit(ButtonPort,Button2);
    RC6 = 1;

    //================================================
//    #define JP_Port         PORTA
#define Icom_DDR      TRISA4
#define Icom_Pin      PORTAbits.RA4
#define DataPort_DDR     TRISD4      // drugi jumper
#define DataPort_Pin  PORTDbits.RD4
    Icom_DDR = 1;           // wejście na pin Icom jako input
    DataPort_DDR = 1;       // drugi jumper jako wejście
    PORTDbits.RD5 = 1;      // trzeci jumper jako wejście - wybór pomiędzy 50MHz a 5MHz
    TRISD7 = 1;         // wejście włączające dziesiąte pasmo
    while (1) 
    {
        PrintResults();
        if (ADC_READ(GridCurrent) <= 3)   //если подана раскачка блокируется переключение контуров ВКС; blokada przełączania pasm przy wysterowaniu
        {
            ChangeBand();
        }
		check_for_dirty_configuration();
    }
}

void privetstvie(void) 
{
    unsigned char i;
    unsigned char j;

    //strcpy(buffer, (PGM_P)&(string_table[0]));
    strcpy(buffer, string_table[0]);
    lcd_gotoxy(0, 0);
    lcd_puts(buffer);
    strcpy(buffer, string_table[4]); //"PA controller-1500W"
    lcd_gotoxy(0, 1);
    lcd_puts(buffer);
    strcpy(buffer, (string_table[5])); //"    Beta Ver 1.03"
    lcd_gotoxy(0, 2);
    lcd_puts(buffer);
    

    //Задержка для показа сообщения в течении 2 сек
    for (i = 0; i < 4; i++) 
    {
#ifndef DEBUGGING
        __delay_ms(0xFF);
#else
        _delay_ms(0x30); //��� �������
#endif
    }
}

void screen1() 
{

    strcpy(buffer, string_table[1]); //"P=    W     SWR= .  "
    lcd_gotoxy(0, 0);
    lcd_puts(buffer);
    lcd_gotoxy(0, 1);
    strcpy(buffer, string_table[2]); //"Fr=   mHz   U=     v"
    lcd_puts(buffer);
    lcd_gotoxy(0, 2);
    strcpy(buffer, string_table[3]); //"T=    C     I=    mA"
    lcd_puts(buffer);
}

void SelectBand() 
{
    // RD5 = 0 -> pasmo 5MHz zamiast 50MHz
    if (band == _6metr && RD5 == 0)
    {
        strcpy(buffer, freq_table[band + 1]);            
    }
    else
    {
        strcpy(buffer, freq_table[band]);
    }
    lcd_gotoxy(5, 1);
    lcd_puts(buffer); //Пишем на дисплей диапазон     
    BandPort &= ~(_A + _B + _C + _D); //сброс всех диапазонов    
    EnableBand(band);
	config_dirty = 1;
	brudny_czas = czas;
}
void DataPortControl()
{
    // Z3: RD5 "5MHz"; nóżka 28
    // Z4: RD7 wyłącza pasmo 6m; nóżka 30

    unsigned char DataPortCode = PORTD & 0b00001111;
    //unsigned char band;
    switch (DataPortCode)
    {
        case 0x01:
            band = _160metr;
            break;
        case 0x02:
            band = _80metr;
            break;
        case 0x03:
            band = _40metr;
            break;
        case 0x04:
            band = _30metr;
            break;
        case 0x05:
            band = _20metr;
            break;
        case 0x06:
            band = _17metr;
            break;
        case 0x07:
            band = _15metr;
            break;
        case 0x08:
            band = _12metr;
            break;
        case 0x09:
            band = _10metr;
            break;
        case 0x0A:
            band = _6metr;
            break;
        default:
            ;
            //band = 0x0F;                    
    }
    if (band <= _6metr)
    {
        SelectBand();
    }
}
void KeyPadControl()
{
    unsigned char i;
    //unsigned char band;
    if (Button1 == 0) //Проверяем нажатие кнопки1 переключения
        //диапазонов (ножка RC7) UP
    {
        for (i = 0; i < 200; i++) 
        {
#ifndef DEBUGGING
            __delay_ms(10);
#else
            __delay_us(255);
#endif
            if (Button1)
                break;
        }
        if (i > 2) //защита от дребезга
        {
            //band = OldBand;
            if (RD7 == 0)   // pasmo 6m wyłączone
            {
                if (band < _10metr) band++;
                else band = _160metr;
            }
            else
            {
                // pasmo 6m włączone
                if (band < _6metr) band++;
                else band = _160metr;
            }
            SelectBand();
            OldBand = band;
#ifdef DEBUGGING
            __delay_ms(60);
#else
            for (i = 0; i < 4; i++) {
                __delay_ms(0xFF);
            }
#endif
        }

    }
    if (Button2 == 0) //Проверяем нажатие кнопки1 переключения
        //диапазонов (ножка RC6)  DOWN
    {

        for (i = 0; i < 200; i++) {
#ifndef DEBUGGING
            __delay_ms(10);
#else
            __delay_us(255);
#endif
            if (Button2)
                break;
        }
        if (i > 2) //защита от дребезга
        {
            //band = OldBand;
            if (RD7 == 0)   // pasmo 6m wyłączone
            {
                if (band > _160metr) band--;
                else band = _10metr;
            }
            else
            {
                if (band > _160metr) band--;
                else band = _6metr;
            }
            SelectBand();
            OldBand = band;
#ifdef DEBUGGING
            _delay_ms(60);
#else
            for (i = 0; i < 4; i++) {
                __delay_ms(0xFF);
            }
#endif
        }
    } 
}
void ChangeBand(void)
{
    // stan aktywny wysoki (brak zworki)
    // brak zworek Z1 i Z2 -> sterowanie ręczne zmiany pasma (z przycisków)
    
    // Z1: RA4 ICOM port; nóżka 6
    // Z2: RD4 DATA port; nóżka 27
    
    // Z3: RD5 "5MHz"; Z3; nóżka 28
    // Z4: RD7 włączanie dziesiątego pasma; nóżka 30
    
    if (RA4 == 1 && RD4 == 0)
    {
        icom();
    }
    if (RA4 == 0 && RD4 == 1)
    {
        unsigned char DataPortCode = PORTD & 0b00001111;
        DataPortControl();
    }
    if (RA4 == 0 && RD4 == 0)
    {
        KeyPadControl();
    }
}

void PrintResults(void)
{    
    U_plate=ADC_READ(PlateVoltage); //измерение анодного напряжения
    PrintPlateVoltage(U_plate);
    //
    I_plate=ADC_READ(PlateCurrent); //измерение анодного тока
    PrintPlateCurrent(I_plate);
    PrintGridCurrent(ADC_READ(GridCurrent));   // prąd siatki pierwszej
    //
    U_forward = ADC_READ(FEW);
    U_reflect = ADC_READ(REW);
    PrintSwr();
    PrintPower();
    PowerScale();
}
unsigned int ADC_READ(unsigned char channel)
{
//unsigned char AdcMux;
//unsigned int result;
//AdcMux=channel & 7;
//ADMUX |= AdcMux;
//ADCSRA |=_BV(ADSC);     // запуск АЦП
//
//while (BitIsClear(ADCSRA, ADIF)); // ожидание завершения преобразования
//ADMUX &=~AdcMux;
//result= ADCL;//Чтение младших 8 битов результата
//result+=((unsigned int)ADCH)<<8;
//return(result);
if(channel > 7)              //Channel range is 0 ~ 7
    return 0;
//unsigned int result;
  ADCON0 &= 0xC5;              //Clearing channel selection bits
  ADCON0 |= channel<<3;        //Setting channel selection bits
  __delay_ms(2);               //Acquisition time to charge hold capacitor
  GO_nDONE = 1;                //Initializes A/D conversion
  while(GO_nDONE);             //Waiting for conversion to complete
  //result = ADRESL;
  return ((ADRESH<<8)+ADRESL); //Return result
  //result+=((unsigned int)ADRESH)<<8;
  //return result;
}
void ADC_Init()
{
//#ifdef DEBUGGING
//ADMUX =0;    //внешний источник на AREF
//#else
//ADMUX |= _BV(REFS1)+_BV(REFS0);//0x80+0x40
////         _BV(REFS1) _BV(REFS0) - внутренний источник 2,56В
//#endif
//ADCSRA =_BV(ADEN)+_BV(ADPS2)+_BV(ADPS1);
////       АЦП вкл        f=8000000/64  
  ADCON0 = 0b10000001;               //Turn ON ADC and Clock Selection - ok
  // ToDo ustawić zewnętrzne źródło REF
  // VREF na AN3
  ADCON1 = 0b10000001;               //All pins as Analog Input and setting Reference Voltages
}

void PrintSwr(void) 
{
    if ((U_forward == 0 && U_reflect == 0))
    {
        Lcd_Chr(0, 16, 0x20);
        Lcd_Chr(0, 18, 0x20);
        Lcd_Chr(0, 19, 0x20);
        RE2 = 0;
    }
    else
    {
        if (U_forward <= U_reflect)
            swr = 9990;
        else
            swr = (((unsigned long) (U_forward + U_reflect))*1000) / (U_forward - U_reflect); //Вычисляем КСВ
        if (swr > 9990) swr = 9990; //Условие на ограничение 9.99
        Lcd_Chr(0, 16, (swr / 1000) % 10 + 0x30); //разряд единиц КСВ
        Lcd_Chr(0, 18, (swr / 100) % 10 + 0x30); //разряд десятых КСВ
        Lcd_Chr(0, 19, (swr % 10) + 0x30); //разряд сотых х КСВ
        if (swr > 3000)
        {
            RE2 = 1;
        }
        else 
        {
            RE2 = 0;
        }
    }
}

void PrintPlateVoltage(unsigned int u_dat) 
{
    unsigned long Voltage;
    Voltage = MaxU * u_dat;
    Voltage = Voltage/1023;
    PrintValue4(Voltage, 15, 1); //Пишем на дисплей анодное напряжение
}

void PrintPlateCurrent(unsigned int i_dat) 
{
    unsigned long Current;
    //Current=i_dat; 
    //Current =(MaxI*i_dat)/1000; 
    Current = MaxI * i_dat;
    Current = Current / 1023;
    //Current = i_dat; 
    PrintValue4(Current, 14, 2); //prąd anodowy
}
void PrintGridCurrent(unsigned int is_dat)
{
    unsigned long Current;
    Current = MaxIs * is_dat;
    Current = Current / 1023;
    PrintValue4(Current, 3, 2); //prąd siatki pierwszej   
}
void PrintPower() 
{
    unsigned long PWRout;
    unsigned int K;

    U_forward = ADC_READ(FEW);
    U_reflect = ADC_READ(REW);
    K = (U_reflect << 10) / U_forward;
    if (U_forward > 960) U_forward = 960; //ограничение на счёт. соответствует КСВ=10
    //при проходящей мощности ~ в 3 раза меньше 
    //на выходе РА при условии реализации в счете
    //всех 10 бит
    x = U_forward;
    x *= a; //преобразование
    x >>= 10; //получили напряжениеd в V
    PWRin = x * x*koeff;
    PWRin /= Z_line; //получили мощность
    //...............................................................
    PWRout = (((PWRin * (0x400 + K)) >> 10)*(0x400 - K)) >> 10;
    //этот изврат для сохранения точности, чтобы не прибегать к плавающей точке
    //.....................................................................
    PrintValue4((unsigned int) PWRout, 6, 0);
}
//...................................................................
//рисует градусник мощности
//для выбора типа индикатора раскоментарить соответствующую строку 

void PowerScale() 
{
#ifdef _1602
#define MaxSell 16
#endif

#ifdef _1604
#define MaxSell 16
#endif

#ifdef _2004
#define MaxSell 20
#endif

    unsigned int PWRkoef = (MaxPower / (3 * MaxSell));
    unsigned int length, sell;
    unsigned char i = 0;
    unsigned char ost;
    length = PWRin / PWRkoef; //определение количества палок в грудуснике
    sell = length / 3; //вычисление количества знакомест под 3-х палочный знак
    ost = length % 3; //остаток равен скан-коду(адресу) последнего выводимого
    //символа из CGRAM 
    if (sell >= 1) 
    {
        for (i = 0; i < sell; i++) 
        {
            buffer[i] = 1; //сначала выводим по три полоски
        }
    }
    if (ost != 0) 
    {
        if (ost == 1)
        {
            buffer[i] = 3; //одна верхняя полоска
        }
        if (ost == 2)
        {
            buffer[i] = 2; //две верхние полоски
        }
        i++;
    }
    for (; i < MaxSell; i++) 
    {
        buffer[i] = 0x20; //пробел
    }
    lcd_gotoxy(0, 3);
    lcd_puts(buffer);
}

void PrintValue4(unsigned int value, unsigned char pos, unsigned char line) 
{
    sprintf(buffer, "%i", value);
    lcd_gotoxy(pos, line);
    if (value >= 1000) {
        lcd_puts(buffer);
    } else {
        Lcd_Chr(line, pos, 0x20);
        if (value >= 100)
            lcd_puts(buffer);
        else {
            Lcd_Chr(line, pos + 1, 0x20);
            if (value >= 10)
                lcd_puts(buffer);
            else {
                Lcd_Chr(line, pos + 2, 0x20);
                lcd_puts(buffer);
            }
        }
    }
}

void icom(void) 
{
    unsigned int u_icom;
    //unsigned char band;
    /*
    #define IcomPin     PINA2
    #define IcomPort    PORTA
    #define IcomPortDDR DDRA
     */
    //unsigned char PortBand;
    //напряжения , соответствующие диапазонам.
    //         icom        ADC        результат преобразования
    //1,8 -    7.5v        2.5v       1000
    //3.5 -    6.1v        2.0v       813
    //7 -      5.1v        1.7v       680
    //10 -     0v            0v        0
    //14 -     4.1v        1.36v      546
    //18/21 -  3.2v        1.0v       427
    //24/28 -  2.25v      0.75v       300
    // 160m = 7.0-8.0V
    //80-75m = 6.0-6.8V
    //40m = 5.0-5.8V
    //30m = 0V
    //20m = 4.0-4.8V
    //17m-15m = 3.0-3.8V
    //12m-10m = 2.0-2.8V
    //6m-2m = 1.0-1.9V
    //
    /*
    enum
    {
    _160metr,
    _80metr,
    _40metr,
    _20metr,
    _30metr,
    _15metr,
    _10metr
    }; 
     */
    u_icom = ADC_READ(IcomPin);
    //выбор диапазона
#ifdef DEBUGGING
    if (u_icom < 120)
        band = _30metr;
    else
        if (u_icom < 300)
        band = _10metr;
    else
        if (u_icom < 490)
        band = _15metr;
    else
        if (u_icom < 670)
        band = _20metr;
    else
        if (u_icom < 760)
        band = _40metr;
    else
        if (u_icom < 900)
        band = _80metr;
    else
        band = _160metr;
    //...........................
#else
//    if (u_icom < 100)
//        band = _30metr;
//    else
//        if (u_icom < 350)
//        band = _10metr;
//    else
//        if (u_icom < 490)
//        band = _15metr;
//    else
//        if (u_icom < 610)
//        band = _20metr;
//    else
//        if (u_icom < 745)
//        band = _40metr;
//    else
//        if (u_icom < 900)
//        band = _80metr;
//    else
//        band = _160metr;
    //
    // Podział napięcia sterującego równo na dziesięć części
    //
    if (u_icom < 57)
        band = _160metr;
    else
        if (u_icom < 171)
        band = _80metr;
    else
        if (u_icom < 285)
        band = _40metr;
    else
        if (u_icom < 399)
        band = _30metr;
    else
        if (u_icom < 513)
        band = _20metr;
    else
        if (u_icom < 627)
        band = _17metr;
    else
        if (u_icom < 741)
        band = _15metr;
    else
        if (u_icom < 855)
        band = _12metr;
    else
        if (u_icom < 969)
        band = _10metr;
    else
        band = _6metr;
#endif
    SelectBand();
}
void check_for_dirty_configuration()
{
	if (config_dirty)
	{
		if (czas - brudny_czas > ZAPIS_CO)
		{
			//EEPROM_WriteByte(0, band);
            eeprom_write(0, band);
        	config_dirty = 0;
#ifdef PASMA
            zapis = ~zapis;
            PORTDbits.RD7 = zapis;
#endif
		}
	}
}
