//Драйвер работы с знакосинтезирующим ЖКИ
//**************************************
// Функции работы с ЖКИ 
// Версия 2.0
//
//для компилятора WinAVR 20080610, хотя по идее должно 
//компилиться практически на любой версии, не имеющей глюков по
//макросу _BV() (проверить на этот косяк) 
//**************************************
//------------------------------------------------------------
// Функции работы с ЖКИ в 4х битном режиме
// для компилятора WinAVR 
//Составлено для случая, когда все управляющие
//выводы индикатора подключены к ОДНОМУ из 
//портов контроллера, а шина данных полубайта подключена к младшему
// или старшему полубайту порта.
//Если необходимо подключить шину данных к старшему полубайту,
//то нужно закоментарить фразу #define LowNibble
//подключение выводов E,RS,RW индикатора может быть произвольным,
//следует только определить это в описании подключения выводов.
//-------------------------------------------------------------
//Применим для индикаторов 1602,1604,2004.
//При использовании конкретного индикатора оставить раскоментаренной
//одну из строк #define _1602/1604/2004 
//.......................................................
//Часть функций имеют имена, совпадающие с наименованием
//в CV Avr, и написана для получения соместимости при переносе
//с CV Avr на WinAvr
//-------------------------------------------------------------
//Неиспользуемая для управления индикатором свободная линия порта
//может использоваться по произвольному назначению
//Все операции по смене пола для нее должны быть определены
//своем тексте программы отдельно.
//...............................................
//В качестве подосновы использовался исходник от Арсена
//"Победитель"
   
//Если в основном теле программы использованы эти 2 библиотеки,
//то их подключение можно закоментарить

//#include <avr/io.h>
//#include <avr/pgmspace.h>

//#define DELAY
#include <xc.h>
#define _XTAL_FREQ 16000000


#ifndef DELAY
//#include <util/delay.h>
#endif

//#define _1602   //�������������� ��� ������� 16*2 ������
#define _2004     //�������������� ��� ������� 20*4 ������
//#define _1604     //�������������� ��� ������� 20*4 ������




//����������� �������������
#define setbit(var,bitnomber) ((var) |= 1 << (bitnomber))
#define clrbit(var,bitnomber) ((var) &= ~(1 << (bitnomber)))
#define getbit(var,bitnomber) (((var)>>(bitnomber))&1)
#define BitIsClear(var,bitnomber)  (!(var & (1<<bitnomber)))
#define BitIsSet(var,bitnomber)    (var & (1<<bitnomber))
//--------------------------------------------------------------
//����������� ����������� ���������� � �����
//� ������� ��������� � ������ ����� ����� �����

#define LcdPortDDR   TRISB
#define LcdPort   PORTB


#define LowNibble

#ifdef  LowNibble

//Для случая подключения ЖКИ к младшему полубайту порта 

//#define RS        PB6
#define RS          RB4
//#define RW        PB5
#define RW          RB7
//#define E         PB4
#define E           RB5
//#define D7        PB3
#define D7          RB3
//#define D6        PB2
#define D6          RB2
//#define D5        PB1
#define D5          RB1
//#define D4        PB0
#define D4          RB0


//#define NIBBLE   ((1<<D4)+(1<<D5)+(1<<D6)+(1<<D7))
//#define WRITE_HIGH_NIBBLE(var)  ( LcdPort = (LcdPort & (~NIBBLE)) | ((var)>>4) )
//#define WRITE_LOW_NIBBLE(var)  ( LcdPort = (LcdPort & (~NIBBLE)) | ((var) & (NIBBLE)))

#define WRITE_HIGH_NIBBLE(x)  ( LcdPort = (LcdPort & 0xF0) | ((x) >> 4) )
#define WRITE_LOW_NIBBLE(x)   ( LcdPort = (LcdPort & 0xF0) | ((x) & 0x0F) )



#else

//��� ������ ����������� ��� � �������� ��������� ����� 

#define D7        PB7
#define D6        PB6
#define D5        PB5
#define D4        PB4
#define RS        PB2
#define RW        PB1
#define E         PB0

//#define NIBBLE   ((1<<D4)+(1<<D5)+(1<<D6)+(1<<D7))
//#define WRITE_HIGH_NIBBLE(var) ( LcdPort = (LcdPort & (~NIBBLE)) | ((var) & (NIBBLE)))
//#define WRITE_LOW_NIBBLE(var)  ( LcdPort = (LcdPort & (~NIBBLE)) | ((var)<<4))

#define WRITE_HIGH_NIBBLE(x)  ( LcdPort = (LcdPort & 0x0F) | ((x) & 0xF0) )
#define WRITE_LOW_NIBBLE(x)   ( LcdPort = (LcdPort & 0x0F) | ((x) << 4) )

#endif

//������� ��������� � ������ ����� �����
/*
#define SET_D7        setbit(LcdPort,D7)
#define SET_D6        setbit(LcdPort,D6)
#define SET_D5        setbit(LcdPort,D5)
#define SET_D4        setbit(LcdPort,D4)
*/
#define SET_D7   D7 = 1;
#define SET_D6   D6 = 1;
#define SET_D5   D5 = 1;
#define SET_D4   D4 = 1;
//#define SET_RS        setbit(LcdPort,RS)
#define SET_RS          RS = 1
//#define CLR_RS        clrbit(LcdPort,RS)
#define CLR_RS          RS = 0
//#define SET_E         setbit(LcdPort,E)
#define SET_E         E = 1;
//#define CLR_E         clrbit(LcdPort,E)
#define CLR_E         E = 0;
//#define SET_RW        setbit(LcdPort,RW)
#define SET_RW        RW = 1;
//#define CLR_RW        clrbit(LcdPort,RW)
#define CLR_RW        RW = 0;
//#define CLR_ALL()    LcdPort &=~(_BV(RS)+_BV(RW)+_BV(E)+_BV(D4)+_BV(D5)+_BV(D6)+_BV(D7))
#define CLR_ALL()    LcdPort = 0x00;


//---------------------------------------------------------------------------
// ��������� ���������, ������������ � �������� ��������� ��������

//#define F_CPU 8000000 	// cpu frequancy ���� �� ������� ��� ����������, ��������������
//#define CYCLES_PER_US ((F_CPU+500000)/1000000) 	// cpu cycles per microsecond

#define MKS_PBYT 2 /* ����� �� �������� ��������� �� ������� */
#define MKS_BYTE 40 /* ����� ����� �������� ����� �� ������� */
#define StartDelay 40 //�������� ��� ������������ ����������� ������ �������

#ifdef DELAY
//�������� ������ �� ����������� ����������, ����� �� �������� ��� �����
//���� ��� ����� �������, �������� ��������������� ���� ���
//����������� �� ������� 
// delay for a minimum of <us> microseconds 
// the time resolution is dependent on the time the loop takes 
// e.g. with 4Mhz and 5 cycles per loop, the resolution is 1.25 us 
void __delay_us(unsigned short time_us) 
{
	unsigned short delay_loops;
	register unsigned short i;

	delay_loops = (time_us+3)/5*CYCLES_PER_US; // +3 for rounding up (dirty) 

	// one loop takes 5 cpu cycles 
	for (i=0; i < delay_loops; i++) {};
}

void __delay_ms(unsigned char time_ms)
{
	unsigned short delay_count = F_CPU / 4000;

	unsigned short cnt;
	asm volatile ("\n"
                  "L_dl1%=:\n\t"
                  "mov %A0, %A2\n\t"
                  "mov %B0, %B2\n"
                  "L_dl2%=:\n\t"
                  "sbiw %A0, 1\n\t"
                  "brne L_dl2%=\n\t"
                  "dec %1\n\t" "brne L_dl1%=\n\t":"=&w" (cnt)
                  :"r"(time_ms), "r"((unsigned short) (delay_count))
	);
}
#endif


//=======================================================
//Прототипы функций, взаимодествующих с индикатором 
//=======================================================
//низкоуровневые функции, не используемые CV Avr
//-------------------------------------------------------
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
//void lcd_putsf(char flash *str);

//*********************************************************
// Функции для ЖКИ
//*********************************************************

//=================================================================================
//Низкоуровневые функции управления индикатором, 
//которые отсутствуют в CV Avr
//=================================================================================

//--------------------------------------------------------
//Имя функции:    void PrintLcd(char value)
//Описание:       Вывод символа на экран ЖКИ 
//Вход:           ch -> ASCII код символа для вывода
//Выход:          нет
//----------------------------------------------------------


void PrintLcd(unsigned char value) 
{
	SET_RS;
	_lcd_write_data (value);
	CLR_RS;
	__delay_us(MKS_BYTE);
}


//-------------------------------------------------------------------------------
//Имя функции:    void enable_half_byte(void)
//Описание:       выдача сигнала "Е" на ЖКИ с укороченным временем
//                ожидания (между вводом полубайтов) 
//-------------------------------------------------------------------------------

void enable_half_byte (void)
{
	__delay_us(MKS_PBYT); 
	SET_E;
	__delay_us(MKS_PBYT);
	CLR_E;
	__delay_us(MKS_PBYT);		// ����� ����� �����������
}	

//-------------------------------------------------------------------------------
//Имя функции:    void enable (void)
//Описание:       выдача сигнала "Е" на ЖКИ с полным временем
//                ожидания (после выдачи всей команды)
//-------------------------------------------------------------------------------

void enable (void)
{
	__delay_us(MKS_PBYT); 
	SET_E;
	__delay_us(MKS_PBYT);
	CLR_E;
	__delay_us(MKS_BYTE);		//����� ����� ���������
}

//-------------------------------------------------------------------------------
//Низкоуровневые функции управления индикатором, 
//которые имеются в CV Avr
//-------------------------------------------------------------------------------
void _lcd_ready(void)
{

clrbit(LcdPortDDR,D7);        // D7 ��������� �� ����
SET_RW;
clrbit(LcdPort,D7);
CLR_RS;
__delay_us(MKS_PBYT);
SET_E;
while (BitIsSet(LcdPort,D7)) //�������� ����� ��������� LCD
{__delay_us(MKS_PBYT);}
CLR_E;
setbit(LcdPortDDR,D7);        // D7 ��������� �� �����
CLR_RW;
}
//-------------------------------------------------------------------------------
//Имя функции:    void _lcd_write_data(char value)
//Описание:       Запись байта команды в ЖКИ 
//-------------------------------------------------------------------------------
// nieużywana
void _lcd_write_data(unsigned char value)
//void _lcd_write_data(char value)
{
	__delay_us(MKS_PBYT);
	WRITE_HIGH_NIBBLE(value);
	enable_half_byte();
	WRITE_LOW_NIBBLE(value);
	enable();
  __delay_us(MKS_BYTE);
}
//-------------------------------------------------------------------------------
//Имя функции:  void lcd_write_byte(unsigned char addr, unsigned char data);
//Описание:     Запись байта в память знакогенератора CG
//              или данных DD ЖКИ
//-------------------------------------------------------------------------------

//writes a byte to the LCD display RAM.
void lcd_write_byte(unsigned char addr, unsigned char data)
{
	CLR_RS;
    CLR_RW;
	_lcd_write_data(addr|0x80);
	SET_RS;
	_lcd_write_data(data);
	CLR_RS;
}


//-------------------------------------------------------------------------------
//Имя функции:  void writeCGRAM(unsigned char addr, unsigned char data);
//Описание:     Запись байта в память знакогенератора CG ЖКИ
//              
//-------------------------------------------------------------------------------

//writes a byte to the LCD character generator.
void writeCGRAM(unsigned char addr, unsigned char data)
{
	CLR_RS;
    CLR_RW;
	_lcd_write_data(addr|0x40);
	SET_RS;
	_lcd_write_data(data);
	CLR_RS;
}




//===============================================================================
//Высокоуровневые функции обращения к индикаторру      //
//===============================================================================

//-------------------------------------------------------------------------------
//Имя функции:    void lcd_init(unsigned char lcd_columns)
//-------------------------------------------------------------------------------
//Описание:       Начальная инициализация ЖКИ 
//Самая первая функция, которую необходимо вызвать до вызова всех
//остальных высокоуровневых LCD функций в CV Avr.
//Реализована пока только двухстрочная версия на 16 поз. в строке.
//Курсор не отображается.
//Очищает дисплей,записывая в память данных DD код пробела (0x20) и устанавливает
//счетчик памяти данных в значение 0x00 (начало),
//что соответствует row=0, column=0
//--------------------------------------------------------------------------------
void lcd_init(void)
{
// ToDo LcdPortDDR |= _BV(RS)+_BV(RW)+_BV(E)+_BV(D4)+_BV(D5)+_BV(D6)+_BV(D7);
    TRISB = 0;
//0xff;// назначаем все линии порта B на выход
CLR_ALL();
// и устанавливаем на них низкий уровень
//_delay_ms(StartDelay); // ждем более 30 мс
__delay_ms(MKS_BYTE);
// Трижды устанавливаем 8-битный режим - начальная инициализация 
SET_D5; SET_D4;
enable();
enable();
enable();
CLR_ALL();
//LcdPort = 0x00;
SET_D5;
enable();
_lcd_write_data(0x28);//0b00101000 функция инициализации+4 разрядная шина+2строчное табло
                      //+матрица индицируемых знаков 5х8
ClrScr();             //очистка экрана
_lcd_write_data(0x06);//0b00000110 режим ввода данных - инкремент счетчика адреса на 1
                      //после записи в память DD/CG или чтения из нее.Активизируется 
					  //курсор и функция мерцания.
					  //Содержимое табло НЕ сдвигается вправо/влево
_lcd_write_data(0x0C);//0b00001100 функция настройки табло. выбран режим табло включено+
                      //курсор скрыт, символ над курсором не мерцает
}


//-------------------------------------------------------------------------------
//Имя функции:  void ClrScr(void)
//-------------------------------------------------------------------------------
//записывает в память данных DD код пробела (0x20) и устанавливает
//счетчик памяти данных в значение 0x00 (начало),
//что соответствует row=0, column=0
//-------------------------------------------------------------------------------
void ClrScr(void)
{
//_lcd_ready();
//_lcd_write_data(0xc); // ��������� �����,cursor off, �������� ����.
//_lcd_ready();
_lcd_write_data (0x01);//������� �����
__delay_ms(3);
}
//-------------------------------------------------------------------------------
//...............................................................................

//-------------------------------------------------------------------------------
//Имя:        void lcd_gotoxy(unsigned char x, unsigned char y)
//Описание:   Установка курсора в желаемую позицию на экране ЖКИ
//Вход:       X -> X позиция в строке на экране ЖКИ
//                   пределы изменения 0-39 для 2 строк
//                   выводиться на табло будут область 16 байт,
//                   установленная с помощью функции сдвига курсора.
//                   Когда курсор установлен в 0,
//                   то вывод с 0 байта в строке 0
//                   и 0x40 байта в строке 1 при двухстрочном индикаторе
//                   при 4 строчном х 20
//                   вывод  по строкам 0-0x13, 0x40-0x53,0x14-0x27,0x54-0x67
//            Y -> Y номер Строки на экране ЖКИ
//                пределы изменения 0,1,2,3
//то, что происходит описано неверно, на самом деле устанавливается адрес в памяти
//DD, куда пишется или откуда читается байт, но внешне выглядит как описано выше.
//-------------------------------------------------------------------------------
void lcd_gotoxy(unsigned char x, unsigned char y)
{
#ifdef _1602
unsigned char address=0;
if (y ==0)
address = x;
if (y ==1)
address = 0x40 + x;

_lcd_write_data(address | 0x80);

#endif
//...................................

#ifdef _2004
unsigned char address=0;
if (y ==0)
address = x;
if (y ==1)
address = 0x40 + x;
if (y==2)
address = 0x14 + x;
if (y==3)
address = 0x54 + x;

_lcd_write_data(address | 0x80);

#endif
//.............................
#ifdef _1604
unsigned char address=0;
if (y ==0)
address = x;
if (y ==1)
address = 0x40 + x;
if (y==2)
address = 0x10 + x;
if (y==3)
address = 0x50 + x;

_lcd_write_data(address | 0x80);

#endif
//............................................

}
//-------------------------------------------------------------------------------
//...............................................................................
//-------------------------------------------------------------------------------
//Имя:       void lcd_puts(char *str)
//Описание:     Вывод текстовой строки на ЖКИ-дисплей 
//Вход:         Укзатель на строку char *str в оперативной памяти
//-------------------------------------------------------------------------------
void lcd_puts(char *str)
{
unsigned char i;

#ifdef _1602
#define StrEnd 40
#endif


#ifdef _1604
#define StrEnd 16
#endif

#ifdef _2004
#define StrEnd 20
#endif





for (i=0;i<StrEnd;i++)
   {
   if (*str == 0x00) break;
   PrintLcd(*str);
   *str++ = 0x00;
   }
}
//-------------------------------------------------------------------------------
//...............................................................................
//-------------------------------------------------------------------------------
//Имя:       Lcd_Chr(uint8_t line,uint8_t column,char ch)
//Описание:     Вывод символа в требуемой позиции требуемой строки 
//Вход:         № строки, № позиции в строке,символ
//-------------------------------------------------------------------------------

void Lcd_Chr(unsigned char line,unsigned char column,char ch)
{
lcd_gotoxy(column,line);
PrintLcd(ch);
}



