//#include <Arduino.h>
//#include <inttypes.h>
#include "ModbusRtu.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <ModbusRtu.h>



#define F_OSC 16000000 // Clock frequency
#define BAUD_RATE 250000

#define MOSFET 1
#define RELAY 2
/*
Here you have to select the output mode accordingly to the receiver type you are using.
 Choose MOSFET or RELAY for OUTPUT_MODE
 */
//#define OUTPUT_MODE RELAY
#define OUTPUT_MODE MOSFET
#define THR 20 //threshold for analogRead

#if OUTPUT_MODE == RELAY

#define PWMpin4 6 // PWM OUT4
#define PWMpin1 10 // PWM OUT1
#define PWMpin2 9 // PWM OUT2
#define PWMpin3 5 // PWM OUT3

#else

#define PWMpin1 6 // PWM OUT1
#define PWMpin4 10 // PWM OUT4
#define PWMpin3 9 // PWM OUT3
#define PWMpin2 5 // PWM OUT2

#endif


// Define Function Prototypes that use User Types below here or use a .h file
//


// Define Functions below here or use other .ino or cpp files
//
#define DE 2
#define LED 21
#define SerialTxControl 3   //RS485 управляющий контакт на arduino pin 10
#define RS485Transmit    HIGH
#define RS485Receive     LOW 
#define ENBL HIGH
#define DISBL LOW

//---- здесь описываем все переменные нужные нам для работы с modbus ModbusRtu

uint16_t au16data[16]; //!< массив данных, используемый как буфер для обмена данными через modbus
uint16_t au16data_i[16];
uint8_t u8state; //!< состояние машины
uint8_t u8query; //!< указатель сообщение
unsigned long u32wait; //отслеживаем тайминги работы системы

/**
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  port : serial port
 *  u8txenpin : 0 for RS-232 and USB-FTDI 
 *               or any pin number > 1 for RS-485
 */
Modbus master(0,Serial2,4); // this is master and RS-232 or USB-FTDI
//Modbus master1(0,Serial2,4);
//Modbus master2(0,Serial2,4);
//Modbus master3(0,Serial2,4);
/**
 * This is an structe which contains a query to an slave device
 */
modbus_t telegram[5];
modbus_t telegram_i[5];

//---------------------------------------------------------------------------

enum {
	BREAK, STARTB, STARTADD, DATA
};

volatile unsigned int dmxStatus;
volatile unsigned int dmxStartAddress;
volatile unsigned int dmxCount = 0;
volatile unsigned int ch1, ch2, ch3, ch4;
volatile unsigned int channel[4]={0,0,0,0};
volatile unsigned int MASTER;
volatile int iChannel;
char dig[10]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
char sOut[6]={' ',' ',' ',' ',' ',' '};

void writemodbus1()
{

    telegram[1].u8id = 1; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 49999; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	au16data[0] = 1148;
	//au16data[9] = map(channel[iChannel-1] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

    telegram[1].u8id = 1; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 50009; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	//au16data[0] = 1148;
	au16data[0] = map(channel[0] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[0]  );
	//Serial.write(dig);
   //}
}

void writemodbus2()
{

    telegram[1].u8id = 2; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 49999; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	au16data[0] = 1148;
	//au16data[9] = map(channel[iChannel-1] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

    telegram[1].u8id = 2; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 50009; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	//au16data[0] = 1148;
	au16data[0] = map(channel[1] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[1]  );
	//Serial.write(dig);
   //}
}

void writemodbus3()
{

    telegram[1].u8id = 3; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 49999; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	au16data[0] = 1148;
	//au16data[9] = map(channel[iChannel-1] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

    telegram[1].u8id = 3; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 50009; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	//au16data[0] = 1148;
	au16data[0] = map(channel[2] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[2]  );
	//Serial.write(dig);
   //}
}

void writemodbus4()
{

    telegram[1].u8id = 4; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 49999; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	au16data[0] = 1148;
	//au16data[9] = map(channel[iChannel-1] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

    telegram[1].u8id = 4; // slave address
	telegram[1].u8fct = 6;
	telegram[1].u16RegAdd = 50009; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data;
	//au16data[0] = 1148;
	au16data[0] = map(channel[3] ,0,255, 0,16384);
	master.query(telegram[1]);
	master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[3]  );
	//Serial.write(dig);
   //}
}

void startDrive(int iNum)
{
    telegram_i[iNum].u8id = iNum; // slave address
	telegram_i[iNum].u8fct = 6;
	telegram_i[iNum].u16RegAdd = 49999; // start address in slave
    telegram_i[iNum].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram_i[iNum].au16reg = au16data_i;
	au16data_i[0] = 1148;
	//master.query(telegram_i[0]);
	//delay(100);
	//master.poll();
    //Serial.println("Start DRIVE!");
}


void writemodbus(int iC)
{

	//startDrive(iC);


    telegram[iC].u8id = iC; // slave address
	telegram[iC].u8fct = 6;
	telegram[iC].u16RegAdd = 50009; // start address in slave
    telegram[iC].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[iC].au16reg = au16data+iC-1;
	//au16data[0] = 1148;
	au16data[iC-1] = map(channel[iC-1] ,0,255, 0,16384);
	//master.query(telegram[1]);
	//master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[iC-1]  );
	//Serial.write(dig);
   //}
}


void writemodbus_m(int iC)
{

	//startDrive(iC);


    telegram[iC].u8id = iC; // slave address
	telegram[iC].u8fct = 16;
	telegram[iC].u16RegAdd = 49999; // start address in slave
    telegram[iC].u16CoilsNo = 11; // number of elements (coils or registers) to read
    telegram[iC].au16reg = au16data;
	au16data[0] = 1148;
	au16data[10] = map(channel[iC-1] ,0,255, 0,16384);
	//master.query(telegram[1]);
	//master.poll();

   //if (iC == 1 )
   //{
	//sprintf(dig, "%d %d\n",iC, au16data[0], channel[iC-1]  );
	//Serial.write(dig);
   //}
}


/*Инициализируем USART*/
void init_USART()
{
	UBRR1L = (uint8_t)(F_CPU / (BAUD_RATE * 16L) - 1); //устанавливаем скорость  250 kbit/s
	UBRR1H = (F_CPU / (BAUD_RATE * 16L) - 1) >> 8;     // 
	UDR1 = 0;
	UCSR1A = 0;	                            // отчищаем флажки ошибок, запрещаем U2X и MPCM
	UCSR1B = (1 << RXCIE1) | (1 << RXEN1);	// Включаем на прием
	UCSR1C = (1 << USBS1) | (3 << UCSZ10);	// 8bit 2 stop
}






//UART обработка прерывания приема данных
SIGNAL(USART1_RX_vect)
{
	int temp = UCSR1A;  //получаем байт состояния
	int dmxByte = UDR1; //получаем данные из регистра данных 
    //char bDMX[4];
	//sprintf(bDMX,"%d\n", dmxByte);
	digitalWrite(LED, HIGH);
    //Serial.write("start int ");
    //Serial.write(bDMX);
	
	if (temp&(1 << DOR1))	// Data Overrun?
	{
		dmxStatus = BREAK;	// wait for reset (BREAK)
		UCSR1A &= ~(1 << DOR1);
		goto tail;
	}

	if (temp&(1 << FE1))	//BREAK or FramingError?
	{

		dmxCount = 0;	    // сбрасываем счетчик данных
		dmxStatus = STARTB;	// считаем, что пришел сигнал BREAK ;-) ->ждем стартовый байт
		UCSR1A &= ~(1 << FE1);
		goto tail;
	}


	switch (dmxStatus)
	{
	case STARTB:

		if (dmxByte == 0)  //This is our star byte
		{

			if (dmxStartAddress == 1) dmxStatus = DATA; // the FE WAS a BREAK -> the next byte is our first channel
			else dmxStatus = STARTADD;	// the FE WAS a BREAK -> wait for the right channel
			dmxCount = 1;


		}
		else
		{
			dmxStatus = BREAK;	// wait for reset (BREAK) it was a framing error
		}
		goto tail;
		break;

	case STARTADD:

		if (dmxCount == dmxStartAddress - 1) //Is the next byte channel one?
		{
			dmxStatus = DATA; //Yes, so let's wait the data
		}
		dmxCount++;

		break;


	case DATA:	// HERE YOU SHOULD PROCESS THE CHOSEN DMX CHANNELS!!!
	    
		if (dmxCount == dmxStartAddress)
		{
			ch1 = dmxByte;
			channel[0] = ch1;
			iChannel = 1;
		    //sprintf(sOut, "%d#%d#\n", iChannel, dmxByte );
	        //Serial.write(sOut);
            //writemodbus(iChannel);
			dmxCount++;
		}
		else if (dmxCount == (dmxStartAddress + 1))
		{
			ch2 = dmxByte;
			channel[1] = ch2;
			iChannel = 2;
            //writemodbus(iChannel);
			dmxCount++;
		    //sprintf(sOut, "%d#%d#\n", iChannel, dmxByte );
	        //Serial.write(sOut);

		}
		else if (dmxCount == (dmxStartAddress + 2))
		{

			ch3 = dmxByte;
			channel[2] = ch3;
			iChannel = 3;
            //writemodbus(iChannel);
			dmxCount++;
		    //sprintf(sOut, "%d#%d#\n", iChannel, dmxByte );
	        //Serial.write(sOut);


		}
		else  if (dmxCount == (dmxStartAddress + 3))
		{
			ch4 = dmxByte;
			channel[3] = ch4;
			iChannel = 4;
            //writemodbus(iChannel);
			dmxCount = 1;
			dmxStatus = BREAK;	// ALL CHANNELS RECEIVED

			analogWrite(PWMpin1, channel[0]);	// update mosfet outputs
			analogWrite(PWMpin2, channel[1]);
			analogWrite(PWMpin3, channel[2]);
			analogWrite(PWMpin4, channel[3]);
			startDrive(1);
			writemodbus(1);
            startDrive(2);
			writemodbus(2);
			startDrive(3);
			writemodbus(3);
			startDrive(4);
			writemodbus(4);

		    sprintf(sOut, "%d#%d#%d#%d\n", channel[0], channel[1], channel[2], channel[3] );
	        Serial.write(sOut);

            
		}

	}




tail:
	asm("nop");
}




//test demo funtion
void demo()
{
	int bright;
	if (OUTPUT_MODE == MOSFET)
	{
		for (;;)
		{

			for (bright = 0; bright < 255; bright++)	// infinite loop
			{
				analogWrite(PWMpin1, bright);
				analogWrite(PWMpin2, bright);
				analogWrite(PWMpin3, bright);
				analogWrite(PWMpin4, bright);
				delay(10);
			}

			for (bright = 255; bright >= 0; bright--)	// infinite loop
			{
				analogWrite(PWMpin1, bright);
				analogWrite(PWMpin2, bright);
				analogWrite(PWMpin3, bright);
				analogWrite(PWMpin4, bright);
				delay(10);
			}

			analogWrite(PWMpin1, 255);
			analogWrite(PWMpin3, 255);
			delay(10);
			analogWrite(PWMpin1, 0);
			analogWrite(PWMpin3, 0);
			analogWrite(PWMpin2, 255);
			analogWrite(PWMpin4, 255);
			delay(10);
		}
	}
	else if (OUTPUT_MODE == RELAY)
	{
		for (;;)
		{
			analogWrite(PWMpin1, 255);
			analogWrite(PWMpin2, 0);
			analogWrite(PWMpin3, 0);
			analogWrite(PWMpin4, 0);
			delay(1000);
			analogWrite(PWMpin1, 0);
			analogWrite(PWMpin2, 255);
			analogWrite(PWMpin3, 0);
			analogWrite(PWMpin4, 0);

			delay(1000);
			analogWrite(PWMpin1, 0);
			analogWrite(PWMpin2, 0);
			analogWrite(PWMpin3, 255);
			analogWrite(PWMpin4, 0);

			delay(1000);
			analogWrite(PWMpin1, 0);
			analogWrite(PWMpin2, 0);
			analogWrite(PWMpin3, 0);
			analogWrite(PWMpin4, 255);
			delay(1000);
		}
	}
}


void setup() {
  // put your setup code here, to run once:
	//инициализируем инфо диоды
	pinMode(DE, OUTPUT); // enable Tx Rx
	pinMode(LED, OUTPUT); // led DMX
	pinMode(SerialTxControl, OUTPUT);
	digitalWrite(SerialTxControl, RS485Receive);
	//start DMX receive
	init_USART();
	//start modbus here
	Serial2.begin(38400, SERIAL_8E1);
	master.setTimeOut(50);
//	master1.setTimeOut(500);
//	master2.setTimeOut(1000);
//	master3.setTimeOut(1500);
//	master3.setTimeOut(50);
	master.start();
//	master1.start();
//	master2.start();
//	master3.start();
//	//master.setTimeOut( 5000 );
	//master.begin(19200);

	Serial.begin(9600);

    u32wait = millis() + 500;
    u8state  = 0;
	u8query  = 1; 	

}

//loop() - основной цикл прораммы
void loop() {
  // put your main code here, to run repeatedly:
  	volatile unsigned int address1, address2, address3, address4, address5, address6, address7, address8, address9;
	//Serial.write("send telegram1 \n");
	cli(); //disable interrupt
	//переменный для установки адреса с которого следует слушать DMX поток
	//по умолчанию равен 1, если адрес равен 0, запускается функция demo()
	address1 = 9;
	address2 = 0;
	address3 = 0;
	address4 = 0;
	address5 = 0;
	address6 = 0;
	address7 = 0;
	address8 = 0;
	address9 = 0;
	//start to get DMX flow begin from 1 
	/*Вычисление dmxStartAddress*/
	dmxStartAddress = (address1 * 1) + (address2 * 2) + (address3 * 4) + (address4 * 8) + (address5 * 16) + (address6 * 32) + (address7 * 64) + (address8 * 128) + (address9 * 256);

	if (dmxStartAddress == 0) { //If all dipswitches are 0 
		demo();                //call the demo function
	}

	else if (dmxStartAddress >= 509)  //The receiver manages 4 channels, so you can't set a start address above 509;
		dmxStartAddress = 509;

	digitalWrite(DE, LOW);


	sei(); //enable global interrupt
    Serial.write("send telegram2 \n");
    //telegram[0].u8id = 1;                 // slave address
    //telegram[0].u8fct = 3;                // function code (registers read multiple  3) 
    //telegram[0].u16RegAdd = 16130;            // start address in slave -  direccion de Inicio 0
    //telegram[0].u16CoilsNo = 11;          // number of elements (coils or registers) to read  0 - 16 
    //telegram[0].au16reg = au16data;       // pointer to a memory array in the Arduino - Almacenamiento en Array de memoria de arduino
    //master.query(telegram[0]);
	//master.poll();


  /*  
	telegram[1].u8id = 1; // slave address
    telegram[1].u8fct = 6; // function code (this one is write a single register)
    telegram[1].u16RegAdd = 49999; // start address in slave
    telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[1].au16reg = au16data; // pointer to a memory array in the Arduino
    //Даем команду ПУСК
	au16data[0] = 1148;
	//master.query(telegram[1]);
	//master.poll();

	telegram[2].u8id = 2; // slave address
    telegram[2].u8fct = 6; // function code (this one is write a single register)
    telegram[2].u16RegAdd = 49999; // start address in slave
    telegram[2].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[2].au16reg = au16data+1; // pointer to a memory array in the Arduino
    //Даем команду ПУСК
	au16data[1] = 1148;
	//master.query(telegram[2]);
	//master.poll();

	telegram[3].u8id = 3; // slave address
    telegram[3].u8fct = 6; // function code (this one is write a single register)
    telegram[3].u16RegAdd = 49999; // start address in slave
    telegram[3].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[3].au16reg = au16data+2; // pointer to a memory array in the Arduino
    //Даем команду ПУСК
	au16data[2] = 1148;
	//master.query(telegram[3]);
	//master.poll();

	telegram[4].u8id = 4; // slave address
    telegram[4].u8fct = 6; // function code (this one is write a single register)
    telegram[4].u16RegAdd = 49999; // start address in slave
    telegram[4].u16CoilsNo = 1; // number of elements (coils or registers) to read
    telegram[4].au16reg = au16data+3; // pointer to a memory array in the Arduino
    //Даем команду ПУСК
	au16data[3] = 1148;
	//master.query(telegram[4]);
	//master.poll();
    //int i=1;
	//int can_write = 1;
    //while(1)
	//{
    //    if ((i<5) && can_write)  {master.query(telegram[i]); can_write=0;}
    //    Serial.write('init FC\n');
	//	master.poll();
	//	if (master.getState() == COM_IDLE) {i++; can_write = 1;}
	//	if (i==5) goto next_step;
	//}

	

    startDrive(1);
    startDrive(2);
    startDrive(3);
    startDrive(4);
  */

    startDrive(1);
    startDrive(2);
    startDrive(3);
    startDrive(4);
	int stage = 1;
    int initSys = 1;

    while(initSys)
	{
			if (u8state == 0)
               if (millis() > u32wait) u8state++; // обрабатываем по времени
            
            if (u8state == 1)
			{
			   if (stage == 1) master.query( telegram_i[u8query]);
               if (stage == 2) master.query( telegram[u8query] ); // отправляем запрос частотнику , только один раз!
               u8state++;
	           u8query++;
	           if (u8query > 4) u8query = 1;

			}


			if (u8state ==2 ) 
			{
			  //проверка состояния линии, занята или нет	
			  master.poll();
			  if (master.getState() == COM_IDLE) 
		 	  {
			    //Serial.write("CHECK STATE MODBUS1 \n");
                u8state = 0;
				stage = 2; //в регистр состояния записно разрешение на запуск, можно работать
                u32wait = millis() + 500; 
                initSys = 0;            
			  }
			}
        

	}
   
    Serial.write("start cycle\n");
	for (;;) {
			//telegram[1].u8id = 1; // slave address
            //telegram[1].u8fct = 6; // function code (this one is write a single register)
            //telegram[1].u16RegAdd = 1; //50000; // start address in slave
            //telegram[1].u16CoilsNo = 1; // number of elements (coils or registers) to read
            //telegram[1].au16reg = au16data; // pointer to a memory array in the Arduino
            //Даем команду ПУСК
			//au16data[0] = 1148;
			//master.query(telegram[1]);

			if (u8state == 0)
               if (millis() > u32wait) u8state++; // обрабатываем по времени
            
            if (u8state == 1)
			{
			   if (stage == 1) master.query( telegram_i[u8query]);
               if (stage == 2) master.query( telegram[u8query] ); // отправляем запрос частотнику , только один раз!
               u8state++;
	           u8query++;
	           if (u8query > 4) u8query = 1;

			}


			if (u8state ==2 ) 
			{
			  //проверка состояния линии, занята или нет	
			  master.poll();
			  if (master.getState() == COM_IDLE) 
		 	  {
			     //	Serial.write("CHECK STATE1 \n");/* code */
                u8state = 0;
				stage = 2; //в регистр состояния записно разрешение на запускБ можно работать
                u32wait = millis() + 500; 
              
			  }

			}
			//записываем частоту вращения!

	}

}


