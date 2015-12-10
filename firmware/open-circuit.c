//
// HEADER FILES
//
#include <system.h>
#include <rand.h>
#include <eeprom.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 8000000

//
// TYPE DEFS
//
typedef unsigned char byte;

//
// MACRO DEFS
//

#define P_LED		lata.2

#define P_SRDAT		latc.3
#define P_SRCLK		lata.5
#define P_SRLAT		lata.4


// MIDI beat clock messages
#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc


//
// GLOBAL DATA
//

// timer stuff

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 20
volatile byte rxBuffer[SZ_RXBUFFER];
volatile byte rxHead = 0;
volatile byte rxTail = 0;

volatile byte beatcount = 0;
void interrupt( void )
{
		
	// serial rx ISR
	if(pir1.5)
	{	
		// get the byte
		byte b = rcreg;

		if(b==0xf8) {
			if(++beatcount == 24) {
				P_LED = 1;
				beatcount = 0;
			}
			else {
				P_LED = 0;
			}			
		}
		 
/*		
		// calculate next buffer head
		byte nextHead = (rxHead + 1);
		if(nextHead >= SZ_RXBUFFER) 
		{
			nextHead -= SZ_RXBUFFER;
		}
		
		// if buffer is not full
		if(nextHead != rxTail)
		{
			// store the byte
			rxBuffer[rxHead] = b;
			rxHead = nextHead;
		}		*/
		pir1.5 = 0;
	}
	
}

////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void initUSART()
{
	pir1.1 = 1;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 0;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 15;		// brg low byte (31250)	
	
}

/*
////////////////////////////////////////////////////////////
// RUN MIDI THRU
void midiThru()
{
	// loop until there is no more data or
	// we receive a full message
	for(;;)
	{
		// buffer overrun error?
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
		// any data in the buffer?
		if(rxHead == rxTail)
		{
			// no data ready
			return;
		}
		
		// read the character out of buffer
		byte q = rxBuffer[rxTail];
		if(++rxTail >= SZ_RXBUFFER) 
			rxTail -= SZ_RXBUFFER;

		// Check for MIDI realtime message (e.g. clock)
		if((q & 0xF8) == 0xF8)
		{
			// if we are not passing realtime messages then skip it
			if(!(_options & OPTION_PASSREALTIMEMSG))
				continue;
		}
		else
		{
			// if we are not passing non-realtime messages then skip it
			if(!(_options & OPTION_PASSOTHERMSG))
				continue;
		}
		
		// should we animate the LEDs based on thru traffic?
		if(MODE_NOCLOCK == _mode && (_options & OPTION_THRUANIMATE))
		{
			// animate and send
			duty[q%6] = q%INITIAL_DUTY;
			send(q);
		}
		// should we indicate thru traffic with flickering LEDs?
		else 
		{					
			// flicker and send
			P_LED2 = 1;
			P_LED3 = 1;
			send(q);
			P_LED2 = 0;
			P_LED3 = 0;
		}
	}		
}
*/

////////////////////////////////////////////////////////////
// I2C MASTER
////////////////////////////////////////////////////////////
void i2cInit() {
	// disable output drivers on i2c pins
	trisc.0 = 1;
	trisc.1 = 1;
	
	//ssp1con1.7 = 
	//ssp1con1.6 = 
	ssp1con1.5 = 1; // Enable synchronous serial port
	ssp1con1.4 = 1; // Enable SCL
	ssp1con1.3 = 1; // }
	ssp1con1.2 = 0; // }
	ssp1con1.1 = 0; // }
	ssp1con1.0 = 0; // } I2C Master with clock = Fosc/(4(SSPxADD+1))
	
	ssp1stat.7 = 1;	// slew rate disabled	
	ssp1add = 19;	// 100kHz baud rate
}
void i2cWrite(byte data) {
	ssp1buf = data;
	while((ssp1con2 & 0b00011111) || // SEN, RSEN, PEN, RCEN or ACKEN
		(ssp1stat.2)); // data transmit in progress	
}
void i2cBeginWrite(byte address) {
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.0 = 1; // signal start condition
	while(!pir1.3); // wait for it to complete
	i2cWrite(address<<1); // address + WRITE(0) bit
}

void i2cEndMsg() {
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.2 = 1; // signal stop condition
	while(!pir1.3); // wait for it to complete
}

void sendDAC(int n0, int v0, int n1, int v1) {
  i2cBeginWrite(0b1100000);
  i2cWrite((n1>>8) & 0xF);
  i2cWrite(n1 & 0xFF);
  i2cWrite((v1>>8) & 0xF);
  i2cWrite(v1 & 0xFF);
  i2cWrite((v0>>8) & 0xF);
  i2cWrite(v0 & 0xFF);
  i2cWrite((n0>>8) & 0xF);
  i2cWrite(n0 & 0xFF);
  i2cEndMsg();
}

////////////////////////////////////////////////////////////
// COMPARATOR
////////////////////////////////////////////////////////////
void cmpInit() 
{
	cm1con0.7 = 1;	// C1ON enable
	//cm1con0.6 = 0;
	cm1con0.5 = 0;	// C1OE disabled
	//cm1con0.4 = 0;
	//cm1con0.3 = 0;
	cm1con0.2 = 0;	// C1SP low power mode
	cm1con0.1 = 0;	// C1HYS hysteresis disabled
	cm1con0.0 = 0;  // C1SYNC asynchronous
	
	cm1con1.7 = 0;  // C1INTP off
	cm1con1.6 = 0;  // C1INTN off
	cm1con1.5 = 1;  // }
	cm1con1.4 = 0;  // } C1PCH positive input is fixed voltage reference
	//cm1con1.3 = 0;
	//cm1con1.2 = 0;
	cm1con1.1 = 1; 	// }
	cm1con1.0 = 1;	// } C1VN connected to..
	
	
	fvrcon.7 = 1; // FVREN enable fixed voltage reference
	//fvrcon.6 = 0;
	//fvrcon.5 = 0;
	//fvrcon.4 = 0;
	fvrcon.3 = 1; // } 
	fvrcon.2 = 1; // } CDAFVR=10; FVR at 2.048V
	//fvrcon.1 = 0;
	//fvrcon.0 = 0;
}

void load_sr(byte d) {
	byte m = 0x80;
	P_SRLAT = 0;
	while(m) {
		P_SRCLK = 0;
		P_SRDAT = !!(d&m);
		P_SRCLK = 1;
		m>>=1;
	}
	P_SRLAT = 1;
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 	
	// osc control / 8MHz / internal
	osccon = 0b01110010;


	// configure io
	trisa = 0b00000010;              	
    trisc = 0b00110000;   	
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;

	// initialise MIDI comms
//	initUSART();

#define SRB_GATA	0x01
#define SRB_GATB	0x02
#define SRB_DRM1 	0x08
#define SRB_DRM2 	0x10
#define SRB_DRM3 	0x04
#define SRB_DRM4 	0x20
#define SRB_SYNC	0x40

i2cInit();
//cmpInit();
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	// App loop
	int i=0;
	for(;;)
	{	
		P_LED = !!(i&0x80); // comparator output
		load_sr((i&0x2)?0x01:0x00);
++i;
		sendDAC(0,i,0,0);
		i+=1;
		if(i>4095)
			i=0;
	//	P_LED = !!(cmout.0); // comparator output
		delay_ms(10);
	}

}



