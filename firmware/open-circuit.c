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

#define P_LED1		lata.2
#define P_LED2		latc.2

#define P_SRDAT		latc.3
#define P_SRCLK		lata.5
#define P_SRLAT		lata.4


// MIDI beat clock messages
#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc

#define SRB_GATA	0x01
#define SRB_GATB	0x02
#define SRB_DRM1 	0x08
#define SRB_DRM2 	0x10
#define SRB_DRM3 	0x04
#define SRB_DRM4 	0x20
#define SRB_SYNC	0x40

#define GATE_A		0
#define GATE_B		1
#define GATE_DRM1	2
#define GATE_DRM2	3
#define GATE_DRM3	4
#define GATE_DRM4	5
#define GATE_SYNC	6

#define CHANNEL_A 	0
#define CHANNEL_B 	1
#define CHANNEL_DRUMS 	9

#define NOTE_DRUM1	0x3c
#define NOTE_DRUM2	0x3e
#define NOTE_DRUM3	0x40
#define NOTE_DRUM4	0x41

#define VEL_ACCENT 100

#define TRIG_PULSE 5 //ms
#define LED_TIME 10 //ms
//
// GLOBAL DATA
//

#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 20
volatile byte rxBuffer[SZ_RXBUFFER];
volatile byte rxHead = 0;
volatile byte rxTail = 0;

volatile byte beatcount = 0;


// state variables
byte srData = 0;
byte midiInRunningStatus;
byte midiNumParams;
byte midiParams[2];
char midiParamIndex;

char gateTime[8] = {0};
char ledTime = 0;

int cvPitchA = 0;
int cvPitchB = 0;
int cvVolA = 0;
int cvVolB = 0;

volatile byte msTick = 0;
void interrupt( void )
{
	// TIMER0 OVERFLOW
	// Timer 0 overflow is used to 
	// create a once per millisecond
	// count
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		msTick = 1;
		intcon.2 = 0;		
	}		
	// serial rx ISR
	if(pir1.5)
	{	
		// get the byte
		byte b = rcreg;

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
		}		
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

////////////////////////////////////////////////////////////
// RUN MIDI IN
byte midiIn()
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
			return 0;
		}
		
		// read the character out of buffer
		byte ch = rxBuffer[rxTail];
		if(++rxTail >= SZ_RXBUFFER) 
			rxTail -= SZ_RXBUFFER;

		// REALTIME MESSAGE
		if((ch & 0xf0) == 0xf0)
		{
			switch(ch)
			{
			case MIDI_SYNCH_TICK:
			case MIDI_SYNCH_START:
			case MIDI_SYNCH_CONTINUE:
			case MIDI_SYNCH_STOP:
			break;
			}
		}      
		// CHANNEL STATUS MESSAGE
		else if(!!(ch & 0x80))
		{
			midiParamIndex = 0;
			midiInRunningStatus = ch; 
			switch(ch & 0xF0)
			{
			case 0xA0: //  Aftertouch  1  key  touch  
			case 0xC0: //  Patch change  1  instrument #   
			case 0xD0: //  Channel Pressure  1  pressure  
				midiNumParams = 1;
				break;    
			case 0x80: //  Note-off  2  key  velocity  
			case 0x90: //  Note-on  2  key  veolcity  
			case 0xB0: //  Continuous controller  2  controller #  controller value  
			case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
			default:
				midiNumParams = 2;
				break;        
			}
		}    
		else if(midiInRunningStatus)
		{
			// gathering parameters
			midiParams[midiParamIndex++] = ch;
			if(midiParamIndex >= midiNumParams)
			{
				midiParamIndex = 0;
				switch(midiInRunningStatus & 0xF0)
				{
				case 0x80: // note off
				case 0x90: // note on
					return midiInRunningStatus; // return to the arp engine
				default:
					break;
				}
			}
		}
	}
	return 0;
}

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

void startNote(byte which, byte note, byte vel) {
	long dacNote = ((long)note * 500)/12;
	long dacVol = ((long)vel * 2500)/127;
	if(dacNote < 0) dacNote = 0;
	if(dacNote > 4095) dacNote = 4095;
	if(dacVol< 0) dacVol = 0;
	if(dacVol > 4095) dacVol = 4095;
	if(which == 1) {
		cvPitchB = dacNote;
		cvVolB = dacVol;
		srData |= SRB_GATB;
		load_sr(srData);
	}
	else {
		cvPitchA = dacNote;
		cvVolA = dacVol;
		srData |= SRB_GATA;
		load_sr(srData);
	}
	sendDAC(cvPitchA, cvVolA, cvPitchB, cvVolB);
}

void stopNote(byte which) {
	if(which == 1) {
		cvVolB = 0;
		srData &= ~SRB_GATB;
		load_sr(srData);
	}
	else {
		cvVolA = 0;
		srData &= ~SRB_GATA;
		load_sr(srData);
	}
	sendDAC(cvPitchA, cvVolA, cvPitchB, cvVolB);
}

void initTimer0() {
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 2MHz
	// 	prescaled 1/8 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 0; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag	
}

void trigPulse(int which, int ms) {
	byte d = srData;
	switch(which) {
		case GATE_A: 	srData |= SRB_GATA; break;
		case GATE_B: 	srData |= SRB_GATB; break;
		case GATE_DRM1: srData |= SRB_DRM1; break;
		case GATE_DRM2: srData |= SRB_DRM2; break;
		case GATE_DRM3: srData |= SRB_DRM3; break;
		case GATE_DRM4: srData |= SRB_DRM4; break;
		case GATE_SYNC: srData |= SRB_SYNC; break;
		default: return;
	}	
	if(d != srData) {
		load_sr(srData);
	}
	gateTime[which] = ms;
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
	initUSART();
initTimer0();

i2cInit();
//cmpInit();
load_sr(0);
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	// App loop
	int i=0;
	for(;;)
	{
		if(msTick) {
			byte d = srData;
			for(int g = 0; g<8; ++g) {
				if(gateTime[g]) {
					if(--gateTime[g] == 0) {
						switch(g) {
							case GATE_A: srData &= ~SRB_GATA; break;
							case GATE_B: srData &= ~SRB_GATB; break;
							case GATE_DRM1: srData &= ~SRB_DRM1; break;
							case GATE_DRM2: srData &= ~SRB_DRM2; break;
							case GATE_DRM3: srData &= ~SRB_DRM3; break;
							case GATE_DRM4: srData &= ~SRB_DRM4; break;
							case GATE_SYNC: srData &= ~SRB_SYNC; break;
						}
					}
				}
			}
			if(d != srData) {
				load_sr(srData);
			}
			if(ledTime && !--ledTime) {
				P_LED1 = 0;
			}
			msTick = 0;
		}
		
		
		byte msg = midiIn();		
		switch(msg & 0xF0) {
			case 0x90:			
				P_LED1 = 1;
				ledTime = LED_TIME;
			case 0x80:
				if((msg & 0x0F) == CHANNEL_A) {
					if(midiParams[1] && (msg & 0xF0) == 0x90) {
						startNote(0, midiParams[0], midiParams[1]);							
					}
					else {
						stopNote(0);							
					}
				}
				else
				if((msg & 0x0F) == CHANNEL_B) {
					if(midiParams[1] && (msg & 0xF0) == 0x90) {
						startNote(1, midiParams[0], midiParams[1]);							
					}
					else {
						stopNote(1);							
					}
				}
				else
				if((msg & 0x0F) == CHANNEL_DRUMS) {
					if(midiParams[1] && (msg & 0xF0) == 0x90) {					
						switch(midiParams[0]) {
							case NOTE_DRUM1: trigPulse(GATE_DRM1, TRIG_PULSE); break;
							case NOTE_DRUM2: trigPulse(GATE_DRM2, TRIG_PULSE); break;
							case NOTE_DRUM3: trigPulse(GATE_DRM3, TRIG_PULSE); break;
							case NOTE_DRUM4: trigPulse(GATE_DRM4, TRIG_PULSE); break;
						}
						if(midiParams[1] >= VEL_ACCENT) {
							trigPulse(GATE_SYNC, TRIG_PULSE); 
						}
						break;
					}
				}
				break;
				
				
				
				if(midiParams[1] > 0) {
					switch(midiParams[0]) {
						case NOTE_DRUM1:
						case NOTE_DRUM2:
						case NOTE_DRUM3:
						case NOTE_DRUM4:
							break;
						default:
							startNote(0, midiParams[0], midiParams[1]);							
							break;
					}
					break;
				}
		}
		
		/*
		*/
		
		//i+=1;
		//if(i>4095)
//			i=0;
	//	P_LED = !!(cmout.0); // comparator output
	}

}



