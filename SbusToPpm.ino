// ServoDecodeTest

#include <avr/eeprom.h>

// SERIAL_MODE options NP57600 and EP100K
//#define SERIAL_MODE_NP57600
#define SERIAL_MODE_EP100K

//#define	DEBUG	1

#define FAILSAFE_TIMEOUT_MS     500

// Hardware pin mapping
#define DDR_IO2			DDRD
#define PORT_IO2		PORTD
#define PIN_IO2			PIND
#define BIT_IO2			2
#define DDR_IO3			DDRD
#define PORT_IO3		PORTD
#define PIN_IO3			PIND
#define BIT_IO3			3
#define DDR_IO4			DDRD
#define PORT_IO4		PORTD
#define PIN_IO4			PIND
#define BIT_IO4			4
#define DDR_IO5			DDRD
#define PORT_IO5		PORTD
#define PIN_IO5			PIND
#define BIT_IO5			5
#define DDR_IO6			DDRD
#define PORT_IO6		PORTD
#define PIN_IO6			PIND
#define BIT_IO6			6
#define DDR_IO7			DDRD
#define PORT_IO7		PORTD
#define PIN_IO7			PIND
#define BIT_IO7			7

#define DDR_IO8			DDRB
#define PORT_IO8		PORTB
#define PIN_IO8			PINB
#define BIT_IO8			0
#define DDR_IO9			DDRB
#define PORT_IO9		PORTB
#define PIN_IO9			PINB
#define BIT_IO9			1
#define DDR_IO10		DDRB
#define PORT_IO10		PORTB
#define PIN_IO10		PINB
#define BIT_IO10		2
#define DDR_IO11		DDRB
#define PORT_IO11		PORTB
#define PIN_IO11		PINB
#define BIT_IO11		3
#define DDR_IO12		DDRB
#define PORT_IO12		PORTB
#define PIN_IO12		PINB
#define BIT_IO12		4
#define DDR_IO13		DDRB
#define PORT_IO13		PORTB
#define PIN_IO13		PINB
#define BIT_IO13		5

#define DDR_ADC0		DDRC
#define PORT_ADC0		PORTC
#define PIN_ADC0		PINC
#define BIT_ADC0		0
#define DDR_ADC1		DDRC
#define PORT_ADC1		PORTC
#define PIN_ADC1		PINC
#define BIT_ADC1		1
#define DDR_ADC2		DDRC
#define PORT_ADC2		PORTC
#define PIN_ADC2		PINC
#define BIT_ADC2		2
#define DDR_ADC3		DDRC
#define PORT_ADC3		PORTC
#define PIN_ADC3		PINC
#define BIT_ADC3		3
#define DDR_ADC4		DDRC
#define PORT_ADC4		PORTC
#define PIN_ADC4		PINC
#define BIT_ADC4		4
#define DDR_ADC5		DDRC
#define PORT_ADC5		PORTC
#define PIN_ADC5		PINC
#define BIT_ADC5		5

#define NUMBER_CHANNELS	16

uint8_t *Ports[NUMBER_CHANNELS] = {
	(uint8_t *)&PORT_IO2,
	(uint8_t *)&PORT_IO3,
	(uint8_t *)&PORT_IO4,
	(uint8_t *)&PORT_IO5,
	(uint8_t *)&PORT_IO6,
	(uint8_t *)&PORT_IO7,
	(uint8_t *)&PORT_IO8,
	(uint8_t *)&PORT_IO9,
	(uint8_t *)&PORT_IO10,
	(uint8_t *)&PORT_IO11,
	(uint8_t *)&PORT_IO12,
	(uint8_t *)&PORT_IO13,
	(uint8_t *)&PORT_ADC0,
	(uint8_t *)&PORT_ADC1,
	(uint8_t *)&PORT_ADC2,
	(uint8_t *)&PORT_ADC3
} ;

uint8_t Bits[NUMBER_CHANNELS] =
{
	(1<<BIT_IO2),
	(1<<BIT_IO3),
	(1<<BIT_IO4),
	(1<<BIT_IO5),
	(1<<BIT_IO6),
	(1<<BIT_IO7),
	(1<<BIT_IO8),
	(1<<BIT_IO9),
	(1<<BIT_IO10),
	(1<<BIT_IO11),
	(1<<BIT_IO12),
	(1<<BIT_IO13),
	(1<<BIT_ADC0),
	(1<<BIT_ADC1),
	(1<<BIT_ADC2),
	(1<<BIT_ADC3)
} ;

void set_outputs()
{
	DDR_IO2 |= (1<<BIT_IO2) ;
	DDR_IO3 |= (1<<BIT_IO3) ;
	DDR_IO4 |= (1<<BIT_IO4) ;
	DDR_IO5 |= (1<<BIT_IO5) ;
	DDR_IO6 |= (1<<BIT_IO6) ;
	DDR_IO7 |= (1<<BIT_IO7) ;
	DDR_IO8 |= (1<<BIT_IO8) ;
	DDR_IO9 |= (1<<BIT_IO9) ;
	DDR_IO10 |= (1<<BIT_IO10) ;
	DDR_IO11 |= (1<<BIT_IO11) ;
	DDR_IO12 |= (1<<BIT_IO12) ;
	DDR_IO13 |= (1<<BIT_IO13) ;
	DDR_ADC0 |= (1<<BIT_ADC0) ;
	DDR_ADC1 |= (1<<BIT_ADC1) ;
	DDR_ADC2 |= (1<<BIT_ADC2) ;
	DDR_ADC3 |= (1<<BIT_ADC3) ;
}


// Times for 57600 baud
#if F_CPU == 20000000L   // 20MHz clock 
//  #define TICKS2COUNT         348  // Ticks between two bits.
  #define TICKS2WAITONE       348  // Wait one bit period.
  #define TICKS2WAITONE_HALF  520    // Wait one and a half bit period.
#elif F_CPU == 16000000L  // 16MHz clock                                                  
//  #define TICKS2COUNT57         278  // Ticks between two bits.
  #define TICKS2WAITONE57       277  // Wait one bit period.
  #define TICKS2WAITONE_HALF57  415    // Wait one and a half bit period.
//  #define TICKS2COUNT100         160  // Ticks between two bits.
  #define TICKS2WAITONE100       160  // Wait one bit period.
  #define TICKS2WAITONE_HALF100  240  // Wait one and a half bit period.
#elif F_CPU == 8000000L   // 8MHz clock
  // Assumes a 8MHz clock                                                   
//  #define TICKS2COUNT         139  // Ticks between two bits.
  #define TICKS2WAITONE       139  // Wait one bit period.
  #define TICKS2WAITONE_HALF  208    // Wait one and a half bit period.
#else
  #error Unsupported clock speed
#endif

#if F_CPU == 20000000L     // 20MHz clock                                                  
  #define INTERRUPT_EXEC_CYCL   112      // Cycles to execute interrupt routines from interrupt.
  #define INTERRUPT_EARLY_BIAS  40       // Cycles to allow of other interrupts.

#elif F_CPU == 16000000L   // 16MHz clock                                                  
  #define INTERRUPT_EXEC_CYCL   100 //90       // Cycles to execute interrupt routines from interrupt.
  #define INTERRUPT_EARLY_BIAS  32       // Cycles to allow of other interrupts.

#elif F_CPU == 8000000L    // 8MHz clock                                                   
  #define INTERRUPT_EXEC_CYCL   45       // Cycles to execute interrupt routines from interrupt.
  #define INTERRUPT_EARLY_BIAS  16       // Cycles to allow of other interrupts.

#else
  #error Unsupported clock speed
#endif


#define TRXDDR  DDRD
#define TRXPORT PORTD
#define TRXPIN  PIND


#define ENABLE_TIMER_INTERRUPT( )       ( TIMSK1 |= ( 1<< OCIE1A ) )
#define DISABLE_TIMER_INTERRUPT( )      ( TIMSK1 &= ~( 1<< OCIE1A ) )
#define CLEAR_TIMER_INTERRUPT( )        ( TIFR1 = (1 << OCF1A) )

#define	ONE_TO_FOUR			0
#define	FIVE_TO_EIGHT		1
#define	NINE_TO_TWELVE	2
#define	THIRTEEN_TO_SIXTEEN	3
#define	END_PULSES			4

// UART's state.
#define   IDLE				0       // Idle state, both transmit and receive possible.
#define 	PULSING			1				// PPM pulses
//#define   RECEIVE     2       // Receiving byte.
//#define	  WAITING     3

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))

uint32_t micros() ;
void setPulseTimes( uint8_t Five2_8 ) ;

uint16_t FailsafeTimes[NUMBER_CHANNELS] ;
uint16_t PulseTimes[NUMBER_CHANNELS] ;

static volatile uint8_t State = IDLE ;     //!< Holds the state of the UART.
static volatile uint8_t Parity ;
static volatile uint8_t SwUartRXData;     //!< Storage for received bits.
static volatile unsigned char SwUartRXBitCount; //!< RX bit counter.

uint8_t PulsesNeeded ;
uint16_t LastPulsesStartTime ;
uint8_t ChannelSwap ;
uint8_t EightOnly ;

struct t_pulses
{
	uint8_t *port ;
	uint8_t bit ;	
	uint8_t start ;
	uint16_t nextTime ;
} Pulses[8] ;

uint8_t PulsesIndex = 0 ;

uint8_t Sbuffer[28] = {0} ;
uint8_t Sindex = 0 ;
volatile uint16_t Lastrcv = 0 ;
unsigned long Timer = 0 ;

uint32_t LastSbusReceived ;
uint8_t SbusHasBeenReceived ;

uint16_t TicksWaitOne ;
uint16_t TicksWaitOneHalf ;
uint8_t RxIsOne ;
uint8_t SerialMode ;

// Copy the failsafe values into the pulse array

void checkInput()
{
	uint16_t x ;
	uint8_t y ;
	while ( UCSR0A & (1<<RXC0) )
	{
		y = UDR0 ;
		if ( Sindex || ( y == 0x0F ) )
		{
			Sbuffer[Sindex] = y ;
			cli() ;
			x = TCNT1 ;
			sei() ;
			Lastrcv = x ;
 	  	Sindex += 1 ;
			if (Sindex > 27)
 	  	{
 	  	  Sindex = 27 ;
			}
		}
	}
}

void enterFailsafe()
{
	uint8_t i ;
	for ( i = 0 ; i < NUMBER_CHANNELS ; i += 1 )
	{
		PulseTimes[i] = FailsafeTimes[i] ;
	}
}

uint16_t CopyPulseTimes[8] ;
uint8_t CopyChannelNumbers[8] ;

void sortToCopy( uint16_t *pulsePtr )
{
	uint8_t i ;
	uint8_t j ;
	uint8_t k ;
	uint16_t m ;

	for ( i = 0 ; i < 4 ; i += 1 )
	{
		CopyPulseTimes[i] = pulsePtr[i] ;
	}
	for ( i = 0 ; i < 4 ; i += 1 )
	{
		m = CopyPulseTimes[0] ;
		k = 0 ;
		for ( j = 1 ; j < 4 ; j += 1 )
		{
			if ( CopyPulseTimes[j] < m )						// If this one is shorter
			{
				m = CopyPulseTimes[i] ;							// Note value
				k = j ;											// Note index
			}
		}
		CopyChannelNumbers[i] = k ;
		CopyPulseTimes[k] = 0xFFFF ;
	}
}

static uint8_t processSBUSframe()
{
	uint8_t inputbitsavailable = 0 ;
	uint8_t i ;
	uint32_t inputbits = 0 ;
	uint8_t *sbus = Sbuffer ;
	if ( Sindex < 25 )
	{
		return 0 ;
	}
	if ( sbus[24] != 0 )
	{
//      UDR0 = sbus[24] ;
//      UDR0 = sbus[0] ;
		Sindex = 0 ;
		return 0 ;		// Not a valid SBUS frame
	}

	if ( *sbus++ != 0x0F )
	{
		Sindex = 0 ;
		return 0 ;		// Not a valid SBUS frame
	}
//  UDR0 = sbus[20] ;
//  UDR0 = sbus[21] ;

#ifdef DEBUG
	PORTC ^= 0x20 ;
#endif

	LastSbusReceived = millis() ;
	SbusHasBeenReceived = 1 ;

	for ( i = 0 ; i < 16 ; i += 1 )
	{
    uint16_t temp ;
		while ( inputbitsavailable < 11 )
		{
			inputbits |= (uint32_t)*sbus++ << inputbitsavailable ;
			inputbitsavailable += 8 ;
		}
    temp = ( (int16_t)( inputbits & 0x7FF ) - 0x3E0 ) * 5 / 8 + 1500 ;
		if ( ( temp > 800 ) && ( temp < 2200 ) )
		{
			PulseTimes[i] = temp ;
		}
		inputbitsavailable -= 11 ;
		inputbits >>= 11 ;
	}
		Sindex = 0 ;
	return 1 ;
}

void setup()	// run once, when the sketch starts
{
	PORTB = 0 ;			// Outputs low
	PORTC &= 0xF0 ;	// Outputs low
	PORTD &= 0x03 ;	// Outputs low
	set_outputs() ;

}

#if F_CPU == 16000000L  // 16MHz clock                                                  
#define DELAY150		2400
#define DELAY20			 320
#define DELAY40			 640
#define DELAY60			 960
#define TIME_LONG	 40000
#define PULSE_SCALE		16
#elif F_CPU == 8000000L   // 8MHz clock
#define DELAY150		1200
#define DELAY20			 160
#define DELAY40			 320
#define DELAY60			 480
#define TIME_LONG	 20000
#define PULSE_SCALE		 8
#endif		


void setPulseTimes( uint8_t Five2_8 )
{
	uint16_t *pulsePtr = PulseTimes ;
	uint16_t times[4] ;				// The 4 pulse lengths to process
	uint8_t i ;
	uint8_t j ;
	uint8_t k ;
	uint16_t m ;
 	
	k = 0 ;								// Offset into Ports and Bits
	if ( Five2_8 == FIVE_TO_EIGHT )
	{
		pulsePtr += 4 ;			// Move on to second 4 pulses
		k = 4 ;
	}

	if ( Five2_8 == NINE_TO_TWELVE )
	{
		pulsePtr += 8 ;			// Move on to second 4 pulses
		k = 8 ;
	}

	if ( Five2_8 == THIRTEEN_TO_SIXTEEN )
	{
		pulsePtr += 12 ;			// Move on to second 4 pulses
		k = 12 ;
	}

	if ( ChannelSwap )		// Link on AD5
	{
		// swap chanels 1-8 and 9-16
		if ( k >= 8 )
		{
			k -= 8 ;
		}
		else
		{
			k += 8 ;
		}
	}


	for ( i = 0 ; i < 4 ; i += 1 )
	{
		times[i] = pulsePtr[i] ;		// Make local copy of pulses to process
	}
	
	m = times[0] ;								// First pulse
	j = 0 ;
	for ( i = 1 ; i < 4 ; i += 1 )	// Find shortest pulse
	{
		if ( times[i] < m )						// If this one is shorter
		{
			m = times[i] ;							// Note value
			j = i ;											// Note index
		}
	}
	times[j] = 0xFFFF ;							// Make local copy very large
	j += k ;												// Add on offset
	Pulses[4].port = Pulses[0].port = Ports[j] ;	// Set the port for this pulse
	Pulses[4].bit = Pulses[0].bit = Bits[j] ;			// Set the bit for this pulse
	Pulses[0].start = 1 ;													// Mark the start
	Pulses[4].start = 0 ;                         // Mark the end
  cli() ;
	uint16_t time = TCNT1 + DELAY150 ;								// Start the pulses in 150 uS
  sei() ;																							// Gives time for this code to finish
	Pulses[0].nextTime = time + DELAY20 ;							// Next pulse starts in 20 uS
	Pulses[3].nextTime = time + m * PULSE_SCALE ;					// This pulse ends at this time
	
	checkInput() ;
	
	m = times[0] ;                // First pulse
	j = 0 ;
	for ( i = 1 ; i < 4 ; i += 1 )  // Find next shortest pulse
	{
		if ( times[i] < m )           // If this one is shorter
		{
			m = times[i] ;							// Note value
			j = i ;                     // Note index
		}
	}
	times[j] = 0xFFFF ;             // Make local copy very large
	j += k ;                        // Add on offset
	Pulses[5].port = Pulses[1].port = Ports[j] ;	// Set the port for this pulse
	Pulses[5].bit = Pulses[1].bit = Bits[j] ;     // Set the bit for this pulse
	Pulses[1].start = 1 ;                         // Mark the start
	Pulses[5].start = 0 ;                         // Mark the end
	Pulses[1].nextTime = time + DELAY40 ;             // Next pulse starts in another 20 uS
	Pulses[4].nextTime = time + DELAY20 + m * PULSE_SCALE ;    // This pulse ends at this time
	                                              
	checkInput() ;
	
	m = times[0] ;                                
	j = 0 ;
	for ( i = 1 ; i < 4 ; i += 1 )
	{
		if ( times[i] < m )
		{
			m = times[i] ;
			j = i ;
		}
	}
	times[j] = 0xFFFF ;
	j += k ;
	Pulses[6].port = Pulses[2].port = Ports[j] ;
	Pulses[6].bit = Pulses[2].bit = Bits[j] ;
	Pulses[2].start = 1 ;
	Pulses[6].start = 0 ;
	Pulses[2].nextTime = time + DELAY60 ;
	Pulses[5].nextTime = time + DELAY40 + m * PULSE_SCALE ;

	checkInput() ;

	m = times[0] ;
	j = 0 ;
	for ( i = 1 ; i < 4 ; i += 1 )
	{
		if ( times[i] < m )
		{
			m = times[i] ;
			j = i ;
		}
	}
	j += k ;
	Pulses[7].port = Pulses[3].port = Ports[j] ;
	Pulses[7].bit = Pulses[3].bit = Bits[j] ;
	Pulses[3].start = 1 ;
	Pulses[7].start = 0 ;
	Pulses[6].nextTime = time + DELAY60 + m * PULSE_SCALE ;
	Pulses[7].nextTime = time + TIME_LONG ;			// Some time well ahead, shouldn't be used
	cli() ;
	OCR1A = time ;				// Set for first interrupt
  sei() ;
}

ISR(TIMER1_COMPA_vect)
{
	if ( State == PULSING )
	{
		uint8_t *port ;
		uint8_t bit ;
		uint16_t index = PulsesIndex ;
		if ( index < 8 )
		{
			port = Pulses[index].port ;				// Output port to modify
			bit = Pulses[index].bit ;					// Output bit to modify
			OCR1A = Pulses[index].nextTime ;	// Time of next action
			if ( Pulses[index].start )				// If starting pulse
			{
				*port |= bit ;									// Set the output
			}
			else
			{
				*port &= ~bit ;									// Else clear it
			}
			index += 1 ;
			PulsesIndex = index ;					// Next entry
		}
		if ( index >= 8 )
		{
			DISABLE_TIMER_INTERRUPT( ) ;			// We have finished the 4 pulses
			State = IDLE ;
		}
	}
	else
	{
	 	// Unknown state.
		State = IDLE;                           // Error, should not occur. Going to a safe state.
	}	
} // End of ISR

void readFailsafe()
{
  eeprom_read_block( FailsafeTimes, (const void*)0, 32 ) ;
}

void eeprom_write_byte_cmp (uint8_t dat, uint16_t pointer_eeprom)
{
  //see /home/thus/work/avr/avrsdk4/avr-libc-1.4.4/libc/misc/eeprom.S:98 143
  while(EECR & (1<<EEPE)) /* make sure EEPROM is ready */
  {
  }
  EEAR  = pointer_eeprom ;

  EECR |= 1<<EERE ;
  if(dat == EEDR) return ;

  EEDR  = dat ;
  uint8_t flags=SREG ;
  cli() ;
  EECR |= 1<<EEMPE ;
  EECR |= 1<<EEPE ;
  SREG = flags ;
}

void eeWriteBlockCmp( const void *i_pointer_ram, uint16_t i_pointer_eeprom, size_t size )
{
  const char* pointer_ram = (const char*)i_pointer_ram ;
  uint16_t    pointer_eeprom = i_pointer_eeprom ;
  while(size)
	{
    eeprom_write_byte_cmp( *pointer_ram++, pointer_eeprom++ ) ;
    size -= 1 ;
  }
}

void writeFailsafe()
{
	eeWriteBlockCmp( (const void*)FailsafeTimes, (uint16_t)0, 32 ) ;
}

void setSerialMode( uint8_t mode )
{
	if ( mode == 0 )		// 57600
	{
#if F_CPU == 16000000L  // 16MHz clock                                                  
		UBRR0L = 16 ;		// For 57600 baud, use 9 for 100000 baud
#elif F_CPU == 8000000L   // 8MHz clock
		UBRR0L = 8 ;		// For 57600 baud, use 9 for 100000 baud
#endif		
		UCSR0C = (1<<UCSZ00) | (1<<UCSZ01 ) ;
	}
	else
	{
#if F_CPU == 16000000L  // 16MHz clock                                                  
		UBRR0L = 9 ;		// For 100000 baud
#elif F_CPU == 8000000L   // 8MHz clock
		UBRR0L = 4 ;		// For 100000 baud
#endif		
		UCSR0C = (1<<UCSZ00) | (1<<UCSZ01 ) | (1<<UPM01) ;
		UCSR0B &= ~TXEN0 ;
	}
	SerialMode = mode ;
}

static void initUart()
{
	UBRR0H = 0 ;
	setSerialMode( 1 ) ;
	UCSR0A = 0 ;
	PORTD |= 3 ;		// Enable pullup

	UCSR0B = (1<<RXEN0) ;	// Enable receiver

  // Internal State Variable
  State = IDLE ;
}

void init()
{
	uint8_t i ;
	uint8_t j ;
  // Timer1
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1
	TIFR1 = (1 << OCF1A) ;
	TIMSK1 |= ( 1<< OCIE1A ) ;

	initUart() ;
	readFailsafe() ;
	j = 0 ;
	for ( i = 0 ; i < NUMBER_CHANNELS ; i += 1 )
	{
		uint16_t x ;
		x = FailsafeTimes[i] ;
		if ( ( x < 800 ) || ( x > 2200 ) )
		{
			x = 1500 ;
			FailsafeTimes[i] = x ;
			j = 1 ;
		}
	}
	if ( j )
	{
		writeFailsafe() ;	// Need to update them
	}
	DDRC &= ~0x30 ;
	PORTC |= 0x30 ;	// AD4/5 digital input with pullup

#ifdef DEBUG
	DDRC |= 0x20 ;
	PORTC &= ~0x20 ;
#endif

	DDRD &= ~0x02 ;
	PORTD |= 0x02 ;

	sei();
}

uint32_t LastSwitchTime ;
uint8_t LastSwitchState ;

void checkSwitch()
{
	if ( millis() - LastSwitchTime < 500 )
	{
		return ;
	}
	if ( ( PINC & 0x10 ) == 0 )		// Switch pressed
	{
		if ( LastSwitchState == 0 )
		{
			// Just pressed
			LastSwitchTime = millis() ;
			LastSwitchState = 1 ;
			uint8_t i ;
			for ( i = 0 ; i < NUMBER_CHANNELS ; i += 1 )
			{
				FailsafeTimes[i] = PulseTimes[i] ;
			}
			writeFailsafe() ;
		}
	}
	else
	{
		if ( LastSwitchState )
		{
			LastSwitchState = 0 ;
			LastSwitchTime = millis() ;
		}
	}
}


int main()	// run over and over again
{
	uint16_t x ;
  init() ;
  setup() ;

	enterFailsafe() ;			
	LastSbusReceived = millis() - (FAILSAFE_TIMEOUT_MS + 100) ;	// Force failsafe at startup

	if ( ( PINC & 0x20 ) == 0 )		// Link on AD5
	{
		ChannelSwap = 1 ;
	}
	if ( ( PIND & 0x02 ) == 0 )		// Link on PD1
	{
		EightOnly = 1 ;
	}

	for(;;)
	{
		checkInput() ;
		checkSwitch() ;
		checkInput() ;

		uint16_t y = micros() ;
		if ( ( y - LastPulsesStartTime ) > 20000 )
		{
			LastPulsesStartTime += 20000 ;
			setPulseTimes( ONE_TO_FOUR ) ;	// First 4 pulses
			CLEAR_TIMER_INTERRUPT( ) ;			// Clear flag in case it is set
			PulsesIndex = 0 ;								// Start here
			State = PULSING ;
			ENABLE_TIMER_INTERRUPT( ) ;			// Allow interrupt to run
			Timer = micros() ;
			PulsesNeeded = FIVE_TO_EIGHT ;
		}

		checkInput() ;

		cli() ;
		x = TCNT1 ;
		sei() ;

#if F_CPU == 16000000L  // 16MHz clock                                                  
		if ( ( x - Lastrcv ) > 8000 )
#elif F_CPU == 8000000L   // 8MHz clock
		if ( ( x - Lastrcv ) > 4000 )
#endif		
		{
			if ( Sindex )
			{
				if ( Sindex > 24 )
				{
					if ( !processSBUSframe() )
					{
						Sindex = 0 ;
					}
					else
					{
						if ( SerialMode )		// 100K mode
						{
							if ( State == IDLE )
							{
								y = micros() ;
								uint16_t rate = 17900 ;
								if ( EightOnly )
								{
									rate = 8900 ;
								}

								if ( ( y - LastPulsesStartTime ) > rate )
								{
									LastPulsesStartTime = y - 21000 ;	// Will start the pulses
								}
							}
						}
					}
				}
				else
				{
					if ( ( x - Lastrcv ) > 48000 )	// 3mS
					{
						Sindex = 0 ;
					}	
				}
			}
		}
		
		checkInput() ;
	
		if ( PulsesNeeded == FIVE_TO_EIGHT )		// Second 4 pulses
		{
			if ( ( micros() - Timer ) > 3000 )	// Time for them
			{
				if ( EightOnly )
				{
					PulsesNeeded = END_PULSES ;
				}
				else
				{
					PulsesNeeded = NINE_TO_TWELVE ;
				}
				setPulseTimes( FIVE_TO_EIGHT ) ;	// Second 4 pulses
				CLEAR_TIMER_INTERRUPT( ) ;				// Clear flag in case it is set
				PulsesIndex = 0 ;									// Start here
				State = PULSING ;
				Timer = micros() ;
				ENABLE_TIMER_INTERRUPT( ) ;				// Allow interrupt to run
			}
		}
		checkInput() ;

		if ( PulsesNeeded == NINE_TO_TWELVE )		// Second 4 pulses
		{
			if ( ( micros() - Timer ) > 3000 )	// Time for them
			{
				PulsesNeeded = THIRTEEN_TO_SIXTEEN ;
				setPulseTimes( NINE_TO_TWELVE ) ;	// Next 4 pulses
				CLEAR_TIMER_INTERRUPT( ) ;				// Clear flag in case it is set
				PulsesIndex = 0 ;									// Start here
				State = PULSING ;
				Timer = micros() ;
				ENABLE_TIMER_INTERRUPT( ) ;				// Allow interrupt to run
			}
		}
		checkInput() ;
	
		if ( PulsesNeeded == THIRTEEN_TO_SIXTEEN )		// Second 4 pulses
		{
			if ( ( micros() - Timer ) > 3000 )	// Time for them
			{
				PulsesNeeded = END_PULSES ;
				setPulseTimes( THIRTEEN_TO_SIXTEEN ) ;	// Last 4 pulses
				CLEAR_TIMER_INTERRUPT( ) ;				// Clear flag in case it is set
				PulsesIndex = 0 ;									// Start here
				State = PULSING ;
				ENABLE_TIMER_INTERRUPT( ) ;				// Allow interrupt to run
			}
		}
		checkInput() ;

		if ( PulsesNeeded == END_PULSES )		// Second 4 pulses
		{
			if ( State == IDLE )
			{
				PulsesNeeded = 0 ;
			}
		}

		if ( ( millis() - LastSbusReceived ) > FAILSAFE_TIMEOUT_MS )
		{
			enterFailsafe() ;
		}
		
		if ( SbusHasBeenReceived == 0 )
		{
			if ( ( millis() - LastSbusReceived ) > 100 )
			{
				LastSbusReceived = millis() ;
				setSerialMode( !SerialMode ) ;				
			}
		}

		checkInput() ;

	}
} 

// replacement millis() and micros()
// These work polled, no interrupts
// micros() MUST be called at least once every 4 milliseconds
uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;
uint8_t Correction ;

uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	elapsed += Correction ;
#if F_CPU == 16000000L  // 16MHz clock                                                  
	Correction = elapsed & 0x0F ;
	elapsed >>= 4 ;
#elif F_CPU == 8000000L   // 8MHz clock
	Correction = elapsed & 0x07 ;
	elapsed >>= 3 ;
#endif		
	
	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
#if F_CPU == 8000000L   // 8MHz clock
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
#endif		
	if ( elapsed  > 3999 )
	{
		millisToAdd += 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd += 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd += 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd += 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}

void delay(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();
	uint16_t lms = ms ;

	while (lms > 0) {
		if (((uint16_t)micros() - start) >= 1000) {
			lms--;
			start += 1000;
		}
	}
}
 

