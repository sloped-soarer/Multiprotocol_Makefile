

# 1 "src/Multiprotocol.ino" // Helps debugging !
/*********************************************************
					Multiprotocol Tx code
               by Midelic and Pascal Langer(hpnuts)
	http://www.rcgroups.com/forums/showthread.php?t=2165676
    https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/edit/master/README.md

	Thanks to PhracturedBlue, Hexfet, Goebish, Victzh and all protocol developers
				Ported  from deviation firmware 

 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <avr/pgmspace.h>

//#define DEBUG_PIN		// Use pin TX for AVR and SPI_CS for STM32 => DEBUG_PIN_on, DEBUG_PIN_off, DEBUG_PIN_toggle
//#define DEBUG_SERIAL	// Only for STM32_BOARD, compiled with Upload method "Serial"->usart1, "STM32duino bootloader"->USB serial

#ifdef __arm__			// Let's automatically select the board if arm is selected
	#define STM32_BOARD
#endif
#if defined (ARDUINO_AVR_XMEGA32D4) || defined (ARDUINO_MULTI_ORANGERX)
	#include "MultiOrange.h"
#endif

#include "Multiprotocol.h"

//Multiprotocol module configuration file
#include "_Config.h"

//Personal config file
#if defined(USE_MY_CONFIG)
#include "_MyConfig.h"
#endif

#include "Pins.h"
#include "TX_Def.h"
#include "Validate.h"

#ifndef STM32_BOARD
	#include <avr/eeprom.h>
#else
	#include <libmaple/usart.h>
	#include <libmaple/timer.h>
	//#include <libmaple/spi.h>
	#include <SPI.h>
	#include <EEPROM.h>	
	HardwareTimer HWTimer2(2);
#if defined  SPORT_POLLING
#ifdef INVERT_TELEMETRY
	HardwareTimer HWTimer4(4);
#endif
#endif
	void PPM_decode();
	void ISR_COMPB();
	extern "C"
	{
		void __irq_usart2(void);
		void __irq_usart3(void);
	}
#endif

//Global constants/variables
uint32_t MProtocol_id;//tx id,
uint32_t MProtocol_id_master;
uint32_t blink=0,last_signal=0;
//
uint16_t counter;
uint8_t  channel;
uint8_t  packet[40];

#define NUM_CHN 16
// Servo data
uint16_t Channel_data[NUM_CHN];
uint8_t  Channel_AUX;
#ifdef FAILSAFE_ENABLE
	uint16_t Failsafe_data[NUM_CHN];
#endif

// Protocol variables
uint8_t  cyrfmfg_id[6];//for dsm2 and devo
uint8_t  rx_tx_addr[5];
uint8_t  rx_id[5];
uint8_t  phase;
uint16_t bind_counter;
uint8_t  bind_phase;
uint8_t  binding_idx;
uint16_t packet_period;
uint8_t  packet_count;
uint8_t  packet_sent;
uint8_t  packet_length;
uint8_t  hopping_frequency[50];
uint8_t  *hopping_frequency_ptr;
uint8_t  hopping_frequency_no=0;
uint8_t  rf_ch_num;
uint8_t  throttle, rudder, elevator, aileron;
uint8_t  flags;
uint16_t crc;
uint8_t  crc8;
uint16_t seed;
uint16_t failsafe_count;
uint16_t state;
uint8_t  len;
uint8_t  armed, arm_flags, arm_channel_previous;

#if defined(FRSKYX_CC2500_INO) || defined(SFHSS_CC2500_INO) || defined(HITEC_CC2500_INO)
	uint8_t calData[48];
#endif

#ifdef CHECK_FOR_BOOTLOADER
	uint8_t BootTimer ;
	uint8_t BootState ;
	uint8_t NotBootChecking ;
	uint8_t BootCount ;

	#define BOOT_WAIT_30_IDLE	0
	#define BOOT_WAIT_30_DATA	1
	#define BOOT_WAIT_20		2
	#define BOOT_READY			3
#endif

//Channel mapping for protocols
const uint8_t CH_AETR[]={AILERON, ELEVATOR, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
const uint8_t CH_TAER[]={THROTTLE, AILERON, ELEVATOR, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
const uint8_t CH_RETA[]={RUDDER, ELEVATOR, THROTTLE, AILERON, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};
const uint8_t CH_EATR[]={ELEVATOR, AILERON, THROTTLE, RUDDER, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16};

// Mode_select variables
uint8_t mode_select;
uint8_t protocol_flags=0,protocol_flags2=0;

#ifdef ENABLE_PPM
// PPM variable
volatile uint16_t PPM_data[NUM_CHN];
volatile uint8_t  PPM_chan_max=0;
#endif

#if not defined (ORANGE_TX) && not defined (STM32_BOARD)
//Random variable
volatile uint32_t gWDT_entropy=0;
#endif

//Serial protocol
uint8_t sub_protocol;
uint8_t protocol;
uint8_t option;
uint8_t cur_protocol[3];
uint8_t prev_option;
uint8_t prev_power=0xFD; // unused power value
uint8_t  RX_num;

//Serial RX variables
#define BAUD 100000
#define RXBUFFER_SIZE 26
volatile uint8_t rx_buff[RXBUFFER_SIZE];
volatile uint8_t rx_ok_buff[RXBUFFER_SIZE];
volatile uint8_t discard_frame = 0;

// Telemetry
#define MAX_PKT 29
uint8_t pkt[MAX_PKT];//telemetry receiving packets
#if defined(TELEMETRY)
	#ifdef INVERT_TELEMETRY
		#if not defined(ORANGE_TX) && not defined(STM32_BOARD)
			// enable bit bash for serial
			#define	BASH_SERIAL 1
		#endif
		#define	INVERT_SERIAL 1
	#endif
	uint8_t pass = 0;
	uint8_t pktt[MAX_PKT];//telemetry receiving packets
	#ifdef BASH_SERIAL
	// For bit-bashed serial output
		#define TXBUFFER_SIZE 192
		volatile struct t_serial_bash
		{
			uint8_t head ;
			uint8_t tail ;
			uint8_t data[TXBUFFER_SIZE] ;
			uint8_t busy ;
			uint8_t speed ;
		} SerialControl ;
	#else
		#define TXBUFFER_SIZE 96
		volatile uint8_t tx_buff[TXBUFFER_SIZE];
		volatile uint8_t tx_head=0;
		volatile uint8_t tx_tail=0;
	#endif // BASH_SERIAL
	uint8_t v_lipo1;
	uint8_t v_lipo2;
	uint8_t RX_RSSI;
	uint8_t TX_RSSI;
	uint8_t RX_LQI;
	uint8_t TX_LQI;
	uint8_t telemetry_link=0; 
	uint8_t telemetry_counter=0;
	uint8_t telemetry_lost;
	#ifdef SPORT_POLLING
		#define MAX_SPORT_BUFFER 64
		uint8_t	SportData[MAX_SPORT_BUFFER];
		bool	ok_to_send = false;
		uint8_t	sport_idx = 0;
		uint8_t	sport_index = 0;
	#endif
#endif // TELEMETRY

// Callback
typedef uint16_t (*void_function_t) (void);//pointer to a function with no parameters which return an uint16_t integer
void_function_t remote_callback = 0;

// Init
void setup()
{
	// Setup diagnostic uart before anything else
	#ifdef DEBUG_SERIAL
		Serial.begin(115200,SERIAL_8N1);
		while (!Serial); // Wait for ever for the serial port to connect...
		debugln("Multiprotocol version: %d.%d.%d.%d", VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION, VERSION_PATCH_LEVEL);
	#endif

	// General pinout
	#ifdef ORANGE_TX
		//XMEGA
		PORTD.OUTSET = 0x17 ;
		PORTD.DIRSET = 0xB2 ;
		PORTD.DIRCLR = 0x4D ;
		PORTD.PIN0CTRL = 0x18 ;
		PORTD.PIN2CTRL = 0x18 ;
		PORTE.DIRSET = 0x01 ;
		PORTE.DIRCLR = 0x02 ;
		// Timer1 config
		// TCC1 16-bit timer, clocked at 0.5uS
		EVSYS.CH3MUX = 0x80 + 0x04 ;	// Prescaler of 16
		TCC1.CTRLB = 0; TCC1.CTRLC = 0; TCC1.CTRLD = 0; TCC1.CTRLE = 0;
		TCC1.INTCTRLA = 0; TIMSK1 = 0;
		TCC1.PER = 0xFFFF ;
		TCNT1 = 0 ;
		TCC1.CTRLA = 0x0B ;	// Event3 (prescale of 16)
	#elif defined STM32_BOARD
		//STM32
		afio_cfg_debug_ports(AFIO_DEBUG_NONE);
		pinMode(LED2_pin,OUTPUT);
		pinMode(A7105_CSN_pin,OUTPUT);
		pinMode(CC25_CSN_pin,OUTPUT);
		pinMode(NRF_CSN_pin,OUTPUT);
		pinMode(CYRF_CSN_pin,OUTPUT);
		pinMode(SPI_CSN_pin,OUTPUT);
		pinMode(CYRF_RST_pin,OUTPUT);
		pinMode(PE1_pin,OUTPUT);
		pinMode(PE2_pin,OUTPUT);
		pinMode(TX_INV_pin,OUTPUT);
		pinMode(RX_INV_pin,OUTPUT);
		#if defined TELEMETRY
			#if defined INVERT_SERIAL
				TX_INV_on;	//activate inverter for both serial TX and RX signals
				RX_INV_on;
			#else
				TX_INV_off;
				RX_INV_off;
			#endif	
		#endif
		pinMode(BIND_pin,INPUT_PULLUP);
		pinMode(PPM_pin,INPUT);
		pinMode(S1_pin,INPUT_PULLUP);//dial switch
		pinMode(S2_pin,INPUT_PULLUP);
		pinMode(S3_pin,INPUT_PULLUP);
		pinMode(S4_pin,INPUT_PULLUP);
		//Random pins
		pinMode(PB0, INPUT_ANALOG); // set up pin for analog input
		pinMode(PB1, INPUT_ANALOG); // set up pin for analog input

		//Timers
		init_HWTimer();			//0.5us
	#else
		//ATMEGA328p
		// all inputs
		DDRB=0x00;DDRC=0x00;DDRD=0x00;
		// outputs
		SDI_output;
		SCLK_output;
		#ifdef A7105_CSN_pin
			A7105_CSN_output;
		#endif
		#ifdef CC25_CSN_pin
			CC25_CSN_output;
		#endif
		#ifdef CYRF_CSN_pin
			CYRF_RST_output;
			CYRF_CSN_output;
		#endif
		#ifdef NRF_CSN_pin
			NRF_CSN_output;
		#endif
		PE1_output;
		PE2_output;
		SERIAL_TX_output;

		// pullups
		PROTO_DIAL1_port |= _BV(PROTO_DIAL1_pin);
		PROTO_DIAL2_port |= _BV(PROTO_DIAL2_pin);
		PROTO_DIAL3_port |= _BV(PROTO_DIAL3_pin);
		PROTO_DIAL4_port |= _BV(PROTO_DIAL4_pin);
		BIND_port |= _BV(BIND_pin);

		// Timer1 config
		TCCR1A = 0;
		TCCR1B = (1 << CS11);	//prescaler8, set timer1 to increment every 0.5us(16Mhz) and start timer

		// Random
		random_init();
	#endif

	LED2_on;
	
	// Set Chip selects
	#ifdef A7105_CSN_pin
		A7105_CSN_on;
	#endif
	#ifdef CC25_CSN_pin
		CC25_CSN_on;
	#endif
	#ifdef CYRF_CSN_pin
		CYRF_CSN_on;
	#endif
	#ifdef NRF_CSN_pin
		NRF_CSN_on;
	#endif
	//	Set SPI lines
	#ifdef	STM32_BOARD
		initSPI2();
	#else
		SDI_on;
		SCLK_off;
	#endif

	//Wait for every component to start
	delayMilliseconds(100);
	
	// Read status of bind button
	if( IS_BIND_BUTTON_on )
	{
		BIND_BUTTON_FLAG_on;	// If bind button pressed save the status
		BIND_IN_PROGRESS;		// Request bind
	}
	else
		BIND_DONE;

	// Read status of mode select binary switch
	// after this mode_select will be one of {0000, 0001, ..., 1111}
	#ifndef ENABLE_PPM
		mode_select = MODE_SERIAL ;	// force serial mode
	#elif defined STM32_BOARD
		mode_select= 0x0F -(uint8_t)(((GPIOA->regs->IDR)>>4)&0x0F);
	#else
		mode_select =
			((PROTO_DIAL1_ipr & _BV(PROTO_DIAL1_pin)) ? 0 : 1) + 
			((PROTO_DIAL2_ipr & _BV(PROTO_DIAL2_pin)) ? 0 : 2) +
			((PROTO_DIAL3_ipr & _BV(PROTO_DIAL3_pin)) ? 0 : 4) +
			((PROTO_DIAL4_ipr & _BV(PROTO_DIAL4_pin)) ? 0 : 8);
	#endif
	//mode_select=1;
    debugln("Protocol selection switch reads as %d", mode_select);

	#ifdef ENABLE_PPM
		uint8_t bank=bank_switch();
	#endif

	// Set default channels' value
	InitChannel();
	#ifdef ENABLE_PPM
		InitPPM();
	#endif

	// Update LED
	LED_off;
	LED_output;

	//Init RF modules
	modules_reset();

#ifndef ORANGE_TX
	//Init the seed with a random value created from watchdog timer for all protocols requiring random values
	#ifdef STM32_BOARD
		randomSeed((uint32_t)analogRead(PB0) << 10 | analogRead(PB1));			
	#else
		randomSeed(random_value());
	#endif
#endif

	// Read or create protocol id
	MProtocol_id_master=random_id(10,false);

	debugln("Module Id: %lx", MProtocol_id_master);
	
#ifdef ENABLE_PPM
	//Protocol and interrupts initialization
	if(mode_select != MODE_SERIAL)
	{ // PPM
		#ifndef MY_PPM_PROT
			const PPM_Parameters *PPM_prot_line=&PPM_prot[bank*14+mode_select-1];
		#else
			const PPM_Parameters *PPM_prot_line=&My_PPM_prot[bank*14+mode_select-1];
		#endif
		
		protocol		=	PPM_prot_line->protocol;
		cur_protocol[1] = protocol;
		sub_protocol   	=	PPM_prot_line->sub_proto;
		RX_num			=	PPM_prot_line->rx_num;

		//Forced frequency tuning values for CC2500 protocols
		#if defined(FORCE_FRSKYD_TUNING) && defined(FRSKYD_CC2500_INO)
			if(protocol==PROTO_FRSKYD) 
				option			=	FORCE_FRSKYD_TUNING;		// Use config-defined tuning value for FrSkyD
			else
		#endif
		#if defined(FORCE_FRSKYV_TUNING) && defined(FRSKYV_CC2500_INO)
			if(protocol==PROTO_FRSKYV)
				option			=	FORCE_FRSKYV_TUNING;		// Use config-defined tuning value for FrSkyV
			else
		#endif
		#if defined(FORCE_FRSKYX_TUNING) && defined(FRSKYX_CC2500_INO)
			if(protocol==PROTO_FRSKYX)
				option			=	FORCE_FRSKYX_TUNING;		// Use config-defined tuning value for FrSkyX
			else
		#endif 
		#if defined(FORCE_SFHSS_TUNING) && defined(SFHSS_CC2500_INO)
			if (protocol==PROTO_SFHSS)
				option			=	FORCE_SFHSS_TUNING;			// Use config-defined tuning value for SFHSS
			else
		#endif
		#if defined(FORCE_CORONA_TUNING) && defined(CORONA_CC2500_INO)
			if (protocol==PROTO_CORONA)
				option			=	FORCE_CORONA_TUNING;		// Use config-defined tuning value for CORONA
			else
		#endif
		#if defined(FORCE_HITEC_TUNING) && defined(HITEC_CC2500_INO)
			if (protocol==PROTO_HITEC)
				option			=	FORCE_HITEC_TUNING;		// Use config-defined tuning value for HITEC
			else
		#endif
				option			=	PPM_prot_line->option;	// Use radio-defined option value

		if(PPM_prot_line->power)		POWER_FLAG_on;
		if(PPM_prot_line->autobind)
		{
			AUTOBIND_FLAG_on;
			BIND_IN_PROGRESS;	// Force a bind at protocol startup
		}

		protocol_init();

		#ifndef STM32_BOARD
			//Configure PPM interrupt
			#if PPM_pin == 2
				EICRA |= _BV(ISC01);	// The rising edge of INT0 pin D2 generates an interrupt request
				EIMSK |= _BV(INT0);		// INT0 interrupt enable
			#elif PPM_pin == 3
				EICRA |= _BV(ISC11);	// The rising edge of INT1 pin D3 generates an interrupt request
				EIMSK |= _BV(INT1);		// INT1 interrupt enable
			#else
				#error PPM pin can only be 2 or 3
			#endif
		#else
			attachInterrupt(PPM_pin,PPM_decode,FALLING);
		#endif

		#if defined(TELEMETRY)
			PPM_Telemetry_serial_init();// Configure serial for telemetry
		#endif
	}
	else
#endif //ENABLE_PPM
	{ // Serial
		#ifdef ENABLE_SERIAL
			for(uint8_t i=0;i<3;i++)
				cur_protocol[i]=0;
			protocol=0;
			#ifdef CHECK_FOR_BOOTLOADER
				Mprotocol_serial_init(1); 	// Configure serial and enable RX interrupt
			#else
				Mprotocol_serial_init(); 	// Configure serial and enable RX interrupt
			#endif
		#endif //ENABLE_SERIAL
	}
	LED2_on;
	debugln("Init complete");
}

// Main
// Protocol scheduler
void loop()
{ 
	uint16_t next_callback,diff=0xFFFF;

	while(1)
	{
		if(remote_callback==0 || IS_WAIT_BIND_on || diff>2*200)
		{
			do
			{
				Update_All();
			}
			while(remote_callback==0 || IS_WAIT_BIND_on);
		}
		#ifndef STM32_BOARD
			if( (TIFR1 & OCF1A_bm) != 0)
			{
				cli();					// Disable global int due to RW of 16 bits registers
				OCR1A=TCNT1;			// Callback should already have been called... Use "now" as new sync point.
				sei();					// Enable global int
			}
			else
				while((TIFR1 & OCF1A_bm) == 0); // Wait before callback
		#else
			if((TIMER2_BASE->SR & TIMER_SR_CC1IF)!=0)
			{
				debugln("Callback miss");
				cli();
				OCR1A = TCNT1;
				sei();
			}
			else
				while((TIMER2_BASE->SR & TIMER_SR_CC1IF )==0); // Wait before callback
		#endif
		do
		{
			TX_MAIN_PAUSE_on;
			tx_pause();
			if(IS_INPUT_SIGNAL_on && remote_callback!=0)
				next_callback=remote_callback();
			else
				next_callback=2000;					// No PPM/serial signal check again in 2ms...
			TX_MAIN_PAUSE_off;
			tx_resume();
			while(next_callback>4000)
			{ // start to wait here as much as we can...
				next_callback-=2000;				// We will wait below for 2ms
				cli();								// Disable global int due to RW of 16 bits registers
				OCR1A += 2000*2 ;					// set compare A for callback
				#ifndef STM32_BOARD	
					TIFR1=OCF1A_bm;					// clear compare A=callback flag
				#else
					TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC1IF;	// Clear Timer2/Comp1 interrupt flag
				#endif
				sei();								// enable global int
				if(Update_All())					// Protocol changed?
				{
					next_callback=0;				// Launch new protocol ASAP
					break;
				}
				#ifndef STM32_BOARD	
					while((TIFR1 & OCF1A_bm) == 0);	// wait 2ms...
				#else
					while((TIMER2_BASE->SR & TIMER_SR_CC1IF)==0);//2ms wait
				#endif
			}
			// at this point we have a maximum of 4ms in next_callback
			next_callback *= 2 ;
			cli();									// Disable global int due to RW of 16 bits registers
			OCR1A+= next_callback ;					// set compare A for callback
			#ifndef STM32_BOARD			
				TIFR1=OCF1A_bm;						// clear compare A=callback flag
			#else
				TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC1IF;	// Clear Timer2/Comp1 interrupt flag
			#endif		
			diff=OCR1A-TCNT1;						// compare timer and comparator
			sei();									// enable global int
		}
		while(diff&0x8000);	 						// Callback did not took more than requested time for next callback
													// so we can launch Update_All before next callback
	}
}

uint8_t Update_All()
{
	#ifdef ENABLE_SERIAL
		#ifdef CHECK_FOR_BOOTLOADER
			if ( (mode_select==MODE_SERIAL) && (NotBootChecking == 0) )
				pollBoot() ;
			else
		#endif
		if(mode_select==MODE_SERIAL && IS_RX_FLAG_on)		// Serial mode and something has been received
		{
			update_serial_data();							// Update protocol and data
			update_channels_aux();
			INPUT_SIGNAL_on;								//valid signal received
			last_signal=millis();
		}
	#endif //ENABLE_SERIAL
	#ifdef ENABLE_PPM
		if(mode_select!=MODE_SERIAL && IS_PPM_FLAG_on)		// PPM mode and a full frame has been received
		{
			for(uint8_t i=0;i<PPM_chan_max;i++)
			{ // update servo data without interrupts to prevent bad read
				uint16_t val;
				cli();										// disable global int
				val = PPM_data[i];
				sei();										// enable global int
				val=map16b(val,PPM_MIN_100*2,PPM_MAX_100*2,CHANNEL_MIN_100,CHANNEL_MAX_100);
				if(val&0x8000) 					val=CHANNEL_MIN_125;
				else if(val>CHANNEL_MAX_125)	val=CHANNEL_MAX_125;
				Channel_data[i]=val;
			}
			PPM_FLAG_off;									// wait for next frame before update
			update_channels_aux();
			INPUT_SIGNAL_on;								// valid signal received
			last_signal=millis();
		}
	#endif //ENABLE_PPM
	update_led_status();
	#if defined(TELEMETRY)
		#if ( !( defined(MULTI_TELEMETRY) || defined(MULTI_STATUS) ) )
			if( (protocol==PROTO_FRSKYD) || (protocol==PROTO_BAYANG) || (protocol==PROTO_NCC1701) || (protocol==PROTO_BUGS) || (protocol==PROTO_BUGSMINI) || (protocol==PROTO_HUBSAN) || (protocol==PROTO_AFHDS2A) || (protocol==PROTO_FRSKYX) || (protocol==PROTO_DSM) || (protocol==PROTO_CABELL)  || (protocol==PROTO_HITEC))
		#endif
				TelemetryUpdate();
	#endif
	#ifdef ENABLE_BIND_CH
		if(IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && Channel_data[BIND_CH-1]>CHANNEL_MAX_COMMAND && Channel_data[THROTTLE]<(CHANNEL_MIN_100+50))
		{ // Autobind is on and BIND_CH went up and Throttle is low
			CHANGE_PROTOCOL_FLAG_on;							//reload protocol
			BIND_IN_PROGRESS;									//enable bind
			BIND_CH_PREV_on;
		}
		if(IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_on && Channel_data[BIND_CH-1]<CHANNEL_MIN_COMMAND)
		{ // Autobind is on and BIND_CH went down
			BIND_CH_PREV_off;
			//Request protocol to terminate bind
			#if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO) || defined(FRSKYV_CC2500_INO)
			if(protocol==PROTO_FRSKYD || protocol==PROTO_FRSKYX || protocol==PROTO_FRSKYV)
				BIND_DONE;
			else
			#endif
			if(bind_counter>2)
				bind_counter=2;
		}
	#endif //ENABLE_BIND_CH
	if(IS_CHANGE_PROTOCOL_FLAG_on)
	{ // Protocol needs to be changed or relaunched for bind
		protocol_init();									//init new protocol
		return 1;
	}
	return 0;
}

// Update channels direction and Channel_AUX flags based on servo AUX positions
static void update_channels_aux(void)
{
	//Reverse channels direction
	#ifdef REVERSE_AILERON
		reverse_channel(AILERON);
	#endif
	#ifdef REVERSE_ELEVATOR
		reverse_channel(ELEVATOR);
	#endif
	#ifdef REVERSE_THROTTLE
		reverse_channel(THROTTLE);
	#endif
	#ifdef REVERSE_RUDDER
		reverse_channel(RUDDER);
	#endif
		
	//Calc AUX flags
	Channel_AUX=0;
	for(uint8_t i=0;i<8;i++)
		if(Channel_data[CH5+i]>CHANNEL_SWITCH)
			Channel_AUX|=1<<i;
}

// Update led status based on binding and serial
static void update_led_status(void)
{
	if(IS_INPUT_SIGNAL_on)
		if(millis()-last_signal>70)
			INPUT_SIGNAL_off;							//no valid signal (PPM or Serial) received for 70ms
	if(blink<millis())
	{
		if(IS_INPUT_SIGNAL_off)
		{
			if(mode_select==MODE_SERIAL)
				blink+=BLINK_SERIAL_TIME;				//blink slowly if no valid serial input
			else
				blink+=BLINK_PPM_TIME;					//blink more slowly if no valid PPM input
		}
		else
			if(remote_callback == 0)
			{ // Invalid protocol
				if(IS_LED_on)							//flash to indicate invalid protocol
					blink+=BLINK_BAD_PROTO_TIME_LOW;
				else
					blink+=BLINK_BAD_PROTO_TIME_HIGH;
			}
			else
			{
				if(IS_WAIT_BIND_on)
				{
					if(IS_LED_on)							//flash to indicate WAIT_BIND
						blink+=BLINK_WAIT_BIND_TIME_LOW;
					else
						blink+=BLINK_WAIT_BIND_TIME_HIGH;
				}
				else
				{
					if(IS_BIND_DONE)
						LED_off;							//bind completed force led on
					blink+=BLINK_BIND_TIME;					//blink fastly during binding
				}
			}
		LED_toggle;
	}
}

#ifdef ENABLE_PPM
uint8_t bank_switch(void)
{
	uint8_t bank=eeprom_read_byte((EE_ADDR)EEPROM_BANK_OFFSET);
	if(bank>=NBR_BANKS)
	{ // Wrong number of bank
		eeprom_write_byte((EE_ADDR)EEPROM_BANK_OFFSET,0x00);	// set bank to 0
		bank=0;
	}
	debugln("Using bank %d", bank);

	phase=3;
	uint32_t check=millis();
	blink=millis();
	while(mode_select==15)
	{ //loop here if the dial is on position 15 for user to select the bank
		if(blink<millis())
		{
			switch(phase & 0x03)
			{ // Flash bank number of times
				case 0:
					LED_on;
					blink+=BLINK_BANK_TIME_HIGH;
					phase++;
					break;
				case 1:
					LED_off;
					blink+=BLINK_BANK_TIME_LOW;
					phase++;
					break;
				case 2:
					if( (phase>>2) >= bank)
					{
						phase=0;
						blink+=BLINK_BANK_REPEAT;
					}
					else
						phase+=2;
					break;
				case 3:
					LED_output;
					LED_off;
					blink+=BLINK_BANK_TIME_LOW;
					phase=0;
					break;
			}
		}
		if(check<millis())
		{
			//Test bind button: for AVR it's shared with the LED so some extra work is needed to check it...
			#ifndef STM32_BOARD
				bool led=IS_LED_on;
				BIND_SET_INPUT;
				BIND_SET_PULLUP;
			#endif
			bool test_bind=IS_BIND_BUTTON_on;
			#ifndef STM32_BOARD
				if(led)
					LED_on;
				else
					LED_off;
				LED_output;
			#endif
			if( test_bind )
			{	// Increase bank
				LED_on;
				bank++;
				if(bank>=NBR_BANKS)
					bank=0;
				eeprom_write_byte((EE_ADDR)EEPROM_BANK_OFFSET,bank);
				debugln("Using bank %d", bank);
				phase=3;
				blink+=BLINK_BANK_REPEAT;
				check+=2*BLINK_BANK_REPEAT;
			}
			check+=1;
		}
	}
	return bank;
}
#endif

inline void tx_pause()
{
	#ifdef TELEMETRY
	// Pause telemetry by disabling transmitter interrupt
		#ifdef ORANGE_TX
			USARTC0.CTRLA &= ~0x03 ;
		#else
			#ifndef BASH_SERIAL
				#ifdef STM32_BOARD
					USART3_BASE->CR1 &= ~ USART_CR1_TXEIE;
				#else
					UCSR0B &= ~_BV(UDRIE0);
				#endif
			#endif
		#endif
	#endif
}

inline void tx_resume()
{
	#ifdef TELEMETRY
	// Resume telemetry by enabling transmitter interrupt
		#ifndef SPORT_POLLING
		if(!IS_TX_PAUSE_on)
		#endif
		{
			#ifdef ORANGE_TX
				cli() ;
				USARTC0.CTRLA = (USARTC0.CTRLA & 0xFC) | 0x01 ;
				sei() ;
			#else
				#ifndef BASH_SERIAL
					#ifdef STM32_BOARD
						USART3_BASE->CR1 |= USART_CR1_TXEIE;
					#else
						UCSR0B |= _BV(UDRIE0);			
					#endif
				#else
					resumeBashSerial();
				#endif
			#endif
		}
	#endif
}

// Protocol start
static void protocol_init()
{
	static uint16_t next_callback;
	if(IS_WAIT_BIND_off)
	{
		remote_callback = 0;			// No protocol
		next_callback=0;				// Default is immediate call back
		LED_off;						// Led off during protocol init
		modules_reset();				// Reset all modules

		// reset telemetry
		#ifdef TELEMETRY
			tx_pause();
			pass=0;
			telemetry_link=0;
			telemetry_lost=1;
			#ifdef BASH_SERIAL
				TIMSK0 = 0 ;			// Stop all timer 0 interrupts
				#ifdef INVERT_SERIAL
					SERIAL_TX_off;
				#else
					SERIAL_TX_on;
				#endif
				SerialControl.tail=0;
				SerialControl.head=0;
				SerialControl.busy=0;
			#else
				tx_tail=0;
				tx_head=0;
			#endif
			TX_RX_PAUSE_off;
			TX_MAIN_PAUSE_off;
		#endif

		//Set global ID and rx_tx_addr
		MProtocol_id = RX_num + MProtocol_id_master;
		set_rx_tx_addr(MProtocol_id);
		
		#ifdef FAILSAFE_ENABLE
			InitFailsafe();
		#endif
	
		blink=millis();

		PE1_on;							//NRF24L01 antenna RF3 by default
		PE2_off;						//NRF24L01 antenna RF3 by default
		
		debugln("Protocol selected: %d, sub proto %d, rxnum %d, option %d", protocol, sub_protocol, RX_num, option);

		switch(protocol)				// Init the requested protocol
		{
			#ifdef A7105_INSTALLED
				#if defined(FLYSKY_A7105_INO)
					case PROTO_FLYSKY:
						PE1_off;	//antenna RF1
						next_callback = initFlySky();
						remote_callback = ReadFlySky;
						break;
				#endif
				#if defined(AFHDS2A_A7105_INO)
					case PROTO_AFHDS2A:
						PE1_off;	//antenna RF1
						next_callback = initAFHDS2A();
						remote_callback = ReadAFHDS2A;
						break;
				#endif
				#if defined(HUBSAN_A7105_INO)
					case PROTO_HUBSAN:
						PE1_off;	//antenna RF1
						if(IS_BIND_BUTTON_FLAG_on) random_id(EEPROM_ID_OFFSET,true); // Generate new ID if bind button is pressed.
						next_callback = initHubsan();
						remote_callback = ReadHubsan;
						break;
				#endif
				#if defined(BUGS_A7105_INO)
					case PROTO_BUGS:
						PE1_off;	//antenna RF1
						next_callback = initBUGS();
						remote_callback = ReadBUGS;
						break;
				#endif
			#endif
			#ifdef CC2500_INSTALLED
				#if defined(FRSKYD_CC2500_INO)
					case PROTO_FRSKYD:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initFrSky_2way();
						remote_callback = ReadFrSky_2way;
						break;
				#endif
				#if defined(FRSKYV_CC2500_INO)
					case PROTO_FRSKYV:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initFRSKYV();
						remote_callback = ReadFRSKYV;
						break;
				#endif
				#if defined(FRSKYX_CC2500_INO)
					case PROTO_FRSKYX:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initFrSkyX();
						remote_callback = ReadFrSkyX;
						break;
				#endif
				#if defined(SFHSS_CC2500_INO)
					case PROTO_SFHSS:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initSFHSS();
						remote_callback = ReadSFHSS;
						break;
				#endif
				#if defined(CORONA_CC2500_INO)
					case PROTO_CORONA:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initCORONA();
						remote_callback = ReadCORONA;
						break;
				#endif
				#if defined(HITEC_CC2500_INO)
					case PROTO_HITEC:
						PE1_off;	//antenna RF2
						PE2_on;
						next_callback = initHITEC();
						remote_callback = ReadHITEC;
						break;
				#endif
			#endif
			#ifdef CYRF6936_INSTALLED
				#if defined(DSM_CYRF6936_INO)
					case PROTO_DSM:
						PE2_on;	//antenna RF4
						next_callback = initDsm();
						remote_callback = ReadDsm;
						break;
				#endif
				#if defined(WFLY_CYRF6936_INO)
					case PROTO_WFLY:
						PE2_on;	//antenna RF4
						next_callback = initWFLY();
						remote_callback = ReadWFLY;
						break;
				#endif
				#if defined(DEVO_CYRF6936_INO)
					case PROTO_DEVO:
						#ifdef ENABLE_PPM
							if(mode_select) //PPM mode
							{
								if(IS_BIND_BUTTON_FLAG_on)
								{
									eeprom_write_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num),0x00);	// reset to autobind mode for the current model
									option=0;
								}
								else
								{	
									option=eeprom_read_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num));	// load previous mode: autobind or fixed id
									if(option!=1) option=0;								// if not fixed id mode then it should be autobind
								}
							}
						#endif //ENABLE_PPM
						PE2_on;	//antenna RF4
						next_callback = DevoInit();
						remote_callback = devo_callback;
						break;
				#endif
				#if defined(WK2x01_CYRF6936_INO)
					case PROTO_WK2x01:
						#ifdef ENABLE_PPM
							if(mode_select) //PPM mode
							{
								if(IS_BIND_BUTTON_FLAG_on)
								{
									eeprom_write_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num),0x00);	// reset to autobind mode for the current model
									option=0;
								}
								else
								{	
									option=eeprom_read_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num));	// load previous mode: autobind or fixed id
									if(option!=1) option=0;								// if not fixed id mode then it should be autobind
								}
							}
						#endif //ENABLE_PPM
						PE2_on;	//antenna RF4
						next_callback = WK_setup();
						remote_callback = WK_cb;
						break;
				#endif
				#if defined(J6PRO_CYRF6936_INO)
					case PROTO_J6PRO:
						PE2_on;	//antenna RF4
						next_callback = initJ6Pro();
						remote_callback = ReadJ6Pro;
						break;
				#endif
			#endif
			#ifdef NRF24L01_INSTALLED
				#if defined(HISKY_NRF24L01_INO)
					case PROTO_HISKY:
						next_callback=initHiSky();
						remote_callback = hisky_cb;
						break;
				#endif
				#if defined(V2X2_NRF24L01_INO)
					case PROTO_V2X2:
						next_callback = initV2x2();
						remote_callback = ReadV2x2;
						break;
				#endif
				#if defined(YD717_NRF24L01_INO)
					case PROTO_YD717:
						next_callback=initYD717();
						remote_callback = yd717_callback;
						break;
				#endif
				#if defined(KN_NRF24L01_INO)
					case PROTO_KN:
						next_callback = initKN();
						remote_callback = kn_callback;
						break;
				#endif
				#if defined(SYMAX_NRF24L01_INO)
					case PROTO_SYMAX:
						next_callback = initSymax();
						remote_callback = symax_callback;
						break;
				#endif
				#if defined(SLT_NRF24L01_INO)
					case PROTO_SLT:
						next_callback=initSLT();
						remote_callback = SLT_callback;
						break;
				#endif
				#if defined(CX10_NRF24L01_INO)
					case PROTO_Q2X2:
						sub_protocol|=0x08;		// Increase the number of sub_protocols for CX-10
					case PROTO_CX10:
						next_callback=initCX10();
						remote_callback = CX10_callback;
						break;
				#endif
				#if defined(CG023_NRF24L01_INO)
					case PROTO_CG023:
						next_callback=initCG023();
						remote_callback = CG023_callback;
						break;
				#endif
				#if defined(BAYANG_NRF24L01_INO)
					case PROTO_BAYANG:
						next_callback=initBAYANG();
						remote_callback = BAYANG_callback;
						break;
				#endif
				#if defined(ESKY_NRF24L01_INO)
					case PROTO_ESKY:
						next_callback=initESKY();
						remote_callback = ESKY_callback;
						break;
				#endif
				#if defined(MT99XX_NRF24L01_INO)
					case PROTO_MT99XX:
						next_callback=initMT99XX();
						remote_callback = MT99XX_callback;
						break;
				#endif
				#if defined(MJXQ_NRF24L01_INO)
					case PROTO_MJXQ:
						next_callback=initMJXQ();
						remote_callback = MJXQ_callback;
						break;
				#endif
				#if defined(SHENQI_NRF24L01_INO)
					case PROTO_SHENQI:
						next_callback=initSHENQI();
						remote_callback = SHENQI_callback;
						break;
				#endif
				#if defined(FY326_NRF24L01_INO)
					case PROTO_FY326:
						next_callback=initFY326();
						remote_callback = FY326_callback;
						break;
				#endif
				#if defined(FQ777_NRF24L01_INO)
					case PROTO_FQ777:
						next_callback=initFQ777();
						remote_callback = FQ777_callback;
						break;
				#endif
				#if defined(ASSAN_NRF24L01_INO)
					case PROTO_ASSAN:
						next_callback=initASSAN();
						remote_callback = ASSAN_callback;
						break;
				#endif
				#if defined(HONTAI_NRF24L01_INO)
					case PROTO_HONTAI:
						next_callback=initHONTAI();
						remote_callback = HONTAI_callback;
						break;
				#endif
				#if defined(Q303_NRF24L01_INO)
					case PROTO_Q303:
						next_callback=initQ303();
						remote_callback = Q303_callback;
						break;
				#endif
				#if defined(GW008_NRF24L01_INO)
					case PROTO_GW008:
						next_callback=initGW008();
						remote_callback = GW008_callback;
						break;
				#endif
				#if defined(DM002_NRF24L01_INO)
					case PROTO_DM002:
						next_callback=initDM002();
						remote_callback = DM002_callback;
						break;
				#endif
				#if defined(CABELL_NRF24L01_INO)
					case PROTO_CABELL:
						next_callback=initCABELL();
						remote_callback = CABELL_callback;
						break;
				#endif
				#if defined(ESKY150_NRF24L01_INO)
					case PROTO_ESKY150:
						next_callback=initESKY150();
						remote_callback = ESKY150_callback;
						break;
				#endif
				#if defined(H8_3D_NRF24L01_INO)
					case PROTO_H8_3D:
						next_callback=initH8_3D();
						remote_callback = H8_3D_callback;
						break;
				#endif
				#if defined(CFLIE_NRF24L01_INO)
					case PROTO_CFLIE:
						next_callback=initCFlie();
						remote_callback = cflie_callback;
						break;
				#endif
				#if defined(BUGSMINI_NRF24L01_INO)
					case PROTO_BUGSMINI:
						next_callback=initBUGSMINI();
						remote_callback = BUGSMINI_callback;
						break;
				#endif
				#if defined(NCC1701_NRF24L01_INO)
					case PROTO_NCC1701:
						next_callback=initNCC();
						remote_callback = NCC_callback;
						break;
				#endif
				#if defined(E01X_NRF24L01_INO)
					case PROTO_E01X:
						next_callback=initE01X();
						remote_callback = E01X_callback;
						break;
				#endif
				#if defined(V911S_NRF24L01_INO)
					case PROTO_V911S:
						next_callback=initV911S();
						remote_callback = V911S_callback;
						break;
				#endif
				#if defined(GD00X_NRF24L01_INO)
					case PROTO_GD00X:
						next_callback=initGD00X();
						remote_callback = GD00X_callback;
						break;
				#endif
				#if defined(TEST_NRF24L01_INO)
					case PROTO_TEST:
						next_callback=initTest();
						remote_callback = Test_callback;
						break;
				#endif
			#endif
		}
	}

	#if defined(WAIT_FOR_BIND) && defined(ENABLE_BIND_CH)
		if( IS_AUTOBIND_FLAG_on && IS_BIND_CH_PREV_off && (cur_protocol[1]&0x80)==0 && mode_select == MODE_SERIAL)
		{ // Autobind is active but no bind requested by either BIND_CH or BIND. But do not wait if in PPM mode...
			WAIT_BIND_on;
			return;
		}
	#endif
	WAIT_BIND_off;
	CHANGE_PROTOCOL_FLAG_off;

	if(next_callback>32000)
	{ // next_callback should not be more than 32767 so we will wait here...
		uint16_t temp=(next_callback>>10)-2;
		delayMilliseconds(temp);
		next_callback-=temp<<10;				// between 2-3ms left at this stage
	}
	cli();										// disable global int
	OCR1A = TCNT1 + next_callback*2;			// set compare A for callback
	#ifndef STM32_BOARD
		TIFR1 = OCF1A_bm ;						// clear compare A flag
	#else
		TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC1IF;	// Clear Timer2/Comp1 interrupt flag
	#endif	
	sei();										// enable global int
	BIND_BUTTON_FLAG_off;						// do not bind/reset id anymore even if protocol change
}

void update_serial_data()
{
	RX_DONOTUPDATE_on;
	RX_FLAG_off;								//data is being processed
	#ifdef SAMSON	// Extremely dangerous, do not enable this unless you know what you are doing...
		if( rx_ok_buff[0]==0x55 && (rx_ok_buff[1]&0x1F)==PROTO_FRSKYD && rx_ok_buff[2]==0x7F && rx_ok_buff[24]==217 && rx_ok_buff[25]==202 )
		{//proto==FRSKYD+sub==7+rx_num==7+CH15==73%+CH16==73%
			rx_ok_buff[1]=(rx_ok_buff[1]&0xE0) | PROTO_FLYSKY;			// change the protocol to Flysky
			memcpy((void*)(rx_ok_buff+4),(void*)(rx_ok_buff+4+11),11);	// reassign channels 9-16 to 1-8
		}
	#endif
	if(rx_ok_buff[1]&0x20)						//check range
		RANGE_FLAG_on;
	else
		RANGE_FLAG_off;
	if(rx_ok_buff[1]&0x40)						//check autobind
		AUTOBIND_FLAG_on;
	else
		AUTOBIND_FLAG_off;
	if(rx_ok_buff[2]&0x80)						//if rx_ok_buff[2] ==1,power is low ,0-power high
		POWER_FLAG_off;							//power low
	else
		POWER_FLAG_on;							//power high

	//Forced frequency tuning values for CC2500 protocols
	#if defined(FORCE_FRSKYD_TUNING) && defined(FRSKYD_CC2500_INO)
		if(protocol==PROTO_FRSKYD) 
			option=FORCE_FRSKYD_TUNING;	// Use config-defined tuning value for FrSkyD
		else
	#endif
	#if defined(FORCE_FRSKYV_TUNING) && defined(FRSKYV_CC2500_INO)
		if(protocol==PROTO_FRSKYV)
			option=FORCE_FRSKYV_TUNING;	// Use config-defined tuning value for FrSkyV
		else
	#endif
	#if defined(FORCE_FRSKYX_TUNING) && defined(FRSKYX_CC2500_INO)
		if(protocol==PROTO_FRSKYX)
			option=FORCE_FRSKYX_TUNING;	// Use config-defined tuning value for FrSkyX
		else
	#endif 
	#if defined(FORCE_SFHSS_TUNING) && defined(SFHSS_CC2500_INO)
		if (protocol==PROTO_SFHSS)
			option=FORCE_SFHSS_TUNING;	// Use config-defined tuning value for SFHSS
		else
	#endif
	#if defined(FORCE_CORONA_TUNING) && defined(CORONA_CC2500_INO)
		if (protocol==PROTO_CORONA)
			option=FORCE_CORONA_TUNING;	// Use config-defined tuning value for CORONA
		else
	#endif
	#if defined(FORCE_HITEC_TUNING) && defined(HITEC_CC2500_INO)
		if (protocol==PROTO_HITEC)
			option=FORCE_HITEC_TUNING;	// Use config-defined tuning value for HITEC
		else
	#endif
			option=rx_ok_buff[3];		// Use radio-defined option value
	
	#ifdef FAILSAFE_ENABLE
		bool failsafe=false;
		if(rx_ok_buff[0]&0x02)
		{ // Packet contains failsafe instead of channels
			failsafe=true;
			rx_ok_buff[0]&=0xFD;				//remove the failsafe flag
			FAILSAFE_VALUES_on;					//failsafe data has been received
		}
	#endif
	#ifdef BONI
		if(CH14_SW)
			rx_ok_buff[2]=(rx_ok_buff[2]&0xF0)|((rx_ok_buff[2]+1)&0x0F);	// Extremely dangerous, do not enable this!!! This is really for a special case...
	#endif
	if( (rx_ok_buff[0] != cur_protocol[0]) || ((rx_ok_buff[1]&0x5F) != (cur_protocol[1]&0x5F)) || ( (rx_ok_buff[2]&0x7F) != (cur_protocol[2]&0x7F) ) )
	{ // New model has been selected
		CHANGE_PROTOCOL_FLAG_on;				//change protocol
		WAIT_BIND_off;
		if((rx_ok_buff[1]&0x80)!=0 || IS_AUTOBIND_FLAG_on)
			BIND_IN_PROGRESS;					//launch bind right away if in autobind mode or bind is set
		else
			BIND_DONE;
		protocol=(rx_ok_buff[0]==0x55?0:32) + (rx_ok_buff[1]&0x1F);	//protocol no (0-63) bits 4-6 of buff[1] and bit 0 of buf[0]
		sub_protocol=(rx_ok_buff[2]>>4)& 0x07;	//subprotocol no (0-7) bits 4-6
		RX_num=rx_ok_buff[2]& 0x0F;				// rx_num bits 0---3
	}
	else
		if( ((rx_ok_buff[1]&0x80)!=0) && ((cur_protocol[1]&0x80)==0) )		// Bind flag has been set
		{ // Restart protocol with bind
			CHANGE_PROTOCOL_FLAG_on;
			BIND_IN_PROGRESS;
		}
		else
			if( ((rx_ok_buff[1]&0x80)==0) && ((cur_protocol[1]&0x80)!=0) )	// Bind flag has been reset
			{ // Request protocol to end bind
				#if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO) || defined(FRSKYV_CC2500_INO)
				if(protocol==PROTO_FRSKYD || protocol==PROTO_FRSKYX || protocol==PROTO_FRSKYV)
					BIND_DONE;
				else
				#endif
				if(bind_counter>2)
					bind_counter=2;
			}
			
	//store current protocol values
	for(uint8_t i=0;i<3;i++)
		cur_protocol[i] =  rx_ok_buff[i];

	// decode channel/failsafe values
	volatile uint8_t *p=rx_ok_buff+3;
	uint8_t dec=-3;
	for(uint8_t i=0;i<NUM_CHN;i++)
	{
		dec+=3;
		if(dec>=8)
		{
			dec-=8;
			p++;
		}
		p++;
		uint16_t temp=((*((uint32_t *)p))>>dec)&0x7FF;
		#ifdef FAILSAFE_ENABLE
			if(failsafe)
				Failsafe_data[i]=temp;			//value range 0..2047, 0=no pulses, 2047=hold
			else
		#endif
				Channel_data[i]=temp;			//value range 0..2047, 0=-125%, 2047=+125%
	}
	RX_DONOTUPDATE_off;
	#ifdef ORANGE_TX
		cli();
	#else
		UCSR0B &= ~_BV(RXCIE0);					// RX interrupt disable
	#endif
	if(IS_RX_MISSED_BUFF_on)					// If the buffer is still valid
	{	memcpy((void*)rx_ok_buff,(const void*)rx_buff,RXBUFFER_SIZE);// Duplicate the buffer
		RX_FLAG_on;								// data to be processed next time...
		RX_MISSED_BUFF_off;
	}
	#ifdef ORANGE_TX
		sei();
	#else
		UCSR0B |= _BV(RXCIE0) ;					// RX interrupt enable
	#endif
	#ifdef FAILSAFE_ENABLE
		if(failsafe)
			debugln("RX_FS:%d,%d,%d,%d",Failsafe_data[0],Failsafe_data[1],Failsafe_data[2],Failsafe_data[3]);
	#endif
}

void modules_reset()
{
	#ifdef	CC2500_INSTALLED
		CC2500_Reset();
	#endif
	#ifdef	A7105_INSTALLED
		A7105_Reset();
	#endif
	#ifdef	CYRF6936_INSTALLED
		CYRF_Reset();
	#endif
	#ifdef	NRF24L01_INSTALLED
		NRF24L01_Reset();
	#endif

	//Wait for every component to reset
	delayMilliseconds(100);
	prev_power=0xFD;		// unused power value
}

#ifdef CHECK_FOR_BOOTLOADER
	void Mprotocol_serial_init( uint8_t boot )
#else
	void Mprotocol_serial_init()
#endif
{
	#ifdef ORANGE_TX
		PORTC.OUTSET = 0x08 ;
		PORTC.DIRSET = 0x08 ;

		USARTC0.BAUDCTRLA = 19 ;
		USARTC0.BAUDCTRLB = 0 ;
		
		USARTC0.CTRLB = 0x18 ;
		USARTC0.CTRLA = (USARTC0.CTRLA & 0xCC) | 0x11 ;
		USARTC0.CTRLC = 0x2B ;
		UDR0 ;
		#ifdef INVERT_SERIAL
			PORTC.PIN3CTRL |= 0x40 ;
		#endif
		#ifdef CHECK_FOR_BOOTLOADER
			if ( boot )
			{
				USARTC0.BAUDCTRLB = 0 ;
				USARTC0.BAUDCTRLA = 33 ;		// 57600
				USARTC0.CTRLA = (USARTC0.CTRLA & 0xC0) ;
				USARTC0.CTRLC = 0x03 ;			// 8 bit, no parity, 1 stop
				USARTC0.CTRLB = 0x18 ;			// Enable Tx and Rx
				PORTC.PIN3CTRL &= ~0x40 ;
			}
		#endif // CHECK_FOR_BOOTLOADER
	#elif defined STM32_BOARD
		#ifdef CHECK_FOR_BOOTLOADER
			if ( boot )
			{
				usart2_begin(57600,SERIAL_8N1);
				USART2_BASE->CR1 &= ~USART_CR1_RXNEIE ;
				(void)UDR0 ;
			}
			else
		#endif // CHECK_FOR_BOOTLOADER
		{
			usart2_begin(100000,SERIAL_8E2);
			USART2_BASE->CR1 |= USART_CR1_PCE_BIT;
		}
		usart3_begin(100000,SERIAL_8E2);
		#ifndef SPORT_POLLING
			USART3_BASE->CR1 &= ~ USART_CR1_RE;	//disable receive
		#endif		
		USART2_BASE->CR1 &= ~ USART_CR1_TE;		//disable transmit
	#else
		//ATMEGA328p
		#include <util/setbaud.h>	
		UBRR0H = UBRRH_VALUE;
		UBRR0L = UBRRL_VALUE;
		UCSR0A = 0 ;	// Clear X2 bit
		//Set frame format to 8 data bits, even parity, 2 stop bits
		UCSR0C = _BV(UPM01)|_BV(USBS0)|_BV(UCSZ01)|_BV(UCSZ00);
		while ( UCSR0A & (1 << RXC0) )	//flush receive buffer
			UDR0;
		//enable reception and RC complete interrupt
		UCSR0B = _BV(RXEN0)|_BV(RXCIE0);//rx enable and interrupt
		#ifndef DEBUG_PIN
			#if defined(TELEMETRY)
				initTXSerial( SPEED_100K ) ;
			#endif //TELEMETRY
		#endif //DEBUG_PIN
		#ifdef CHECK_FOR_BOOTLOADER
			if ( boot )
			{
				UBRR0H = 0;
				UBRR0L = 33;			// 57600
				UCSR0C &= ~_BV(UPM01);	// No parity
				UCSR0B &= ~_BV(RXCIE0);	// No rx interrupt
				UCSR0A |= _BV(U2X0);	// Double speed mode USART0
			}
		#endif // CHECK_FOR_BOOTLOADER
	#endif //ORANGE_TX
}

#ifdef STM32_BOARD
	void usart2_begin(uint32_t baud,uint32_t config )
	{
		usart_init(USART2); 
		usart_config_gpios_async(USART2,GPIOA,PIN_MAP[PA3].gpio_bit,GPIOA,PIN_MAP[PA2].gpio_bit,config);
		LED2_output;
		usart_set_baud_rate(USART2, STM32_PCLK1, baud);
		usart_enable(USART2);
	}
	void usart3_begin(uint32_t baud,uint32_t config )
	{
		usart_init(USART3);
		usart_config_gpios_async(USART3,GPIOB,PIN_MAP[PB11].gpio_bit,GPIOB,PIN_MAP[PB10].gpio_bit,config);
		usart_set_baud_rate(USART3, STM32_PCLK1, baud);
		usart_enable(USART3);
	}
	void init_HWTimer()
	{	
		HWTimer2.pause();									// Pause the timer2 while we're configuring it
		
		TIMER2_BASE->PSC = 35;								// 36-1;for 72 MHZ /0.5sec/(35+1)
		TIMER2_BASE->ARR = 0xFFFF;							// Count until 0xFFFF
		
		HWTimer2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);	// Main scheduler
		HWTimer2.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);	// Serial check

		TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC2IF;			// Clear Timer2/Comp2 interrupt flag
		HWTimer2.attachInterrupt(TIMER_CH2,ISR_COMPB);		// Assign function to Timer2/Comp2 interrupt
		TIMER2_BASE->DIER &= ~TIMER_DIER_CC2IE;				// Disable Timer2/Comp2 interrupt
		
		HWTimer2.refresh();									// Refresh the timer's count, prescale, and overflow
		HWTimer2.resume();
	}
#endif

#ifdef CHECK_FOR_BOOTLOADER
void pollBoot()
{
	uint8_t rxchar ;
	uint8_t lState = BootState ;
	uint8_t millisTime = millis();				// Call this once only

	#ifdef ORANGE_TX
	if ( USARTC0.STATUS & USART_RXCIF_bm )
	#elif defined STM32_BOARD
	if ( USART2_BASE->SR & USART_SR_RXNE )
	#else
	if ( UCSR0A & ( 1 << RXC0 ) )
	#endif
	{
		rxchar = UDR0 ;
		BootCount += 1 ;
		if ( ( lState == BOOT_WAIT_30_IDLE ) || ( lState == BOOT_WAIT_30_DATA ) )
		{
			if ( lState == BOOT_WAIT_30_IDLE )	// Waiting for 0x30
				BootTimer = millisTime ;		// Start timeout
			if ( rxchar == 0x30 )
				lState = BOOT_WAIT_20 ;
			else
				lState = BOOT_WAIT_30_DATA ;
		}
		else
			if ( lState == BOOT_WAIT_20 && rxchar == 0x20 )	// Waiting for 0x20
				lState = BOOT_READY ;
	}
	else // No byte received
	{
		if ( lState != BOOT_WAIT_30_IDLE )		// Something received
		{
			uint8_t time = millisTime - BootTimer ;
			if ( time > 5 )
			{
				#ifdef	STM32_BOARD
				if ( BootCount > 4 )
				#else
				if ( BootCount > 2 )
				#endif
				{ // Run normally
					NotBootChecking = 0xFF ;
					Mprotocol_serial_init( 0 ) ;
				}
				else if ( lState == BOOT_READY )
				{
					#ifdef	STM32_BOARD
						nvic_sys_reset();
						while(1);						/* wait until reset */
					#else
						cli();							// Disable global int due to RW of 16 bits registers
						void (*p)();
						#ifndef ORANGE_TX
							p = (void (*)())0x3F00 ;	// Word address (0x7E00 byte)
						#else
							p = (void (*)())0x4000 ;	// Word address (0x8000 byte)
						#endif
						(*p)() ;						// go to boot
					#endif
				}
				else
				{
					lState = BOOT_WAIT_30_IDLE ;
					BootCount = 0 ;
				}
			}
		}
	}
	BootState = lState ;
}
#endif //CHECK_FOR_BOOTLOADER

#if defined(TELEMETRY)
void PPM_Telemetry_serial_init()
{
	if( (protocol==PROTO_FRSKYD) || (protocol==PROTO_HUBSAN) || (protocol==PROTO_AFHDS2A) || (protocol==PROTO_BAYANG)|| (protocol==PROTO_NCC1701) || (protocol==PROTO_CABELL)  || (protocol==PROTO_HITEC) || (protocol==PROTO_BUGS) || (protocol==PROTO_BUGSMINI))
		initTXSerial( SPEED_9600 ) ;
	if(protocol==PROTO_FRSKYX)
		initTXSerial( SPEED_57600 ) ;
	if(protocol==PROTO_DSM)
		initTXSerial( SPEED_125K ) ;
}
#endif

// Convert 32b id to rx_tx_addr
static void set_rx_tx_addr(uint32_t id)
{ // Used by almost all protocols
	rx_tx_addr[0] = (id >> 24) & 0xFF;
	rx_tx_addr[1] = (id >> 16) & 0xFF;
	rx_tx_addr[2] = (id >>  8) & 0xFF;
	rx_tx_addr[3] = (id >>  0) & 0xFF;
	rx_tx_addr[4] = (rx_tx_addr[2]&0xF0)|(rx_tx_addr[3]&0x0F);
}

static uint32_t random_id(uint16_t address, uint8_t create_new)
{
	#ifndef FORCE_GLOBAL_ID
		uint32_t id=0;

		if(eeprom_read_byte((EE_ADDR)(address+10))==0xf0 && !create_new)
		{  // TXID exists in EEPROM
			for(uint8_t i=4;i>0;i--)
			{
				id<<=8;
				id|=eeprom_read_byte((EE_ADDR)address+i-1);
			}
			if(id!=0x2AD141A7)	//ID with seed=0
			{
				debugln("Read ID from EEPROM");
				return id;
			}
		}
		// Generate a random ID
		#if defined STM32_BOARD
			#define STM32_UUID ((uint32_t *)0x1FFFF7E8)
			if (!create_new)
			{
				id = STM32_UUID[0] ^ STM32_UUID[1] ^ STM32_UUID[2];
				debugln("Generated ID from STM32 UUID");
			}
			else
		#endif
				id = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);

		for(uint8_t i=0;i<4;i++)
			eeprom_write_byte((EE_ADDR)address+i,id >> (i*8));
		eeprom_write_byte((EE_ADDR)(address+10),0xf0);//write bind flag in eeprom.
		return id;
	#else
		(void)address;
		(void)create_new;
		return FORCE_GLOBAL_ID;
	#endif
}

/**************************/
/**************************/
/**  Interrupt routines  **/
/**************************/
/**************************/

//PPM
#ifdef ENABLE_PPM
	#ifdef ORANGE_TX
		#if PPM_pin == 2
			ISR(PORTD_INT0_vect)
		#else
			ISR(PORTD_INT1_vect)
		#endif
	#elif defined STM32_BOARD
		void PPM_decode()
	#else
		#if PPM_pin == 2
			ISR(INT0_vect, ISR_NOBLOCK)
		#else
			ISR(INT1_vect, ISR_NOBLOCK)
		#endif
	#endif
	{	// Interrupt on PPM pin
		static int8_t chan=0,bad_frame=1;
		static uint16_t Prev_TCNT1=0;
		uint16_t Cur_TCNT1;

		Cur_TCNT1 = TCNT1 - Prev_TCNT1 ;	// Capture current Timer1 value
		if(Cur_TCNT1<1600)
			bad_frame=1;					// bad frame
		else
			if(Cur_TCNT1>4400)
			{  //start of frame
				if(chan>=MIN_PPM_CHANNELS)
				{
					PPM_FLAG_on;			// good frame received if at least 4 channels have been seen
					if(chan>PPM_chan_max) PPM_chan_max=chan;	// Saving the number of channels received
				}
				chan=0;						// reset channel counter
				bad_frame=0;
			}
			else
				if(bad_frame==0)			// need to wait for start of frame
				{  //servo values between 800us and 2200us will end up here
					PPM_data[chan]=Cur_TCNT1;
					if(chan++>=MAX_PPM_CHANNELS)
						bad_frame=1;		// don't accept any new channels
				}
		Prev_TCNT1+=Cur_TCNT1;
	}
#endif //ENABLE_PPM

//Serial RX
#ifdef ENABLE_SERIAL
	#ifdef ORANGE_TX
		ISR(USARTC0_RXC_vect)
	#elif defined STM32_BOARD
		void __irq_usart2()			
	#else
		ISR(USART_RX_vect)
	#endif
	{	// RX interrupt
		static uint8_t idx=0;
		#ifdef ORANGE_TX
			if((USARTC0.STATUS & 0x1C)==0)		// Check frame error, data overrun and parity error
		#elif defined STM32_BOARD
			if((USART2_BASE->SR & USART_SR_RXNE) && (USART2_BASE->SR &0x0F)==0)					
		#else
			UCSR0B &= ~_BV(RXCIE0) ;			// RX interrupt disable
			sei() ;
			if((UCSR0A&0x1C)==0)				// Check frame error, data overrun and parity error
		#endif
		{ // received byte is ok to process
			if(idx==0||discard_frame==1)
			{	// Let's try to sync at this point
				idx=0;discard_frame=0;
				RX_MISSED_BUFF_off;			// If rx_buff was good it's not anymore...
				rx_buff[0]=UDR0;
				#ifdef FAILSAFE_ENABLE
					if((rx_buff[0]&0xFC)==0x54)	// If 1st byte is 0x54, 0x55, 0x56 or 0x57 it looks ok
				#else
					if((rx_buff[0]&0xFE)==0x54)	// If 1st byte is 0x54 or 0x55 it looks ok
				#endif
				{
					TX_RX_PAUSE_on;
					tx_pause();
					#if defined STM32_BOARD
						TIMER2_BASE->CCR2=TIMER2_BASE->CNT+(6500L);	// Full message should be received within timer of 3250us
						TIMER2_BASE->SR = 0x1E5F & ~TIMER_SR_CC2IF;	// Clear Timer2/Comp2 interrupt flag
						TIMER2_BASE->DIER |= TIMER_DIER_CC2IE;		// Enable Timer2/Comp2 interrupt
					#else
						OCR1B = TCNT1+(6500L) ;	// Full message should be received within timer of 3250us
						TIFR1 = OCF1B_bm ;		// clear OCR1B match flag
						SET_TIMSK1_OCIE1B ;		// enable interrupt on compare B match
					#endif
					idx++;
				}
			}
			else
			{
				rx_buff[idx++]=UDR0;		// Store received byte
				if(idx>=RXBUFFER_SIZE)
				{	// A full frame has been received
					if(!IS_RX_DONOTUPDATE_on)
					{ //Good frame received and main is not working on the buffer
						memcpy((void*)rx_ok_buff,(const void*)rx_buff,RXBUFFER_SIZE);// Duplicate the buffer
						RX_FLAG_on;			// flag for main to process servo data
					}
					else
						RX_MISSED_BUFF_on;	// notify that rx_buff is good
					discard_frame=1; 		// start again
				}
			}
		}
		else
		{
			idx=UDR0;						// Dummy read
			discard_frame=1;				// Error encountered discard full frame...
			debugln("Bad frame RX");
		}
		if(discard_frame==1)
		{
			#ifdef STM32_BOARD
				TIMER2_BASE->DIER &= ~TIMER_DIER_CC2IE;	// Disable Timer2/Comp2 interrupt
			#else							
				CLR_TIMSK1_OCIE1B;			// Disable interrupt on compare B match
			#endif
			TX_RX_PAUSE_off;
			tx_resume();
		}
		#if not defined (ORANGE_TX) && not defined (STM32_BOARD)
			cli() ;
			UCSR0B |= _BV(RXCIE0) ;			// RX interrupt enable
		#endif
	}

	//Serial timer
	#ifdef ORANGE_TX
		ISR(TCC1_CCB_vect)
	#elif defined STM32_BOARD
		void ISR_COMPB()
	#else
		ISR(TIMER1_COMPB_vect, ISR_NOBLOCK )
	#endif
	{	// Timer1 compare B interrupt
		discard_frame=1;
		#ifdef STM32_BOARD
			TIMER2_BASE->DIER &= ~TIMER_DIER_CC2IE;	// Disable Timer2/Comp2 interrupt
			debugln("Bad frame timer");
		#else
			CLR_TIMSK1_OCIE1B;						// Disable interrupt on compare B match
		#endif
		tx_resume();
	}
#endif //ENABLE_SERIAL

#if not defined (ORANGE_TX) && not defined (STM32_BOARD)
	static void random_init(void)
	{
		cli();					// Temporarily turn off interrupts, until WDT configured
		MCUSR = 0;				// Use the MCU status register to reset flags for WDR, BOR, EXTR, and POWR
		WDTCSR |= _BV(WDCE);	// WDT control register, This sets the Watchdog Change Enable (WDCE) flag, which is  needed to set the prescaler
		WDTCSR = _BV(WDIE);		// Watchdog interrupt enable (WDIE)
		sei();					// Turn interupts on
	}

	static uint32_t random_value(void)
	{
		while (!gWDT_entropy);
		return gWDT_entropy;
	}

	// Random interrupt service routine called every time the WDT interrupt is triggered.
	// It is only enabled at startup to generate a seed.
	ISR(WDT_vect)
	{
		static uint8_t gWDT_buffer_position=0;
		#define gWDT_buffer_SIZE 32
		static uint8_t gWDT_buffer[gWDT_buffer_SIZE];
		gWDT_buffer[gWDT_buffer_position] = TCNT1L; // Record the Timer 1 low byte (only one needed) 
		gWDT_buffer_position++;                     // every time the WDT interrupt is triggered
		if (gWDT_buffer_position >= gWDT_buffer_SIZE)
		{
			// The following code is an implementation of Jenkin's one at a time hash
			for(uint8_t gWDT_loop_counter = 0; gWDT_loop_counter < gWDT_buffer_SIZE; ++gWDT_loop_counter)
			{
				gWDT_entropy += gWDT_buffer[gWDT_loop_counter];
				gWDT_entropy += (gWDT_entropy << 10);
				gWDT_entropy ^= (gWDT_entropy >> 6);
			}
			gWDT_entropy += (gWDT_entropy << 3);
			gWDT_entropy ^= (gWDT_entropy >> 11);
			gWDT_entropy += (gWDT_entropy << 15);
			WDTCSR = 0;	// Disable Watchdog interrupt
		}
	}
#endif


# 1 "src/SPI.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
/********************/
/**  SPI routines  **/
/********************/
#ifdef STM32_BOARD

#ifdef DEBUG_SERIAL
//	#define DEBUG_SPI
#endif

SPIClass SPI_2(2); 								//Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port

void initSPI2()
{
	//SPI_DISABLE();
	SPI_2.end();
	SPI2_BASE->CR1 &= ~SPI_CR1_DFF_8_BIT;		//8 bits format, this bit should be written only when SPI is disabled (SPE = ?0?) for correct operation.
	SPI_2.begin();								//Initialize the SPI_2 port.

	SPI2_BASE->CR1 &= ~SPI_CR1_LSBFIRST;		// Set the SPI_2 bit order MSB first
	SPI2_BASE->CR1 &= ~(SPI_CR1_CPOL|SPI_CR1_CPHA);	// Set the SPI_2 data mode 0: Clock idles low, data captured on rising edge (first transition)
	SPI2_BASE->CR1 &= ~(SPI_CR1_BR);
	SPI2_BASE->CR1 |= SPI_CR1_BR_PCLK_DIV_8;	// Set the speed (36 / 8 = 4.5 MHz SPI_2 speed) SPI_CR1_BR_PCLK_DIV_8
}
	
void SPI_Write(uint8_t command)
{//working OK	
	SPI2_BASE->DR = command;					//Write the first data item to be transmitted into the SPI_DR register (this clears the TXE flag).
	#ifdef DEBUG_SPI
		debug("%02X ",command);
	#endif
	while (!(SPI2_BASE->SR & SPI_SR_RXNE));
	command = SPI2_BASE->DR;					// ... and read the last received data.
}

uint8_t SPI_Read(void)
{
	SPI_Write(0x00);		
	return SPI2_BASE->DR;
}

uint8_t SPI_SDI_Read()
{	
	uint8_t rx=0;
	cli();	//Fix Hubsan droputs??
	while(!(SPI2_BASE->SR & SPI_SR_TXE));
	while((SPI2_BASE->SR & SPI_SR_BSY));	
	//	
	SPI_DISABLE();
	SPI_SET_BIDIRECTIONAL();
	volatile uint8_t x = SPI2_BASE->DR;
	(void)x;
	SPI_ENABLE();
	//
	SPI_DISABLE();				  
	while(!(SPI2_BASE->SR& SPI_SR_RXNE));
	rx=SPI2_BASE->DR;
	SPI_SET_UNIDIRECTIONAL();
	SPI_ENABLE();
	sei();//fix Hubsan dropouts??
	return rx;
}

void SPI_ENABLE()
{
	SPI2_BASE->CR1 |= SPI_CR1_SPE;
}

void SPI_DISABLE()
{
	SPI2_BASE->CR1 &= ~SPI_CR1_SPE;
}

void SPI_SET_BIDIRECTIONAL()
{
	SPI2_BASE->CR1 |= SPI_CR1_BIDIMODE;
	SPI2_BASE->CR1  &= ~ SPI_CR1_BIDIOE;//receive only
}

void SPI_SET_UNIDIRECTIONAL()
{
	SPI2_BASE->CR1 &= ~SPI_CR1_BIDIMODE;
}

#else

#ifdef ORANGE_TX
	#define XNOP() NOP()
#else
	#define XNOP()
#endif

void SPI_Write(uint8_t command)
{
	uint8_t n=8; 

	SCLK_off;//SCK start low
	XNOP();
	SDI_off;
	XNOP();
	do
	{
		if(command&0x80)
			SDI_on;
		else
			SDI_off;
		XNOP();
		SCLK_on;
		XNOP();
		XNOP();
		command = command << 1;
		SCLK_off;
		XNOP();
	}
	while(--n) ;
	SDI_on;
}

uint8_t SPI_Read(void)
{
	uint8_t result=0,i;
	for(i=0;i<8;i++)
	{
		result=result<<1;
		if(SDO_1)
			result |= 0x01;
		SCLK_on;
		XNOP();
		XNOP();
		NOP();
		SCLK_off;
		XNOP();
		XNOP();
	}
	return result;
}

#ifdef A7105_INSTALLED
uint8_t SPI_SDI_Read(void)
{
	uint8_t result=0;
	SDI_input;
	for(uint8_t i=0;i<8;i++)
	{                    
		result=result<<1;
		if(SDI_1)  ///if SDIO =1 
			result |= 0x01;
		SCLK_on;
		NOP();
		SCLK_off;
	}
	SDI_output;
	return result;
}
#endif

#endif//STM32_BOARD

# 1 "src/A7105_SPI.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
/********************/
/** A7105 routines **/
/********************/
#ifdef A7105_INSTALLED
#include "iface_a7105.h"

void A7105_WriteData(uint8_t len, uint8_t channel)
{
	uint8_t i;
	A7105_CSN_off;
	SPI_Write(A7105_RST_WRPTR);
	SPI_Write(A7105_05_FIFO_DATA);
	for (i = 0; i < len; i++)
		SPI_Write(packet[i]);
	A7105_CSN_on;
	if(protocol!=PROTO_FLYSKY)
	{
		A7105_Strobe(A7105_STANDBY);	//Force standby mode, ie cancel any TX or RX...
		A7105_SetTxRxMode(TX_EN);		//Switch to PA
	}
	A7105_WriteReg(A7105_0F_PLL_I, channel);
	A7105_Strobe(A7105_TX);
}

void A7105_ReadData(uint8_t len)
{
	uint8_t i;
	A7105_Strobe(A7105_RST_RDPTR);
	A7105_CSN_off;
	SPI_Write(0x40 | A7105_05_FIFO_DATA);	//bit 6 =1 for reading
	for (i=0;i<len;i++)
		packet[i]=SPI_SDI_Read();
	A7105_CSN_on;
}

void A7105_WriteReg(uint8_t address, uint8_t data) {
	A7105_CSN_off;
	SPI_Write(address); 
	NOP();
	SPI_Write(data);  
	A7105_CSN_on;
} 

uint8_t A7105_ReadReg(uint8_t address)
{ 
	uint8_t result;
	A7105_CSN_off;
	SPI_Write(address |=0x40);				//bit 6 =1 for reading
	result = SPI_SDI_Read();  
	A7105_CSN_on;
	return(result); 
} 

//------------------------
void A7105_SetTxRxMode(uint8_t mode)
{
	if(mode == TX_EN)
	{
		A7105_WriteReg(A7105_0B_GPIO1_PIN1, 0x33);
		A7105_WriteReg(A7105_0C_GPIO2_PIN_II, 0x31);
	}
	else
		if (mode == RX_EN)
		{
			A7105_WriteReg(A7105_0B_GPIO1_PIN1, 0x31);
			A7105_WriteReg(A7105_0C_GPIO2_PIN_II, 0x33);
		}
		else
		{
			//The A7105 seems to some with a cross-wired power-amp (A7700)
			//On the XL7105-D03, TX_EN -> RXSW and RX_EN -> TXSW
			//This means that sleep mode is wired as RX_EN = 1 and TX_EN = 1
			//If there are other amps in use, we'll need to fix this
			A7105_WriteReg(A7105_0B_GPIO1_PIN1, 0x33);
			A7105_WriteReg(A7105_0C_GPIO2_PIN_II, 0x33);
		}
}

//------------------------
uint8_t A7105_Reset()
{
	uint8_t result;
	
	A7105_WriteReg(A7105_00_MODE, 0x00);
	delayMilliseconds(1);
	A7105_SetTxRxMode(TXRX_OFF);			//Set both GPIO as output and low
	result=A7105_ReadReg(A7105_10_PLL_II) == 0x9E;	//check if is reset.
	A7105_Strobe(A7105_STANDBY);
	return result;
}

void A7105_WriteID(uint32_t ida)
{
	A7105_CSN_off;
	SPI_Write(A7105_06_ID_DATA);			//ex id=0x5475c52a ;txid3txid2txid1txid0
	SPI_Write((ida>>24)&0xff);				//53 
	SPI_Write((ida>>16)&0xff);				//75
	SPI_Write((ida>>8)&0xff);				//c5
	SPI_Write((ida>>0)&0xff);				//2a
	A7105_CSN_on;
}

/*
static void A7105_SetPower_Value(int power)
{
	//Power amp is ~+16dBm so:
	//TXPOWER_100uW  = -23dBm == PAC=0 TBG=0
	//TXPOWER_300uW  = -20dBm == PAC=0 TBG=1
	//TXPOWER_1mW    = -16dBm == PAC=0 TBG=2
	//TXPOWER_3mW    = -11dBm == PAC=0 TBG=4
	//TXPOWER_10mW   = -6dBm  == PAC=1 TBG=5
	//TXPOWER_30mW   = 0dBm   == PAC=2 TBG=7
	//TXPOWER_100mW  = 1dBm   == PAC=3 TBG=7
	//TXPOWER_150mW  = 1dBm   == PAC=3 TBG=7
	uint8_t pac, tbg;
	switch(power) {
		case 0: pac = 0; tbg = 0; break;
		case 1: pac = 0; tbg = 1; break;
		case 2: pac = 0; tbg = 2; break;
		case 3: pac = 0; tbg = 4; break;
		case 4: pac = 1; tbg = 5; break;
		case 5: pac = 2; tbg = 7; break;
		case 6: pac = 3; tbg = 7; break;
		case 7: pac = 3; tbg = 7; break;
		default: pac = 0; tbg = 0; break;
	};
	A7105_WriteReg(0x28, (pac << 3) | tbg);
}
*/

void A7105_SetPower()
{
	uint8_t power=A7105_BIND_POWER;
	if(IS_BIND_DONE)
		#ifdef A7105_ENABLE_LOW_POWER
			power=IS_POWER_FLAG_on?A7105_HIGH_POWER:A7105_LOW_POWER;
		#else
			power=A7105_HIGH_POWER;
		#endif
	if(IS_RANGE_FLAG_on)
		power=A7105_RANGE_POWER;
	if(prev_power != power)
	{
		A7105_WriteReg(A7105_28_TX_TEST, power);
		prev_power=power;
	}
}

void A7105_Strobe(uint8_t address) {
	A7105_CSN_off;
	SPI_Write(address);
	A7105_CSN_on;
}

// Fine tune A7105 LO base frequency
// this is required for some A7105 modules and/or RXs with inaccurate crystal oscillator
void A7105_AdjustLOBaseFreq(uint8_t cmd)
{
	static int16_t old_offset=2048;
	int16_t offset=1024;
	if(cmd==0)
	{	// Called at init of the A7105
		old_offset=2048;
		switch(protocol)
		{
			case PROTO_HUBSAN:
				#ifdef FORCE_HUBSAN_TUNING
					offset=(int16_t)FORCE_HUBSAN_TUNING;
				#endif
				break;
			case PROTO_BUGS:
				#ifdef FORCE_HUBSAN_TUNING
					offset=(int16_t)FORCE_HUBSAN_TUNING;
				#endif
				break;
			case PROTO_FLYSKY:
				#ifdef FORCE_FLYSKY_TUNING
					offset=(int16_t)FORCE_FLYSKY_TUNING;
				#endif
				break;
			case PROTO_AFHDS2A:
				#ifdef FORCE_AFHDS2A_TUNING
					offset=(int16_t)FORCE_AFHDS2A_TUNING;
				#endif
				break;
		}
	}
	if(offset==1024)	// Use channel 15 as an input
		offset=convert_channel_16b_nolimit(CH15,-300,300);

	if(old_offset==offset)	// offset is the same as before...
			return;
	old_offset=offset;

	// LO base frequency = 32e6*(bip+(bfp/(2^16)))
	uint8_t bip;	// LO base frequency integer part
	uint16_t bfp;	// LO base frequency fractional part
	offset++;		// as per datasheet, not sure why recommended, but that's a +1kHz drift only ...
	offset<<=1;
	if(offset < 0)
	{
		bip = 0x4a;	// 2368 MHz
		bfp = 0xffff + offset;
	}
	else
	{
		bip = 0x4b;	// 2400 MHz (default)
		bfp = offset;
	}
	A7105_WriteReg( A7105_11_PLL_III, bip);
	A7105_WriteReg( A7105_12_PLL_IV, (bfp >> 8) & 0xff);
	A7105_WriteReg( A7105_13_PLL_V, bfp & 0xff);
	//debugln("Channel: %d, offset: %d, bip: %2x, bfp: %4x", Channel_data[14], offset, bip, bfp);
}

static void __attribute__((unused)) A7105_SetVCOBand(uint8_t vb1, uint8_t vb2)
{	// Set calibration band value to best match
	uint8_t diff1, diff2;

	if (vb1 >= 4)
		diff1 = vb1 - 4;
	else
		diff1 = 4 - vb1;

	if (vb2 >= 4)
		diff2 = vb2 - 4;
	else
		diff2 = 4 - vb2;

	if (diff1 == diff2 || diff1 > diff2)
		A7105_WriteReg(A7105_25_VCO_SBCAL_I, vb1 | 0x08);
	else
		A7105_WriteReg(A7105_25_VCO_SBCAL_I, vb2 | 0x08);
}

#ifdef HUBSAN_A7105_INO
const uint8_t PROGMEM HUBSAN_A7105_regs[] = {
	0xFF, 0x63, 0xFF, 0x0F, 0xFF, 0xFF, 0xFF ,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x05, 0x04, 0xFF,	// 00 - 0f
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x2B, 0xFF, 0xFF, 0x62, 0x80, 0xFF, 0xFF, 0x0A, 0xFF, 0xFF, 0x07,	// 10 - 1f
	0x17, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x47, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,	// 20 - 2f
	0xFF, 0xFF // 30 - 31
};
#endif
#ifdef FLYSKY_A7105_INO
const uint8_t PROGMEM FLYSKY_A7105_regs[] = {
	0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,	// 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,	// 10 - 1f
	0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,	// 20 - 2f
	0x01, 0x0f // 30 - 31
};
#endif
#ifdef AFHDS2A_A7105_INO
const uint8_t PROGMEM AFHDS2A_A7105_regs[] = {
	0xFF, 0x42 | (1<<5), 0x00, 0x25, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x01, 0x3c, 0x05, 0x00, 0x50,	// 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x4f, 0x62, 0x80, 0xFF, 0xFF, 0x2a, 0x32, 0xc3, 0x1f,				// 10 - 1f
	0x1e, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,				// 20 - 2f
	0x01, 0x0f // 30 - 31
};
#endif
#ifdef BUGS_A7105_INO
const uint8_t PROGMEM BUGS_A7105_regs[] = {
	0xFF, 0x42, 0x00, 0x15, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x05, 0x01, 0x50,	// 00 - 0f
	0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x40, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,	// 10 - 1f
	0x16, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x3b, 0x00, 0x0b, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,	// 20 - 2f
	0x01, 0x0f // 30 - 31
};
#endif

#define ID_NORMAL 0x55201041
#define ID_PLUS   0xAA201041
void A7105_Init(void)
{
	uint8_t *A7105_Regs=0;
    uint8_t vco_calibration0, vco_calibration1;
	
	#ifdef BUGS_A7105_INO
		if(protocol==PROTO_BUGS)
			A7105_Regs=(uint8_t*)BUGS_A7105_regs;
		else
	#endif
	#ifdef HUBSAN_A7105_INO
		if(protocol==PROTO_HUBSAN)
		{
			A7105_WriteID(ID_NORMAL);
			A7105_Regs=(uint8_t*)HUBSAN_A7105_regs;
		}
		else
	#endif
		{
			A7105_WriteID(0x5475c52A);//0x2Ac57554
			#ifdef FLYSKY_A7105_INO
				if(protocol==PROTO_FLYSKY)
					A7105_Regs=(uint8_t*)FLYSKY_A7105_regs;
				else
			#endif
				{
					#ifdef AFHDS2A_A7105_INO
						A7105_Regs=(uint8_t*)AFHDS2A_A7105_regs;
					#endif
				}
		}

	for (uint8_t i = 0; i < 0x32; i++)
	{
		uint8_t val=pgm_read_byte_near(&A7105_Regs[i]);
		#ifdef FLYSKY_A7105_INO
			if(protocol==PROTO_FLYSKY && sub_protocol==CX20)
			{
				if(i==0x0E) val=0x01;
				if(i==0x1F) val=0x1F;
				if(i==0x20) val=0x1E;
			}
		#endif
		if( val != 0xFF)
			A7105_WriteReg(i, val);
	}
	A7105_Strobe(A7105_STANDBY);

	//IF Filter Bank Calibration
	A7105_WriteReg(A7105_02_CALC,1);
	while(A7105_ReadReg(A7105_02_CALC));			// Wait for calibration to end
//	A7105_ReadReg(A7105_22_IF_CALIB_I);
//	A7105_ReadReg(A7105_24_VCO_CURCAL);

	if(protocol!=PROTO_HUBSAN)
	{
		//VCO Current Calibration
		A7105_WriteReg(A7105_24_VCO_CURCAL,0x13);	//Recommended calibration from A7105 Datasheet
		//VCO Bank Calibration
		A7105_WriteReg(A7105_26_VCO_SBCAL_II,0x3b);	//Recommended calibration from A7105 Datasheet
	}

	//VCO Bank Calibrate channel 0
	A7105_WriteReg(A7105_0F_CHANNEL, 0);
	A7105_WriteReg(A7105_02_CALC,2);
	while(A7105_ReadReg(A7105_02_CALC));			// Wait for calibration to end
	vco_calibration0 = A7105_ReadReg(A7105_25_VCO_SBCAL_I);
	
	//VCO Bank Calibrate channel A0
	A7105_WriteReg(A7105_0F_CHANNEL, 0xa0);
	A7105_WriteReg(A7105_02_CALC, 2);
	while(A7105_ReadReg(A7105_02_CALC));			// Wait for calibration to end
	vco_calibration1 = A7105_ReadReg(A7105_25_VCO_SBCAL_I);

	if(protocol==PROTO_BUGS)
		A7105_SetVCOBand(vco_calibration0 & 0x07, vco_calibration1 & 0x07);	// Set calibration band value to best match
	else
		if(protocol!=PROTO_HUBSAN)
			A7105_WriteReg(A7105_25_VCO_SBCAL_I,protocol==PROTO_FLYSKY?0x08:0x0A);	//Reset VCO Band calibration

	A7105_SetTxRxMode(TX_EN);
	A7105_SetPower();

	A7105_AdjustLOBaseFreq(0);
	
	A7105_Strobe(A7105_STANDBY);
}
#endif

# 1 "src/CC2500_SPI.ino" // Helps debugging !

/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
//-------------------------------
//-------------------------------
//CC2500 SPI routines
//-------------------------------
//-------------------------------
#ifdef CC2500_INSTALLED
#include "iface_cc2500.h"

//----------------------------
void CC2500_WriteReg(uint8_t address, uint8_t data)
{
	CC25_CSN_off;
	SPI_Write(address); 
	NOP();
	SPI_Write(data);
	CC25_CSN_on;
} 

//----------------------
static void CC2500_ReadRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
	CC25_CSN_off;
	SPI_Write(CC2500_READ_BURST | address);
	for(uint8_t i = 0; i < length; i++)
		data[i] = SPI_Read();
	CC25_CSN_on;
}

//--------------------------------------------
static uint8_t CC2500_ReadReg(uint8_t address)
{ 
	uint8_t result;
	CC25_CSN_off;
	SPI_Write(CC2500_READ_SINGLE | address);
	result = SPI_Read();  
	CC25_CSN_on;
	return(result); 
} 

//------------------------
void CC2500_ReadData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_ReadRegisterMulti(CC2500_3F_RXFIFO, dpbuffer, len);
}

//*********************************************
void CC2500_Strobe(uint8_t state)
{
	CC25_CSN_off;
	SPI_Write(state);
	CC25_CSN_on;
}

static void CC2500_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t length)
{
	CC25_CSN_off;
	SPI_Write(CC2500_WRITE_BURST | address);
	for(uint8_t i = 0; i < length; i++)
		SPI_Write(data[i]);
	CC25_CSN_on;
}

void CC2500_WriteData(uint8_t *dpbuffer, uint8_t len)
{
	CC2500_Strobe(CC2500_SFTX);
	CC2500_WriteRegisterMulti(CC2500_3F_TXFIFO, dpbuffer, len);
	CC2500_Strobe(CC2500_STX);
}

void CC2500_SetTxRxMode(uint8_t mode)
{
	if(mode == TX_EN)
	{//from deviation firmware
		CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
		CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F | 0x40);
	}
	else
		if (mode == RX_EN)
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F | 0x40);
		}
		else
		{
			CC2500_WriteReg(CC2500_02_IOCFG0, 0x2F);
			CC2500_WriteReg(CC2500_00_IOCFG2, 0x2F);
		}
}

//------------------------
/*static void cc2500_resetChip(void)
{
	// Toggle chip select signal
	CC25_CSN_on;
	delayMicroseconds(30);
	CC25_CSN_off;
	delayMicroseconds(30);
	CC25_CSN_on;
	delayMicroseconds(45);
	CC2500_Strobe(CC2500_SRES);
	_delay_ms(100);
}
*/
uint8_t CC2500_Reset()
{
	CC2500_Strobe(CC2500_SRES);
	delayMilliseconds(1);
	CC2500_SetTxRxMode(TXRX_OFF);
	return CC2500_ReadReg(CC2500_0E_FREQ1) == 0xC4;//check if reset
}
/*
static void CC2500_SetPower_Value(uint8_t power)
{
	const unsigned char patable[8]=	{
		0xC5,  // -12dbm
		0x97, // -10dbm
		0x6E, // -8dbm
		0x7F, // -6dbm
		0xA9, // -4dbm
		0xBB, // -2dbm
		0xFE, // 0dbm
		0xFF // 1.5dbm
	};
	if (power > 7)
		power = 7;
	CC2500_WriteReg(CC2500_3E_PATABLE,  patable[power]);
}
*/
void CC2500_SetPower()
{
	uint8_t power=CC2500_BIND_POWER;
	if(IS_BIND_DONE)
		#ifdef CC2500_ENABLE_LOW_POWER
			power=IS_POWER_FLAG_on?CC2500_HIGH_POWER:CC2500_LOW_POWER;
		#else
			power=CC2500_HIGH_POWER;
		#endif
	if(IS_RANGE_FLAG_on)
		power=CC2500_RANGE_POWER;
	if(prev_power != power)
	{
		CC2500_WriteReg(CC2500_3E_PATABLE, power);
		prev_power=power;
	}
}
#endif

# 1 "src/CYRF6936_SPI.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifdef CYRF6936_INSTALLED
#include "iface_cyrf6936.h"

void CYRF_WriteRegister(uint8_t address, uint8_t data)
{
	CYRF_CSN_off;
	SPI_Write(0x80 | address);
	SPI_Write(data);
	CYRF_CSN_on;
}

static void CYRF_WriteRegisterMulti(uint8_t address, const uint8_t data[], uint8_t length)
{
	uint8_t i;

	CYRF_CSN_off;
	SPI_Write(0x80 | address);
	for(i = 0; i < length; i++)
		SPI_Write(data[i]);
	CYRF_CSN_on;
}

static void CYRF_ReadRegisterMulti(uint8_t address, uint8_t data[], uint8_t length)
{
	uint8_t i;

	CYRF_CSN_off;
	SPI_Write(address);
	for(i = 0; i < length; i++)
		data[i] = SPI_Read();
	CYRF_CSN_on;
}

uint8_t CYRF_ReadRegister(uint8_t address)
{
	uint8_t data;
	CYRF_CSN_off;
	SPI_Write(address);
	data = SPI_Read();
	CYRF_CSN_on;
	return data;
}
//

uint8_t CYRF_Reset()
{
#ifdef CYRF_RST_HI
	CYRF_RST_HI;										//Hardware reset
	delayMicroseconds(100);
	CYRF_RST_LO;
	delayMicroseconds(100);		  
#endif
	CYRF_WriteRegister(CYRF_1D_MODE_OVERRIDE, 0x01);	//Software reset
	delayMicroseconds(200);
	CYRF_WriteRegister(CYRF_0C_XTAL_CTRL, 0xC0);		//Enable XOUT as GPIO
	CYRF_WriteRegister(CYRF_0D_IO_CFG, 0x04);			//Enable PACTL as GPIO
	CYRF_SetTxRxMode(TXRX_OFF);
	//Verify the CYRF chip is responding
	return (CYRF_ReadRegister(CYRF_10_FRAMING_CFG) == 0xa5);
}

/*
*
*/
void CYRF_GetMfgData(uint8_t data[])
{
#ifndef FORCE_CYRF_ID
	/* Fuses power on */
	CYRF_WriteRegister(CYRF_25_MFG_ID, 0xFF);

	CYRF_ReadRegisterMulti(CYRF_25_MFG_ID, data, 6);

	/* Fuses power off */
	CYRF_WriteRegister(CYRF_25_MFG_ID, 0x00); 
#else
	memcpy(data,FORCE_CYRF_ID,6);
#endif
}

/*
* 1 - Tx else Rx
*/
void CYRF_SetTxRxMode(uint8_t mode)
{
	if(mode==TXRX_OFF)
	{
		if(protocol!=PROTO_WFLY)
			CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24); // 4=IDLE, 8=TX, C=RX
		CYRF_WriteRegister(CYRF_0E_GPIO_CTRL,0x00); // XOUT=0 PACTL=0
	}
	else
	{
		//Set the post tx/rx state
		if(protocol!=PROTO_WFLY)
			CYRF_WriteRegister(CYRF_0F_XACT_CFG, mode == TX_EN ? 0x28 : 0x2C); // 4=IDLE, 8=TX, C=RX
		if(mode == TX_EN)
#ifdef ORANGE_TX_BLUE
			CYRF_WriteRegister(CYRF_0E_GPIO_CTRL,0x20); // XOUT=1, PACTL=0
		else
			CYRF_WriteRegister(CYRF_0E_GPIO_CTRL,0x80);	// XOUT=0, PACTL=1
#else
			CYRF_WriteRegister(CYRF_0E_GPIO_CTRL,0x80); // XOUT=1, PACTL=0
		else
			CYRF_WriteRegister(CYRF_0E_GPIO_CTRL,0x20);	// XOUT=0, PACTL=1
#endif
	}
}
/*
*
*/
void CYRF_ConfigRFChannel(uint8_t ch)
{
	CYRF_WriteRegister(CYRF_00_CHANNEL,ch);
}

/*
static void CYRF_SetPower_Value(uint8_t power)
{
	uint8_t val = CYRF_ReadRegister(CYRF_03_TX_CFG) & 0xF8;
	CYRF_WriteRegister(CYRF_03_TX_CFG, val | (power & 0x07));
}
*/

void CYRF_SetPower(uint8_t val)
{
	uint8_t power=CYRF_BIND_POWER;
	if(IS_BIND_DONE)
		#ifdef CYRF6936_ENABLE_LOW_POWER
			power=IS_POWER_FLAG_on?CYRF_HIGH_POWER:CYRF_LOW_POWER;
		#else
			power=CYRF_HIGH_POWER;
		#endif
	if(IS_RANGE_FLAG_on)
		power=CYRF_RANGE_POWER;
	power|=val;
	if(prev_power != power)
	{
		CYRF_WriteRegister(CYRF_03_TX_CFG,power);
		prev_power=power;
	}
}

/*
*
*/
void CYRF_ConfigCRCSeed(uint16_t crc)
{
	CYRF_WriteRegister(CYRF_15_CRC_SEED_LSB,crc & 0xff);
	CYRF_WriteRegister(CYRF_16_CRC_SEED_MSB,crc >> 8);
}
/*
* these are the recommended sop codes from Cyrpress
* See "WirelessUSB LP/LPstar and PRoC LP/LPstar Technical Reference Manual"
*/
void CYRF_ConfigSOPCode(const uint8_t *sopcodes)
{
	//NOTE: This can also be implemented as:
	//for(i = 0; i < 8; i++) WriteRegister)0x23, sopcodes[i];
	CYRF_WriteRegisterMulti(CYRF_22_SOP_CODE, sopcodes, 8);
}

void CYRF_ConfigDataCode(const uint8_t *datacodes, uint8_t len)
{
	//NOTE: This can also be implemented as:
	//for(i = 0; i < len; i++) WriteRegister)0x23, datacodes[i];
	CYRF_WriteRegisterMulti(CYRF_23_DATA_CODE, datacodes, len);
}

void CYRF_WritePreamble(uint32_t preamble)
{
	CYRF_CSN_off;
	SPI_Write(0x80 | 0x24);
	SPI_Write(preamble & 0xff);
	SPI_Write((preamble >> 8) & 0xff);
	SPI_Write((preamble >> 16) & 0xff);
	CYRF_CSN_on;
}
/*
*
*/
/*static void CYRF_ReadDataPacket(uint8_t dpbuffer[])
{
	CYRF_ReadRegisterMulti(CYRF_21_RX_BUFFER, dpbuffer, 0x10);
}
*/
void CYRF_ReadDataPacketLen(uint8_t dpbuffer[], uint8_t length)
{
    CYRF_ReadRegisterMulti(CYRF_21_RX_BUFFER, dpbuffer, length);
}

static void CYRF_WriteDataPacketLen(const uint8_t dpbuffer[], uint8_t len)
{
	CYRF_WriteRegister(CYRF_01_TX_LENGTH, len);
	CYRF_WriteRegister(CYRF_02_TX_CTRL, 0x43);	// 0x40
	CYRF_WriteRegisterMulti(CYRF_20_TX_BUFFER, dpbuffer, len);
	CYRF_WriteRegister(CYRF_02_TX_CTRL, 0x83);	// 0xBF
}

void CYRF_WriteDataPacket(const uint8_t dpbuffer[])
{
	CYRF_WriteDataPacketLen(dpbuffer, 16);
}

/*static uint8_t CYRF_ReadRSSI(uint8_t dodummyread)
{
	uint8_t result;
	if(dodummyread)
		CYRF_ReadRegister(CYRF_13_RSSI);
	result = CYRF_ReadRegister(CYRF_13_RSSI);
	if(result & 0x80)
		result = CYRF_ReadRegister(CYRF_13_RSSI);
	return (result & 0x0F);
}
*/
//NOTE: This routine will reset the CRC Seed
void CYRF_FindBestChannels(uint8_t *channels, uint8_t len, uint8_t minspace, uint8_t min, uint8_t max)
{
	#define NUM_FREQ 80
	#define FREQ_OFFSET 4
	uint8_t rssi[NUM_FREQ];

	if (min < FREQ_OFFSET)
		min = FREQ_OFFSET;
	if (max > NUM_FREQ)
		max = NUM_FREQ;

	uint8_t i;
	int8_t j;
	memset(channels, 0, sizeof(uint8_t) * len);
	CYRF_ConfigCRCSeed(0x0000);
	CYRF_SetTxRxMode(RX_EN);
	//Wait for pre-amp to switch from send to receive
	delayMilliseconds(1);
	for(i = 0; i < NUM_FREQ; i++)
	{
		CYRF_ConfigRFChannel(i);
		delayMicroseconds(270);					//slow channel require 270usec for synthesizer to settle
        if( !(CYRF_ReadRegister(CYRF_05_RX_CTRL) & 0x80)) {
            CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x80); //Prepare to receive
            delayMicroseconds(15);
            CYRF_ReadRegister(CYRF_13_RSSI);	//dummy read
            delayMicroseconds(15);				//The conversion can occur as often as once every 12us
        }
		rssi[i] = CYRF_ReadRegister(CYRF_13_RSSI)&0x1F;
	}

	for (i = 0; i < len; i++)
	{
		channels[i] = min;
		for (j = min; j < max; j++)
			if (rssi[j] < rssi[channels[i]])
				channels[i] = j;
		for (j = channels[i] - minspace; j < channels[i] + minspace; j++) {
			//Ensure we don't reuse any channels within minspace of the selected channel again
			if (j < 0 || j >= NUM_FREQ)
				continue;
			rssi[j] = 0xff;
		}
	}
	CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x20);		// Abort RX operation
	CYRF_SetTxRxMode(TX_EN);
	CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x20);		// Clear abort RX
}

#if defined(DEVO_CYRF6936_INO) || defined(J6PRO_CYRF6936_INO)
const uint8_t PROGMEM DEVO_j6pro_sopcodes[][8] = {
    /* Note these are in order transmitted (LSB 1st) */
    {0x3C, 0x37, 0xCC, 0x91, 0xE2, 0xF8, 0xCC, 0x91},
    {0x9B, 0xC5, 0xA1, 0x0F, 0xAD, 0x39, 0xA2, 0x0F},
    {0xEF, 0x64, 0xB0, 0x2A, 0xD2, 0x8F, 0xB1, 0x2A},
    {0x66, 0xCD, 0x7C, 0x50, 0xDD, 0x26, 0x7C, 0x50},
    {0x5C, 0xE1, 0xF6, 0x44, 0xAD, 0x16, 0xF6, 0x44},
    {0x5A, 0xCC, 0xAE, 0x46, 0xB6, 0x31, 0xAE, 0x46},
    {0xA1, 0x78, 0xDC, 0x3C, 0x9E, 0x82, 0xDC, 0x3C},
    {0xB9, 0x8E, 0x19, 0x74, 0x6F, 0x65, 0x18, 0x74},
    {0xDF, 0xB1, 0xC0, 0x49, 0x62, 0xDF, 0xC1, 0x49},
    {0x97, 0xE5, 0x14, 0x72, 0x7F, 0x1A, 0x14, 0x72},
#if defined(J6PRO_CYRF6936_INO)
    {0x82, 0xC7, 0x90, 0x36, 0x21, 0x03, 0xFF, 0x17},
    {0xE2, 0xF8, 0xCC, 0x91, 0x3C, 0x37, 0xCC, 0x91}, //Note: the '03' was '9E' in the Cypress recommended table
    {0xAD, 0x39, 0xA2, 0x0F, 0x9B, 0xC5, 0xA1, 0x0F}, //The following are the same as the 1st 8 above,
    {0xD2, 0x8F, 0xB1, 0x2A, 0xEF, 0x64, 0xB0, 0x2A}, //but with the upper and lower word swapped
    {0xDD, 0x26, 0x7C, 0x50, 0x66, 0xCD, 0x7C, 0x50},
    {0xAD, 0x16, 0xF6, 0x44, 0x5C, 0xE1, 0xF6, 0x44},
    {0xB6, 0x31, 0xAE, 0x46, 0x5A, 0xCC, 0xAE, 0x46},
    {0x9E, 0x82, 0xDC, 0x3C, 0xA1, 0x78, 0xDC, 0x3C},
    {0x6F, 0x65, 0x18, 0x74, 0xB9, 0x8E, 0x19, 0x74},
#endif
};
#endif
static void __attribute__((unused)) CYRF_PROGMEM_ConfigSOPCode(const uint8_t *data)
{
	uint8_t code[8];
	for(uint8_t i=0;i<8;i++)
		code[i]=pgm_read_byte_near(&data[i]);
	CYRF_ConfigSOPCode(code);
}
#endif

# 1 "src/NRF24l01_SPI.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifdef NRF24L01_INSTALLED
#include "iface_nrf24l01.h"


//---------------------------
// NRF24L01+ SPI Specific Functions
//---------------------------

uint8_t rf_setup;

void NRF24L01_Initialize()
{
    rf_setup = 0x09;
	prev_power = 0x00;	// Make sure prev_power is inline with current power
	XN297_SetScrambledMode(XN297_SCRAMBLED);
}  

void NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
	NRF_CSN_off;
	SPI_Write(W_REGISTER | (REGISTER_MASK & reg));
	SPI_Write(data);
	NRF_CSN_on;
}

void NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t * data, uint8_t length)
{
	NRF_CSN_off;

	SPI_Write(W_REGISTER | ( REGISTER_MASK & reg));
	for (uint8_t i = 0; i < length; i++)
		SPI_Write(data[i]);
	NRF_CSN_on;
}

void NRF24L01_WritePayload(uint8_t * data, uint8_t length)
{
	NRF_CSN_off;
	SPI_Write(W_TX_PAYLOAD);
	for (uint8_t i = 0; i < length; i++)
		SPI_Write(data[i]);
	NRF_CSN_on;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
	NRF_CSN_off;
	SPI_Write(R_REGISTER | (REGISTER_MASK & reg));
	uint8_t data = SPI_Read();
	NRF_CSN_on;
	return data;
}

/*static void NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t * data, uint8_t length)
{
	NRF_CSN_off;
	SPI_Write(R_REGISTER | (REGISTER_MASK & reg));
	for(uint8_t i = 0; i < length; i++)
		data[i] = SPI_Read();
	NRF_CSN_on;
}
*/

static void NRF24L01_ReadPayload(uint8_t * data, uint8_t length)
{
	NRF_CSN_off;
	SPI_Write(R_RX_PAYLOAD);
	for(uint8_t i = 0; i < length; i++)
		data[i] = SPI_Read();
	NRF_CSN_on; 
}

static void  NRF24L01_Strobe(uint8_t state)
{
	NRF_CSN_off;
	SPI_Write(state);
	NRF_CSN_on;
}

void NRF24L01_FlushTx()
{
	NRF24L01_Strobe(FLUSH_TX);
}

void NRF24L01_FlushRx()
{
	NRF24L01_Strobe(FLUSH_RX);
}

static uint8_t __attribute__((unused)) NRF24L01_GetStatus()
{
	return SPI_Read();
}

static uint8_t NRF24L01_GetDynamicPayloadSize()
{
	NRF_CSN_off;
    SPI_Write(R_RX_PL_WID);
    uint8_t len = SPI_Read();
	NRF_CSN_on; 
    return len;
}

void NRF24L01_Activate(uint8_t code)
{
	NRF_CSN_off;
	SPI_Write(ACTIVATE);
	SPI_Write(code);
	NRF_CSN_on;
}

void NRF24L01_SetBitrate(uint8_t bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    prev_power=(rf_setup>>1)&0x03;	// Make sure prev_power is inline with current power
	NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

/*
	static void NRF24L01_SetPower_Value(uint8_t power)
{
    uint8_t nrf_power = 0;
    switch(power) {
        case TXPOWER_100uW: nrf_power = 0; break;
        case TXPOWER_300uW: nrf_power = 0; break;
        case TXPOWER_1mW:   nrf_power = 0; break;
        case TXPOWER_3mW:   nrf_power = 1; break;
        case TXPOWER_10mW:  nrf_power = 1; break;
        case TXPOWER_30mW:  nrf_power = 2; break;
        case TXPOWER_100mW: nrf_power = 3; break;
        case TXPOWER_150mW: nrf_power = 3; break;
        default:            nrf_power = 0; break;
    };
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);
    NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}
*/
void NRF24L01_SetPower()
{
	uint8_t power=NRF_BIND_POWER;
	if(IS_BIND_DONE)
		#ifdef NRF24L01_ENABLE_LOW_POWER
			power=IS_POWER_FLAG_on?NRF_HIGH_POWER:NRF_LOW_POWER;
		#else
			power=NRF_HIGH_POWER;
		#endif
	if(IS_RANGE_FLAG_on)
		power=NRF_POWER_0;
	if(prev_power != power)
	{
		rf_setup = (rf_setup & 0xF9) | (power << 1);
		NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
		prev_power=power;
	}
}

void NRF24L01_SetTxRxMode(enum TXRX_State mode)
{
	if(mode == TX_EN) {
		NRF_CE_off;
		NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
											| (1 << NRF24L01_07_TX_DS)
											| (1 << NRF24L01_07_MAX_RT));
		NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
											| (1 << NRF24L01_00_CRCO)
											| (1 << NRF24L01_00_PWR_UP));
		delayMicroseconds(130);
		NRF_CE_on;
	}
	else
		if (mode == RX_EN)
		{
			NRF_CE_off;
			NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
			NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
												| (1 << NRF24L01_07_TX_DS)
												| (1 << NRF24L01_07_MAX_RT));
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
												| (1 << NRF24L01_00_CRCO)
												| (1 << NRF24L01_00_PWR_UP)
												| (1 << NRF24L01_00_PRIM_RX));
			delayMicroseconds(130);
			NRF_CE_on;
		}
		else
		{
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
			NRF_CE_off;
		}
}

void NRF24L01_Reset()
{
	//** not in deviation but needed to hot switch between models
	NRF24L01_Activate(0x73);                          // Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits off
	NRF24L01_Activate(0x73);
	//**

	NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_Strobe(0xff);			// NOP
    NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_SetTxRxMode(TXRX_OFF);
	delayMicroseconds(100);
}

uint8_t NRF24L01_packet_ack()
{
    switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)))
	{
		case _BV(NRF24L01_07_TX_DS):
			return PKT_ACKED;
		case _BV(NRF24L01_07_MAX_RT):
			return PKT_TIMEOUT;
    }
	return PKT_PENDING;
}


///////////////
// XN297 emulation layer
uint8_t xn297_scramble_enabled=XN297_SCRAMBLED;	//enabled by default
uint8_t xn297_addr_len;
uint8_t xn297_tx_addr[5];
uint8_t xn297_rx_addr[5];
uint8_t xn297_crc = 0;

static const uint8_t xn297_scramble[] = {
	0xe3, 0xb1, 0x4b, 0xea, 0x85, 0xbc, 0xe5, 0x66,
	0x0d, 0xae, 0x8c, 0x88, 0x12, 0x69, 0xee, 0x1f,
	0xc7, 0x62, 0x97, 0xd5, 0x0b, 0x79, 0xca, 0xcc,
	0x1b, 0x5d, 0x19, 0x10, 0x24, 0xd3, 0xdc, 0x3f,
	0x8e, 0xc5, 0x2f};

const uint16_t PROGMEM xn297_crc_xorout_scrambled[] = {
	0x0000, 0x3448, 0x9BA7, 0x8BBB, 0x85E1, 0x3E8C,
	0x451E, 0x18E6, 0x6B24, 0xE7AB, 0x3828, 0x814B,
	0xD461, 0xF494, 0x2503, 0x691D, 0xFE8B, 0x9BA7,
	0x8B17, 0x2920, 0x8B5F, 0x61B1, 0xD391, 0x7401,
	0x2138, 0x129F, 0xB3A0, 0x2988};

const uint16_t PROGMEM xn297_crc_xorout[] = {
	0x0000, 0x3d5f, 0xa6f1, 0x3a23, 0xaa16, 0x1caf,
	0x62b2, 0xe0eb, 0x0821, 0xbe07, 0x5f1a, 0xaf15,
	0x4f0a, 0xad24, 0x5e48, 0xed34, 0x068c, 0xf2c9,
	0x1852, 0xdf36, 0x129d, 0xb17c, 0xd5f5, 0x70d7,
	0xb798, 0x5133, 0x67db, 0xd94e};

static uint8_t bit_reverse(uint8_t b_in)
{
    uint8_t b_out = 0;
    for (uint8_t i = 0; i < 8; ++i)
	{
        b_out = (b_out << 1) | (b_in & 1);
        b_in >>= 1;
    }
    return b_out;
}

static const uint16_t polynomial = 0x1021;
static uint16_t crc16_update(uint16_t crc, uint8_t a, uint8_t bits)
{
	crc ^= a << 8;
    while(bits--)
        if (crc & 0x8000)
            crc = (crc << 1) ^ polynomial;
		else
            crc = crc << 1;
    return crc;
}

void XN297_SetTXAddr(const uint8_t* addr, uint8_t len)
{
	if (len > 5) len = 5;
	if (len < 3) len = 3;
	uint8_t buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
	xn297_addr_len = len;
	if (xn297_addr_len < 4)
		for (uint8_t i = 0; i < 4; ++i)
			buf[i] = buf[i+1];
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, buf, 5);
	// Receive address is complicated. We need to use scrambled actual address as a receive address
	// but the TX code now assumes fixed 4-byte transmit address for preamble. We need to adjust it
	// first. Also, if the scrambled address begins with 1 nRF24 will look for preamble byte 0xAA
	// instead of 0x55 to ensure enough 0-1 transitions to tune the receiver. Still need to experiment
	// with receiving signals.
	memcpy(xn297_tx_addr, addr, len);
}

void XN297_SetRXAddr(const uint8_t* addr, uint8_t len)
{
	if (len > 5) len = 5;
	if (len < 3) len = 3;
	uint8_t buf[] = { 0, 0, 0, 0, 0 };
	memcpy(buf, addr, len);
	memcpy(xn297_rx_addr, addr, len);
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		buf[i] = xn297_rx_addr[i];
		if(xn297_scramble_enabled)
			buf[i] ^= xn297_scramble[xn297_addr_len-i-1];
	}
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, len-2);
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, buf, 5);
}

void XN297_Configure(uint8_t flags)
{
	xn297_crc = !!(flags & _BV(NRF24L01_00_EN_CRC));
	flags &= ~(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO));
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, flags & 0xFF);
}

void XN297_SetScrambledMode(const uint8_t mode)
{
    xn297_scramble_enabled = mode;
}

void XN297_WritePayload(uint8_t* msg, uint8_t len)
{
	uint8_t buf[32];
	uint8_t last = 0;

	if (xn297_addr_len < 4)
	{
		// If address length (which is defined by receive address length)
		// is less than 4 the TX address can't fit the preamble, so the last
		// byte goes here
		buf[last++] = 0x55;
	}
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		buf[last] = xn297_tx_addr[xn297_addr_len-i-1];
		if(xn297_scramble_enabled)
			buf[last] ^=  xn297_scramble[i];
		last++;
	}
	for (uint8_t i = 0; i < len; ++i)
	{
		// bit-reverse bytes in packet
		uint8_t b_out = bit_reverse(msg[i]);
		buf[last] = b_out;
		if(xn297_scramble_enabled)
			buf[last] ^= xn297_scramble[xn297_addr_len+i];
		last++;
	}
	if (xn297_crc)
	{
		uint8_t offset = xn297_addr_len < 4 ? 1 : 0;
		uint16_t crc = 0xb5d2;
		for (uint8_t i = offset; i < last; ++i)
			crc = crc16_update(crc, buf[i], 8);
		if(xn297_scramble_enabled)
			crc ^= pgm_read_word(&xn297_crc_xorout_scrambled[xn297_addr_len - 3 + len]);
		else
			crc ^= pgm_read_word(&xn297_crc_xorout[xn297_addr_len - 3 + len]);
		buf[last++] = crc >> 8;
		buf[last++] = crc & 0xff;
	}
	NRF24L01_WritePayload(buf, last);
}


void XN297_WriteEnhancedPayload(uint8_t* msg, uint8_t len, uint8_t noack, uint16_t crc_xorout)
{
	uint8_t packet[32];
	uint8_t scramble_index=0;
	uint8_t last = 0;
	static uint8_t pid=0;

	// address
	if (xn297_addr_len < 4)
	{
		// If address length (which is defined by receive address length)
		// is less than 4 the TX address can't fit the preamble, so the last
		// byte goes here
		packet[last++] = 0x55;
	}
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		packet[last] = xn297_tx_addr[xn297_addr_len-i-1];
		if(xn297_scramble_enabled)
			packet[last] ^= xn297_scramble[scramble_index++];
		last++;
	}

	// pcf
	packet[last] = (len << 1) | (pid>>1);
	if(xn297_scramble_enabled)
		packet[last] ^= xn297_scramble[scramble_index++];
	last++;
	packet[last] = (pid << 7) | (noack << 6);

	// payload
	packet[last]|= bit_reverse(msg[0]) >> 2; // first 6 bit of payload
	if(xn297_scramble_enabled)
		packet[last] ^= xn297_scramble[scramble_index++];

	for (uint8_t i = 0; i < len-1; ++i)
	{
		last++;
		packet[last] = (bit_reverse(msg[i]) << 6) | (bit_reverse(msg[i+1]) >> 2);
		if(xn297_scramble_enabled)
			packet[last] ^= xn297_scramble[scramble_index++];
	}

	last++;
	packet[last] = bit_reverse(msg[len-1]) << 6; // last 2 bit of payload
	if(xn297_scramble_enabled)
		packet[last] ^= xn297_scramble[scramble_index++] & 0xc0;

	// crc
	if (xn297_crc)
	{
		uint8_t offset = xn297_addr_len < 4 ? 1 : 0;
		uint16_t crc = 0xb5d2;
		for (uint8_t i = offset; i < last; ++i)
			crc = crc16_update(crc, packet[i], 8);
		crc = crc16_update(crc, packet[last] & 0xc0, 2);
		crc ^= crc_xorout;

		packet[last++] |= (crc >> 8) >> 2;
		packet[last++] = ((crc >> 8) << 6) | ((crc & 0xff) >> 2);
		packet[last++] = (crc & 0xff) << 6;
	}
	NRF24L01_WritePayload(packet, last);

	pid++;
	if(pid>3)
		pid=0;
}

boolean XN297_ReadPayload(uint8_t* msg, uint8_t len)
{ //!!! Don't forget if using CRC to do a +2 on any of the used NRF24L01_11_RX_PW_Px !!!
	uint8_t buf[32];
	if (xn297_crc)
		NRF24L01_ReadPayload(buf, len+2);	// Read payload + CRC 
	else
		NRF24L01_ReadPayload(buf, len);
	// Decode payload
	for(uint8_t i=0; i<len; i++)
	{
		uint8_t b_in=buf[i];
		if(xn297_scramble_enabled)
			b_in ^= xn297_scramble[i+xn297_addr_len];
		msg[i] = bit_reverse(b_in);
	}
	if (!xn297_crc)
		return true;	// No CRC so OK by default...

	// Calculate CRC
	uint16_t crc = 0xb5d2;
	//process address
	for (uint8_t i = 0; i < xn297_addr_len; ++i)
	{
		uint8_t b_in=xn297_tx_addr[xn297_addr_len-i-1];
		if(xn297_scramble_enabled)
			b_in ^=  xn297_scramble[i];
		crc = crc16_update(crc, b_in, 8);
	}
	//process payload
	for (uint8_t i = 0; i < len; ++i)
		crc = crc16_update(crc, buf[i], 8);
	//xorout
	if(xn297_scramble_enabled)
		crc ^= pgm_read_word(&xn297_crc_xorout_scrambled[xn297_addr_len - 3 + len]);
	else
		crc ^= pgm_read_word(&xn297_crc_xorout[xn297_addr_len - 3 + len]);
	//test
	if( (crc >> 8) == buf[len] && (crc & 0xff) == buf[len+1])
		return true;	// CRC  OK
	return false;		// CRC NOK
}

uint8_t XN297_ReadEnhancedPayload(uint8_t* msg, uint8_t len)
{
	uint8_t buffer[32];
	uint8_t pcf_size; // pcf payload size
	NRF24L01_ReadPayload(buffer, len+2); // pcf + payload
	pcf_size = buffer[0];
	if(xn297_scramble_enabled)
		pcf_size ^= xn297_scramble[xn297_addr_len];
	pcf_size = pcf_size >> 1;
	for(int i=0; i<len; i++)
	{
		msg[i] = bit_reverse((buffer[i+1] << 2) | (buffer[i+2] >> 6));
		if(xn297_scramble_enabled)
			msg[i] ^= bit_reverse((xn297_scramble[xn297_addr_len+i+1] << 2) | 
									(xn297_scramble[xn297_addr_len+i+2] >> 6));
	}
	return pcf_size;
}
 
 // End of XN297 emulation

//
// HS6200 emulation layer
///////////////////////////
static uint8_t hs6200_crc;
static uint16_t hs6200_crc_init;
static uint8_t hs6200_tx_addr[5];
static uint8_t hs6200_address_length;

static const uint8_t hs6200_scramble[] = {
	0x80,0xf5,0x3b,0x0d,0x6d,0x2a,0xf9,0xbc,
	0x51,0x8e,0x4c,0xfd,0xc1,0x65,0xd0 }; // todo: find all 32 bytes ...

void HS6200_SetTXAddr(const uint8_t* addr, uint8_t len)
{
	if(len < 4)
		len = 4;
	else if(len > 5)
		len = 5;

	// use nrf24 address field as a longer preamble
	if(addr[len-1] & 0x80)
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)"\x55\x55\x55\x55\x55", 5);
	else
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)"\xaa\xaa\xaa\xaa\xaa", 5);

	// precompute address crc
	hs6200_crc_init = 0xffff;
	for(int i=0; i<len; i++)
		hs6200_crc_init = crc16_update(hs6200_crc_init, addr[len-1-i], 8);
	memcpy(hs6200_tx_addr, addr, len);
	hs6200_address_length = len;
}

static uint16_t hs6200_calc_crc(uint8_t* msg, uint8_t len)
{
    uint8_t pos;
    uint16_t crc = hs6200_crc_init;
    
    // pcf + payload
    for(pos=0; pos < len-1; pos++)
        crc = crc16_update(crc, msg[pos], 8);
    // last byte (1 bit only)
    if(len > 0)
        crc = crc16_update(crc, msg[pos+1], 1);
    return crc;
}

void HS6200_Configure(uint8_t flags)
{
	hs6200_crc = !!(flags & _BV(NRF24L01_00_EN_CRC));
	flags &= ~(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO));
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, flags & 0xff);      
}

void HS6200_WritePayload(uint8_t* msg, uint8_t len)
{
	uint8_t payload[32];
	const uint8_t no_ack = 1; // never ask for an ack
	static uint8_t pid;
	uint8_t pos = 0;

	if(len > sizeof(hs6200_scramble))
		len = sizeof(hs6200_scramble);

	// address
	for(int i=hs6200_address_length-1; i>=0; i--)
		payload[pos++] = hs6200_tx_addr[i];

	// guard bytes
	payload[pos++] = hs6200_tx_addr[0];
	payload[pos++] = hs6200_tx_addr[0];

	// packet control field
	payload[pos++] = ((len & 0x3f) << 2) | (pid & 0x03);
	payload[pos] = (no_ack & 0x01) << 7;
	pid++;

	// scrambled payload
	if(len > 0)
	{
		payload[pos++] |= (msg[0] ^ hs6200_scramble[0]) >> 1; 
		for(uint8_t i=1; i<len; i++)
			payload[pos++] = ((msg[i-1] ^ hs6200_scramble[i-1]) << 7) | ((msg[i] ^ hs6200_scramble[i]) >> 1);
		payload[pos] = (msg[len-1] ^ hs6200_scramble[len-1]) << 7; 
	}

	// crc
	if(hs6200_crc)
	{
		uint16_t crc = hs6200_calc_crc(&payload[hs6200_address_length+2], len+2);
		uint8_t hcrc = crc >> 8;
		uint8_t lcrc = crc & 0xff;
		payload[pos++] |= (hcrc >> 1);
		payload[pos++] = (hcrc << 7) | (lcrc >> 1);
		payload[pos++] = lcrc << 7;
	}

	NRF24L01_WritePayload(payload, pos);
	delayMicroseconds(option);
	NRF24L01_WritePayload(payload, pos);
}
//
// End of HS6200 emulation
////////////////////////////

///////////////
// LT8900 emulation layer
uint8_t LT8900_buffer[64];
uint8_t LT8900_buffer_start;
uint16_t LT8900_buffer_overhead_bits;
uint8_t LT8900_addr[8];
uint8_t LT8900_addr_size;
uint8_t LT8900_Preamble_Len;
uint8_t LT8900_Tailer_Len;
uint8_t LT8900_CRC_Initial_Data;
uint8_t LT8900_Flags;
#define LT8900_CRC_ON 6
#define LT8900_SCRAMBLE_ON 5
#define LT8900_PACKET_LENGTH_EN 4
#define LT8900_DATA_PACKET_TYPE_1 3
#define LT8900_DATA_PACKET_TYPE_0 2
#define LT8900_FEC_TYPE_1 1
#define LT8900_FEC_TYPE_0 0

void LT8900_Config(uint8_t preamble_len, uint8_t trailer_len, uint8_t flags, uint8_t crc_init)
{
	//Preamble 1 to 8 bytes
	LT8900_Preamble_Len=preamble_len;
	//Trailer 4 to 18 bits
	LT8900_Tailer_Len=trailer_len;
	//Flags
	// CRC_ON: 1 on, 0 off
	// SCRAMBLE_ON: 1 on, 0 off
	// PACKET_LENGTH_EN: 1 1st byte of payload is payload size
	// DATA_PACKET_TYPE: 00 NRZ, 01 Manchester, 10 8bit/10bit line code, 11 interleave data type
	// FEC_TYPE: 00 No FEC, 01 FEC13, 10 FEC23, 11 reserved
	LT8900_Flags=flags;
	//CRC init constant
	LT8900_CRC_Initial_Data=crc_init;
}

void LT8900_SetChannel(uint8_t channel)
{
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel +2);	//NRF24L01 is 2400+channel but LT8900 is 2402+channel
}

void LT8900_SetTxRxMode(enum TXRX_State mode)
{
	if(mode == TX_EN)
	{
		//Switch to TX
		NRF24L01_SetTxRxMode(TXRX_OFF);
		NRF24L01_SetTxRxMode(TX_EN);
		//Disable CRC
		NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_PWR_UP));
	}
	else
		if (mode == RX_EN)
		{
			NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);		// Enable data pipe 0 only
			NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 32);
			//Switch to RX
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_FlushRx();
			NRF24L01_SetTxRxMode(RX_EN);
			// Disable CRC
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_PWR_UP) | (1 << NRF24L01_00_PRIM_RX) );
		}
		else
			NRF24L01_SetTxRxMode(TXRX_OFF);
}

void LT8900_BuildOverhead()
{
	uint8_t pos;

	//Build overhead
	//preamble
	memset(LT8900_buffer,LT8900_addr[0]&0x01?0xAA:0x55,LT8900_Preamble_Len-1);
	pos=LT8900_Preamble_Len-1;
	//address
	for(uint8_t i=0;i<LT8900_addr_size;i++)
	{
		LT8900_buffer[pos]=bit_reverse(LT8900_addr[i]);
		pos++;
	}
	//trailer
	memset(LT8900_buffer+pos,(LT8900_buffer[pos-1]&0x01)==0?0xAA:0x55,3);
	LT8900_buffer_overhead_bits=pos*8+LT8900_Tailer_Len;
	//nrf address length max is 5
	pos+=LT8900_Tailer_Len/8;
	LT8900_buffer_start=pos>5?5:pos;
}

void LT8900_SetAddress(uint8_t *address,uint8_t addr_size)
{
	uint8_t addr[5];
	
	//Address size (SyncWord) 2 to 8 bytes, 16/32/48/64 bits
	LT8900_addr_size=addr_size;
	for (uint8_t i = 0; i < addr_size; i++)
		LT8900_addr[i] = address[addr_size-1-i];

	//Build overhead
	LT8900_BuildOverhead();

	//Set NRF RX&TX address based on overhead content
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, LT8900_buffer_start-2);
	for(uint8_t i=0;i<LT8900_buffer_start;i++)	// reverse bytes order
		addr[i]=LT8900_buffer[LT8900_buffer_start-i-1];
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0,	addr,LT8900_buffer_start);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,	addr,LT8900_buffer_start);
}

uint8_t LT8900_ReadPayload(uint8_t* msg, uint8_t len)
{
	uint8_t i,pos=0,shift,end,buffer[32];
	unsigned int crc=LT8900_CRC_Initial_Data,a;
	pos=LT8900_buffer_overhead_bits/8-LT8900_buffer_start;
	end=pos+len+(LT8900_Flags&_BV(LT8900_PACKET_LENGTH_EN)?1:0)+(LT8900_Flags&_BV(LT8900_CRC_ON)?2:0);
	//Read payload
	NRF24L01_ReadPayload(buffer,end+1);
	//Check address + trail
	for(i=0;i<pos;i++)
		if(LT8900_buffer[LT8900_buffer_start+i]!=buffer[i])
			return 0; // wrong address...
	//Shift buffer to remove trail bits
	shift=LT8900_buffer_overhead_bits&0x7;
	for(i=pos;i<end;i++)
	{
		a=(buffer[i]<<8)+buffer[i+1];
		a<<=shift;
		buffer[i]=(a>>8)&0xFF;
	}
	//Check len
	if(LT8900_Flags&_BV(LT8900_PACKET_LENGTH_EN))
	{
		crc=crc16_update(crc,buffer[pos],8);
		if(bit_reverse(len)!=buffer[pos++])
			return 0; // wrong len...
	}
	//Decode message 
	for(i=0;i<len;i++)
	{
		crc=crc16_update(crc,buffer[pos],8);
		msg[i]=bit_reverse(buffer[pos++]);
	}
	//Check CRC
	if(LT8900_Flags&_BV(LT8900_CRC_ON))
	{
		if(buffer[pos++]!=((crc>>8)&0xFF)) return 0;	// wrong CRC...
		if(buffer[pos]!=(crc&0xFF)) return 0;			// wrong CRC...
	}
	//Everything ok
	return 1;
}

void LT8900_WritePayload(uint8_t* msg, uint8_t len)
{
	unsigned int crc=LT8900_CRC_Initial_Data,a,mask;
	uint8_t i, pos=0,tmp, buffer[64], pos_final,shift;
	//Add packet len
	if(LT8900_Flags&_BV(LT8900_PACKET_LENGTH_EN))
	{
		tmp=bit_reverse(len);
		buffer[pos++]=tmp;
		crc=crc16_update(crc,tmp,8);
	}
	//Add payload
	for(i=0;i<len;i++)
	{
		tmp=bit_reverse(msg[i]);
		buffer[pos++]=tmp;
		crc=crc16_update(crc,tmp,8);
	}
	//Add CRC
	if(LT8900_Flags&_BV(LT8900_CRC_ON))
	{
		buffer[pos++]=crc>>8;
		buffer[pos++]=crc;
	}
	//Shift everything to fit behind the trailer (4 to 18 bits)
	shift=LT8900_buffer_overhead_bits&0x7;
	pos_final=LT8900_buffer_overhead_bits/8;
	mask=~(0xFF<<(8-shift));
	LT8900_buffer[pos_final+pos]=0xFF;
	for(i=pos-1;i!=0xFF;i--)
	{
		a=buffer[i]<<(8-shift);
		LT8900_buffer[pos_final+i]=(LT8900_buffer[pos_final+i]&mask>>8)|a>>8;
		LT8900_buffer[pos_final+i+1]=(LT8900_buffer[pos_final+i+1]&mask)|a;
	}
	if(shift)
		pos++;
	//Send everything
	NRF24L01_WritePayload(LT8900_buffer+LT8900_buffer_start,pos_final+pos-LT8900_buffer_start);
}
// End of LT8900 emulation
#endif

# 1 "src/NCC1701_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(NCC1701_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define NCC_WRITE_WAIT      2000
#define NCC_PACKET_INTERVAL 10333
#define NCC_TX_PACKET_LEN	16
#define NCC_RX_PACKET_LEN	13

enum {
	NCC_BIND_TX1=0,
	NCC_BIND_RX1,
	NCC_BIND_TX2,
	NCC_BIND_RX2,
	NCC_TX3,
	NCC_RX3,
};

static void __attribute__((unused)) NCC_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);

	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);     // 5-byte RX/TX address
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)"\xE7\xE7\xC7\xD7\x67",5);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    (uint8_t*)"\xE7\xE7\xC7\xD7\x67",5);
	
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);				// No Auto Acknowledgment on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);			// Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, NCC_RX_PACKET_LEN);	// Enable rx pipe 0
	NRF24L01_SetBitrate(NRF24L01_BR_250K);					// NRF24L01_BR_1M, NRF24L01_BR_2M, NRF24L01_BR_250K
	NRF24L01_SetPower();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, (0 << NRF24L01_00_EN_CRC)   // switch to TX mode and disable CRC
										| (1 << NRF24L01_00_CRCO)
										| (1 << NRF24L01_00_PWR_UP)
										| (0 << NRF24L01_00_PRIM_RX));
}

const uint8_t NCC_xor[]={0x80, 0x44, 0x64, 0x75, 0x6C, 0x71, 0x2A, 0x36, 0x7C, 0xF1, 0x6E, 0x52, 0x09, 0x9D};
static void __attribute__((unused)) NCC_Crypt_Packet()
{
	uint16_t crc=0;
	for(uint8_t i=0; i< NCC_TX_PACKET_LEN-2; i++)
	{
		packet[i]^=NCC_xor[i];
		crc=crc16_update(crc, packet[i], 8);
	}
	crc^=0x60DE;
	packet[NCC_TX_PACKET_LEN-2]=crc>>8;
	packet[NCC_TX_PACKET_LEN-1]=crc;
}
static boolean __attribute__((unused)) NCC_Decrypt_Packet()
{
	uint16_t crc=0;
	debug("RX: ");
	for(uint8_t i=0; i< NCC_RX_PACKET_LEN-2; i++)
	{
		crc=crc16_update(crc, packet[i], 8);
		packet[i]^=NCC_xor[i];
		debug("%02X ",packet[i]);
	}
	crc^=0xA950;
	if( (crc>>8)==packet[NCC_RX_PACKET_LEN-2] && (crc&0xFF)==packet[NCC_RX_PACKET_LEN-1] )
	{// CRC match
		debugln("OK");
		return true;
	}
	debugln("NOK");
	return false;
}

static void __attribute__((unused)) NCC_Write_Packet()
{
	packet[0]=0xAA;
	packet[1]=rx_tx_addr[0];
	packet[2]=rx_tx_addr[1];
	packet[3]=rx_id[0];
	packet[4]=rx_id[1];
	packet[5]=convert_channel_8b(THROTTLE)>>2;	// 00-3D
	packet[6]=convert_channel_8b(ELEVATOR);		// original: 61-80-9F but works with 00-80-FF
	packet[7]=convert_channel_8b(AILERON );		// original: 61-80-9F but works with 00-80-FF
	packet[8]=convert_channel_8b(RUDDER  );		// original: 61-80-9F but works with 00-80-FF
	packet[9]=rx_id[2];
	packet[10]=rx_id[3];
	packet[11]=rx_id[4];
	packet[12]=GET_FLAG(CH5_SW, 0x02);			// Warp:0x00 -> 0x02
	packet[13]=packet[5]+packet[6]+packet[7]+packet[8]+packet[12];
	if(phase==NCC_BIND_TX1)
	{
		packet[0]=0xBB;
		packet[5]=0x01;
		packet[6]=rx_tx_addr[2];
		memset((void *)(packet+7),0x55,7);
		hopping_frequency_no^=1;
	}
	else
	{
		hopping_frequency_no++;
		if(hopping_frequency_no>2) hopping_frequency_no=0;
	}
	// change frequency
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
	// switch to TX mode and disable CRC
	NRF24L01_SetTxRxMode(TXRX_OFF);
	NRF24L01_SetTxRxMode(TX_EN);
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, (0 << NRF24L01_00_EN_CRC)
										| (1 << NRF24L01_00_CRCO)
										| (1 << NRF24L01_00_PWR_UP)
										| (0 << NRF24L01_00_PRIM_RX));
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	// send packet
	NCC_Crypt_Packet();
	NRF24L01_WritePayload(packet,NCC_TX_PACKET_LEN);
	NRF24L01_SetPower();
}

uint16_t NCC_callback()
{
	switch(phase)
	{
		case NCC_BIND_TX1:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				NRF24L01_ReadPayload(packet, NCC_RX_PACKET_LEN);
				if(NCC_Decrypt_Packet() && packet[1]==rx_tx_addr[0] && packet[2]==rx_tx_addr[1])

				{
					rx_id[0]=packet[3];
					rx_id[1]=packet[4];
					NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
					phase=NCC_BIND_TX2;
					return NCC_PACKET_INTERVAL;
				}
			}
			NCC_Write_Packet();
			phase = NCC_BIND_RX1;
			return NCC_WRITE_WAIT;
		case NCC_BIND_RX1:
			// switch to RX mode and disable CRC
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(RX_EN);
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (0 << NRF24L01_00_EN_CRC)
												| (1 << NRF24L01_00_CRCO)
												| (1 << NRF24L01_00_PWR_UP)
												| (1 << NRF24L01_00_PRIM_RX));
			NRF24L01_FlushRx();
			phase = NCC_BIND_TX1;
			return NCC_PACKET_INTERVAL - NCC_WRITE_WAIT;
		case NCC_BIND_TX2:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				NRF24L01_ReadPayload(packet, NCC_RX_PACKET_LEN);
				if(NCC_Decrypt_Packet() && packet[1]==rx_tx_addr[0] && packet[2]==rx_tx_addr[1] && packet[3]==rx_id[0] && packet[4]==rx_id[1])
				{
					rx_id[2]=packet[8];
					rx_id[3]=packet[9];
					rx_id[4]=packet[10];
					NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
					BIND_DONE;
					phase=NCC_TX3;
					return NCC_PACKET_INTERVAL;
				}
			}
			NCC_Write_Packet();
			phase = NCC_BIND_RX2;
			return NCC_WRITE_WAIT;
		case NCC_BIND_RX2:
			// switch to RX mode and disable CRC
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(RX_EN);
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (0 << NRF24L01_00_EN_CRC)
												| (1 << NRF24L01_00_CRCO)
												| (1 << NRF24L01_00_PWR_UP)
												| (1 << NRF24L01_00_PRIM_RX));
			NRF24L01_FlushRx();
			phase = NCC_BIND_TX2;
			return NCC_PACKET_INTERVAL - NCC_WRITE_WAIT;
		case NCC_TX3:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				NRF24L01_ReadPayload(packet, NCC_RX_PACKET_LEN);
				if(NCC_Decrypt_Packet() && packet[1]==rx_tx_addr[0] && packet[2]==rx_tx_addr[1] && packet[3]==rx_id[0] && packet[4]==rx_id[1])
				{
					//Telemetry
					//packet[5] and packet[7] roll angle
					//packet[6] crash detect: 0x00 no crash, 0x02 crash
					#ifdef NCC1701_HUB_TELEMETRY
						v_lipo1 = packet[6]?0xFF:0x00;	// Crash indication
						v_lipo2 = 0x00;
						RX_RSSI = 0x7F;					// Dummy RSSI
						TX_RSSI = 0x7F;					// Dummy RSSI
						telemetry_link=1;
					#endif
				}
			}
			NCC_Write_Packet();
			phase = NCC_RX3;
			return NCC_WRITE_WAIT;
		case NCC_RX3:
			// switch to RX mode and disable CRC
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(RX_EN);
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, (0 << NRF24L01_00_EN_CRC)
												| (1 << NRF24L01_00_CRCO)
												| (1 << NRF24L01_00_PWR_UP)
												| (1 << NRF24L01_00_PRIM_RX));
			NRF24L01_FlushRx();
			phase = NCC_TX3;
			return NCC_PACKET_INTERVAL - NCC_WRITE_WAIT;
	}
	return 0;
}

const uint8_t PROGMEM NCC_TX_DATA[][6]= {
	{ 0x6D, 0x97, 0x04, 0x48, 0x43, 0x26 }, 
	{ 0x35, 0x4B, 0x80, 0x44, 0x4C, 0x0B },
	{ 0x50, 0xE2, 0x32, 0x2D, 0x4B, 0x0A },
	{ 0xBF, 0x34, 0xF3, 0x45, 0x4D, 0x0D },
	{ 0xDD, 0x7D, 0x5A, 0x46, 0x28, 0x23 },
	{ 0xED, 0x19, 0x06, 0x2C, 0x4A, 0x09 },
	{ 0xE9, 0xA8, 0x91, 0x2B, 0x49, 0x07 },
	{ 0x66, 0x17, 0x7D, 0x48, 0x43, 0x26 },
	{ 0xC2, 0x93, 0x55, 0x44, 0x4C, 0x0B },
};

uint16_t initNCC(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
	
	// Load TX data
	uint8_t rand=rx_tx_addr[3]%9;
	for(uint8_t i=0; i<3; i++)
	{
		rx_tx_addr[i]=pgm_read_byte_near(&NCC_TX_DATA[rand][i]);
		hopping_frequency[i]=pgm_read_byte_near(&NCC_TX_DATA[rand][i+3]);
	}

	// RX data is acquired during bind
	rx_id[0]=0x00;
	rx_id[1]=0x00;
	rx_id[2]=0x20;
	rx_id[3]=0x20;
	rx_id[4]=0x20;

	hopping_frequency[4]=0x08;	// bind channel 1
	hopping_frequency[5]=0x2A;	// bind channel 2
	hopping_frequency_no=4;		// start with bind
	NCC_init();
	phase=NCC_BIND_TX1;
	#ifdef NCC1701_HUB_TELEMETRY
		init_frskyd_link_telemetry();
	#endif
	return 10000;
}

#endif


# 1 "src/MT99xx_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with MT99xx, Eachine H7, Yi Zhan i6S and LS114/124
// Last sync with Goebish mt99xx_nrf24l01.c dated 2016-01-29

#if defined(MT99XX_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define MT99XX_BIND_COUNT		928
#define MT99XX_PACKET_PERIOD_FY805 2460
#define MT99XX_PACKET_PERIOD_MT 2625
#define MT99XX_PACKET_PERIOD_YZ 3125
#define MT99XX_INITIAL_WAIT     500
#define MT99XX_PACKET_SIZE		9

#define checksum_offset	rf_ch_num
#define channel_offset	phase

enum{
    // flags going to packet[6] (MT99xx, H7)
    FLAG_MT_RATE1   = 0x01, // (H7 high rate)
    FLAG_MT_RATE2   = 0x02, // (MT9916 only)
    FLAG_MT_VIDEO   = 0x10,
    FLAG_MT_SNAPSHOT= 0x20,
    FLAG_MT_FLIP    = 0x80,
};

enum{
    // flags going to packet[6] (LS)
    FLAG_LS_INVERT  = 0x01,
    FLAG_LS_RATE    = 0x02,
    FLAG_LS_HEADLESS= 0x10,
    FLAG_LS_SNAPSHOT= 0x20,
    FLAG_LS_VIDEO   = 0x40,
    FLAG_LS_FLIP    = 0x80,
};

enum{
    // flags going to packet[7] (FY805)
    FLAG_FY805_HEADLESS= 0x10,
};

enum {
    MT99XX_INIT = 0,
    MT99XX_BIND,
    MT99XX_DATA
};

const uint8_t h7_mys_byte[] = {
	0x01, 0x11, 0x02, 0x12, 0x03, 0x13, 0x04, 0x14, 
	0x05, 0x15, 0x06, 0x16, 0x07, 0x17, 0x00, 0x10
};

static const uint8_t ls_mys_byte[] = {
	0x05, 0x15, 0x25, 0x06, 0x16, 0x26,
	0x07, 0x17, 0x27, 0x00, 0x10, 0x20,
	0x01, 0x11, 0x21, 0x02, 0x12, 0x22,
	0x03, 0x13, 0x23, 0x04, 0x14, 0x24
};

static void __attribute__((unused)) MT99XX_send_packet()
{
	const uint8_t yz_p4_seq[] = {0xa0, 0x20, 0x60};
	static uint8_t yz_seq_num=0;
	static uint8_t ls_counter=0;

	if(sub_protocol != YZ)
	{ // MT99XX & H7 & LS
		packet[0] = convert_channel_16b_limit(THROTTLE,0xE1,0x00); // throttle
		packet[1] = convert_channel_16b_limit(RUDDER  ,0x00,0xE1); // rudder
		packet[2] = convert_channel_16b_limit(AILERON ,0xE1,0x00); // aileron
		packet[3] = convert_channel_16b_limit(ELEVATOR,0x00,0xE1); // elevator
		packet[4] = 0x20; // pitch trim (0x3f-0x20-0x00)
		packet[5] = 0x20; // roll trim (0x00-0x20-0x3f)
		packet[6] = GET_FLAG( CH5_SW, FLAG_MT_FLIP );
		packet[7] = h7_mys_byte[hopping_frequency_no];		// next rf channel index ?

		if(sub_protocol==H7)
			packet[6]|=FLAG_MT_RATE1; // max rate on H7
		else
			if(sub_protocol==MT99)
				packet[6] |= 0x40 | FLAG_MT_RATE2
				  | GET_FLAG( CH7_SW, FLAG_MT_SNAPSHOT )
				  | GET_FLAG( CH8_SW, FLAG_MT_VIDEO );	// max rate on MT99xx
			else
				if(sub_protocol==FY805)
				{
					packet[6]=0x20;
					//Rate 0x01?
					//Flip ?
					packet[7]=0x01
						|GET_FLAG( CH5_SW, FLAG_MT_FLIP )
						|GET_FLAG( CH9_SW, FLAG_FY805_HEADLESS );	//HEADLESS
					checksum_offset=0;
				}
				else //LS
				{
					packet[6] |= FLAG_LS_RATE							// max rate
						| GET_FLAG( CH6_SW, FLAG_LS_INVERT )		//INVERT
						| GET_FLAG( CH7_SW, FLAG_LS_SNAPSHOT )		//SNAPSHOT
						| GET_FLAG( CH8_SW, FLAG_LS_VIDEO )			//VIDEO
						| GET_FLAG( CH9_SW, FLAG_LS_HEADLESS );		//HEADLESS
					packet[7] = ls_mys_byte[ls_counter++];
					if(ls_counter >= sizeof(ls_mys_byte))
						ls_counter=0;
				}

		uint8_t result=checksum_offset;
		for(uint8_t i=0; i<8; i++)
			result += packet[i];
		packet[8] = result;
	}
	else
	{ // YZ
		packet[0] = convert_channel_16b_limit(THROTTLE,0x00,0x64); // throttle
		packet[1] = convert_channel_16b_limit(RUDDER  ,0x64,0x00); // rudder
		packet[2] = convert_channel_16b_limit(ELEVATOR,0x00,0x64); // elevator
		packet[3] = convert_channel_16b_limit(AILERON ,0x64,0x00); // aileron
		if(packet_count++ >= 23)
		{
			yz_seq_num ++;
			if(yz_seq_num > 2)
				yz_seq_num = 0;
			packet_count=0;
		}
		packet[4] = yz_p4_seq[yz_seq_num]; 
		packet[5] = 0x02 // expert ? (0=unarmed, 1=normal)
					| GET_FLAG(CH8_SW, 0x10)		//VIDEO
					| GET_FLAG(CH5_SW, 0x80)		//FLIP
					| GET_FLAG(CH9_SW, 0x04)		//HEADLESS
					| GET_FLAG(CH7_SW, 0x20);		//SNAPSHOT
		packet[6] =   GET_FLAG(CH6_SW, 0x80);		//LED
		packet[7] = packet[0];            
		for(uint8_t idx = 1; idx < MT99XX_PACKET_SIZE-2; idx++)
			packet[7] += packet[idx];
		packet[8] = 0xff;
	}

	if(sub_protocol == LS)
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x2D); // LS always transmits on the same channel
	else
		if(sub_protocol==FY805)
			NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x4B); // FY805 always transmits on the same channel
		else
			NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no] + channel_offset);
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, MT99XX_PACKET_SIZE);

	hopping_frequency_no++;
	if(sub_protocol == YZ)
		hopping_frequency_no++; // skip every other channel

	if(hopping_frequency_no > 15)
		hopping_frequency_no = 0;

	NRF24L01_SetPower();
}

static void __attribute__((unused)) MT99XX_init()
{
    NRF24L01_Initialize();
    if(sub_protocol == YZ)
		XN297_SetScrambledMode(XN297_UNSCRAMBLED);
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_FlushTx();
    XN297_SetTXAddr((uint8_t *)"\xCC\xCC\xCC\xCC\xCC", 5);
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);		// Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);		// Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);		// 5 bytes address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);	// no auto retransmit
    if(sub_protocol == YZ)
        NRF24L01_SetBitrate(NRF24L01_BR_250K);			// 250Kbps (nRF24L01+ only)
    else
        NRF24L01_SetBitrate(NRF24L01_BR_1M);          // 1Mbps
    NRF24L01_SetPower();
	
    XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) );
	
}

static void __attribute__((unused)) MT99XX_initialize_txid()
{
	rx_tx_addr[3] = 0xCC;
	rx_tx_addr[4] = 0xCC;
    if(sub_protocol == YZ)
	{
		rx_tx_addr[0] = 0x53; // test (SB id)
		rx_tx_addr[1] = 0x00;
		rx_tx_addr[2] = 0x00;
	}
	else
		if(sub_protocol == FY805)
		{
			rx_tx_addr[0] = 0x81; // test (SB id)
			rx_tx_addr[1] = 0x0F;
			rx_tx_addr[2] = 0x00;
		}
		else
			if(sub_protocol == LS)
				rx_tx_addr[0] = 0xCC;
			else //MT99 & H7
				rx_tx_addr[2] = 0x00;
	checksum_offset = rx_tx_addr[0] + rx_tx_addr[1] + rx_tx_addr[2];
	channel_offset = (((checksum_offset & 0xf0)>>4) + (checksum_offset & 0x0f)) % 8;
}

uint16_t MT99XX_callback()
{
	if(IS_BIND_DONE)
		MT99XX_send_packet();
	else
	{
		if (bind_counter == 0)
		{
            // set tx address for data packets
            XN297_SetTXAddr(rx_tx_addr, 5);
			BIND_DONE;
		}
		else
		{
			if(sub_protocol == LS)
				NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x2D); // LS always transmits on the same channel
			else
				if(sub_protocol==FY805)
					NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x4B); // FY805 always transmits on the same channel
				else
					NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
			NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
			NRF24L01_FlushTx();
			XN297_WritePayload(packet, MT99XX_PACKET_SIZE); // bind packet
			hopping_frequency_no++;
			if(sub_protocol == YZ)
				hopping_frequency_no++; // skip every other channel
			if(hopping_frequency_no > 15)
				hopping_frequency_no = 0;
			bind_counter--;
		}
	}

    return packet_period;
}

uint16_t initMT99XX(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    bind_counter = MT99XX_BIND_COUNT;

	memcpy(hopping_frequency,"\x02\x48\x0C\x3e\x16\x34\x20\x2A\x2A\x20\x34\x16\x3e\x0c\x48\x02",16);
	hopping_frequency_no=0;
	
	MT99XX_initialize_txid();
	MT99XX_init();

	packet[0] = 0x20;
	packet_period = MT99XX_PACKET_PERIOD_MT;
	switch(sub_protocol)
	{ // MT99 & H7
		case MT99:
		case H7:
			packet[1] = 0x14;
			packet[2] = 0x03;
			packet[3] = 0x25;
			break;
		case YZ:
			packet_period = MT99XX_PACKET_PERIOD_YZ;
			packet[1] = 0x15;
			packet[2] = 0x05;
			packet[3] = 0x06;
			break;
		case LS:
			packet[1] = 0x14;
			packet[2] = 0x05;
			packet[3] = 0x11;
			break;
		case FY805:
			packet_period = MT99XX_PACKET_PERIOD_FY805;
			packet[1] = 0x15;
			packet[2] = 0x12;
			packet[3] = 0x17;
			break;
	}
	packet[4] = rx_tx_addr[0];
	packet[5] = rx_tx_addr[1];
	packet[6] = rx_tx_addr[2];
    packet[7] = checksum_offset; // checksum offset
    packet[8] = 0xAA;			// fixed
	packet_count=0;
	return	MT99XX_INITIAL_WAIT+MT99XX_PACKET_PERIOD_MT;
}
#endif


# 1 "src/Hontai_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(HONTAI_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define HONTAI_BIND_COUNT 80
#define HONTAI_PACKET_PERIOD    13500
#define FQ777_951_PACKET_PERIOD	10000
#define HONTAI_INITIAL_WAIT       500
#define HONTAI_BIND_PACKET_SIZE   10
#define HONTAI_PACKET_SIZE        12
#define HONTAI_RF_BIND_CHANNEL    0

enum{
    HONTAI_FLAG_FLIP      = 0x01, 
    HONTAI_FLAG_PICTURE   = 0x02, 
    HONTAI_FLAG_VIDEO     = 0x04, 
    HONTAI_FLAG_HEADLESS  = 0x08, 
    HONTAI_FLAG_RTH       = 0x10,
    HONTAI_FLAG_CALIBRATE = 0x20,
};

// proudly swiped from http://www.drdobbs.com/implementing-the-ccitt-cyclical-redundan/199904926
#define HONTAI_POLY 0x8408
static void __attribute__((unused)) crc16(uint8_t *data_p, uint8_t length)
{
	uint16_t crc = 0xffff;

	length -= 2;
	do
	{
		for (uint8_t i = 0, data = (uint8_t)*data_p++;
		i < 8;
		i++, data >>= 1)
		{
			if ((crc & 0x01) ^ (data & 0x01))
				crc = (crc >> 1) ^ HONTAI_POLY;
			else
				crc >>= 1;
		}
	} while (--length);

	crc = ~crc;
	*data_p++ = crc & 0xff;
	*data_p   = crc >> 8;
}

static void __attribute__((unused)) HONTAI_send_packet(uint8_t bind)
{
	if (bind)
	{
		memcpy(packet, rx_tx_addr, 5);
		memset(&packet[5], 0, 3);
	}
	else
	{
		memset(packet,0,HONTAI_PACKET_SIZE);
		packet[3] = convert_channel_16b_limit(THROTTLE, 0, 127) << 1;	// Throttle
		packet[4] = convert_channel_16b_limit(AILERON, 63, 0);			// Aileron
		packet[5] = convert_channel_16b_limit(ELEVATOR, 0, 63);			// Elevator
		packet[6] = convert_channel_16b_limit(RUDDER, 0, 63);			// Rudder
		if(sub_protocol == X5C1)
			packet[7] = convert_channel_16b_limit(AILERON, 0, 63)-31;	// Aileron trim
		else
			packet[7] = convert_channel_16b_limit(AILERON, 0, 32)-16;	// Aileron trim
		packet[8] = convert_channel_16b_limit(RUDDER, 0, 32)-16;			// Rudder trim
		if (sub_protocol == X5C1)
			packet[9] = convert_channel_16b_limit(ELEVATOR, 0, 63)-31;	// Elevator trim
		else
			packet[9] = convert_channel_16b_limit(ELEVATOR, 0, 32)-16;	// Elevator trim
		switch(sub_protocol)
		{
			case HONTAI:
				packet[0]  = 0x0B;
				packet[3] |= GET_FLAG(CH7_SW, 0x01);				// Picture
				packet[4] |= GET_FLAG(CH10_SW, 0x80)					// RTH
						  |  GET_FLAG(CH9_SW, 0x40);				// Headless
				packet[5] |= GET_FLAG(CH11_SW, 0x80)					// Calibrate
						  |  GET_FLAG(CH5_SW, 0x40);				// Flip
				packet[6] |= GET_FLAG(CH8_SW, 0x80);				// Video
				break;
			case JJRCX1:
				packet[0]  = GET_FLAG(CH6_SW, 0x02);				// Arm
				packet[3] |= GET_FLAG(CH7_SW, 0x01);				// Picture
				packet[4] |= 0x80;										// unknown
				packet[5] |= GET_FLAG(CH11_SW, 0x80)					// Calibrate
						  |  GET_FLAG(CH5_SW, 0x40);				// Flip
				packet[6] |= GET_FLAG(CH8_SW, 0x80);				// Video
				packet[8]  = 0xC0										// high rate, no rudder trim
						  |  GET_FLAG(CH10_SW, 0x02)					// RTH
						  |  GET_FLAG(CH9_SW, 0x01);				// Headless
				break;
			case X5C1:
				packet[0]  = 0x0B;
				packet[3] |= GET_FLAG(CH7_SW, 0x01);				// Picture
				packet[4]  = 0x80										// unknown
						  |  GET_FLAG(CH6_SW, 0x40);				// Lights
				packet[5] |= GET_FLAG(CH11_SW, 0x80)					// Calibrate
						  |  GET_FLAG(CH5_SW, 0x40);				// Flip
				packet[6] |= GET_FLAG(CH8_SW, 0x80);				// Video
				packet[8]  = 0xC0										// high rate, no rudder trim
						  |  GET_FLAG(CH10_SW, 0x02)					// RTH
						  |  GET_FLAG(CH9_SW, 0x01);				// Headless
				break;
			case FQ777_951:
				packet[0]  = GET_FLAG(CH7_SW, 0x01)					// Picture
						  |  GET_FLAG(CH8_SW, 0x02);				// Video
				packet[3] |= GET_FLAG(CH5_SW, 0x01);				// Flip
				packet[4] |= 0xC0;										// High rate (mid=0xa0, low=0x60)
				packet[5] |= GET_FLAG(CH11_SW, 0x80);				// Calibrate
				packet[6] |= GET_FLAG(CH9_SW, 0x40);				// Headless
				break;
		}
	}
	crc16(packet, bind ? HONTAI_BIND_PACKET_SIZE:HONTAI_PACKET_SIZE);

	// Power on, TX mode, 2byte CRC
	if(sub_protocol == JJRCX1)
		NRF24L01_SetTxRxMode(TX_EN);
	else
		XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));

	NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? HONTAI_RF_BIND_CHANNEL : hopping_frequency[hopping_frequency_no++]);
	hopping_frequency_no %= 3;

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	if(sub_protocol == JJRCX1)
		NRF24L01_WritePayload(packet, bind ? HONTAI_BIND_PACKET_SIZE:HONTAI_PACKET_SIZE);
	else
		XN297_WritePayload(packet, bind ? HONTAI_BIND_PACKET_SIZE:HONTAI_PACKET_SIZE);

	NRF24L01_SetPower();
}

static void __attribute__((unused)) HONTAI_init()
{
	NRF24L01_Initialize();

	NRF24L01_SetTxRxMode(TX_EN);

	if(sub_protocol == JJRCX1)
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)"\xd2\xb5\x99\xb3\x4a", 5);
	else
		XN297_SetTXAddr((const uint8_t*)"\xd2\xb5\x99\xb3\x4a", 5);

	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);				// No Auto Acknowldgement on all data pipes
	NRF24L01_SetBitrate(NRF24L01_BR_1M);					// 1Mbps
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);								// Activate feature register
	if(sub_protocol == JJRCX1)
	{
		NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xff);	// JJRC uses dynamic payload length
		NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3f);			// match other stock settings even though AA disabled...
		NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);
	}
	else
	{
		NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);	// no retransmits
		NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);			// Disable dynamic payload length on all pipes
		NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);
	}
	NRF24L01_Activate(0x73);								// Deactivate feature register
}

const uint8_t PROGMEM HONTAI_hopping_frequency_nonels[][3] = {
	{0x05, 0x19, 0x28},     // Hontai
	{0x0a, 0x1e, 0x2d}};    // JJRC X1

const uint8_t PROGMEM HONTAI_addr_vals[4][16] = {
	{0x24, 0x26, 0x2a, 0x2c, 0x32, 0x34, 0x36, 0x4a, 0x4c, 0x4e, 0x54, 0x56, 0x5a, 0x64, 0x66, 0x6a},
	{0x92, 0x94, 0x96, 0x9a, 0xa4, 0xa6, 0xac, 0xb2, 0xb4, 0xb6, 0xca, 0xcc, 0xd2, 0xd4, 0xd6, 0xda},
	{0x93, 0x95, 0x99, 0x9b, 0xa5, 0xa9, 0xab, 0xad, 0xb3, 0xb5, 0xc9, 0xcb, 0xcd, 0xd3, 0xd5, 0xd9},
	{0x25, 0x29, 0x2b, 0x2d, 0x33, 0x35, 0x49, 0x4b, 0x4d, 0x59, 0x5b, 0x65, 0x69, 0x6b, 0x6d, 0x6e}};

static void __attribute__((unused)) HONTAI_init2()
{
	uint8_t data_tx_addr[5];

	//TX address
	data_tx_addr[0] = pgm_read_byte_near( &HONTAI_addr_vals[0][ rx_tx_addr[3]       & 0x0f]);
	data_tx_addr[1] = pgm_read_byte_near( &HONTAI_addr_vals[1][(rx_tx_addr[3] >> 4) & 0x0f]);
	data_tx_addr[2] = pgm_read_byte_near( &HONTAI_addr_vals[2][ rx_tx_addr[4]       & 0x0f]);
	data_tx_addr[3] = pgm_read_byte_near( &HONTAI_addr_vals[3][(rx_tx_addr[4] >> 4) & 0x0f]);
	data_tx_addr[4] = 0x24;
	if(sub_protocol == JJRCX1)
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, data_tx_addr, sizeof(data_tx_addr));
	else
		XN297_SetTXAddr(data_tx_addr, sizeof(data_tx_addr));

	//Hopping frequency table
	for(uint8_t i=0;i<3;i++)
		hopping_frequency[i]=pgm_read_byte_near( &HONTAI_hopping_frequency_nonels[sub_protocol == JJRCX1?1:0][i] );
	hopping_frequency_no=0;
}

static void __attribute__((unused)) HONTAI_initialize_txid()
{
	rx_tx_addr[4] = rx_tx_addr[2]; 
	if(sub_protocol == HONTAI || sub_protocol == FQ777_951)
	{
		rx_tx_addr[0] = 0x4c; // first three bytes some kind of model id? - set same as stock tx
		rx_tx_addr[1] = 0x4b;
		rx_tx_addr[2] = 0x3a;
	}
	else
	{
		rx_tx_addr[0] = 0x4b; // JJRC X1
		rx_tx_addr[1] = 0x59;
		rx_tx_addr[2] = 0x3a;
	}
}

uint16_t HONTAI_callback()
{
	if(bind_counter!=0)
	{
		HONTAI_send_packet(1);
		bind_counter--;
		if (bind_counter == 0)
		{
			HONTAI_init2();
			BIND_DONE;
		}
	}
	else
		HONTAI_send_packet(0);

	return sub_protocol == FQ777_951 ? FQ777_951_PACKET_PERIOD : HONTAI_PACKET_PERIOD;
}

uint16_t initHONTAI()
{
	BIND_IN_PROGRESS;	// autobind protocol
	bind_counter = HONTAI_BIND_COUNT;
	HONTAI_initialize_txid();
	HONTAI_init();
	return HONTAI_INITIAL_WAIT;
}
#endif


# 1 "src/ASSAN_nrf24l01.ino" // Helps debugging !


/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(ASSAN_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define ASSAN_PACKET_SIZE		20
#define ASSAN_RF_BIND_CHANNEL	0x03
#define ASSAN_ADDRESS_LENGTH	4

enum {
    ASSAN_BIND0=0,
    ASSAN_BIND1,
    ASSAN_BIND2,
    ASSAN_DATA0,
    ASSAN_DATA1,
    ASSAN_DATA2,
    ASSAN_DATA3,
    ASSAN_DATA4,
    ASSAN_DATA5
};

void ASSAN_init()
{
    NRF24L01_Initialize();
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);			// 4 bytes rx/tx address
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)"\x80\x80\x80\xB8", ASSAN_ADDRESS_LENGTH);		// Bind address
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t *)"\x80\x80\x80\xB8", ASSAN_ADDRESS_LENGTH);	// Bind address
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);				// No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);			// Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, ASSAN_PACKET_SIZE);
    NRF24L01_SetPower();
}

void ASSAN_send_packet()
{
	for(uint8_t i=0;i<8;i++)
	{
		uint16_t val=Channel_data[i];
		val=((val<<2)+val)+(860<<3);					// PPM value <<3
		
		packet[2*i]=val>>8;
		packet[2*i+1]=val;
	}
	for(uint8_t i=0;i<ASSAN_ADDRESS_LENGTH;i++)
		packet[16+i]=packet[23-i];
 	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);		// Clear data ready, data sent, and retransmit
	NRF24L01_FlushTx();
	NRF24L01_WritePayload(packet, ASSAN_PACKET_SIZE);
}

uint16_t ASSAN_callback()
{
	switch (phase)
	{
	// Bind
		case ASSAN_BIND0:
			//Config RX @1M
			NRF24L01_WriteReg(NRF24L01_05_RF_CH, ASSAN_RF_BIND_CHANNEL);
			NRF24L01_SetBitrate(NRF24L01_BR_1M);					// 1Mbps
			NRF24L01_SetTxRxMode(RX_EN);
			phase++;
		case ASSAN_BIND1:
			//Wait for receiver to send the frames
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ //Something has been received
				NRF24L01_ReadPayload(packet, ASSAN_PACKET_SIZE);
				if(packet[19]==0x13)
				{ //Last frame received
					phase++;
					//Switch to TX
					NRF24L01_SetTxRxMode(TXRX_OFF);
					NRF24L01_SetTxRxMode(TX_EN);
					//Prepare bind packet
					memset(packet,0x05,ASSAN_PACKET_SIZE-5);
					packet[15]=0x99;
					for(uint8_t i=0;i<ASSAN_ADDRESS_LENGTH;i++)
						packet[16+i]=packet[23-i];
					packet_count=0;
					delayMilliseconds(260);
					return 10000;	// Wait 270ms in total...
				}
			}
			return 1000;
		case ASSAN_BIND2:
			// Send 20 packets
			packet_count++;
			if(packet_count==20)
				packet[15]=0x13;	// different value for last packet
			NRF24L01_WritePayload(packet, ASSAN_PACKET_SIZE);
			if(packet_count==20)
			{
				phase++;
				delayMilliseconds(2165);
			}
			return 22520;
	// Normal operation
		case ASSAN_DATA0:
			// Bind Done
			BIND_DONE;
			NRF24L01_SetBitrate(NRF24L01_BR_250K);					// 250Kbps
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(TX_EN);
		case ASSAN_DATA1:
		case ASSAN_DATA4:
			// Change ID and RF channel
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,packet+20+4*hopping_frequency_no, ASSAN_ADDRESS_LENGTH);
			NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
			hopping_frequency_no^=0x01;
			NRF24L01_SetPower();
			phase=ASSAN_DATA2;
			return 2000;
		case ASSAN_DATA2:
		case ASSAN_DATA3:
			ASSAN_send_packet();
			phase++;	// DATA 3 or 4
			return 5000;
	}
	return 0;
}

static void __attribute__((unused)) ASSAN_initialize_txid()
{
/*	//Renaud TXID with Freq=36 and alternate Freq 67 or 68 or 69 or 70 or 71 or 73 or 74 or 75 or 78 and may be more...
	packet[23]=0x22;
	packet[22]=0x37;
	packet[21]=0xFA;
	packet[20]=0x53; */
	// Using packet[20..23] to store the ID1 and packet[24..27] to store the ID2
	uint8_t freq=0,freq2;
	for(uint8_t i=0;i<ASSAN_ADDRESS_LENGTH;i++)
	{
		uint8_t temp=rx_tx_addr[i];
		packet[i+20]=temp;
		packet[i+24]=temp+1;
		freq+=temp;
	}	

	// Main frequency
	freq=((freq%25)+2)<<1;
	if(freq&0x02)	freq|=0x01;
	hopping_frequency[0]=freq;
	// Alternate frequency has some random
	do
	{
		freq2=random(0xfefefefe)%9;
		freq2+=freq*2-5;
	}
	while( (freq2>118) || (freq2<freq+1) || (freq2==2*freq) );
	hopping_frequency[1]=freq2;
}

uint16_t initASSAN()
{
	ASSAN_initialize_txid();
	ASSAN_init();
	hopping_frequency_no = 0;

	if(IS_BIND_IN_PROGRESS)
		phase=ASSAN_BIND0;
	else 
		phase=ASSAN_DATA0;
	return 1000;
}

#endif


# 1 "src/FlySky_a7105.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/flysky_a7105.c dated 2015-09-28

#if defined(FLYSKY_A7105_INO)

#include "iface_a7105.h"

//FlySky constants & variables
#define FLYSKY_BIND_COUNT 2500

enum {
	// flags going to byte 10
	FLAG_V9X9_VIDEO = 0x40,
	FLAG_V9X9_CAMERA= 0x80,
	// flags going to byte 12
	FLAG_V9X9_FLIP   = 0x10,
	FLAG_V9X9_LED   = 0x20,
};

enum {
	// flags going to byte 13
	FLAG_V6X6_HLESS1= 0x80,
	// flags going to byte 14
	FLAG_V6X6_VIDEO = 0x01,
	FLAG_V6X6_YCAL  = 0x02,
	FLAG_V6X6_XCAL  = 0x04,
	FLAG_V6X6_RTH   = 0x08,
	FLAG_V6X6_CAMERA= 0x10,
	FLAG_V6X6_HLESS2= 0x20,
	FLAG_V6X6_LED   = 0x40,
	FLAG_V6X6_FLIP  = 0x80,
};

enum {
	// flags going to byte 14
	FLAG_V912_TOPBTN= 0x40,
	FLAG_V912_BTMBTN= 0x80,
};

const uint8_t PROGMEM V912_X17_SEQ[10] =  { 0x14, 0x31, 0x40, 0x49, 0x49,    // sometime first byte is 0x15 ?
											0x49, 0x49, 0x49, 0x49, 0x49, }; 

static void __attribute__((unused)) flysky_apply_extension_flags()
{
	switch(sub_protocol)
	{
		case V9X9:
			if(CH5_SW)
				packet[12] |= FLAG_V9X9_FLIP;
			if(CH6_SW)
				packet[12] |= FLAG_V9X9_LED;
			if(CH7_SW)
				packet[10] |= FLAG_V9X9_CAMERA;
			if(CH8_SW)
				packet[10] |= FLAG_V9X9_VIDEO;
			break;
			
		case V6X6:
			packet[13] = 0x03; // 3 = 100% rate (0=40%, 1=60%, 2=80%)
			packet[14] = 0x00;
			if(CH5_SW) 
				packet[14] |= FLAG_V6X6_FLIP;
			if(CH6_SW) 
				packet[14] |= FLAG_V6X6_LED;
			if(CH7_SW) 
				packet[14] |= FLAG_V6X6_CAMERA;
			if(CH8_SW) 
				packet[14] |= FLAG_V6X6_VIDEO;
			if(CH9_SW)
			{ 
				packet[13] |= FLAG_V6X6_HLESS1;
				packet[14] |= FLAG_V6X6_HLESS2;
			}
			if(CH10_SW)
				packet[14] |= FLAG_V6X6_RTH;
			if(CH11_SW) 
				packet[14] |= FLAG_V6X6_XCAL;
			if(CH12_SW) 
				packet[14] |= FLAG_V6X6_YCAL;
			packet[15] = 0x10; // unknown
			packet[16] = 0x10; // unknown
			packet[17] = 0xAA; // unknown
			packet[18] = 0xAA; // unknown
			packet[19] = 0x60; // unknown, changes at irregular interval in stock TX
			packet[20] = 0x02; // unknown
			break;
			
		case V912:
			packet_count++;
			if( packet_count > 9)
				packet_count = 0;
			packet[12] |= 0x20; // bit 6 is always set ?
			packet[13] = 0x00;  // unknown
			packet[14] = 0x00;
			if(CH5_SW)
				packet[14]  = FLAG_V912_BTMBTN;
			if(CH6_SW)
				packet[14] |= FLAG_V912_TOPBTN;
			packet[15] = 0x27; // [15] and [16] apparently hold an analog channel with a value lower than 1000
			packet[16] = 0x03; // maybe it's there for a pitch channel for a CP copter ?
			packet[17] = pgm_read_byte( &V912_X17_SEQ[packet_count] ) ; // not sure what [17] & [18] are for
			if(packet_count == 0)                    // V912 Rx does not even read those bytes... [17-20]
				packet[18] = 0x02;
			else
				packet[18] = 0x00;
			packet[19] = 0x00; // unknown
			packet[20] = 0x00; // unknown
			break;
			
		case CX20:
			packet[19] = 0x00; // unknown
			packet[20] = (hopping_frequency_no<<4)|0x0A;
			break;
		default:
			break; 
	}
}

static void __attribute__((unused)) flysky_build_packet(uint8_t init)
{
    uint8_t i;
	//servodata timing range for flysky.
	//-100% =~ 0x03e8//=1000us(min)
	//+100% =~ 0x07ca//=1994us(max)
	//Center = 0x5d9//=1497us(center)
	//channel order AIL;ELE;THR;RUD;CH5;CH6;CH7;CH8
    packet[0] = init ? 0xaa : 0x55;
    packet[1] = rx_tx_addr[3];
    packet[2] = rx_tx_addr[2];
    packet[3] = rx_tx_addr[1];
    packet[4] = rx_tx_addr[0];
	for(i = 0; i < 8; i++)
	{
		uint16_t temp=convert_channel_ppm(CH_AETR[i]);
		if(sub_protocol == CX20 && CH_AETR[i]==ELEVATOR)
			temp=3000-temp;
		packet[5 + i*2]=temp&0xFF;		//low byte of servo timing(1000-2000us)
		packet[6 + i*2]=(temp>>8)&0xFF;	//high byte of servo timing(1000-2000us)
	}
    flysky_apply_extension_flags();
}

uint16_t ReadFlySky()
{
	#ifndef FORCE_FLYSKY_TUNING
		A7105_AdjustLOBaseFreq(1);
	#endif
	if(IS_BIND_IN_PROGRESS)
	{
		flysky_build_packet(1);
		A7105_WriteData(21, 1);
		bind_counter--;
		if (bind_counter==0)
			BIND_DONE;
	}
	else
	{
		flysky_build_packet(0);
		A7105_WriteData(21, hopping_frequency[hopping_frequency_no & 0x0F]);
		A7105_SetPower();
	}
	hopping_frequency_no++;

	if(sub_protocol==CX20)
		return 3984;
	else
		return 1510;	//1460 on deviation but not working with the latest V911 bricks... Turnigy 9X v2 is 1533, Flysky TX for 9XR/9XR Pro is 1510, V911 TX is 1490.
}

const uint8_t PROGMEM tx_channels[8][4] = {
	{ 0x12, 0x34, 0x56, 0x78},
	{ 0x18, 0x27, 0x36, 0x45},
	{ 0x41, 0x82, 0x36, 0x57},
	{ 0x84, 0x13, 0x65, 0x72},
	{ 0x87, 0x64, 0x15, 0x32},
	{ 0x76, 0x84, 0x13, 0x52},
	{ 0x71, 0x62, 0x84, 0x35},
	{ 0x71, 0x86, 0x43, 0x52}
};

uint16_t initFlySky()
{
	uint8_t chanrow;
	uint8_t chanoffset;
	uint8_t temp;

	A7105_Init();
	
	// limit offset to 9 as higher values don't work with some RX (ie V912)
	// limit offset to 9 as CX20 repeats the same channels after that
	if ((rx_tx_addr[3]&0xF0) > 0x90)
		rx_tx_addr[3]=rx_tx_addr[3]-0x70;

	// Build frequency hop table
	chanrow=rx_tx_addr[3] & 0x0F;
	chanoffset=rx_tx_addr[3]/16;
	for(uint8_t i=0;i<16;i++)
	{
		temp=pgm_read_byte_near(&tx_channels[chanrow>>1][i>>2]);
		if(i&0x02)
			temp&=0x0F;
		else
			temp>>=4;
		temp*=0x0A;
		if(i&0x01)
			temp+=0x50;
		if(sub_protocol==CX20)
		{
			if(temp==0x0A)
				temp+=0x37;
			if(temp==0xA0)
			{
				if (chanoffset<4)
					temp=0x37;
				else if (chanoffset<9)
					temp=0x2D;
				else
					temp=0x29;
			}
		}
		hopping_frequency[((chanrow&1)?15-i:i)]=temp-chanoffset;
	}
	hopping_frequency_no=0;
	packet_count=0;
	if(IS_BIND_IN_PROGRESS)
		bind_counter = FLYSKY_BIND_COUNT;
	else
		bind_counter = 0;
	return 2400;
}
#endif


# 1 "src/Common.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef FAILSAFE_ENABLE
//Convert from percentage to failsafe value
#define FAILSAFE_THROTTLE_LOW_VAL (((FAILSAFE_THROTTLE_LOW+125)*1024)/125)
#if FAILSAFE_THROTTLE_LOW_VAL <= 0
	#undef FAILSAFE_THROTTLE_LOW_VAL
	#define FAILSAFE_THROTTLE_LOW_VAL 1
#elif (FAILSAFE_THROTTLE_LOW_VAL) >= 2046
	#undef FAILSAFE_THROTTLE_LOW_VAL
	#define FAILSAFE_THROTTLE_LOW_VAL 2046
#endif
void InitFailsafe()
{
	for(uint8_t i=0;i<NUM_CHN;i++)
		Failsafe_data[i]=1024;
	Failsafe_data[THROTTLE]=(uint16_t)FAILSAFE_THROTTLE_LOW_VAL;	//1=-125%, 204=-100%
	FAILSAFE_VALUES_on;
	#ifdef FAILSAFE_SERIAL_ONLY
		if(mode_select == MODE_SERIAL)
			FAILSAFE_VALUES_off;
	#endif
}
#endif
#ifdef ENABLE_PPM
void InitPPM()
{
	for(uint8_t i=0;i<NUM_CHN;i++)
		PPM_data[i]=PPM_MAX_100+PPM_MIN_100;
	PPM_data[THROTTLE]=PPM_MIN_100*2;
}
#endif
void InitChannel()
{
	for(uint8_t i=0;i<NUM_CHN;i++)
		Channel_data[i]=1024;
	#ifdef FAILSAFE_THROTTLE_LOW_VAL
		Channel_data[THROTTLE]=(uint16_t)FAILSAFE_THROTTLE_LOW_VAL;	//0=-125%, 204=-100%
	#else
		Channel_data[THROTTLE]=204;
	#endif
}
/************************/
/**  Convert routines  **/
/************************/
// Revert a channel and store it
void reverse_channel(uint8_t num)
{
	uint16_t val=2048-Channel_data[num];
	if(val>=2048) val=2047;
	Channel_data[num]=val;
}
// Channel value is converted to ppm 860<->2140 -125%<->+125% and 988<->2012 -100%<->+100%
uint16_t convert_channel_ppm(uint8_t num)
{
	uint16_t val=Channel_data[num];
	return (((val<<2)+val)>>3)+860;									//value range 860<->2140 -125%<->+125%
}
// Channel value 100% is converted to 10bit values 0<->1023
uint16_t convert_channel_10b(uint8_t num)
{
	uint16_t val=Channel_data[num];
	val=((val<<2)+val)>>3;
	if(val<=128) return 0;
	if(val>=1152) return 1023;
	return val-128;
}
// Channel value 100% is converted to 8bit values 0<->255
uint8_t convert_channel_8b(uint8_t num)
{
	uint16_t val=Channel_data[num];
	val=((val<<2)+val)>>5;
	if(val<=32) return 0;
	if(val>=288) return 255;
	return val-32;
}

// Channel value 100% is converted to value scaled
int16_t convert_channel_16b_limit(uint8_t num,int16_t min,int16_t max)
{
	int32_t val=limit_channel_100(num);			// 204<->1844
	val=(val-CHANNEL_MIN_100)*(max-min)/(CHANNEL_MAX_100-CHANNEL_MIN_100)+min;
	return (uint16_t)val;
}

// Channel value -125%<->125% is scaled to 16bit value with no limit
int16_t convert_channel_16b_nolimit(uint8_t num, int16_t min, int16_t max)
{
	int32_t val=Channel_data[num];				// 0<->2047
	val=(val-CHANNEL_MIN_100)*(max-min)/(CHANNEL_MAX_100-CHANNEL_MIN_100)+min;
	return (uint16_t)val;
}

// Channel value is converted sign + magnitude 8bit values
uint8_t convert_channel_s8b(uint8_t num)
{
	uint8_t ch;
	ch = convert_channel_8b(num);
	return (ch < 128 ? 127-ch : ch);	
}

// Channel value is limited to 100%
uint16_t limit_channel_100(uint8_t num)
{
	if(Channel_data[num]>=CHANNEL_MAX_100)
		return CHANNEL_MAX_100;
	if (Channel_data[num]<=CHANNEL_MIN_100)
		return CHANNEL_MIN_100;
	return Channel_data[num];
}

// Channel value is converted for HK310
void convert_channel_HK310(uint8_t num, uint8_t *low, uint8_t *high)
{
	uint16_t temp=0xFFFF-(3440+((Channel_data[num]*5)>>1))/3;
	*low=(uint8_t)(temp&0xFF);
	*high=(uint8_t)(temp>>8);
}
#ifdef FAILSAFE_ENABLE
// Failsafe value is converted for HK310
void convert_failsafe_HK310(uint8_t num, uint8_t *low, uint8_t *high)
{
	uint16_t temp=0xFFFF-(3440+((Failsafe_data[num]*5)>>1))/3;
	*low=(uint8_t)(temp&0xFF);
	*high=(uint8_t)(temp>>8);
}
#endif

// Channel value for FrSky (PPM is multiplied by 1.5)
uint16_t convert_channel_frsky(uint8_t num)
{
	uint16_t val=Channel_data[num];
	return ((val*15)>>4)+1290;
}

/******************************/
/**  FrSky D and X routines  **/
/******************************/
#if defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO)
enum {
	FRSKY_BIND		= 0,
	FRSKY_BIND_DONE	= 1000,
	FRSKY_DATA1,
	FRSKY_DATA2,
	FRSKY_DATA3,
	FRSKY_DATA4,
	FRSKY_DATA5
};

void Frsky_init_hop(void)
{
	uint8_t val;
	uint8_t channel = rx_tx_addr[0]&0x07;
	uint8_t channel_spacing = rx_tx_addr[1];
	//Filter bad tables
	if(channel_spacing<0x02) channel_spacing+=0x02;
	if(channel_spacing>0xE9) channel_spacing-=0xE7;
	if(channel_spacing%0x2F==0) channel_spacing++;
		
	hopping_frequency[0]=channel;
	for(uint8_t i=1;i<50;i++)
	{
		channel=(channel+channel_spacing) % 0xEB;
		val=channel;
		if((val==0x00) || (val==0x5A) || (val==0xDC))
			val++;
		hopping_frequency[i]=i>46?0:val;
	}
}
#endif
/******************************/
/**  FrSky V, D and X routines  **/
/******************************/
#if defined(FRSKYV_CC2500_INO) || defined(FRSKYD_CC2500_INO) || defined(FRSKYX_CC2500_INO)
	const PROGMEM uint8_t FRSKY_common_startreg_cc2500_conf[]= {
		 CC2500_02_IOCFG0 ,		
		 CC2500_00_IOCFG2 ,
		 CC2500_17_MCSM1 ,
		 CC2500_18_MCSM0 ,
		 CC2500_06_PKTLEN ,
		 CC2500_07_PKTCTRL1 ,
		 CC2500_08_PKTCTRL0 ,
		 CC2500_3E_PATABLE ,
		 CC2500_0B_FSCTRL1 ,
		 CC2500_0C_FSCTRL0 ,	// replaced by option value
		 CC2500_0D_FREQ2 ,	
		 CC2500_0E_FREQ1 ,
		 CC2500_0F_FREQ0 ,
		 CC2500_10_MDMCFG4 ,		
		 CC2500_11_MDMCFG3 ,
		 CC2500_12_MDMCFG2 ,
		 CC2500_13_MDMCFG1 ,
		 CC2500_14_MDMCFG0 ,
		 CC2500_15_DEVIATN  };

	#if defined(FRSKYV_CC2500_INO)
		const PROGMEM uint8_t FRSKYV_cc2500_conf[]= {
		/*02_IOCFG0*/  	 0x06 ,		
		/*00_IOCFG2*/  	 0x06 ,
		/*17_MCSM1*/   	 0x0c ,
		/*18_MCSM0*/   	 0x18 ,
		/*06_PKTLEN*/  	 0xff ,
		/*07_PKTCTRL1*/	 0x04 ,
		/*08_PKTCTRL0*/	 0x05 ,
		/*3E_PATABLE*/ 	 0xfe ,
		/*0B_FSCTRL1*/ 	 0x08 ,
		/*0C_FSCTRL0*/ 	 0x00 ,
		/*0D_FREQ2*/   	 0x5c ,	
		/*0E_FREQ1*/   	 0x58 ,
		/*0F_FREQ0*/   	 0x9d ,
		/*10_MDMCFG4*/ 	 0xAA ,		
		/*11_MDMCFG3*/ 	 0x10 ,
		/*12_MDMCFG2*/ 	 0x93 ,
		/*13_MDMCFG1*/ 	 0x23 ,
		/*14_MDMCFG0*/ 	 0x7a ,
		/*15_DEVIATN*/ 	 0x41  };
	#endif

	#if defined(FRSKYD_CC2500_INO)
		const PROGMEM uint8_t FRSKYD_cc2500_conf[]= {
		/*02_IOCFG0*/  	 0x06 ,		
		/*00_IOCFG2*/  	 0x06 ,
		/*17_MCSM1*/   	 0x0c ,
		/*18_MCSM0*/   	 0x18 ,
		/*06_PKTLEN*/  	 0x19 ,
		/*07_PKTCTRL1*/	 0x04 ,
		/*08_PKTCTRL0*/	 0x05 ,
		/*3E_PATABLE*/ 	 0xff ,
		/*0B_FSCTRL1*/ 	 0x08 ,
		/*0C_FSCTRL0*/ 	 0x00 ,
		/*0D_FREQ2*/   	 0x5c ,	
		/*0E_FREQ1*/   	 0x76 ,
		/*0F_FREQ0*/   	 0x27 ,
		/*10_MDMCFG4*/ 	 0xAA ,		
		/*11_MDMCFG3*/ 	 0x39 ,
		/*12_MDMCFG2*/ 	 0x11 ,
		/*13_MDMCFG1*/ 	 0x23 ,
		/*14_MDMCFG0*/ 	 0x7a ,
		/*15_DEVIATN*/ 	 0x42  };
	#endif

	#if defined(FRSKYX_CC2500_INO)
		const PROGMEM uint8_t FRSKYX_cc2500_conf[]= {
	//FRSKYX
		/*02_IOCFG0*/  	 0x06 ,		
		/*00_IOCFG2*/  	 0x06 ,
		/*17_MCSM1*/   	 0x0c ,
		/*18_MCSM0*/   	 0x18 ,
		/*06_PKTLEN*/  	 0x1E ,
		/*07_PKTCTRL1*/	 0x04 ,
		/*08_PKTCTRL0*/	 0x01 ,
		/*3E_PATABLE*/ 	 0xff ,
		/*0B_FSCTRL1*/ 	 0x0A ,
		/*0C_FSCTRL0*/ 	 0x00 ,
		/*0D_FREQ2*/   	 0x5c ,	
		/*0E_FREQ1*/   	 0x76 ,
		/*0F_FREQ0*/   	 0x27 ,
		/*10_MDMCFG4*/ 	 0x7B ,		
		/*11_MDMCFG3*/ 	 0x61 ,
		/*12_MDMCFG2*/ 	 0x13 ,
		/*13_MDMCFG1*/ 	 0x23 ,
		/*14_MDMCFG0*/ 	 0x7a ,
		/*15_DEVIATN*/ 	 0x51  };
		const PROGMEM uint8_t FRSKYXEU_cc2500_conf[]= {
		/*02_IOCFG0*/  	 0x06 ,		
		/*00_IOCFG2*/  	 0x06 ,
		/*17_MCSM1*/   	 0x0E ,
		/*18_MCSM0*/   	 0x18 ,
		/*06_PKTLEN*/  	 0x23 ,
		/*07_PKTCTRL1*/	 0x04 ,
		/*08_PKTCTRL0*/	 0x01 ,
		/*3E_PATABLE*/ 	 0xff ,
		/*0B_FSCTRL1*/ 	 0x08 ,
		/*0C_FSCTRL0*/ 	 0x00 ,
		/*0D_FREQ2*/   	 0x5c ,	
		/*0E_FREQ1*/   	 0x80 ,
		/*0F_FREQ0*/   	 0x00 ,
		/*10_MDMCFG4*/ 	 0x7B ,		
		/*11_MDMCFG3*/ 	 0xF8 ,
		/*12_MDMCFG2*/ 	 0x03 ,
		/*13_MDMCFG1*/ 	 0x23 ,
		/*14_MDMCFG0*/ 	 0x7a ,
		/*15_DEVIATN*/ 	 0x53  };
	#endif

	const PROGMEM uint8_t FRSKY_common_end_cc2500_conf[][2]= {
		{ CC2500_19_FOCCFG,   0x16 },
		{ CC2500_1A_BSCFG,    0x6c },	
		{ CC2500_1B_AGCCTRL2, 0x43 },
		{ CC2500_1C_AGCCTRL1, 0x40 },
		{ CC2500_1D_AGCCTRL0, 0x91 },
		{ CC2500_21_FREND1,   0x56 },
		{ CC2500_22_FREND0,   0x10 },
		{ CC2500_23_FSCAL3,   0xa9 },
		{ CC2500_24_FSCAL2,   0x0A },
		{ CC2500_25_FSCAL1,   0x00 },
		{ CC2500_26_FSCAL0,   0x11 },
		{ CC2500_29_FSTEST,   0x59 },
		{ CC2500_2C_TEST2,    0x88 },
		{ CC2500_2D_TEST1,    0x31 },
		{ CC2500_2E_TEST0,    0x0B },
		{ CC2500_03_FIFOTHR,  0x07 },
		{ CC2500_09_ADDR,     0x00 } };

	void FRSKY_init_cc2500(const uint8_t *ptr)
	{
		for(uint8_t i=0;i<19;i++)
		{
			uint8_t reg=pgm_read_byte_near(&FRSKY_common_startreg_cc2500_conf[i]);
			uint8_t val=pgm_read_byte_near(&ptr[i]);
			if(reg==CC2500_0C_FSCTRL0)
				val=option;
			CC2500_WriteReg(reg,val);
		}
		prev_option = option ;		// Save option to monitor FSCTRL0 change
		for(uint8_t i=0;i<17;i++)
		{
			uint8_t reg=pgm_read_byte_near(&FRSKY_common_end_cc2500_conf[i][0]);
			uint8_t val=pgm_read_byte_near(&FRSKY_common_end_cc2500_conf[i][1]);
			CC2500_WriteReg(reg,val);
		}
		CC2500_SetTxRxMode(TX_EN);
		CC2500_SetPower();
		CC2500_Strobe(CC2500_SIDLE);    // Go to idle...
	}
#endif


# 1 "src/Corona_cc2500.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(CORONA_CC2500_INO)

#include "iface_cc2500.h"

//#define CORONA_FORCE_ID

#define CORONA_RF_NUM_CHANNELS	3
#define CORONA_ADDRESS_LENGTH	4
#define CORONA_BIND_CHANNEL_V1	0xD1	// also Flydream V3
#define CORONA_BIND_CHANNEL_V2	0xB8
#define CORONA_COARSE			0x00
#define FDV3_BIND_PERIOD		5000
#define FDV3_CHANNEL_PERIOD		4000

const PROGMEM uint8_t CORONA_init_values[] = {
  /* 00 */ 0x29, 0x2E, 0x06, 0x07, 0xD3, 0x91, 0xFF, 0x04,
  /* 08 */ 0x05, 0x00, CORONA_BIND_CHANNEL_V1, 0x06, 0x00, 0x5C, 0x4E, 0xC4 + CORONA_COARSE,
  /* 10 */ 0x5B, 0xF8, 0x03, 0x23, 0xF8, 0x47, 0x07, 0x30,
  /* 18 */ 0x18, 0x16, 0x6C, 0x43, 0x40, 0x91, 0x87, 0x6B,
  /* 20 */ 0xF8, 0x56, 0x10, 0xA9, 0x0A, 0x00, 0x11, 0x41,
  /* 28 */ 0x00, 0x59, 0x7F, 0x3F, 0x81, 0x35, 0x0B
};

uint8_t fdv3_id_send;

static void __attribute__((unused)) CORONA_rf_init()
{
	CC2500_Strobe(CC2500_SIDLE);

	for (uint8_t i = 0; i <= 0x2E; ++i)
		CC2500_WriteReg(i, pgm_read_byte_near(&CORONA_init_values[i]));
	if(sub_protocol==COR_V2)
	{
		CC2500_WriteReg(CC2500_0A_CHANNR, CORONA_BIND_CHANNEL_V2);
		CC2500_WriteReg(CC2500_0E_FREQ1, 0x80);
		CC2500_WriteReg(CC2500_0F_FREQ0, 0x00 + CORONA_COARSE);
		CC2500_WriteReg(CC2500_15_DEVIATN, 0x50);
		CC2500_WriteReg(CC2500_17_MCSM1, 0x00);
	    CC2500_WriteReg(CC2500_1B_AGCCTRL2, 0x67);
		CC2500_WriteReg(CC2500_1C_AGCCTRL1, 0xFB);
		CC2500_WriteReg(CC2500_1D_AGCCTRL0, 0xDC);
	}
	else if(sub_protocol==FD_V3)
	{
		// Flydream receiver captures have deviation 50, tx captures show 47
		CC2500_WriteReg(CC2500_15_DEVIATN, 0x50);
	}
	
	prev_option = option;
	CC2500_WriteReg(CC2500_0C_FSCTRL0, option);

	//not sure what they are doing to the PATABLE since basically only the first byte is used and it's only 8 bytes long. So I think they end up filling the PATABLE fully with 0xFF
	CC2500_WriteRegisterMulti(CC2500_3E_PATABLE,(const uint8_t *)"\x08\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 13);

	CC2500_SetTxRxMode(TX_EN);
	CC2500_SetPower();
}

// Generate id and hopping freq
static void __attribute__((unused)) CORONA_init()
{
	#ifdef CORONA_FORCE_ID
		// Example of ID and channels taken from dumps
		switch(sub_protocol)
		{
			case COR_V1:
				memcpy((void *)rx_tx_addr,(void *)"\x1F\xFE\x6C\x35",CORONA_ADDRESS_LENGTH);
				memcpy((void *)hopping_frequency,(void *)"\x17\x0D\x03\x49",CORONA_RF_NUM_CHANNELS+1);
				break;
			case COR_V2:
				memcpy((void *)rx_tx_addr,(void *)"\xFE\xFE\x02\xFB",CORONA_ADDRESS_LENGTH);
				memcpy((void *)hopping_frequency,(void *)"\x14\x3D\x35",CORONA_RF_NUM_CHANNELS);
			case FD_V3:
				memcpy((void *)rx_tx_addr,(void *)"\x02\xFA\x38\x38",CORONA_ADDRESS_LENGTH);
				memcpy((void *)hopping_frequency,(void *)"\x71\xB9\x30",CORONA_RF_NUM_CHANNELS);
				break;
	}
	#else
		// From dumps channels are anything between 0x00 and 0xC5 on V1.
		// But 0x00 and 0xB8 should be avoided on V2 since they are used for bind.
		// Below code make sure channels are between 0x02 and 0xA0, spaced with
		// a minimum of 2 and not ordered (RX only use the 1st channel unless there is an issue).
		// Extra hopping frequency used for Flydream V3 id packets.
		uint8_t order=rx_tx_addr[3]&0x03;
		for(uint8_t i=0; i<CORONA_RF_NUM_CHANNELS+1; i++)
			hopping_frequency[i^order]=2+rx_tx_addr[3-i]%39+(i<<5)+(i<<3);

		if(sub_protocol!=FD_V3)
		{
			// ID looks random but on the 15 V1 dumps they all show the same odd/even rule
			if(rx_tx_addr[3]&0x01)
			{	// If [3] is odd then [0] is odd and [2] is even 
				rx_tx_addr[0]|=0x01;
				rx_tx_addr[2]&=0xFE;
			}
			else
			{	// If [3] is even then [0] is even and [2] is odd 
				rx_tx_addr[0]&=0xFE;
				rx_tx_addr[2]|=0x01;
			}
			rx_tx_addr[1]=0xFE;			// Always FE in the dumps of V1 and V2
		}
		else
		{
			rx_tx_addr[1]=0xFA;			// Always FA for Flydream V3
			rx_tx_addr[3]=hopping_frequency[CORONA_RF_NUM_CHANNELS];	// channel used for id/freq packets
		}
	#endif
}

static uint16_t __attribute__((unused)) CORONA_build_bind_pkt()
{
	if(sub_protocol==COR_V1)
	{	// V1
		if(bind_counter&1)
		{ // Send TX ID
			packet[0]=0x04;		// 5 bytes to follow
			for(uint8_t i=0; i<CORONA_ADDRESS_LENGTH; i++)
				packet[i+1]=rx_tx_addr[i];
			packet[5]=0xCD;		// Unknown but seems to be always the same value for V1
			return 3689;
		}
		else
		{ // Send hopping freq
			packet[0]=0x03;		// 4 bytes to follow
			for(uint8_t i=0; i<CORONA_RF_NUM_CHANNELS+1; i++)
				packet[i+1]=hopping_frequency[i];
			// Only the first 3 channels of hopping_frequency used for data
			return 3438;
		}
	}
	else
	{	// V2 and FDV3
		packet[0]=0x04;		// 5 bytes to follow
		for(uint8_t i=0; i<CORONA_ADDRESS_LENGTH; i++)
			packet[i+1]=rx_tx_addr[i];
		packet[5]=0x00;		// Unknown but seems to be always the same value for V2 and FDV3
		if(sub_protocol==FD_V3)
			return FDV3_BIND_PERIOD;
		else
			return 26791;
	}
}

// 8 Channels with direct values from PPM
static uint16_t __attribute__((unused)) CORONA_build_packet()
{
	CC2500_SetPower();
	if(state && sub_protocol==COR_V2)
	{	// Send identifier packet for 2.65sec. This is how the RX learns the hopping table after a bind. Why it's not part of the bind like V1 is a mistery...
		// Set channel
		CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
		state--;
		packet[0]=0x07;		// 8 bytes to follow
		// Send hopping freq
		for(uint8_t i=0; i<CORONA_RF_NUM_CHANNELS; i++)
			packet[i+1]=hopping_frequency[i];
		// Send TX ID
		for(uint8_t i=0; i<CORONA_ADDRESS_LENGTH; i++)
			packet[i+4]=rx_tx_addr[i];
		packet[8]=0;
		return 6647;
	}

	// Flydream every fourth packet is identifier packet and is on channel number
	// that is last byte of rx_tx_addr
	if (fdv3_id_send)
	{
		fdv3_id_send = 0;
		CC2500_WriteReg(CC2500_0A_CHANNR, rx_tx_addr[CORONA_ADDRESS_LENGTH-1]);
		packet[0] = 0x07;   // 8 bytes to follow
		// Send TX ID
		for(uint8_t i = 0; i < CORONA_ADDRESS_LENGTH; i++)
			packet[i+1] = rx_tx_addr[i];
		// Send hopping freq
		for(uint8_t i = 0; i < CORONA_RF_NUM_CHANNELS; i++)
			packet[i+1+CORONA_ADDRESS_LENGTH] = hopping_frequency[i];
		packet[8] = 0;
		return 2*FDV3_CHANNEL_PERIOD;  // extra delay after id packet according to captures
	}

	// Set RF channel
	CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[hopping_frequency_no]);

	// Build packet
	packet[0] = 0x10;   // 17 bytes to follow

	// Channels
	memset(packet+9, 0x00, 4);
	for (uint8_t i=0; i<8; i++)
	{ // Channel values are packed
		uint16_t val=convert_channel_ppm(i);
		packet[i+1] = val;
		packet[9 + (i>>1)] |= (i&0x01)?(val>>4)&0xF0:(val>>8)&0x0F;
	}

	// TX ID
	for (uint8_t i=0; i < CORONA_ADDRESS_LENGTH; i++)
		packet[i+13] = rx_tx_addr[i];

	packet[17] = 0x00;

	if (sub_protocol!=FD_V3)
	{
		// Packet period is based on hopping
		switch (hopping_frequency_no)
		{
			case 0:
				packet_period = sub_protocol == COR_V1
				? 4991
				: 4248;
				break;
			case 1: 
				packet_period = sub_protocol == COR_V1
				? 4991
				: 4345;
				break;
			case 2: 
				packet_period = sub_protocol == COR_V1
				? 12520
				: 13468;
				if (sub_protocol == COR_V2)
					packet[17] = 0x03;
				break;
		}
	}
	hopping_frequency_no++;

	if (sub_protocol == FD_V3)
	{
		if (hopping_frequency_no == CORONA_RF_NUM_CHANNELS)
		{
			fdv3_id_send = 1;
			packet_period = 6000; // extra delay before id packet according to captures
		}
		else
			packet_period = FDV3_CHANNEL_PERIOD;
	}

	hopping_frequency_no %= CORONA_RF_NUM_CHANNELS;
	return packet_period;
}

uint16_t ReadCORONA()
{
	// Tune frequency if it has been changed
	if ( prev_option != option )
	{
		CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
		prev_option = option ;
	}

	if(IS_BIND_IN_PROGRESS)
	{
		if (bind_counter-- == 0) BIND_DONE;
		packet_period=CORONA_build_bind_pkt();
	}
	else
		packet_period=CORONA_build_packet();

	// Send packet
	CC2500_WriteData(packet, packet[0]+2);
	return packet_period;
}

uint16_t initCORONA()
{
	switch(sub_protocol)
	{
		case COR_V1:
			bind_counter=1400;		// Stay in bind mode for 5s
			break;
		case COR_V2:
			bind_counter=187;		// Stay in bind mode for 5s
			break;
		case FD_V3:
			bind_counter = 2000;	// Stay in bind mode for 10s
			break;
	}
	state=400;					// Used by V2 to send RF channels + ID for 2.65s at startup
	hopping_frequency_no=0;
	fdv3_id_send = 0;
	CORONA_init();
	CORONA_rf_init();
	return 10000;
}

#endif

# 1 "src/WFLY_cyrf6936.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(WFLY_CYRF6936_INO)

#include "iface_cyrf6936.h"

//#define WFLY_FORCE_ID
#define WFLY_BIND_COUNT 1500	// around 15s
#define WFLY_NUM_FREQUENCE 4
#define WFLY_BIND_CHANNEL 0x09

enum {
	WFLY_BIND_TX=0,
	WFLY_BIND_PREP_RX,
	WFLY_BIND_RX,
	WFLY_PREP_DATA,
	WFLY_DATA,
};

const uint8_t PROGMEM WFLY_sop_bind[]={ 0x5A, 0xCC, 0xAE, 0x46, 0xB6, 0x31, 0xAE, 0x46 };
const uint8_t PROGMEM WFLY_sop_data[]={ 0xEF, 0x64, 0xB0, 0x2A, 0xD2, 0x8F, 0xB1, 0x2A };

//Most of the bytes are unknown... 1C A7 looks to be the bind ID, BF 13 is the TX ID, 15 is the channel used to send the hopping frequencies.
const uint8_t PROGMEM WFLY_bind_packet[]={ 0x1C, 0xA7, 0x60, 0x04, 0x04, 0xBF, 0x13, 0x15, 0xC5, 0x40, 0x8A, 0x37, 0xE0, 0xE8, 0x03, 0xA3 };


const uint8_t PROGMEM WFLY_init_vals[][2] = {
	//Init from dump
	{CYRF_1D_MODE_OVERRIDE, 0x19},			// Reset
	{CYRF_32_AUTO_CAL_TIME, 0x3C},			// Default init value
	{CYRF_35_AUTOCAL_OFFSET, 0x14},			// Default init value
	{CYRF_1B_TX_OFFSET_LSB, 0x55},			// Default init value
	{CYRF_1C_TX_OFFSET_MSB, 0x05},			// Default init value
	{CYRF_06_RX_CFG, 0x48 | 0x02},			// LNA enabled, Fast Turn Mode enabled, adding overwrite enable to not lockup RX
	{CYRF_10_FRAMING_CFG, 0xE8},			// SOP enable
	{CYRF_03_TX_CFG, 0x08 | CYRF_BIND_POWER},	// Original=0x0F, 8DR Mode, 32 chip codes
	{CYRF_0C_XTAL_CTRL, 0xC4},				// Enable XOUT as GPIO
	{CYRF_0D_IO_CFG, 0x04},					// Enable PACTL as GPIO
	{CYRF_0F_XACT_CFG, 0x21},				// Abort current operation
	{CYRF_1E_RX_OVERRIDE, 0x00},			// Accept packets with 0 seed for bind
	{CYRF_15_CRC_SEED_LSB, 0x00},			// CRC seed for bind
	{CYRF_16_CRC_SEED_MSB, 0x00},			// CRC seed for bind
};

static void __attribute__((unused)) WFLY_cyrf_bind_config()
{
	for(uint8_t i = 0; i < sizeof(WFLY_init_vals) / 2; i++)	
		CYRF_WriteRegister(pgm_read_byte_near(&WFLY_init_vals[i][0]), pgm_read_byte_near(&WFLY_init_vals[i][1]));

    CYRF_PROGMEM_ConfigSOPCode(WFLY_sop_bind);
	CYRF_ConfigRFChannel(WFLY_BIND_CHANNEL);
	CYRF_SetTxRxMode(TX_EN);
}

static void __attribute__((unused)) WFLY_cyrf_data_config()
{
	for(uint8_t i = 0; i < (sizeof(WFLY_init_vals) / 2)-3; i++)	
		CYRF_WriteRegister(pgm_read_byte_near(&WFLY_init_vals[i][0]), pgm_read_byte_near(&WFLY_init_vals[i][1]));

	//CYRF_WriteRegister(CYRF_1E_RX_OVERRIDE, 0x08);	// Do not accept CRC with 0 seed but not needed since the RX is not sending any data...
	CYRF_WriteRegister(CYRF_15_CRC_SEED_LSB, rx_tx_addr[2]);
	CYRF_WriteRegister(CYRF_16_CRC_SEED_MSB, rx_tx_addr[3]);
	
    CYRF_PROGMEM_ConfigSOPCode(WFLY_sop_data);
	CYRF_SetTxRxMode(TX_EN);
}

static uint16_t __attribute__((unused)) WFLY_send_data_packet()
{
	packet_count++;
	packet[0] = rx_tx_addr[2];
	packet[1] = rx_tx_addr[3];
	if(packet_count%4==3)
	{	// Send the hopping frequencies
		packet[2]=0x70;		// packet type
		packet[3]=0x04;		// unknown
		packet[4]=0x00;		// unknown
		packet[5]=0x04;		// unknown
		packet[6]=hopping_frequency[0];
		packet[7]=hopping_frequency[0];
		packet[8]=hopping_frequency[1];
		packet[9]=hopping_frequency[2];
		len=10;				// packet[10] contains the checksum
	}
	else
	{	// Send sticks packet
		uint8_t nbr_ch=option;
		if(nbr_ch<4) nbr_ch=9;			// 4 channels min can be sent, default to 9
		if(nbr_ch>9) nbr_ch=9;			// 9 channels max can be sent
		packet[2]=nbr_ch-3;				// nbr of channels to follow
		packet[3]=packet_count>>2;		// packet counter 0x00..0x3F
		len=4;
		for(uint8_t i=0;i<3;i++)
		{ // Channels
			uint16_t ch = convert_channel_16b_nolimit(i*4+0,151,847);
			uint8_t offset=i*5;
			packet[3+offset]|=ch<<6;
			packet[4+offset]=ch>>2;
			len++;
			if(--nbr_ch==0) break;
			ch = convert_channel_16b_nolimit(i*4+1,151,847);
			packet[5+offset]=ch;
			packet[6+offset]=ch>>8;
			len+=2;
			if(--nbr_ch==0) break;
			ch = convert_channel_16b_nolimit(i*4+2,151,847);
			packet[6+offset]|=ch<<2;
			packet[7+offset]=ch>>6;
			len++;
			if(--nbr_ch==0) break;
			ch = convert_channel_16b_nolimit(i*4+3,151,847);
			packet[7+offset]|=ch<<4;
			packet[8+offset]=ch>>4;
			len++;
			if(--nbr_ch==0) break;
		}
	}

	uint8_t sum=0;
	for(uint8_t i = 0; i < len; i++)
		sum += packet[i];
	packet[len] = sum;

	CYRF_ConfigRFChannel(hopping_frequency[(packet_count)%4]);
	CYRF_SetPower(0x08);
	CYRF_WriteDataPacketLen(packet, len+1);

	switch(packet_count%4)
	{
		case 0:
			return 1393;
		case 1:
			return 1330;
		case 2:
			return 1555;
	}
	return 1093;	// case 3
}

uint16_t ReadWFLY()
{
	uint8_t status,len,sum=0,check=0;
	uint8_t start;
	static uint8_t retry;

	switch(phase)
	{
		case WFLY_BIND_TX:
			CYRF_SetTxRxMode(TX_EN);
			CYRF_WriteDataPacketLen(packet, sizeof(WFLY_bind_packet));
			debug("P=");
			for(uint8_t i=0;i<sizeof(WFLY_bind_packet);i++)
				debug(" %02X",packet[i]);
			debugln(" , L=%02X", sizeof(WFLY_bind_packet));
			phase++;
			if(--bind_counter==0)
			{ // Switch to normal mode
				BIND_DONE;
				phase=WFLY_PREP_DATA;
			}
			return 2500;
		case WFLY_BIND_PREP_RX:
			start=micros();
			while ((uint8_t)((uint8_t)micros()-(uint8_t)start) < 200)				// Wait max 200µs for TX to finish
				if((CYRF_ReadRegister(CYRF_02_TX_CTRL) & 0x80) == 0x00)
					break;										// Packet transmission complete
			CYRF_SetTxRxMode(RX_EN);							//Receive mode
			CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x83);			//Prepare to receive
			retry=10;									//Timeout for RX
			phase=WFLY_BIND_RX;
			return 700;
		case WFLY_BIND_RX:
			//Read data from RX
			status = CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			if((status & 0x03) == 0x02)  						// RXC=1, RXE=0 then 2nd check is required (debouncing)
				status |= CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			CYRF_WriteRegister(CYRF_07_RX_IRQ_STATUS, 0x80);	// need to set RXOW before data read
			if((status & 0x07) == 0x02)
			{ // Data received with no errors
				len=CYRF_ReadRegister(CYRF_09_RX_COUNT);
				debugln("L=%02X",len)
				if(len==0x10)
				{
					CYRF_ReadDataPacketLen(pkt, len);
					debug("RX=");
					for(uint8_t i=0;i<0x0F;i++)
					{
						debug(" %02X",pkt[i]);
						if(pkt[i]==packet[i])
							check++;							// Verify quickly the content
						sum+=pkt[i];
					}
					debugln(" %02X",pkt[15]);
					if(sum==pkt[15] && check>=10)
					{ // Good packet received
						if(pkt[2]==0x64)
						{ // Switch to normal mode
							BIND_DONE;
							phase=WFLY_PREP_DATA;
							return 10000;
						}
						memcpy((void *)packet,(void *)pkt,0x10);	// Send back to the RX what we've just received with no modifications
					}
					phase=WFLY_BIND_TX;							
					return 200;
				}
			}
			if(status & 0x85 || --retry == 0)
			{ // RX error or no answer
				debugln("Abort");
				CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x20);		// Enable RX abort
				CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x21);		// Force end state
				CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x00);		// Disable RX abort
				phase=WFLY_BIND_TX;								// Retry sending bind packet
			}
			return 700;
		case WFLY_PREP_DATA:
			WFLY_cyrf_data_config();
			packet_count=0;
			phase++;
		case WFLY_DATA:
			start=micros();
			while ((uint8_t)((uint8_t)micros()-(uint8_t)start) < 200)
				if((CYRF_ReadRegister(CYRF_02_TX_CTRL) & 0x80) == 0x00)
					break;										// Packet transmission complete
			return WFLY_send_data_packet();
	}
	return 1000;
}

uint16_t initWFLY()
{ 
	//Random start channel
	uint8_t ch=0x0A+random(0xfefefefe)%0x0E;
	if(ch%3==0)
		ch++;								// remove these channels as they seem to not be working...
	rf_ch_num=0x0C+(rx_tx_addr[1]%4)*3;		// use the start channels which do not seem to work to send the hopping table instead
	
	#ifdef WFLY_FORCE_ID					// data taken from TX dump
		rx_tx_addr[2]=0xBF;					// ID
		rx_tx_addr[3]=0x13;					// ID
		ch=0x16;							// value seen between 0x0A and 0x17
		rc_ch_num=0x15						// RF channel to send the current hopping table
	#endif

	debug("ID:")
	for(uint8_t i=0;i<2;i++)
		debug(" %02X", rx_tx_addr[2+i]);
	debugln("");

	hopping_frequency[0]=ch;
	hopping_frequency[1]=ch+0x1E;
	hopping_frequency[2]=ch+0x2D;
	hopping_frequency[3]=rf_ch_num;			// RF channel used to send the current hopping table
	
	debug("RF Channels:")
	for(uint8_t i=0;i<WFLY_NUM_FREQUENCE;i++)
		debug(" %02X", hopping_frequency[i]);
	debugln("");

	if(IS_BIND_IN_PROGRESS)
	{
		bind_counter=WFLY_BIND_COUNT;
		WFLY_cyrf_bind_config();
		for(uint8_t i=0;i<sizeof(WFLY_bind_packet);i++)
			packet[i]=pgm_read_byte_near(&WFLY_bind_packet[i]);
		packet[5]=rx_tx_addr[2];
		packet[6]=rx_tx_addr[3];
		packet[7]=rf_ch_num;
		uint8_t sum=0;
		for(uint8_t i = 0; i < 15; i++)
			sum += packet[i];
		packet[15] = sum;
		phase=WFLY_BIND_TX;
	}
	else
		phase = WFLY_PREP_DATA;
	return 10000;
}

#endif


# 1 "src/SLT_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with deviation main github branch

#if defined(SLT_NRF24L01_INO)

#include "iface_nrf24l01.h"

//#define SLT_Q200_FORCE_ID

// For code readability
#define SLT_PAYLOADSIZE_V1 7
#define SLT_PAYLOADSIZE_V2 11
#define SLT_NFREQCHANNELS 15
#define SLT_TXID_SIZE 4

enum{
	// flags going to packet[6] (Q200)
	FLAG_Q200_FMODE	= 0x20,
	FLAG_Q200_VIDON	= 0x10,
	FLAG_Q200_FLIP	= 0x08,
	FLAG_Q200_VIDOFF= 0x04,
};

enum{
	// flags going to packet[6] (MR100 & Q100)
	FLAG_MR100_FMODE	= 0x20,
	FLAG_MR100_FLIP		= 0x04,
	FLAG_MR100_VIDEO	= 0x02,
	FLAG_MR100_PICTURE	= 0x01,
};

enum {
	SLT_BUILD=0,
	SLT_DATA1,
	SLT_DATA2,
	SLT_DATA3,
	SLT_BIND1,
	SLT_BIND2
};

static void __attribute__((unused)) SLT_init()
{
	NRF24L01_Initialize();
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); // 2-bytes CRC, radio off
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);		// Enable data pipe 0
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);		// 4-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);	// Disable auto retransmit
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);		// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 4);			// bytes of data payload for pipe 1
	NRF24L01_SetBitrate(NRF24L01_BR_250K);          	// 256kbps
	NRF24L01_SetPower();
	if(sub_protocol==SLT_V1)
		NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)"\xC3\xC3\xAA\x55", SLT_TXID_SIZE);
	else // V2
		NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)"\x7E\xB8\x63\xA9", SLT_TXID_SIZE);
	NRF24L01_FlushRx();
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, SLT_TXID_SIZE);
	NRF24L01_FlushTx();
	// Turn radio power on
	NRF24L01_SetTxRxMode(TX_EN);
}

static void __attribute__((unused)) SLT_set_freq(void)
{
	// Frequency hopping sequence generation
	for (uint8_t i = 0; i < SLT_TXID_SIZE; ++i)
	{
		uint8_t next_i = (i+1) % SLT_TXID_SIZE; // is & 3 better than % 4 ?
		uint8_t base = i < 2 ? 0x03 : 0x10;
		hopping_frequency[i*4 + 0]  = (rx_tx_addr[i] & 0x3f) + base;
		hopping_frequency[i*4 + 1]  = (rx_tx_addr[i] >> 2) + base;
		hopping_frequency[i*4 + 2]  = (rx_tx_addr[i] >> 4) + (rx_tx_addr[next_i] & 0x03)*0x10 + base;
		hopping_frequency[i*4 + 3]  = (rx_tx_addr[i] >> 6) + (rx_tx_addr[next_i] & 0x0f)*0x04 + base;
	}

	// Unique freq
	uint8_t max_freq=0x50;	//V1 and V2
	if(sub_protocol==Q200)
		max_freq=45;
	for (uint8_t i = 0; i < SLT_NFREQCHANNELS; ++i)
	{
		if(sub_protocol==Q200 && hopping_frequency[i] >= max_freq)
			hopping_frequency[i] = hopping_frequency[i] - max_freq + 0x03;
		uint8_t done = 0;
		while (!done)
		{
			done = 1;
			for (uint8_t j = 0; j < i; ++j)
				if (hopping_frequency[i] == hopping_frequency[j])
				{
					done = 0;
					hopping_frequency[i] += 7;
					if (hopping_frequency[i] >= max_freq)
						hopping_frequency[i] = hopping_frequency[i] - max_freq + 0x03;
				}
		}
	}
}

static void __attribute__((unused)) SLT_wait_radio()
{
	if (packet_sent)
		while (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_TX_DS)));
	packet_sent = 0;
}

static void __attribute__((unused)) SLT_send_packet(uint8_t len)
{
	SLT_wait_radio();
	NRF24L01_FlushTx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, _BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_RX_DR) | _BV(NRF24L01_07_MAX_RT));
	NRF24L01_WritePayload(packet, len);
	packet_sent = 1;
}

static void __attribute__((unused)) SLT_build_packet()
{
	static uint8_t calib_counter=0;
	
	// Set radio channel - once per packet batch
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
	if (++hopping_frequency_no >= SLT_NFREQCHANNELS)
		hopping_frequency_no = 0;

	// aileron, elevator, throttle, rudder, gear, pitch
	uint8_t e = 0; // byte where extension 2 bits for every 10-bit channel are packed
	for (uint8_t i = 0; i < 4; ++i)
	{
		uint16_t v = convert_channel_10b(CH_AETR[i]);
		if(sub_protocol>SLT_V2 && (CH_AETR[i]==THROTTLE || CH_AETR[i]==ELEVATOR) )
			v=1023-v;	// reverse throttle and elevator channels for Q100/Q200/MR100 protocols
		packet[i] = v;
		e = (e >> 2) | (uint8_t) ((v >> 2) & 0xC0);
	}
	// Extra bits for AETR
	packet[4] = e;
	// 8-bit channels
	packet[5] = convert_channel_8b(CH5);
	packet[6] = convert_channel_8b(CH6);
	if(sub_protocol!=SLT_V1)
	{
		if(sub_protocol==Q200)
			packet[6] =  GET_FLAG(CH9_SW , FLAG_Q200_FMODE)
						|GET_FLAG(CH10_SW, FLAG_Q200_FLIP)
						|GET_FLAG(CH11_SW, FLAG_Q200_VIDON)
						|GET_FLAG(CH12_SW, FLAG_Q200_VIDOFF);
		else if(sub_protocol==MR100 || sub_protocol==Q100)
			packet[6] =  GET_FLAG(CH9_SW , FLAG_MR100_FMODE)
						|GET_FLAG(CH10_SW, FLAG_MR100_FLIP)
						|GET_FLAG(CH11_SW, FLAG_MR100_VIDEO)	// Does not exist on the Q100 but...
						|GET_FLAG(CH12_SW, FLAG_MR100_PICTURE);	// Does not exist on the Q100 but...
		packet[7]=convert_channel_8b(CH7);
		packet[8]=convert_channel_8b(CH8);
		packet[9]=0xAA;				//normal mode for Q100/Q200, unknown for V2/MR100
		packet[10]=0x00;			//normal mode for Q100/Q200, unknown for V2/MR100
		if((sub_protocol==Q100 || sub_protocol==Q200) && CH13_SW)
		{//Calibrate
			packet[9]=0x77;			//enter calibration
			if(calib_counter>=20 && calib_counter<=25)	// 7 packets for Q100 / 3 packets for Q200
				packet[10]=0x20;	//launch calibration
			calib_counter++;
			if(calib_counter>250) calib_counter=250;
		}
		else
			calib_counter=0;
	}
}

static void __attribute__((unused)) SLT_send_bind_packet()
{
	SLT_wait_radio();
	BIND_IN_PROGRESS;				//Limit TX power to bind level
	NRF24L01_SetPower();
	BIND_DONE;
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)"\x7E\xB8\x63\xA9", SLT_TXID_SIZE);

	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x50);
	memcpy((void*)packet,(void*)rx_tx_addr,SLT_TXID_SIZE);
	if(phase==SLT_BIND2)
		SLT_send_packet(SLT_TXID_SIZE);
	else // SLT_BIND1
		SLT_send_packet(SLT_PAYLOADSIZE_V2);

	SLT_wait_radio();				//Wait until the packet's sent before changing TX address!

	NRF24L01_SetPower();			//Change power back to normal level
	if(phase==SLT_BIND2) // after V1 bind and V2 second bind packet
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, SLT_TXID_SIZE);
}

#define SLT_TIMING_BUILD		1000
#define SLT_V1_TIMING_PACKET	1000
#define SLT_V2_TIMING_PACKET	2042
#define SLT_V1_TIMING_BIND2		1000
#define SLT_V2_TIMING_BIND1		6507
#define SLT_V2_TIMING_BIND2		2112
uint16_t SLT_callback()
{
	switch (phase)
	{
		case SLT_BUILD:
			SLT_build_packet();
			phase++;
			return SLT_TIMING_BUILD;
		case SLT_DATA1:
		case SLT_DATA2:
			phase++;
			if(sub_protocol==SLT_V1)
			{
				SLT_send_packet(SLT_PAYLOADSIZE_V1);
				return SLT_V1_TIMING_PACKET;
			}
			else //V2
			{
				SLT_send_packet(SLT_PAYLOADSIZE_V2);
				return SLT_V2_TIMING_PACKET;
			}
		case SLT_DATA3:
			if(sub_protocol==SLT_V1)
				SLT_send_packet(SLT_PAYLOADSIZE_V1);
			else //V2
				SLT_send_packet(SLT_PAYLOADSIZE_V2);
			if (++packet_count >= 100)
			{// Send bind packet
				packet_count = 0;
				if(sub_protocol==SLT_V1)
				{
					phase=SLT_BIND2;
					return SLT_V1_TIMING_BIND2;
				}
				else //V2
				{
					phase=SLT_BIND1;
					return SLT_V2_TIMING_BIND1;
				}
			}
			else
			{// Continue to send normal packets
				NRF24L01_SetPower();	// Set tx_power
				phase = SLT_BUILD;
				if(sub_protocol==SLT_V1)
					return 20000-SLT_TIMING_BUILD;
				else //V2
					return 13730-SLT_TIMING_BUILD;
			}
		case SLT_BIND1:
			SLT_send_bind_packet();
			phase++;
			return SLT_V2_TIMING_BIND2;
		case SLT_BIND2:
			SLT_send_bind_packet();
			phase = SLT_BUILD;
			if(sub_protocol==SLT_V1)
				return 20000-SLT_TIMING_BUILD-SLT_V1_TIMING_BIND2;
			else //V2
				return 13730-SLT_TIMING_BUILD-SLT_V2_TIMING_BIND1-SLT_V2_TIMING_BIND2;
	}
	return 19000;
}

uint16_t initSLT()
{
	packet_count = 0;
	packet_sent = 0;
	hopping_frequency_no = 0;
	if(sub_protocol==Q200)
	{ //Q200: Force high part of the ID otherwise it won't bind
		rx_tx_addr[0]=0x01;
		rx_tx_addr[1]=0x02;
		#ifdef SLT_Q200_FORCE_ID	// ID taken from TX dumps
			rx_tx_addr[0]=0x01;rx_tx_addr[1]=0x02;rx_tx_addr[2]=0x6A;rx_tx_addr[3]=0x31;
		/*	rx_tx_addr[0]=0x01;rx_tx_addr[1]=0x02;rx_tx_addr[2]=0x0B;rx_tx_addr[3]=0x57;*/
		#endif
	}
	SLT_set_freq();
	SLT_init();
	phase = SLT_BUILD;
	return 50000;
}

#endif


# 1 "src/FrSkyD_cc2500.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(FRSKYD_CC2500_INO)

#include "iface_cc2500.h"

static void __attribute__((unused)) frsky2way_init(uint8_t bind)
{
	FRSKY_init_cc2500(FRSKYD_cc2500_conf);	

	CC2500_WriteReg(CC2500_09_ADDR, bind ? 0x03 : rx_tx_addr[3]);
	CC2500_WriteReg(CC2500_07_PKTCTRL1, 0x05);
	CC2500_Strobe(CC2500_SIDLE);	// Go to idle...
	//
	CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
	CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
	CC2500_Strobe(CC2500_SFRX);
	//#######END INIT########		
}
	
static void __attribute__((unused)) frsky2way_build_bind_packet()
{
	//11 03 01 d7 2d 00 00 1e 3c 5b 78 00 00 00 00 00 00 01
	//11 03 01 19 3e 00 02 8e 2f bb 5c 00 00 00 00 00 00 01
	packet[0] = 0x11;                
	packet[1] = 0x03;                
	packet[2] = 0x01;                
	packet[3] = rx_tx_addr[3];
	packet[4] = rx_tx_addr[2];
	uint16_t idx = ((state -FRSKY_BIND) % 10) * 5;
	packet[5] = idx;
	packet[6] = hopping_frequency[idx++];
	packet[7] = hopping_frequency[idx++];
	packet[8] = hopping_frequency[idx++];
	packet[9] = hopping_frequency[idx++];
	packet[10] = hopping_frequency[idx++];
	packet[11] = 0x00;
	packet[12] = 0x00;
	packet[13] = 0x00;
	packet[14] = 0x00;
	packet[15] = 0x00;
	packet[16] = 0x00;
	packet[17] = 0x01;
}

static void __attribute__((unused)) frsky2way_data_frame()
{//pachet[4] is telemetry user frame counter(hub)
	//11 d7 2d 22 00 01 c9 c9 ca ca 88 88 ca ca c9 ca 88 88
	//11 57 12 00 00 01 f2 f2 f2 f2 06 06 ca ca ca ca 18 18
	packet[0] = 0x11;             //Length
	packet[1] = rx_tx_addr[3];
	packet[2] = rx_tx_addr[2];
	packet[3] = counter;//	
	#if defined TELEMETRY
		packet[4] = telemetry_counter;
	#else
		packet[4] = 0x00;
	#endif

	packet[5] = 0x01;
	//
	packet[10] = 0;
	packet[11] = 0;
	packet[16] = 0;
	packet[17] = 0;
	for(uint8_t i = 0; i < 8; i++)
	{
		uint16_t value;
			value = convert_channel_frsky(i);
		if(i < 4)
		{
			packet[6+i] = value & 0xff;
			packet[10+(i>>1)] |= ((value >> 8) & 0x0f) << (4 *(i & 0x01));
		} 
		else
		{
			packet[8+i] = value & 0xff;
			packet[16+((i-4)>>1)] |= ((value >> 8) & 0x0f) << (4 * ((i-4) & 0x01));
		}
	}
} 

uint16_t initFrSky_2way()
{
	Frsky_init_hop();
	packet_count=0;
	#if defined TELEMETRY
		init_frskyd_link_telemetry();
	#endif
	if(IS_BIND_IN_PROGRESS)
	{
		frsky2way_init(1);
		state = FRSKY_BIND;
	}
	else
	{
		state = FRSKY_BIND_DONE;
	}
	return 10000;
}	
		
uint16_t ReadFrSky_2way()
{ 
	if (state < FRSKY_BIND_DONE)
	{
		frsky2way_build_bind_packet();
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);		
		CC2500_Strobe(CC2500_SFRX);//0x3A
		CC2500_WriteData(packet, packet[0]+1);
		if(IS_BIND_DONE)
			state = FRSKY_BIND_DONE;
		else
			state++;
		return 9000;
	}
	if (state == FRSKY_BIND_DONE)
	{
		state = FRSKY_DATA2;
		frsky2way_init(0);
		counter = 0;
		BIND_DONE;
	}
	else
		if (state == FRSKY_DATA5)
		{
			CC2500_Strobe(CC2500_SRX);//0x34 RX enable
			state = FRSKY_DATA1;	
			return 9200;
		}
	counter = (counter + 1) % 188;	
	if (state == FRSKY_DATA4)
	{	//telemetry receive
		CC2500_SetTxRxMode(RX_EN);
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[counter % 47]);
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
		state++;
		return 1300;
	}
	else
	{
		if (state == FRSKY_DATA1)
		{
			len = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if (len && len<=(0x11+3))// 20bytes
			{		
				CC2500_ReadData(pkt, len);				//received telemetry packets
				#if defined(TELEMETRY)
					if(pkt[len-1] & 0x80)
					{//with valid crc
						packet_count=0;
						frsky_check_telemetry(pkt,len);	//check if valid telemetry packets and buffer them.
					}
				#endif
			}
			else
			{
				packet_count++;
				// restart sequence on missed packet - might need count or timeout instead of one missed
				if(packet_count>100)
				{//~1sec
					packet_count=0;
					#if defined TELEMETRY
						telemetry_link=0;//no link frames
						pkt[6]=0;//no user frames.
					#endif
				}
			}
			CC2500_SetTxRxMode(TX_EN);
			CC2500_SetPower();	// Set tx_power
		}
		CC2500_Strobe(CC2500_SIDLE);
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[counter % 47]);
		if ( prev_option != option )
		{
			CC2500_WriteReg(CC2500_0C_FSCTRL0,option);	// Frequency offset hack 
			prev_option = option ;
		}
		CC2500_WriteReg(CC2500_23_FSCAL3, 0x89);
		CC2500_Strobe(CC2500_SFRX);        
		frsky2way_data_frame();
		CC2500_WriteData(packet, packet[0]+1);
		state++;
	}				
	return state == FRSKY_DATA4 ? 7500 : 9000;		
}
#endif


# 1 "src/Q303_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(Q303_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define Q303_BIND_COUNT			1500
#define Q303_INITIAL_WAIT		500
#define Q303_RF_BIND_CHANNEL	0x02

#define Q303_BTN_TAKEOFF	1
#define Q303_BTN_DESCEND	2
#define Q303_BTN_SNAPSHOT	4
#define Q303_BTN_VIDEO		8
#define Q303_BTN_RTH		16
#define Q303_BTN_VTX		32

static uint8_t __attribute__((unused)) cx10wd_getButtons()
{
	#define CX10WD_FLAG_LAND	0x20
	#define CX10D_FLAG_LAND		0x80
	#define CX10WD_FLAG_TAKEOFF	0x40

	static uint8_t BTN_state;
	static uint8_t command;

	// startup
	if(packet_count < 50)
	{
		BTN_state = 0;
		command = 0;
		packet_count++;
	}
	// auto land
	else if((Channel_data[CH5]<CHANNEL_MIN_COMMAND) && !(BTN_state & Q303_BTN_DESCEND))
	{
		BTN_state |= Q303_BTN_DESCEND;
		BTN_state &= ~Q303_BTN_TAKEOFF;
		switch(sub_protocol)
		{
			case CX10WD:
				command ^= CX10WD_FLAG_LAND;
				break;
			case CX10D:
				command ^= CX10D_FLAG_LAND;
				break;
		}
	}
	// auto take off
	else if(CH5_SW && !(BTN_state & Q303_BTN_TAKEOFF))
	{
		BTN_state |= Q303_BTN_TAKEOFF;
		BTN_state &= ~Q303_BTN_DESCEND;
		command ^= CX10WD_FLAG_TAKEOFF;
	}

	return command;
}

static uint8_t __attribute__((unused))  cx35_lastButton()
{
	#define CX35_CMD_RATE		0x09
	#define CX35_CMD_TAKEOFF	0x0e
	#define CX35_CMD_DESCEND	0x0f
	#define CX35_CMD_SNAPSHOT	0x0b
	#define CX35_CMD_VIDEO		0x0c
	#define CX35_CMD_RTH		0x11
	#define CX35_CMD_VTX		0x10
	
	static uint8_t BTN_state;
	static uint8_t command;
	// simulate 2 keypress on rate button just after bind
	if(packet_count < 50)
	{
		BTN_state = 0;
		packet_count++;
		command = 0x00; // startup
	}
	else if(packet_count < 150)
	{
		packet_count++;
		command = CX35_CMD_RATE; // 1st keypress
	}
	else if(packet_count < 250)
	{
		packet_count++;
		command |= 0x20; // 2nd keypress
	}
	// descend
	else if(!(GET_FLAG(CH5_SW, 1)) && !(BTN_state & Q303_BTN_DESCEND))
	{
		BTN_state |= Q303_BTN_DESCEND;
		BTN_state &= ~Q303_BTN_TAKEOFF;
		command = CX35_CMD_DESCEND;
	}
	// take off
	else if(GET_FLAG(CH5_SW,1) && !(BTN_state & Q303_BTN_TAKEOFF))
	{
		BTN_state |= Q303_BTN_TAKEOFF;
		BTN_state &= ~Q303_BTN_DESCEND;
		command = CX35_CMD_TAKEOFF;
	}
	// RTH
	else if(GET_FLAG(CH10_SW,1) && !(BTN_state & Q303_BTN_RTH))
	{
		BTN_state |= Q303_BTN_RTH;
		if(command == CX35_CMD_RTH)
			command |= 0x20;
		else
			command = CX35_CMD_RTH;
	}
	else if(!(GET_FLAG(CH10_SW,1)) && (BTN_state & Q303_BTN_RTH))
	{
		BTN_state &= ~Q303_BTN_RTH;
		if(command == CX35_CMD_RTH)
			command |= 0x20;
		else
			command = CX35_CMD_RTH;
	}
	// video
	else if(GET_FLAG(CH8_SW,1) && !(BTN_state & Q303_BTN_VIDEO))
	{
		BTN_state |= Q303_BTN_VIDEO;
		if(command == CX35_CMD_VIDEO)
			command |= 0x20;
		else
			command = CX35_CMD_VIDEO;
	}
	else if(!(GET_FLAG(CH8_SW,1)) && (BTN_state & Q303_BTN_VIDEO))
	{
		BTN_state &= ~Q303_BTN_VIDEO;
		if(command == CX35_CMD_VIDEO)
			command |= 0x20;
		else
			command = CX35_CMD_VIDEO;
	}
	// snapshot
	else if(GET_FLAG(CH7_SW,1) && !(BTN_state & Q303_BTN_SNAPSHOT))
	{
		BTN_state |= Q303_BTN_SNAPSHOT;
		if(command == CX35_CMD_SNAPSHOT)
			command |= 0x20;
		else
			command = CX35_CMD_SNAPSHOT;
	}
	// vtx channel
	else if(GET_FLAG(CH6_SW,1) && !(BTN_state & Q303_BTN_VTX))
	{
		BTN_state |= Q303_BTN_VTX;
		if(command == CX35_CMD_VTX)
			command |= 0x20;
		else
			command = CX35_CMD_VTX;
	}

	if(!(GET_FLAG(CH7_SW,1)))
		BTN_state &= ~Q303_BTN_SNAPSHOT;
	if(!(GET_FLAG(CH6_SW,1)))
		BTN_state &= ~Q303_BTN_VTX;

	return command;
}

static void __attribute__((unused)) Q303_send_packet(uint8_t bind)
{
	uint16_t aileron, elevator, throttle, rudder, slider;
	if(bind)
	{
		packet[0] = 0xaa;
		memcpy(&packet[1], rx_tx_addr + 1, 4);
		memset(&packet[5], 0, packet_length-5);
	}
	else
	{
		packet[0] = 0x55;
		// sticks
		switch(sub_protocol)
		{
			case Q303:
			case CX35:
				aileron  = convert_channel_16b_limit(AILERON,  0, 1000);
				elevator = convert_channel_16b_limit(ELEVATOR, 1000, 0);
				throttle = convert_channel_16b_limit(THROTTLE, 0, 1000);
				rudder   = convert_channel_16b_limit(RUDDER,   1000, 0);
				if(sub_protocol == CX35)
					aileron = 1000 - aileron;
				packet[1] = aileron >> 2;			// 8 bits
				packet[2] = (aileron & 0x03) << 6	// 2 bits
							| (elevator >> 4);		// 6 bits
				packet[3] = (elevator & 0x0f) << 4	// 4 bits
							| (throttle >> 6);		// 4 bits
				packet[4] = (throttle & 0x3f) << 2	// 6 bits 
							| (rudder >> 8);		// 2 bits
				packet[5] = rudder & 0xff;			// 8 bits
				break;
			case CX10D:
			case CX10WD:
				aileron  = convert_channel_16b_limit(AILERON,  2000, 1000);
				elevator = convert_channel_16b_limit(ELEVATOR, 2000, 1000);
				throttle = convert_channel_16b_limit(THROTTLE, 1000, 2000);
				rudder   = convert_channel_16b_limit(RUDDER,   1000, 2000);
				packet[1] = aileron & 0xff;
				packet[2] = aileron >> 8;
				packet[3] = elevator & 0xff;
				packet[4] = elevator >> 8;
				packet[5] = throttle & 0xff;
				packet[6] = throttle >> 8;
				packet[7] = rudder & 0xff;
				packet[8] = rudder >> 8;
				break;
		}

		// buttons
		switch(sub_protocol)
		{
			case Q303:
				packet[6] = 0x10;					// trim(s) ?
				packet[7] = 0x10;					// trim(s) ?
				packet[8] = 0x03					// high rate (0-3)
					| GET_FLAG(CH5_SW,   0x40)
					| GET_FLAG(CH10_SW,	 0x80);
				packet[9] = 0x40					// always set
					| GET_FLAG(CH9_SW,0x08)
					| GET_FLAG(CH6_SW,	0x80)
					| GET_FLAG(CH7_SW,0x10)
					| GET_FLAG(CH8_SW,   0x01);
				if(Channel_data[CH11] < CHANNEL_MIN_COMMAND)
					packet[9] |= 0x04;				// gimbal down
				else if(CH11_SW)
						packet[9] |= 0x20;			// gimbal up
				break;

			case CX35:
				slider = convert_channel_16b_limit(CH11, 731, 342);
				packet[6] = slider >> 2;
				packet[7] = ((slider & 3) << 6)
					| 0x3e;							// ?? 6 bit left (always 111110 ?)
				packet[8] = 0x80;					// always set
				packet[9] = cx35_lastButton();
				break;

			case CX10D:
				packet[8] |= GET_FLAG(CH6_SW, 0x10);
				packet[9] = 0x02; // rate (0-2)
				packet[10]= cx10wd_getButtons();	// auto land / take off management
				break;

			case CX10WD:
				packet[8] |= GET_FLAG(CH6_SW, 0x10);
				packet[9]  = 0x02  // rate (0-2)
						| cx10wd_getButtons();		// auto land / take off management
				packet[10] = 0x00;
				break;
		}
	}

	// Power on, TX mode, CRC enabled
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));

	NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? Q303_RF_BIND_CHANNEL : hopping_frequency[hopping_frequency_no++]);
	hopping_frequency_no %= rf_ch_num;

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	XN297_WritePayload(packet, packet_length);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) Q303_init()
{
	const uint8_t bind_address[] = {0xcc,0xcc,0xcc,0xcc,0xcc};

	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	switch(sub_protocol)
	{
		case CX35:
		case CX10D:
		case CX10WD:
			XN297_SetScrambledMode(XN297_SCRAMBLED);
			NRF24L01_SetBitrate(NRF24L01_BR_1M);
			break;
		case Q303:
			XN297_SetScrambledMode(XN297_UNSCRAMBLED);
			NRF24L01_SetBitrate(NRF24L01_BR_250K);
			break;
	}
	XN297_SetTXAddr(bind_address, 5);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);		// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);	// no retransmits
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);							// Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);			// Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);		// Set feature bits on
	NRF24L01_Activate(0x73);
}

static void __attribute__((unused)) Q303_initialize_txid()
{
	uint8_t i,offset;

	rx_tx_addr[0] = 0x55;

	switch(sub_protocol)
	{
		case Q303:
		case CX10WD:
			offset = rx_tx_addr[1] & 3;
			for(i=0; i<4; i++)
				hopping_frequency[i] = 0x46 + i*2 + offset;
			break;
		case CX35:
		case CX10D:
			// not thoroughly figured out rx_tx_addr/channels mapping yet
			// for now 5 msb of rx_tx_addr[1] must be cleared
			rx_tx_addr[1] &= 7;
			offset = 6+(rx_tx_addr[1]*3);
			hopping_frequency[0] = 0x14; // works only if rx_tx_addr[1] < 8
			for(i=1; i<16; i++)
			{
				hopping_frequency[i] = hopping_frequency[i-1] + offset;
				if(hopping_frequency[i] > 0x41)
					hopping_frequency[i] -= 0x33;
				if(hopping_frequency[i] < 0x14)
					hopping_frequency[i] += offset;
			}
			// CX35 tx uses only 4 of those channels (#0,3,6,9)
			if(sub_protocol == CX35)
				for(i=0; i<4; i++)
					hopping_frequency[i] = hopping_frequency[i*3];
			break;
	}
}

uint16_t Q303_callback()
{
	if(IS_BIND_DONE)
		Q303_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			XN297_SetTXAddr(rx_tx_addr, 5);
			packet_count = 0;
			BIND_DONE;
		}
		else
		{
			Q303_send_packet(1);
			bind_counter--;
		}
	}
	return packet_period;
}

uint16_t initQ303()
{
	Q303_initialize_txid();
	Q303_init();
	bind_counter = Q303_BIND_COUNT;
	switch(sub_protocol)
	{
		case Q303:
			packet_period = 1500;
			packet_length = 10;
			rf_ch_num = 4;
			break;
		case CX35:
			packet_period = 3000;
			packet_length = 10;
			rf_ch_num = 4;
			break;
		case CX10D:
			packet_period = 3000;
			packet_length = 11;
			rf_ch_num = 16;
			break;
		case CX10WD:
			packet_period = 3000;
			packet_length = 11;
			rf_ch_num = 4;
		break;
	}
	hopping_frequency_no = 0;
	BIND_IN_PROGRESS;	// autobind protocol
	return Q303_INITIAL_WAIT;
}

#endif


# 1 "src/V2X2_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with WLToys V2x2, JXD JD38x, JD39x, JJRC H6C, Yizhan Tarantula X6 ...
// Last sync with hexfet new_protocols/v202_nrf24l01.c dated 2015-03-15

#if defined(V2X2_NRF24L01_INO)


#include "iface_nrf24l01.h"


#define V2X2_BIND_COUNT 1000
// Timeout for callback in uSec, 4ms=4000us for V202
#define V2X2_PACKET_PERIOD 4000
//
// Time to wait for packet to be sent (no ACK, so very short)
#define V2X2_PACKET_CHKTIME  100
#define V2X2_PAYLOADSIZE 16

// 
enum {
	V2X2_FLAG_CAMERA = 0x01, // also automatic Missile Launcher and Hoist in one direction
	V2X2_FLAG_VIDEO  = 0x02, // also Sprayer, Bubbler, Missile Launcher(1), and Hoist in the other dir.
	V2X2_FLAG_FLIP   = 0x04,
	V2X2_FLAG_UNK9   = 0x08,
	V2X2_FLAG_LIGHT  = 0x10,
	V2X2_FLAG_UNK10  = 0x20,
	V2X2_FLAG_BIND   = 0xC0,
	// flags going to byte 10
	V2X2_FLAG_HEADLESS  = 0x02,
	V2X2_FLAG_MAG_CAL_X = 0x08,
	V2X2_FLAG_MAG_CAL_Y = 0x20,
    V2X2_FLAG_EMERGENCY = 0x80,	// JXD-506
    // flags going to byte 11 (JXD-506)
    V2X2_FLAG_START_STOP = 0x40,
    V2X2_FLAG_CAMERA_UP  = 0x01,   
    V2X2_FLAG_CAMERA_DN  = 0x02,
};

//

enum {
	V202_INIT2 = 0,
	V202_INIT2_NO_BIND,//1
	V202_BIND1,//2
	V202_BIND2,//3
	V202_DATA//4
};

// This is frequency hopping table for V202 protocol
// The table is the first 4 rows of 32 frequency hopping
// patterns, all other rows are derived from the first 4.
// For some reason the protocol avoids channels, dividing
// by 16 and replaces them by subtracting 3 from the channel
// number in this case.
// The pattern is defined by 5 least significant bits of
// sum of 3 bytes comprising TX id
const uint8_t PROGMEM freq_hopping[][16] = {
	{ 0x27, 0x1B, 0x39, 0x28, 0x24, 0x22, 0x2E, 0x36,
		0x19, 0x21, 0x29, 0x14, 0x1E, 0x12, 0x2D, 0x18 }, //  00
	{ 0x2E, 0x33, 0x25, 0x38, 0x19, 0x12, 0x18, 0x16,
		0x2A, 0x1C, 0x1F, 0x37, 0x2F, 0x23, 0x34, 0x10 }, //  01
	{ 0x11, 0x1A, 0x35, 0x24, 0x28, 0x18, 0x25, 0x2A,
		0x32, 0x2C, 0x14, 0x27, 0x36, 0x34, 0x1C, 0x17 }, //  02
	{ 0x22, 0x27, 0x17, 0x39, 0x34, 0x28, 0x2B, 0x1D,
		0x18, 0x2A, 0x21, 0x38, 0x10, 0x26, 0x20, 0x1F }  //  03
};

static void __attribute__((unused)) v202_init()
{
	NRF24L01_Initialize();

	// 2-bytes CRC, radio off
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xFF); // 4ms retransmit t/o, 15 tries
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);      // Channel 8
	NRF24L01_SetBitrate(NRF24L01_BR_1M);                          // 1Mbps
	NRF24L01_SetPower();
	
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	//    NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00); // no write bits in this field
	//    NRF24L01_WriteReg(NRF24L01_00_CD, 0x00);         // same
	NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
	NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
	NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
	NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, V2X2_PAYLOADSIZE);   // bytes of data payload for pipe 1
	NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, V2X2_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t *)"\x66\x88\x68\x68\x68", 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, (uint8_t *)"\x88\x66\x86\x86\x86", 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)"\x66\x88\x68\x68\x68", 5);
}

static void __attribute__((unused)) V202_init2()
{
	NRF24L01_FlushTx();
	packet_sent = 0;
	hopping_frequency_no = 0;

	// Turn radio power on
    NRF24L01_SetTxRxMode(TX_EN);
	//Done by TX_EN??? => NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
}

static void __attribute__((unused)) V2X2_set_tx_id(void)
{
	uint8_t sum;
	sum = rx_tx_addr[1] + rx_tx_addr[2] + rx_tx_addr[3];
	// Higher 3 bits define increment to corresponding row
	uint8_t increment = (sum & 0x1e) >> 2;
	// Base row is defined by lowest 2 bits
	sum &=0x03;
	for (uint8_t i = 0; i < 16; ++i) {
		uint8_t val = pgm_read_byte_near(&freq_hopping[sum][i]) + increment;
		// Strange avoidance of channels divisible by 16
		hopping_frequency[i] = (val & 0x0f) ? val : val - 3;
	}
}

static void __attribute__((unused)) V2X2_add_pkt_checksum()
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < 15;  ++i)
		sum += packet[i];
	packet[15] = sum;
}

static void __attribute__((unused)) V2X2_send_packet(uint8_t bind)
{
	uint8_t flags2=0;
	if (bind)
	{
		flags     = V2X2_FLAG_BIND;
		packet[0] = 0;
		packet[1] = 0;
		packet[2] = 0;
		packet[3] = 0;
		packet[4] = 0;
		packet[5] = 0;
		packet[6] = 0;
	}
	else
	{
		packet[0] = convert_channel_8b(THROTTLE);
		packet[1] = convert_channel_s8b(RUDDER);
		packet[2] = convert_channel_s8b(ELEVATOR);
		packet[3] = convert_channel_s8b(AILERON);
		// Trims, middle is 0x40
		packet[4] = 0x40; // yaw
		packet[5] = 0x40; // pitch
		packet[6] = 0x40; // roll

		//Flags
		flags=0;
		// Channel 5
		if (CH5_SW)	flags = V2X2_FLAG_FLIP;
		// Channel 6
		if (CH6_SW)	flags |= V2X2_FLAG_LIGHT;
		// Channel 7
		if (CH7_SW)	flags |= V2X2_FLAG_CAMERA;
		// Channel 8
		if (CH8_SW)	flags |= V2X2_FLAG_VIDEO;

		//Flags2
		// Channel 9
		if (CH9_SW)
			flags2 = V2X2_FLAG_HEADLESS;
		if(sub_protocol==JXD506)
		{
			// Channel 11
			if (CH11_SW)
				flags2 |= V2X2_FLAG_EMERGENCY;
		}
		else
		{
			// Channel 10
			if (CH10_SW)
				flags2 |= V2X2_FLAG_MAG_CAL_X;
			// Channel 11
			if (CH11_SW)
				flags2 |= V2X2_FLAG_MAG_CAL_Y;
		}
	}
	// TX id
	packet[7] = rx_tx_addr[1];
	packet[8] = rx_tx_addr[2];
	packet[9] = rx_tx_addr[3];
	// flags
	packet[10] = flags2;
	packet[11] = 0x00;
	packet[12] = 0x00;
	packet[13] = 0x00;
	if(sub_protocol==JXD506)
	{
		// Channel 10
		if (CH10_SW)
			packet[11] = V2X2_FLAG_START_STOP;
		// Channel 12
		if(CH12_SW)
			packet[11] |= V2X2_FLAG_CAMERA_UP;
		else if(Channel_data[CH12] < CHANNEL_MIN_COMMAND)
			packet[11] |= V2X2_FLAG_CAMERA_DN;
		packet[12] = 0x40;
		packet[13] = 0x40;
	}
	packet[14] = flags;
	V2X2_add_pkt_checksum();

	packet_sent = 0;
	uint8_t rf_ch = hopping_frequency[hopping_frequency_no >> 1];
	hopping_frequency_no = (hopping_frequency_no + 1) & 0x1F;
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
	NRF24L01_FlushTx();
	NRF24L01_WritePayload(packet, V2X2_PAYLOADSIZE);
	packet_sent = 1;

	if (! hopping_frequency_no)
		NRF24L01_SetPower();
}

uint16_t ReadV2x2()
{
	switch (phase) {
		case V202_INIT2:
			V202_init2();
			phase = V202_BIND2;
			return 150;
			break;
		case V202_INIT2_NO_BIND:
			V202_init2();
			phase = V202_DATA;
			return 150;
			break;
		case V202_BIND2:
			if (packet_sent && NRF24L01_packet_ack() != PKT_ACKED)
				return V2X2_PACKET_CHKTIME;
			V2X2_send_packet(1);
			if (--bind_counter == 0)
			{
				phase = V202_DATA;
				BIND_DONE;
			}
			break;
		case V202_DATA:
			if (packet_sent && NRF24L01_packet_ack() != PKT_ACKED)
				return V2X2_PACKET_CHKTIME;
			V2X2_send_packet(0);
			break;
	}
	// Packet every 4ms
	return V2X2_PACKET_PERIOD;
}

uint16_t initV2x2()
{	
	v202_init();
	//
	if (IS_BIND_IN_PROGRESS)
	{
		bind_counter = V2X2_BIND_COUNT;
		phase = V202_INIT2;
	}
	else
		phase = V202_INIT2_NO_BIND;
	V2X2_set_tx_id();
	return 50000;
}

#endif


# 1 "src/AFHDS2A_a7105.ino" // Helps debugging !
 /*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/flysky_a7105.c dated 2015-09-28

#ifdef AFHDS2A_A7105_INO

#define AFHDS2A_TXPACKET_SIZE	38
#define AFHDS2A_RXPACKET_SIZE	37
#define AFHDS2A_NUMFREQ			16

enum{
	AFHDS2A_PACKET_STICKS,
	AFHDS2A_PACKET_SETTINGS,
	AFHDS2A_PACKET_FAILSAFE,
};

enum{
	AFHDS2A_BIND1,
	AFHDS2A_BIND2,
	AFHDS2A_BIND3,
	AFHDS2A_BIND4,
	AFHDS2A_DATA_INIT,
	AFHDS2A_DATA,
};

static void AFHDS2A_calc_channels()
{
	uint8_t idx = 0;
	uint32_t rnd = MProtocol_id;
	while (idx < AFHDS2A_NUMFREQ)
	{
		uint8_t i;
		uint8_t count_1_42 = 0, count_43_85 = 0, count_86_128 = 0, count_129_168 = 0;
		rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization

		uint8_t next_ch = ((rnd >> (idx%32)) % 0xa8) + 1;
		// Keep the distance 2 between the channels - either odd or even
		if (((next_ch ^ MProtocol_id) & 0x01 )== 0)
			continue;
		// Check that it's not duplicate and spread uniformly
		for (i = 0; i < idx; i++)
		{
			if(hopping_frequency[i] == next_ch)
				break;
			if(hopping_frequency[i] <= 42)
				count_1_42++;
			else if (hopping_frequency[i] <= 85)
				count_43_85++;
			else if (hopping_frequency[i] <= 128)
				count_86_128++;
			else
				count_129_168++;
		}
		if (i != idx)
			continue;
		if ((next_ch <= 42 && count_1_42 < 5)
			||(next_ch >= 43 && next_ch <= 85 && count_43_85 < 5)
			||(next_ch >= 86 && next_ch <=128 && count_86_128 < 5)
			||(next_ch >= 129 && count_129_168 < 5))
			hopping_frequency[idx++] = next_ch;
	}
}

#if defined(AFHDS2A_FW_TELEMETRY) || defined(AFHDS2A_HUB_TELEMETRY)
// telemetry sensors ID
enum{
	AFHDS2A_SENSOR_RX_VOLTAGE   = 0x00,
	AFHDS2A_SENSOR_RX_ERR_RATE  = 0xfe,
	AFHDS2A_SENSOR_RX_RSSI      = 0xfc,
	AFHDS2A_SENSOR_RX_NOISE     = 0xfb,
	AFHDS2A_SENSOR_RX_SNR       = 0xfa,
	AFHDS2A_SENSOR_A3_VOLTAGE   = 0x03,
};

static void AFHDS2A_update_telemetry()
{
	// Read TX RSSI
	int16_t temp=256-(A7105_ReadReg(A7105_1D_RSSI_THOLD)*8)/5;		// value from A7105 is between 8 for maximum signal strength to 160 or less
	if(temp<0) temp=0;
	else if(temp>255) temp=255;
	TX_RSSI=temp;
	// AA | TXID | rx_id | sensor id | sensor # | value 16 bit big endian | sensor id ......
	// max 7 sensors per packet
	#ifdef AFHDS2A_FW_TELEMETRY
		if (option & 0x80)
		{
			// forward telemetry to TX, skip rx and tx id to save space
			pkt[0]= TX_RSSI;
			for(int i=9;i < AFHDS2A_RXPACKET_SIZE; i++)
				pkt[i-8]=packet[i];

			telemetry_link=2;
			return;
		}
	#endif
	#ifdef AFHDS2A_HUB_TELEMETRY
		for(uint8_t sensor=0; sensor<7; sensor++)
		{
			// Send FrSkyD telemetry to TX
			uint8_t index = 9+(4*sensor);
			switch(packet[index])
			{
				case AFHDS2A_SENSOR_RX_VOLTAGE:
					//v_lipo1 = packet[index+3]<<8 | packet[index+2];
					v_lipo1 = packet[index+2];
					telemetry_link=1;
					break;
				case AFHDS2A_SENSOR_A3_VOLTAGE:
					v_lipo2 = (packet[index+3]<<5) | (packet[index+2]>>3);	// allows to read voltage up to 4S
					telemetry_link=1;
					break;
				case AFHDS2A_SENSOR_RX_ERR_RATE:
					RX_LQI=packet[index+2];
					break;
				case AFHDS2A_SENSOR_RX_RSSI:
					RX_RSSI = -packet[index+2];
					break;
				case 0xff:
					return;
				/*default:
					// unknown sensor ID
					break;*/
			}
		}
	#endif
}
#endif

static void AFHDS2A_build_bind_packet()
{
	uint8_t ch;
	memcpy( &packet[1], rx_tx_addr, 4);
	memset( &packet[5], 0xff, 4);
	packet[10]= 0x00;
	for(ch=0; ch<AFHDS2A_NUMFREQ; ch++)
		packet[11+ch] = hopping_frequency[ch];
	memset( &packet[27], 0xff, 10);
	packet[37] = 0x00;
	switch(phase)
	{
		case AFHDS2A_BIND1:
			packet[0] = 0xbb;
			packet[9] = 0x01;
			break;
		case AFHDS2A_BIND2:
		case AFHDS2A_BIND3:
		case AFHDS2A_BIND4:
			packet[0] = 0xbc;
			if(phase == AFHDS2A_BIND4)
			{
				memcpy( &packet[5], &rx_id, 4);
				memset( &packet[11], 0xff, 16);
			}
			packet[9] = phase-1;
			if(packet[9] > 0x02)
				packet[9] = 0x02;
			packet[27]= 0x01;
			packet[28]= 0x80;
			break;
	}
}

static void AFHDS2A_build_packet(uint8_t type)
{
	uint16_t val;
	memcpy( &packet[1], rx_tx_addr, 4);
	memcpy( &packet[5], rx_id, 4);
	switch(type)
	{
		case AFHDS2A_PACKET_STICKS:		
			packet[0] = 0x58;
			for(uint8_t ch=0; ch<14; ch++)
			{
				uint16_t channelMicros = convert_channel_ppm(CH_AETR[ch]);
				packet[9 +  ch*2] = channelMicros&0xFF;
				packet[10 + ch*2] = (channelMicros>>8)&0xFF;
			}
			#ifdef AFHDS2A_LQI_CH
				// override channel with LQI
				val = 2000 - 10*RX_LQI;
				packet[9+((AFHDS2A_LQI_CH-1)*2)] = val & 0xff;
				packet[10+((AFHDS2A_LQI_CH-1)*2)] = (val >> 8) & 0xff;
			#endif
			break;
		case AFHDS2A_PACKET_FAILSAFE:
			packet[0] = 0x56;
			for(uint8_t ch=0; ch<14; ch++)
			{
				#ifdef FAILSAFE_ENABLE
					uint16_t failsafeMicros = Failsafe_data[CH_AETR[ch]];
					failsafeMicros = (((failsafeMicros<<2)+failsafeMicros)>>3)+860;
					if( failsafeMicros!=FAILSAFE_CHANNEL_HOLD+860)
					{ // Failsafe values
						packet[9 + ch*2] =  failsafeMicros & 0xff;
						packet[10+ ch*2] = ( failsafeMicros >> 8) & 0xff;
					}
					else
				#endif
					{ // no values
						packet[9 + ch*2] = 0xff;
						packet[10+ ch*2] = 0xff;
					}
			}
			break;
		case AFHDS2A_PACKET_SETTINGS:
			packet[0] = 0xaa;
			packet[9] = 0xfd;
			packet[10]= 0xff;
			val=5*(option & 0x7f)+50;	// option value should be between 0 and 70 which gives a value between 50 and 400Hz
			if(val<50 || val>400) val=50;	// default is 50Hz
			packet[11]= val;
			packet[12]= val >> 8;
			if(sub_protocol == PPM_IBUS || sub_protocol == PPM_SBUS)
				packet[13] = 0x01;	// PPM output enabled
			else
				packet[13] = 0x00;
			packet[14]= 0x00;
			for(uint8_t i=15; i<37; i++)
				packet[i] = 0xff;
			packet[18] = 0x05;		// ?
			packet[19] = 0xdc;		// ?
			packet[20] = 0x05;		// ?
			if(sub_protocol == PWM_SBUS || sub_protocol == PPM_SBUS)
				packet[21] = 0xdd;	// SBUS output enabled
			else
				packet[21] = 0xde;	// IBUS
			break;
	}
	packet[37] = 0x00;
}

#define AFHDS2A_WAIT_WRITE 0x80
uint16_t ReadAFHDS2A()
{
	static uint8_t packet_type;
	static uint16_t packet_counter;
	uint8_t data_rx;
	uint16_t start;
	#ifndef FORCE_AFHDS2A_TUNING
		A7105_AdjustLOBaseFreq(1);
	#endif
	switch(phase)
	{
		case AFHDS2A_BIND1:
		case AFHDS2A_BIND2:
		case AFHDS2A_BIND3:
			AFHDS2A_build_bind_packet();
			A7105_WriteData(AFHDS2A_TXPACKET_SIZE, packet_count%2 ? 0x0d : 0x8c);
			if(!(A7105_ReadReg(A7105_00_MODE) & (1<<5 | 1<<6)))
			{ // FECF+CRCF Ok
				A7105_ReadData(AFHDS2A_RXPACKET_SIZE);
				if(packet[0] == 0xbc && packet[9] == 0x01)
				{
					uint8_t temp=AFHDS2A_EEPROM_OFFSET+RX_num*4;
					for(uint8_t i=0; i<4; i++)
					{
						rx_id[i] = packet[5+i];
						eeprom_write_byte((EE_ADDR)(temp+i),rx_id[i]);
					}
					phase = AFHDS2A_BIND4;
					packet_count++;
					return 3850;
				}
			}
			packet_count++;
			phase |= AFHDS2A_WAIT_WRITE;
			return 1700;
		case AFHDS2A_BIND1|AFHDS2A_WAIT_WRITE:
		case AFHDS2A_BIND2|AFHDS2A_WAIT_WRITE:
		case AFHDS2A_BIND3|AFHDS2A_WAIT_WRITE:
			//Wait for TX completion
			start=micros();
			while ((uint16_t)micros()-start < 700)			// Wait max 700µs, using serial+telemetry exit in about 120µs
				if(!(A7105_ReadReg(A7105_00_MODE) & 0x01))
					break;
			A7105_SetPower();
			A7105_SetTxRxMode(TXRX_OFF);					// Turn LNA off since we are in near range and we want to prevent swamping
			A7105_Strobe(A7105_RX);
			phase &= ~AFHDS2A_WAIT_WRITE;
			phase++;
			if(phase > AFHDS2A_BIND3)
				phase = AFHDS2A_BIND1;
			return 2150;
		case AFHDS2A_BIND4:
			AFHDS2A_build_bind_packet();
			A7105_WriteData(AFHDS2A_TXPACKET_SIZE, packet_count%2 ? 0x0d : 0x8c);
			packet_count++;
			bind_phase++;
			if(bind_phase>=4)
			{ 
				hopping_frequency_no=1;
				phase = AFHDS2A_DATA_INIT;
				BIND_DONE;
			}
			return 3850;
		case AFHDS2A_DATA_INIT:
			packet_counter=0;
			packet_type = AFHDS2A_PACKET_STICKS;
			phase = AFHDS2A_DATA;
		case AFHDS2A_DATA:
			AFHDS2A_build_packet(packet_type);
			if((A7105_ReadReg(A7105_00_MODE) & 0x01))		// Check if something has been received...
				data_rx=0;
			else
				data_rx=1;									// Yes
			A7105_WriteData(AFHDS2A_TXPACKET_SIZE, hopping_frequency[hopping_frequency_no++]);
			if(hopping_frequency_no >= AFHDS2A_NUMFREQ)
				hopping_frequency_no = 0;
			if(!(packet_counter % 1313))
				packet_type = AFHDS2A_PACKET_SETTINGS;
			else
			{
				#ifdef FAILSAFE_ENABLE
					if(!(packet_counter % 1569) && IS_FAILSAFE_VALUES_on)
						packet_type = AFHDS2A_PACKET_FAILSAFE;
					else
				#endif
						packet_type = AFHDS2A_PACKET_STICKS;		// todo : check for settings changes
			}
			if(!(A7105_ReadReg(A7105_00_MODE) & (1<<5 | 1<<6)) && data_rx==1)
			{ // RX+FECF+CRCF Ok
				A7105_ReadData(AFHDS2A_RXPACKET_SIZE);
				if(packet[0] == 0xaa)
				{
					if(packet[9] == 0xfc)
						packet_type=AFHDS2A_PACKET_SETTINGS;	// RX is asking for settings
					else
					{
						#ifdef AFHDS2A_LQI_CH
							for(uint8_t sensor=0; sensor<7; sensor++)
							{//read LQI value for RX output
								uint8_t index = 9+(4*sensor);
								if(packet[index]==AFHDS2A_SENSOR_RX_ERR_RATE)
									RX_LQI=packet[index+2];
							}
						#endif
						#if defined(AFHDS2A_FW_TELEMETRY) || defined(AFHDS2A_HUB_TELEMETRY)
							AFHDS2A_update_telemetry();
						#endif
					}
				}
			}
			packet_counter++;
			phase |= AFHDS2A_WAIT_WRITE;
			return 1700;
		case AFHDS2A_DATA|AFHDS2A_WAIT_WRITE:
			//Wait for TX completion
			start=micros();
			while ((uint16_t)micros()-start < 700)			// Wait max 700µs, using serial+telemetry exit in about 120µs
				if(!(A7105_ReadReg(A7105_00_MODE) & 0x01))
					break;
			A7105_SetPower();
			A7105_SetTxRxMode(RX_EN);
			A7105_Strobe(A7105_RX);
			phase &= ~AFHDS2A_WAIT_WRITE;
			return 2150;
	}
	return 3850; // never reached, please the compiler
}

uint16_t initAFHDS2A()
{
	A7105_Init();

	AFHDS2A_calc_channels();
	packet_count = 0;
	bind_phase = 0;
	if(IS_BIND_IN_PROGRESS)
		phase = AFHDS2A_BIND1;
	else
	{
		phase = AFHDS2A_DATA_INIT;
		//Read RX ID from EEPROM based on RX_num, RX_num must be uniq for each RX
		uint8_t temp=AFHDS2A_EEPROM_OFFSET+RX_num*4;
		for(uint8_t i=0;i<4;i++)
			rx_id[i]=eeprom_read_byte((EE_ADDR)(temp+i));
	}
	hopping_frequency_no = 0;
#if defined(AFHDS2A_FW_TELEMETRY) || defined(AFHDS2A_HUB_TELEMETRY)
	init_frskyd_link_telemetry();
#endif
	return 50000;
}
#endif


# 1 "src/MJXQ_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with MJX WLH08, X600, X800, H26D, Eachine E010
// Last sync with hexfet new_protocols/mjxq_nrf24l01.c dated 2016-01-17

#if defined(MJXQ_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define MJXQ_BIND_COUNT		150
#define MJXQ_PACKET_PERIOD	4000  // Timeout for callback in uSec
#define MJXQ_INITIAL_WAIT	500
#define MJXQ_PACKET_SIZE	16
#define MJXQ_RF_NUM_CHANNELS	4
#define MJXQ_ADDRESS_LENGTH	5

// haven't figured out txid<-->rf channel mapping for MJX models
const uint8_t PROGMEM MJXQ_map_txid[][3] = {
				{0xF8, 0x4F, 0x1C},
				{0xC8, 0x6E, 0x02},
				{0x48, 0x6A, 0x40}	};
const uint8_t PROGMEM MJXQ_map_rfchan[][4] = {
				{0x0A, 0x46, 0x3A, 0x42},
				{0x0A, 0x3C, 0x36, 0x3F},
				{0x0A, 0x43, 0x36, 0x3F}	};

const uint8_t PROGMEM E010_map_txid[][2] = {
					{0x4F, 0x1C},
					{0x90, 0x1C},
					{0x24, 0x36},
					{0x7A, 0x40},
					{0x61, 0x31},
					{0x5D, 0x37},
					{0xFD, 0x4F},
					{0x86, 0x3C},
					{0x41, 0x22},
					{0xEE, 0xB3},
					{0x9A, 0xB2},
					{0xC0, 0x44},
					{0x2A, 0xFE},
					{0xD7, 0x6E},
					{0x3C, 0xCD}, // for this ID rx_tx_addr[2]=0x01
					{0xF5, 0x2B} // for this ID rx_tx_addr[2]=0x02
					};
const uint8_t PROGMEM E010_map_rfchan[][2] = {
					{0x3A, 0x35},
					{0x2E, 0x36},
					{0x32, 0x3E},
					{0x2E, 0x3C},
					{0x2F, 0x3B},
					{0x33, 0x3B},
					{0x33, 0x3B},
					{0x34, 0x3E},
					{0x34, 0x2F},
					{0x39, 0x3E},
					{0x2E, 0x38},
					{0x2E, 0x36},
					{0x2E, 0x38},
					{0x3A, 0x41},
					{0x32, 0x3E},
					{0x33, 0x3F}
					};

#define MJXQ_PAN_TILT_COUNT	16   // for H26D - match stock tx timing
#define MJXQ_PAN_DOWN		0x08
#define MJXQ_PAN_UP			0x04
#define MJXQ_TILT_DOWN		0x20
#define MJXQ_TILT_UP		0x10
static uint8_t __attribute__((unused)) MJXQ_pan_tilt_value()
{
// CH12_SW	PAN			// H26D
// CH13_SW	TILT
	uint8_t	pan = 0;
	packet_count++;
	if(packet_count & MJXQ_PAN_TILT_COUNT)
	{
		if(CH12_SW)
			pan=MJXQ_PAN_UP;
		if(Channel_data[CH12]<CHANNEL_MIN_COMMAND)
			pan=MJXQ_PAN_DOWN;
		if(CH13_SW)
			pan+=MJXQ_TILT_UP;
		if(Channel_data[CH13]<CHANNEL_MIN_COMMAND)
			pan+=MJXQ_TILT_DOWN;
	}
	return pan;
}

#define MJXQ_CHAN2TRIM(X) (((X) & 0x80 ? (X) : 0x7f - (X)) >> 1)
static void __attribute__((unused)) MJXQ_send_packet(uint8_t bind)
{
	packet[0] = convert_channel_8b(THROTTLE);
	packet[1] = convert_channel_s8b(RUDDER);
	packet[4] = 0x40;							// rudder does not work well with dyntrim
	packet[2] = 0x80 ^ convert_channel_s8b(ELEVATOR);
	packet[5] = (CH9_SW || CH14_SW) ? 0x40 : MJXQ_CHAN2TRIM(packet[2]);	// trim elevator
	packet[3] = convert_channel_s8b(AILERON);
	packet[6] = (CH9_SW || CH14_SW) ? 0x40 : MJXQ_CHAN2TRIM(packet[3]);	// trim aileron
	packet[7] = rx_tx_addr[0];
	packet[8] = rx_tx_addr[1];
	packet[9] = rx_tx_addr[2];

	packet[10] = 0x00;							// overwritten below for feature bits
	packet[11] = 0x00;							// overwritten below for X600
	packet[12] = 0x00;
	packet[13] = 0x00;

	packet[14] = 0xC0;							// bind value

// CH5_SW	FLIP
// CH6_SW	LED / ARM
// CH7_SW	PICTURE
// CH8_SW	VIDEO
// CH9_SW	HEADLESS
// CH10_SW	RTH
// CH11_SW	AUTOFLIP	// X800, X600
// CH12_SW	PAN
// CH13_SW	TILT
// CH14_SW	XTRM		// Dyntrim, don't use if high.
	switch(sub_protocol)
	{
		case H26WH:
		case H26D:
			packet[10]=MJXQ_pan_tilt_value();
			// fall through on purpose - no break
		case WLH08:
		case E010:
			packet[10] += GET_FLAG(CH10_SW, 0x02)	//RTH
						| GET_FLAG(CH9_SW, 0x01);	//HEADLESS
			if (!bind)
			{
				packet[14] = 0x04
						| GET_FLAG(CH5_SW, 0x01)	//FLIP
						| GET_FLAG(CH7_SW, 0x08)	//PICTURE
						| GET_FLAG(CH8_SW, 0x10)	//VIDEO
						| GET_FLAG(!CH6_SW, 0x20);	// LED or air/ground mode
				if(sub_protocol==H26WH)
				{
					packet[10] |=0x40;					//High rate
					packet[14] &= ~0x24;				// unset air/ground & arm flags
					packet[14] |= GET_FLAG(CH6_SW, 0x02);	// arm
				}
			}
			break;
		case X600:
			packet[10] = GET_FLAG(!CH6_SW, 0x02);	//LED
			packet[11] = GET_FLAG(CH10_SW, 0x01);	//RTH
			if (!bind)
			{
				packet[14] = 0x02						// always high rates by bit2 = 1
						| GET_FLAG(CH5_SW, 0x04)	//FLIP
						| GET_FLAG(CH11_SW, 0x10)	//AUTOFLIP
						| GET_FLAG(CH9_SW, 0x20);	//HEADLESS
			}
			break;
		case X800:
		default:
			packet[10] = 0x10
					| GET_FLAG(!CH6_SW, 0x02)	//LED
					| GET_FLAG(CH11_SW, 0x01);	//AUTOFLIP
			if (!bind)
			{
				packet[14] = 0x02						// always high rates by bit2 = 1
						| GET_FLAG(CH5_SW, 0x04)	//FLIP
						| GET_FLAG(CH7_SW, 0x08)	//PICTURE
						| GET_FLAG(CH8_SW, 0x10);	//VIDEO
			}
			break;
	}

	uint8_t sum = packet[0];
	for (uint8_t i=1; i < MJXQ_PACKET_SIZE-1; i++) sum += packet[i];
	packet[15] = sum;

	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++ / 2]);
	hopping_frequency_no %= 2 * MJXQ_RF_NUM_CHANNELS;	// channels repeated

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	// Power on, TX mode, 2byte CRC and send packet
	if (sub_protocol == H26D || sub_protocol == H26WH)
	{
		NRF24L01_SetTxRxMode(TX_EN);
		NRF24L01_WritePayload(packet, MJXQ_PACKET_SIZE);
	}
	else
	{
		XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
		XN297_WritePayload(packet, MJXQ_PACKET_SIZE);
	}
	NRF24L01_SetPower();
}

static void __attribute__((unused)) MJXQ_init()
{
	uint8_t addr[MJXQ_ADDRESS_LENGTH];
	memcpy(addr, "\x6d\x6a\x77\x77\x77", MJXQ_ADDRESS_LENGTH);
	if (sub_protocol == WLH08)
		memcpy(hopping_frequency, "\x12\x22\x32\x42", MJXQ_RF_NUM_CHANNELS);
	else
		if (sub_protocol == H26D || sub_protocol == H26D || sub_protocol == E010)
			memcpy(hopping_frequency, "\x2e\x36\x3e\x46", MJXQ_RF_NUM_CHANNELS);
		else
		{
			memcpy(hopping_frequency, "\x0a\x35\x42\x3d", MJXQ_RF_NUM_CHANNELS);
			memcpy(addr, "\x6d\x6a\x73\x73\x73", MJXQ_ADDRESS_LENGTH);
		}
	
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);

	if (sub_protocol == H26D || sub_protocol == H26WH)
	{
		NRF24L01_WriteReg(NRF24L01_03_SETUP_AW,		0x03);		// 5-byte RX/TX address
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, addr, MJXQ_ADDRESS_LENGTH);
	}
	else
		XN297_SetTXAddr(addr, MJXQ_ADDRESS_LENGTH);

	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS,		0x70);		// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA,		0x00);		// No Auto Acknowledgment on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR,	0x01);		// Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR,	0x00);		// no retransmits
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0,		MJXQ_PACKET_SIZE);
	if (sub_protocol == E010)
		NRF24L01_SetBitrate(NRF24L01_BR_250K);				// 250K
	else
		NRF24L01_SetBitrate(NRF24L01_BR_1M);				// 1Mbps
	NRF24L01_SetPower();
}

static void __attribute__((unused)) MJXQ_init2()
{
	switch(sub_protocol)
	{
		case H26D:
			memcpy(hopping_frequency, "\x32\x3e\x42\x4e", MJXQ_RF_NUM_CHANNELS);
			break;
		case H26WH:
			memcpy(hopping_frequency, "\x37\x32\x47\x42", MJXQ_RF_NUM_CHANNELS);
			break;
		case E010:
			for(uint8_t i=0;i<2;i++)
			{
				hopping_frequency[i]=pgm_read_byte_near( &E010_map_rfchan[rx_tx_addr[3]&0x0F][i] );
				hopping_frequency[i+2]=hopping_frequency[i]+0x10;
			}
			break;
		case WLH08:
			// do nothing
			break;
		default:
			for(uint8_t i=0;i<MJXQ_RF_NUM_CHANNELS;i++)
				hopping_frequency[i]=pgm_read_byte_near( &MJXQ_map_rfchan[rx_tx_addr[3]%3][i] );
			break;
	}
}

static void __attribute__((unused)) MJXQ_initialize_txid()
{
	switch(sub_protocol)
	{
		case H26WH:
			memcpy(rx_tx_addr, "\xa4\x03\x00", 3); 
			break;
		case E010:
			for(uint8_t i=0;i<2;i++)
				rx_tx_addr[i]=pgm_read_byte_near( &E010_map_txid[rx_tx_addr[3]&0x0F][i] );
			if((rx_tx_addr[3]&0x0E) == 0x0E)
				rx_tx_addr[2]=(rx_tx_addr[3]&0x01)+1;
			else
				rx_tx_addr[2]=0;
			break;
		case WLH08:
			rx_tx_addr[0]&=0xF8;
			rx_tx_addr[2]=rx_tx_addr[3];	// Make use of RX_Num
			break;
		default:
			for(uint8_t i=0;i<3;i++)
				rx_tx_addr[i]=pgm_read_byte_near( &MJXQ_map_txid[rx_tx_addr[3]%3][i] );
			break;
	}
}

uint16_t MJXQ_callback()
{
	if(IS_BIND_DONE)
		MJXQ_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			MJXQ_init2();
			BIND_DONE;
		}
		else
		{
			bind_counter--;
			MJXQ_send_packet(1);
		}
	}

    return MJXQ_PACKET_PERIOD;
}

uint16_t initMJXQ(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
	bind_counter = MJXQ_BIND_COUNT;
    MJXQ_initialize_txid();
    MJXQ_init();
	packet_count=0;
	return MJXQ_INITIAL_WAIT+MJXQ_PACKET_PERIOD;
}

#endif


# 1 "src/Hisky_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/hisky_nrf24l01.c dated 2015-03-27

#if defined(HISKY_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define HISKY_BIND_COUNT 1000
#define HISKY_TXID_SIZE 5
#define HISKY_FREQUENCE_NUM  20
//
uint8_t bind_buf_arry[4][10];

// HiSky protocol uses TX id as an address for nRF24L01, and uses frequency hopping sequence
// which does not depend on this id and is passed explicitly in binding sequence. So we are free
// to generate this sequence as we wish. It should be in the range [02..77]
static void __attribute__((unused)) calc_fh_channels()
{
	uint8_t idx = 0;
	uint32_t rnd = MProtocol_id;

	while (idx < HISKY_FREQUENCE_NUM)
	{
		uint8_t i;
		uint8_t count_2_26 = 0, count_27_50 = 0, count_51_74 = 0;

		rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization
		// Use least-significant byte. 73 is prime, so channels 76..77 are unused
		uint8_t next_ch = ((rnd >> 8) % 73) + 2;
		// Keep the distance 2 between the channels - either odd or even
		if (((next_ch ^ (uint8_t)rx_tx_addr[3]) & 0x01 )== 0)
			continue;
		// Check that it's not duplicated and spread uniformly
		for (i = 0; i < idx; i++) {
			if(hopping_frequency[i] == next_ch)
				break;
			if(hopping_frequency[i] <= 26)
				count_2_26++;
			else if (hopping_frequency[i] <= 50)
				count_27_50++;
			else
				count_51_74++;
		}
		if (i != idx)
			continue;
		if ( (next_ch <= 26 && count_2_26 < 8) || (next_ch >= 27 && next_ch <= 50 && count_27_50 < 8) || (next_ch >= 51 && count_51_74 < 8) )
			hopping_frequency[idx++] = next_ch;//find hopping frequency
	}
}

static void __attribute__((unused)) build_binding_packet(void)
{
	uint8_t i;
	uint16_t sum=0;
	uint8_t sum_l,sum_h;

	for(i=0;i<5;i++)
		sum += rx_tx_addr[i];

	sum_l = (uint8_t)sum;//low byte
	sum >>= 8;
	sum_h = (uint8_t)sum;//high bye

	bind_buf_arry[0][0] = 0xff;
	bind_buf_arry[0][1] = 0xaa;
	bind_buf_arry[0][2] = 0x55;

	for(i=3;i<8;i++)
		bind_buf_arry[0][i] = rx_tx_addr[i-3];

	for(i=1;i<4;i++)
	{
		bind_buf_arry[i][0] = sum_l;
		bind_buf_arry[i][1] = sum_h;
		bind_buf_arry[i][2] = i-1;
	}

	for(i=0;i<7;i++)
	{	bind_buf_arry[1][i+3] = hopping_frequency[i];
		bind_buf_arry[2][i+3] = hopping_frequency[i+7];
		bind_buf_arry[3][i+3] = hopping_frequency[i+14];
	}
}

static void __attribute__((unused)) hisky_init()
{
	NRF24L01_Initialize();

	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknowledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);		// Enable p0 rx
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);		// 5-byte RX/TX address (byte -2)
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 81);			// binding packet must be set in channel 81
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 10);		// payload size = 10
	if(sub_protocol==HK310)
		NRF24L01_SetBitrate(NRF24L01_BR_250K);				// 250Kbps
	else
		NRF24L01_SetBitrate(NRF24L01_BR_1M);				// 1Mbps
	NRF24L01_SetPower();								// Set power
	NRF24L01_SetTxRxMode(TX_EN);						// TX mode, 2-bytes CRC, radio on
}

// HiSky channel sequence: AILE  ELEV  THRO  RUDD  GEAR  PITCH, channel data value is from 0 to 1000
// Channel 7 - Gyro mode, 0 - 6 axis, 3 - 3 axis 
static void __attribute__((unused)) build_ch_data()
{
	uint16_t temp;
	uint8_t i,j;
	for (i = 0; i< 8; i++) {
		j=CH_AETR[i];
		temp=convert_channel_16b_limit(j,0,1000);            			
		if (j == THROTTLE) // It is clear that hisky's throttle stick is made reversely, so I adjust it here on purpose
			temp = 1000 -temp;
		if (j == CH7)
			temp = temp < 400 ? 0 : 3; // Gyro mode, 0 - 6 axis, 3 - 3 axis 
		packet[i] = (uint8_t)(temp&0xFF);
		packet[i<4?8:9]>>=2;
		packet[i<4?8:9]|=(temp>>2)&0xc0;
	}
}

uint16_t hisky_cb()
{
	phase++;
	if(sub_protocol==HK310)
		switch(phase)
		{
			case 1:
				NRF24L01_SetPower();
				phase=2;
				break;
			case 3:
				if (! bind_counter)
					NRF24L01_WritePayload(packet,10); // 2 packets per 5ms
				break;
			case 4:
				phase=6;
				break;
			case 7:	// build packet
				#ifdef FAILSAFE_ENABLE
					if(IS_FAILSAFE_VALUES_on && hopping_frequency_no==0)
					{ //  send failsafe every 100ms
						convert_failsafe_HK310(RUDDER,  &packet[0],&packet[1]);
						convert_failsafe_HK310(THROTTLE,&packet[2],&packet[3]);
						convert_failsafe_HK310(CH5,    &packet[4],&packet[5]);
						packet[7]=0xAA;
						packet[8]=0x5A;
					}
					else
				#endif
					{
						convert_channel_HK310(RUDDER,  &packet[0],&packet[1]);
						convert_channel_HK310(THROTTLE,&packet[2],&packet[3]);
						convert_channel_HK310(CH5,    &packet[4],&packet[5]);
						packet[7]=0x55;
						packet[8]=0x67;
					}
				phase=8;
				break;
		}
	switch(phase)
	{
		case 1:
			NRF24L01_FlushTx();
			break;
		case 2:
			if (bind_counter != 0)
			{
				//Set TX id and channel for bind packet
				NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t *)"\x12\x23\x23\x45\x78", 5);
				NRF24L01_WriteReg(NRF24L01_05_RF_CH, 81);
			}
			break;
		case 3:
			if (bind_counter != 0)
			{
				bind_counter--;//
				if (! bind_counter) //Binding complete
					BIND_DONE;//
				//Send bind packet
				NRF24L01_WritePayload(bind_buf_arry[binding_idx],10);
				binding_idx++;
				if (binding_idx >= 4)
					binding_idx = 0;
			}
			break;
		case 4:
			if (bind_counter != 0)
				NRF24L01_FlushTx();
			break;
		case 5:
			//Set TX power
			NRF24L01_SetPower();
			break;
		case 6:
			//Set TX id and channel for normal packet
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
			NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
			hopping_frequency_no++;
			if (hopping_frequency_no >= HISKY_FREQUENCE_NUM)
				hopping_frequency_no = 0;
			break;
		case 7:
			//Build normal packet
			build_ch_data();
			break;
		case 8:
			break;
		default:
			//Send normal packet
			phase = 0;
			NRF24L01_WritePayload(packet,10);
			break;
	}
	return 1000;  // send 1 binding packet and 1 data packet per 9ms	
}

static void __attribute__((unused)) initialize_tx_id()
{
	//Generate frequency hopping table	
	if(sub_protocol==HK310)
	{
		// for HiSky surface protocol, the transmitter always generates hop channels in sequential order. 
		// The transmitter only generates the first hop channel between 0 and 49. So the channel range is from 0 to 69.
		hopping_frequency_no=rx_tx_addr[0]%50;
		for(uint8_t i=0;i<HISKY_FREQUENCE_NUM;i++)
			hopping_frequency[i]=hopping_frequency_no++;	// Sequential order hop channels...
	}
	else
		calc_fh_channels();
}

uint16_t initHiSky()
{
	initialize_tx_id();
	build_binding_packet();
	hisky_init();
	phase = 0;
	hopping_frequency_no = 0;
	binding_idx = 0;

	if(IS_BIND_IN_PROGRESS)
		bind_counter = HISKY_BIND_COUNT;
	else 
		bind_counter = 0;
	return 1000;
}

#endif


# 1 "src/FrSkyX_cc2500.ino" // Helps debugging !
/*  **************************
	* By Midelic on RCGroups *
	**************************
	This project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	
	Multiprotocol is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	
	You should have received a copy of the GNU General Public License
	along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
*/

#if defined(FRSKYX_CC2500_INO)

#include "iface_cc2500.h"

uint8_t FrX_chanskip;
uint8_t FrX_send_seq ;
uint8_t FrX_receive_seq ;

#define FRX_FAILSAFE_TIMEOUT 1032

static void __attribute__((unused)) frskyX_set_start(uint8_t ch )
{
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_WriteReg(CC2500_25_FSCAL1, calData[ch]);
	CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[ch]);
}		

static void __attribute__((unused)) frskyX_init()
{
	FRSKY_init_cc2500((sub_protocol&2)?FRSKYXEU_cc2500_conf:FRSKYX_cc2500_conf); // LBT or FCC
	//
	for(uint8_t c=0;c < 48;c++)
	{//calibrate hop channels
		CC2500_Strobe(CC2500_SIDLE);    
		CC2500_WriteReg(CC2500_0A_CHANNR,hopping_frequency[c]);
		CC2500_Strobe(CC2500_SCAL);
		delayMicroseconds(900);//
		calData[c] = CC2500_ReadReg(CC2500_25_FSCAL1);
	}
	//#######END INIT########		
}

static void __attribute__((unused)) frskyX_initialize_data(uint8_t adr)
{
	CC2500_WriteReg(CC2500_0C_FSCTRL0,option);	// Frequency offset hack 
	CC2500_WriteReg(CC2500_18_MCSM0,    0x8);	
	CC2500_WriteReg(CC2500_09_ADDR, adr ? 0x03 : rx_tx_addr[3]);
	CC2500_WriteReg(CC2500_07_PKTCTRL1,0x05);
}

//**CRC**
const uint16_t PROGMEM frskyX_CRC_Short[]={
	0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
	0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7 };
static uint16_t __attribute__((unused)) frskyX_CRCTable(uint8_t val)
{
	uint16_t word ;
	word = pgm_read_word(&frskyX_CRC_Short[val&0x0F]) ;
	val /= 16 ;
	return word ^ (0x1081 * val) ;
}
static uint16_t __attribute__((unused)) frskyX_crc_x(uint8_t *data, uint8_t len)
{
	uint16_t crc = 0;
	for(uint8_t i=0; i < len; i++)
		crc = (crc<<8) ^ frskyX_CRCTable((uint8_t)(crc>>8) ^ *data++);
	return crc;
}

static void __attribute__((unused)) frskyX_build_bind_packet()
{
	packet[0] = (sub_protocol & 2 ) ? 0x20 : 0x1D ; // LBT or FCC
	packet[1] = 0x03;
	packet[2] = 0x01;
	//
	packet[3] = rx_tx_addr[3];
	packet[4] = rx_tx_addr[2];
	int idx = ((state -FRSKY_BIND) % 10) * 5;
	packet[5] = idx;
	packet[6] = hopping_frequency[idx++];
	packet[7] = hopping_frequency[idx++];
	packet[8] = hopping_frequency[idx++];
	packet[9] = hopping_frequency[idx++];
	packet[10] = hopping_frequency[idx++];
	packet[11] = 0x02;
	packet[12] = RX_num;
	//
	uint8_t limit = (sub_protocol & 2 ) ? 31 : 28 ;
	memset(&packet[13], 0, limit - 13);
	uint16_t lcrc = frskyX_crc_x(&packet[3], limit-3);
	//
	packet[limit++] = lcrc >> 8;
	packet[limit] = lcrc;
	//
}

// 0-2047, 0 = 817, 1024 = 1500, 2047 = 2182
//64=860,1024=1500,1984=2140//Taranis 125%
static uint16_t  __attribute__((unused)) frskyX_scaleForPXX( uint8_t i )
{	//mapped 860,2140(125%) range to 64,1984(PXX values);
	uint16_t chan_val=convert_channel_frsky(i)-1226;
	if(i>7) chan_val|=2048;   // upper channels offset
	return chan_val;
}
#ifdef FAILSAFE_ENABLE
static uint16_t  __attribute__((unused)) frskyX_scaleForPXX_FS( uint8_t i )
{	//mapped 1,2046(125%) range to 64,1984(PXX values);
	uint16_t chan_val=((Failsafe_data[i]*15)>>4)+64;
	if(Failsafe_data[i]==FAILSAFE_CHANNEL_NOPULSES)
		chan_val=FAILSAFE_CHANNEL_NOPULSES;
	else if(Failsafe_data[i]==FAILSAFE_CHANNEL_HOLD)
		chan_val=FAILSAFE_CHANNEL_HOLD;
	if(i>7) chan_val|=2048;   // upper channels offset
	return chan_val;
}
#endif

#define FRX_FAILSAFE_TIME 1032
static void __attribute__((unused)) frskyX_data_frame()
{
	//0x1D 0xB3 0xFD 0x02 0x56 0x07 0x15 0x00 0x00 0x00 0x04 0x40 0x00 0x04 0x40 0x00 0x04 0x40 0x00 0x04 0x40 0x08 0x00 0x00 0x00 0x00 0x00 0x00 0x96 0x12
	//
	static uint8_t chan_offset=0;
	uint16_t chan_0 ;
	uint16_t chan_1 ; 
	//
    // data frames sent every 9ms; failsafe every 9 seconds
	#ifdef FAILSAFE_ENABLE
		static uint16_t failsafe_count=0;
		static uint8_t FS_flag=0,failsafe_chan=0;
		if (FS_flag == 0  &&  failsafe_count > FRX_FAILSAFE_TIME  &&  chan_offset == 0  &&  IS_FAILSAFE_VALUES_on)
		{
			FS_flag = 0x10;
			failsafe_chan = 0;
		} else if (FS_flag & 0x10 && failsafe_chan < (sub_protocol & 0x01 ? 8-1:16-1))
		{
			FS_flag = 0x10 | ((FS_flag + 2) & 0x0F);	//10, 12, 14, 16, 18, 1A, 1C, 1E - failsafe packet
			failsafe_chan ++;
		} else if (FS_flag & 0x10)
		{
			FS_flag = 0;
			failsafe_count = 0;
		}
		failsafe_count++;
	#endif
	
	packet[0] = (sub_protocol & 0x02 ) ? 0x20 : 0x1D ;	// LBT or FCC
	packet[1] = rx_tx_addr[3];
	packet[2] = rx_tx_addr[2];
	packet[3] = 0x02;
	//  
	packet[4] = (FrX_chanskip<<6)|hopping_frequency_no; 
	packet[5] = FrX_chanskip>>2;
	packet[6] = RX_num;
	//packet[7] = FLAGS 00 - standard packet
	//10, 12, 14, 16, 18, 1A, 1C, 1E - failsafe packet
	//20 - range check packet
	#ifdef FAILSAFE_ENABLE
		packet[7] = FS_flag;
	#else
		packet[7] = 0;
	#endif
	packet[8] = 0;		
	//
	uint8_t startChan = chan_offset;	for(uint8_t i = 0; i <12 ; i+=3)
	{//12 bytes of channel data
		#ifdef FAILSAFE_ENABLE
			if( (FS_flag & 0x10) && ((failsafe_chan & 0x07) == (startChan & 0x07)) )
				chan_0 = frskyX_scaleForPXX_FS(failsafe_chan);
			else
		#endif
				chan_0 = frskyX_scaleForPXX(startChan);
		startChan++;
		//
		#ifdef FAILSAFE_ENABLE
			if( (FS_flag & 0x10) && ((failsafe_chan & 0x07) == (startChan & 0x07)) )
				chan_1 = frskyX_scaleForPXX_FS(failsafe_chan);
			else
		#endif
				chan_1 = frskyX_scaleForPXX(startChan);
		startChan++;
		//
		packet[9+i] = lowByte(chan_0);	//3 bytes*4
		packet[9+i+1]=(((chan_0>>8) & 0x0F)|(chan_1 << 4));
		packet[9+i+2]=chan_1>>4;
	}
	packet[21] = (FrX_receive_seq << 4) | FrX_send_seq ;//8 at start
	
	if(sub_protocol & 0x01 )			// in X8 mode send only 8ch every 9ms
		chan_offset = 0 ;
	else
		chan_offset^=0x08;
	
	uint8_t limit = (sub_protocol & 2 ) ? 31 : 28 ;
	for (uint8_t i=22;i<limit;i++)
		packet[i]=0;
	#if defined SPORT_POLLING
		uint8_t idxs=0;
		if(ok_to_send)
			for (uint8_t i=23;i<limit;i++)
			{//
				if(sport_index==sport_idx)
				{//no new data
					ok_to_send=false;
					break;
				}
				packet[i]=SportData[sport_index];	
				sport_index= (sport_index+1)& (MAX_SPORT_BUFFER-1);
				idxs++;
			}
		packet[22]= idxs;
		#ifdef DEBUG_SERIAL
			for(uint8_t i=0;i<idxs;i++)
			{
				Serial.print(packet[23+i],HEX);
				Serial.print(" ");	
			}		
			Serial.println(" ");
		#endif
	#endif // SPORT_POLLING

	uint16_t lcrc = frskyX_crc_x(&packet[3], limit-3);
	packet[limit++]=lcrc>>8;//high byte
	packet[limit]=lcrc;//low byte
}

uint16_t ReadFrSkyX()
{
	switch(state)
	{	
		default: 
			frskyX_set_start(47);		
			CC2500_SetPower();
			CC2500_Strobe(CC2500_SFRX);
			//		
			frskyX_build_bind_packet();
			CC2500_Strobe(CC2500_SIDLE);
			CC2500_WriteData(packet, packet[0]+1);
			if(IS_BIND_DONE)
				state = FRSKY_BIND_DONE;
			else
				state++;
			return 9000;
		case FRSKY_BIND_DONE:
			frskyX_initialize_data(0);
			hopping_frequency_no=0;
			BIND_DONE;
			state++;			
			break;		
		case FRSKY_DATA1:
			if ( prev_option != option )
			{
				CC2500_WriteReg(CC2500_0C_FSCTRL0,option);	// Frequency offset hack 
				prev_option = option ;
			}
			CC2500_SetTxRxMode(TX_EN);
			frskyX_set_start(hopping_frequency_no);
			CC2500_SetPower();		
			CC2500_Strobe(CC2500_SFRX);
			hopping_frequency_no = (hopping_frequency_no+FrX_chanskip)%47;
			CC2500_Strobe(CC2500_SIDLE);		
			CC2500_WriteData(packet, packet[0]+1);
			//
//			frskyX_data_frame();
			state++;
			return 5200;
		case FRSKY_DATA2:
			CC2500_SetTxRxMode(RX_EN);
			CC2500_Strobe(CC2500_SIDLE);
			state++;
			return 200;
		case FRSKY_DATA3:		
			CC2500_Strobe(CC2500_SRX);
			state++;
			return 3100;
		case FRSKY_DATA4:
			len = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;	
			if (len && (len<=(0x0E + 3)))				//Telemetry frame is 17
			{
				packet_count=0;
				CC2500_ReadData(pkt, len);
				#if defined TELEMETRY
					frsky_check_telemetry(pkt,len);	//check if valid telemetry packets
					//parse telemetry packets here
					//The same telemetry function used by FrSky(D8).
				#endif
			} 
			else
			{
				packet_count++;
				// restart sequence on missed packet - might need count or timeout instead of one missed
				if(packet_count>100)
				{//~1sec
//					seq_last_sent = 0;
//					seq_last_rcvd = 8;
					FrX_send_seq = 0x08 ;
//					FrX_receive_seq = 0 ;
					packet_count=0;
					#if defined TELEMETRY
						telemetry_lost=1;
					#endif
				}
				CC2500_Strobe(CC2500_SFRX);			//flush the RXFIFO
			}
			frskyX_data_frame();
			if ( FrX_send_seq != 0x08 )
			{
				FrX_send_seq = ( FrX_send_seq + 1 ) & 0x03 ;
			}
			state = FRSKY_DATA1;
			return 500;
	}		
	return 1;		
}

uint16_t initFrSkyX()
{
	set_rx_tx_addr(MProtocol_id_master);
	Frsky_init_hop();
	packet_count=0;
	while(!FrX_chanskip)
		FrX_chanskip=random(0xfefefefe)%47;

	//for test***************
	//rx_tx_addr[3]=0xB3;
	//rx_tx_addr[2]=0xFD;
	//************************
	frskyX_init();
#if defined  SPORT_POLLING
#ifdef INVERT_SERIAL
	start_timer4() ;
#endif
#endif
	//
	if(IS_BIND_IN_PROGRESS)
	{	   
		state = FRSKY_BIND;
		frskyX_initialize_data(1);
	}
	else
	{
		state = FRSKY_DATA1;
		frskyX_initialize_data(0);
	}
//	seq_last_sent = 0;
//	seq_last_rcvd = 8;
	FrX_send_seq = 0x08 ;
	FrX_receive_seq = 0 ;
	return 10000;
}	
#endif

# 1 "src/WK2x01_cyrf6936.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(WK2x01_CYRF6936_INO)

#include "iface_cyrf6936.h"

#define WK_BIND_COUNT 2980
#define WK_NUM_WAIT_LOOPS (100 / 5) //each loop is ~5us.  Do not wait more than 100us

enum {
	WK_BIND=0,
	WK_BOUND_1,
	WK_BOUND_2,
	WK_BOUND_3,
	WK_BOUND_4,
	WK_BOUND_5,
	WK_BOUND_6,
	WK_BOUND_7,
	WK_BOUND_8,
};

static const uint8_t WK_sopcodes[8] = {
    /* Note these are in order transmitted (LSB 1st) */
    0xDF,0xB1,0xC0,0x49,0x62,0xDF,0xC1,0x49 //0x49C1DF6249C0B1DF
};
static const uint8_t init_2801[] = {0xc5, 0x34, 0x60, 0x00, 0x25};
static const uint8_t init_2601[] = {0xb9, 0x45, 0xb0, 0xf1, 0x3a};
static const uint8_t init_2401[] = {0xa5, 0x23, 0xd0, 0xf0, 0x00};

uint8_t WK_last_beacon;

static void __attribute__((unused)) WK_add_pkt_crc(uint8_t init)
{
	uint8_t add = init;
	uint8_t xou = init;
	for (uint8_t i = 0; i < 14; i++)
	{
		add += packet[i];
		xou ^= packet[i];
	}
	packet[14] = xou;
	packet[15] = add;
}

static void __attribute__((unused)) WK_build_bind_pkt(const uint8_t *init)
{
	packet[0] = init[0];
	packet[1] = init[1];
	packet[2] = hopping_frequency[0];
	packet[3] = hopping_frequency[1];
	packet[4] = init[2];
	packet[5] = hopping_frequency[2];
	packet[6] = 0xff;
	packet[7] = 0x00;
	packet[8] = 0x00;
	packet[9] = 0x32;
	if (sub_protocol == WK2401)
		packet[10]  = 0x10 | (rx_tx_addr[0]  & 0x0e);
	else
		packet[10]  = rx_tx_addr[0];
	packet[11] = rx_tx_addr[1];
	packet[12] = rx_tx_addr[2] | packet_count;
	packet[13] = init[3];
	WK_add_pkt_crc(init[4]);
}

static int16_t __attribute__((unused)) WK_get_channel(uint8_t ch, int32_t scale, int16_t center, int16_t range)
{
	int16_t value = convert_channel_16b_nolimit(CH_AETR[ch],-scale,scale)+center;
	if (value < center - range) value = center - range;
	if (value > center + range) value = center + range;
	return value;
}

static void __attribute__((unused)) WK_build_data_pkt_2401()
{
	uint16_t msb = 0;
	uint8_t offset = 0;
	for (uint8_t i = 0; i < 4; i++)
	{
		if (i == 2)
			offset = 1;
		int16_t value = WK_get_channel(i, 0x800, 0, 0xA00);	//12 bits, allow value to go to 125%
		uint16_t base = abs(value) >> 2;					//10 bits is the base value
		uint16_t trim = abs(value) & 0x03;					//lowest 2 bits represent trim
		if (base >= 0x200)
		{  //if value is > 100%, remainder goes to trim
			trim = 4 *(base - 0x200);
			base = 0x1ff;
		}
		base = (value >= 0) ? 0x200 + base : 0x200 - base;
		trim = (value >= 0) ? 0x200 + trim : 0x200 - trim;

		packet[2*i+offset]   = base & 0xff;
		packet[2*i+offset+1] = trim & 0xff;
		msb = (msb << 4) | ((base >> 6) & 0x0c) | ((trim >> 8) & 0x03);
	}
	packet[4] = msb >> 8; //Ele/Ail MSB
	packet[9] = msb & 0xff; //Thr/Rud MSB
	packet[10]  = 0xe0 | (rx_tx_addr[0]  & 0x0e);
	packet[11] = rx_tx_addr[1];
	packet[12] = rx_tx_addr[2] | packet_count;
	packet[13] = 0xf0; //FIXME - What is this?
	WK_add_pkt_crc(0x00);
}

#define PCT(pct, max) (((int32_t)(max) * (int32_t)(pct) + 1L) / 1000L)
#define MAXTHR 426 //Measured to provide equal value at +/-0
static void __attribute__((unused)) WK_channels_6plus1_2601(uint8_t frame, int16_t *_v1, int16_t *_v2)
{
	int16_t thr = WK_get_channel(2, 1000, 0, 1000);
	int16_t v1;
	uint8_t thr_rev = 0, pitch_rev = 0;
	if(thr > 0)
	{
		if(thr >= 780)
		{ //78%
			v1 = 0; //thr = 60% * (x - 78%) / 22% + 40%
			thr = PCT(1000-MAXTHR,512) * (thr-PCT(780,1000)) / PCT(220,1000) + PCT(MAXTHR,512);
		}
		else
		{
			v1 = 1023 - 1023 * thr / 780;
			thr = PCT(MAXTHR, 512); //40%
		}
	}
	else
	{
		thr = -thr;
		thr_rev = 1;
		if(thr >= 780)
		{ //78%
			v1 = 1023; //thr = 60% * (x - 78%) / 22% + 40%
			thr = PCT(1000-MAXTHR,512) * (thr-PCT(780,1000)) / PCT(220,1000) + PCT(MAXTHR,512);
		}
		else
		{
			v1 = 1023 * thr / 780;
			thr = PCT(MAXTHR, 512); //40%
		}
	}
	if (thr >= 512)
		thr = 511;
	packet[2] = thr & 0xff;
	packet[4] = (packet[4] & 0xF3) | ((thr >> 6) & 0x04);

	int16_t pitch= WK_get_channel(5, 0x400, 0, 0x400);
	if (pitch < 0)
	{
		pitch_rev = 1;
		pitch = -pitch;
	}
	if (frame == 1)
	{
		//Pitch curve and range
		if (thr > PCT(MAXTHR, 512))
			*_v2 = pitch - pitch * 16 * (thr - PCT(MAXTHR, 512)) / PCT(1000 - MAXTHR, 512) / 100;
		else
			*_v2 = pitch;
		*_v1 = 0;
	}
	else
		if (frame == 2)
		{
			//Throttle curve & Expo
			*_v1 = v1;
			*_v2 = 512;
		}
	packet[7] = (thr_rev << 5) | (pitch_rev << 2); //reverse bits
	packet[8] = 0;
}

static void __attribute__((unused)) WK_channels_5plus1_2601(uint8_t frame, int16_t *v1, int16_t *v2)
{
	(void)v1;
	//Zero out pitch, provide ail, ele, thr, rud, gyr + gear
	if (frame == 1)
		*v2 = 0;					//Pitch curve and range
	packet[7] = 0;
	packet[8] = 0;
}
static void __attribute__((unused)) WK_channels_heli_2601(uint8_t frame, int16_t *v1, int16_t *v2)
{
	//pitch is controlled by rx
	//we can only control fmode, pit-reverse and pit/thr rate
	uint8_t pit_rev = 0;
	if (sub_protocol==W6_HEL_I)
		pit_rev = 1;
	int16_t pit_rate = WK_get_channel(5, 0x400, 0, 0x400);
	uint8_t fmode = 1;
	if (pit_rate < 0)
	{
		pit_rate = -pit_rate;
		fmode = 0;
	}
	if (frame == 1)
	{
		//Pitch curve and range
		*v1 = pit_rate;
		*v2 = (int16_t)(option) * 0x400 / 100 + 0x400;
	}
	packet[7] = (pit_rev << 2);		//reverse bits
	packet[8] = fmode ? 0x02 : 0x00;
}

static void __attribute__((unused)) WK_build_data_pkt_2601()
{
	uint8_t msb = 0;
	uint8_t frame = (packet_count % 3);
	for (uint8_t i = 0; i < 4; i++)
	{
		int16_t value = WK_get_channel(i, 0x190, 0, 0x1FF);
		uint16_t mag = value < 0 ? -value : value;
		packet[i] = mag & 0xff;
		msb = (msb << 2) | ((mag >> 8) & 0x01) | (value < 0 ? 0x02 : 0x00);
	}
	packet[4] = msb;
	int16_t v1 = 0x200, v2 = 0x200;
	if (frame == 0)
	{
		//Gyro & Rudder mix
		v1 = WK_get_channel(6, 0x200, 0x200, 0x200);
		v2 = 0;
	}
	if (sub_protocol == W6_5_1)
		WK_channels_5plus1_2601(frame, &v1, &v2);
	else if (sub_protocol == W6_6_1)
		WK_channels_6plus1_2601(frame, &v1, &v2);
	else
		WK_channels_heli_2601(frame, &v1, &v2);
	if (v1 > 1023)
		v1 = 1023;
	if (v2 > 1023)
		v2 = 1023;
	packet[5] = v2 & 0xff;
	packet[6] = v1 & 0xff;
	//packet[7] handled by channel code
	packet[8] |= (WK_get_channel(4, 0x190, 0, 0x1FF) > 0 ? 1 : 0);
	packet[9] =  ((v1 >> 4) & 0x30) | ((v2 >> 2) & 0xc0) | 0x04 | frame;
	packet[10]  = rx_tx_addr[0];
	packet[11] = rx_tx_addr[1];
	packet[12] = rx_tx_addr[2] | packet_count;
	packet[13] = 0xff;

	WK_add_pkt_crc(0x3A);
}

static void __attribute__((unused)) WK_build_data_pkt_2801()
{
	uint16_t msb = 0;
	uint8_t offset = 0;
	uint8_t sign = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (i == 4) { offset = 1; }
		int16_t value = WK_get_channel(i, 0x1C2, 0, 0x3FF);
		uint16_t mag = value < 0 ? -value : value;
		packet[i+offset] = mag & 0xff;
		msb = (msb << 2) | ((mag >> 8) & 0x03);
		if (value < 0) { sign |= 1 << i; }
	}
	packet[4] = msb >> 8;
	packet[9] = msb  & 0xff;
	packet[10] = rx_tx_addr[0];
	packet[11] = rx_tx_addr[1];
	packet[12] = rx_tx_addr[2] | packet_count;
	packet[13] = sign;
	WK_add_pkt_crc(0x25);
}

static void __attribute__((unused)) WK_build_beacon_pkt_2801()
{
	WK_last_beacon ^= 1;
	uint8_t en = 0;
	uint8_t bind_state;

	#ifdef ENABLE_PPM
	if(mode_select && option==0 && IS_BIND_DONE) 			//PPM mode and option not already set and bind is finished
	{
		BIND_SET_INPUT;
		BIND_SET_PULLUP;										// set pullup
		if(IS_BIND_BUTTON_on)
		{
			eeprom_write_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num),0x01);	// Set fixed id mode for the current model
			option=1;
		}
		BIND_SET_OUTPUT;
	}
	#endif //ENABLE_PPM
    if(prev_option!=option && IS_BIND_DONE)
	{
		set_rx_tx_addr(MProtocol_id);
		rx_tx_addr[2]=rx_tx_addr[3]<<4;		// Make use of RX_num
		bind_counter = WK_BIND_COUNT / 8 + 1;
	}
	if (option)
	{
        if (bind_counter)
            bind_state = 0xe4;
        else
            bind_state = 0x1b;
    }
	else
        bind_state = 0x99;
	
	for (uint8_t i = 0; i < 4; i++)
	{
		#ifdef FAILSAFE_ENABLE
			uint16_t failsafe=Failsafe_data[CH_AETR[i + WK_last_beacon * 4]];
			if(failsafe!=FAILSAFE_CHANNEL_HOLD && IS_FAILSAFE_VALUES_on)
			{
				packet[i+1] = failsafe>>3;	//0..255
				en |= 1 << i;
			}
			else
		#endif
				packet[i+1] = 0;
	}
	packet[0] = en;
	packet[5] = packet[4];
	packet[4] = WK_last_beacon << 6;
	packet[6] = hopping_frequency[0];
	packet[7] = hopping_frequency[1];
	packet[8] = hopping_frequency[2];
	packet[9] = bind_state;
	packet[10] = rx_tx_addr[0];
	packet[11] = rx_tx_addr[1];
	packet[12] = rx_tx_addr[2] | packet_count;
	packet[13] = 0x00; //Does this matter?  in the docs it is the same as the data packet
	WK_add_pkt_crc(0x1C);
}

static void __attribute__((unused)) wk2x01_cyrf_init() {
	/* Initialize CYRF chip */
	CYRF_SetPower(0x28);
	CYRF_WriteRegister(CYRF_06_RX_CFG, 0x4A);
	CYRF_WriteRegister(CYRF_0B_PWR_CTRL, 0x00);
	CYRF_WriteRegister(CYRF_0C_XTAL_CTRL, 0xC0);
	CYRF_WriteRegister(CYRF_0D_IO_CFG, 0x04);
	CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x2C);
	CYRF_WriteRegister(CYRF_10_FRAMING_CFG, 0xEE);
	CYRF_WriteRegister(CYRF_1B_TX_OFFSET_LSB, 0x55);
	CYRF_WriteRegister(CYRF_1C_TX_OFFSET_MSB, 0x05);
	CYRF_WriteRegister(CYRF_1D_MODE_OVERRIDE, 0x18);
	CYRF_WriteRegister(CYRF_32_AUTO_CAL_TIME, 0x3C);
	CYRF_WriteRegister(CYRF_35_AUTOCAL_OFFSET, 0x14);
	CYRF_WriteRegister(CYRF_1E_RX_OVERRIDE, 0x90);
	CYRF_WriteRegister(CYRF_1F_TX_OVERRIDE, 0x00);
	CYRF_WriteRegister(CYRF_01_TX_LENGTH, 0x10);
	CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x2C);
	CYRF_WriteRegister(CYRF_28_CLK_EN, 0x02);
	CYRF_WriteRegister(CYRF_27_CLK_OVERRIDE, 0x02);
	CYRF_ConfigSOPCode(WK_sopcodes);
	CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x28);
	CYRF_WriteRegister(CYRF_1E_RX_OVERRIDE, 0x10);
	CYRF_WriteRegister(CYRF_0E_GPIO_CTRL, 0x20);
	CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x2C);
}

static void __attribute__((unused)) WK_BuildPacket_2801()
{
	switch(phase) {
		case WK_BIND:
			bind_counter--;
			WK_build_bind_pkt(init_2801);
			if (bind_counter == 0)
			{
				BIND_DONE;
				phase++;
			}
			break;
		case WK_BOUND_1:
		case WK_BOUND_2:
		case WK_BOUND_3:
		case WK_BOUND_4:
		case WK_BOUND_5:
		case WK_BOUND_6:
		case WK_BOUND_7:
			WK_build_data_pkt_2801();
			phase++;
			break;
		case WK_BOUND_8:
			WK_build_beacon_pkt_2801();
			phase = WK_BOUND_1;
			if (bind_counter)
			{
				bind_counter--;
				if (bind_counter == 0)
					BIND_DONE;
			}
			break;
	}
}

static void __attribute__((unused)) WK_BuildPacket_2601()
{
	if (bind_counter)
	{
		bind_counter--;
		WK_build_bind_pkt(init_2601);
		if (bind_counter == 0)
			BIND_DONE;
	}
	else
		WK_build_data_pkt_2601();
}

static void __attribute__((unused)) WK_BuildPacket_2401()
{
	if (bind_counter)
	{
		bind_counter--;
		WK_build_bind_pkt(init_2401);
		if(bind_counter == 0)
			BIND_DONE;
	}
	else
		WK_build_data_pkt_2401();
}

uint16_t WK_cb()
{
	if (packet_sent == 0)
	{
		packet_sent = 1;
		if(sub_protocol == WK2801)
			WK_BuildPacket_2801();
		else if(sub_protocol == WK2401)
			WK_BuildPacket_2401();
		else
			WK_BuildPacket_2601();
		packet_count = (packet_count + 1) % 12;
		CYRF_WriteDataPacket(packet);
		return 1600;
	}
	packet_sent = 0;
	uint8_t start=micros();
	while ((uint8_t)micros()-start < 100)			// Wait max 100µs
		if(CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x02)
			break;
	if((packet_count & 0x03) == 0)
	{
		hopping_frequency_no++;
		hopping_frequency_no%=3;
		CYRF_ConfigRFChannel(hopping_frequency[hopping_frequency_no]);
		//Keep transmit power updated
		CYRF_SetPower(0x28);
	}
	return 1200;
}

uint16_t WK_setup()
{
	wk2x01_cyrf_init();
	CYRF_SetTxRxMode(TX_EN);

	hopping_frequency_no=0;
	CYRF_FindBestChannels(hopping_frequency, 3, 4, 4, 80);
	CYRF_ConfigRFChannel(hopping_frequency[0]);

	packet_count = 0;
	packet_sent = 0;
	WK_last_beacon = 0;
	prev_option=option;
	if(sub_protocol!=WK2801 || option==0)
	{
		CYRF_GetMfgData(cyrfmfg_id);
		rx_tx_addr[2]=(hopping_frequency[0] ^ cyrfmfg_id[0] ^ cyrfmfg_id[3])<<4;
		rx_tx_addr[1]=hopping_frequency[1] ^ cyrfmfg_id[1] ^ cyrfmfg_id[4];
		rx_tx_addr[0]=hopping_frequency[2] ^ cyrfmfg_id[2] ^ cyrfmfg_id[5];
		if(sub_protocol == WK2401)
			rx_tx_addr[0] |= 0x01;			//ID must be odd for 2401

		bind_counter = WK_BIND_COUNT;
		phase = WK_BIND;
		BIND_IN_PROGRESS;
	}
	else
	{
		rx_tx_addr[2]=rx_tx_addr[3]<<4;		// Make use of RX_num
		bind_counter = 0;
		phase = WK_BOUND_1;
		BIND_DONE;
	}
	return 2800;
}

#endif


# 1 "src/ESky_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/esky_nrf24l01.c dated 2015-02-13

#if defined(ESKY_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define ESKY_BIND_COUNT		1000
#define ESKY_PACKET_PERIOD	3333
#define ESKY_PAYLOAD_SIZE	13
#define ESKY_PACKET_CHKTIME	100 // Time to wait for packet to be sent (no ACK, so very short)

static void __attribute__((unused)) ESKY_set_data_address()
{
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);     // 4-byte RX/TX address for regular packets
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 4);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    rx_tx_addr, 4);
}

static void __attribute__((unused)) ESKY_init()
{
	NRF24L01_Initialize();

	// 2-bytes CRC, radio off
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);            // No Auto Acknowledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);        // Enable data pipe 0
	if (IS_BIND_IN_PROGRESS)
	{
		NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x01);     // 3-byte RX/TX address for bind packets
		NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)"\x00\x00\x00", 3);
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    (uint8_t*)"\x00\x00\x00", 3);
	}
	else
		ESKY_set_data_address();
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);          // No auto retransmission
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 50);              // Channel 50 for bind packets
	NRF24L01_SetBitrate(NRF24L01_BR_1M);                   // 1Mbps
	NRF24L01_SetPower();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);           // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, ESKY_PAYLOAD_SIZE);  // bytes of data payload for pipe 0
	NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, ESKY_PAYLOAD_SIZE);
	NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, ESKY_PAYLOAD_SIZE);
	NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, ESKY_PAYLOAD_SIZE);
	NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, ESKY_PAYLOAD_SIZE);
	NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, ESKY_PAYLOAD_SIZE);
	NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00);      // Just in case, no real bits to write here
}

static void __attribute__((unused)) ESKY_init2()
{
	NRF24L01_FlushTx();
	hopping_frequency_no = 0;
	uint16_t channel_ord = rx_tx_addr[0] % 74;
	hopping_frequency[12] = 10 + (uint8_t)channel_ord;	//channel_code
	uint8_t channel1, channel2;
	channel1 = 10 + (uint8_t)((37 + channel_ord*5) % 74);
	channel2 = 10 + (uint8_t)((     channel_ord*5) % 74) ;

	hopping_frequency[0] = channel1;
	hopping_frequency[1] = channel1;
	hopping_frequency[2] = channel1;
	hopping_frequency[3] = channel2;
	hopping_frequency[4] = channel2;
	hopping_frequency[5] = channel2;

	//end_bytes
	hopping_frequency[6] = 6;
	hopping_frequency[7] = channel1*2;
	hopping_frequency[8] = channel2*2;
	hopping_frequency[9] = 6;
	hopping_frequency[10] = channel1*2;
	hopping_frequency[11] = channel2*2;

	// Turn radio power on
	NRF24L01_SetTxRxMode(TX_EN);
}

static void __attribute__((unused)) ESKY_send_packet(uint8_t bind)
{
	uint8_t rf_ch = 50; // bind channel
	if (bind)
	{
		// Bind packet
		packet[0]  = rx_tx_addr[2];
		packet[1]  = rx_tx_addr[1];
		packet[2]  = rx_tx_addr[0];
		packet[3]  = hopping_frequency[12]; // channel_code encodes pair of channels to transmit on
		packet[4]  = 0x18;
		packet[5]  = 0x29;
		packet[6]  = 0;
		packet[7]  = 0;
		packet[8]  = 0;
		packet[9]  = 0;
		packet[10] = 0;
		packet[11] = 0;
		packet[12] = 0;
	}
	else
	{
		// Regular packet
		// Each data packet is repeated 3 times on one channel, and 3 times on another channel
		// For arithmetic simplicity, channels are repeated in rf_channels array
		if (hopping_frequency_no == 0)
			for (uint8_t i = 0; i < 6; i++)
			{
				uint16_t val=convert_channel_ppm(CH_AETR[i]);
				packet[i*2]   = val>>8;		//high byte of servo timing(1000-2000us)
				packet[i*2+1] = val&0xFF;	//low byte of servo timing(1000-2000us)
			}
		rf_ch = hopping_frequency[hopping_frequency_no];
		packet[12] = hopping_frequency[hopping_frequency_no+6];	// end_bytes
		hopping_frequency_no++;
		if (hopping_frequency_no > 6) hopping_frequency_no = 0;
	}
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch);
	NRF24L01_FlushTx();
	NRF24L01_WritePayload(packet, ESKY_PAYLOAD_SIZE);
	NRF24L01_SetPower();	//Keep transmit power updated
}

uint16_t ESKY_callback()
{
	if(IS_BIND_DONE)
		ESKY_send_packet(0);
	else
	{
		ESKY_send_packet(1);
		if (--bind_counter == 0)
		{
			ESKY_set_data_address();
			BIND_DONE;
		}
	}
	return ESKY_PACKET_PERIOD;
}

uint16_t initESKY(void)
{
	bind_counter = ESKY_BIND_COUNT;
	rx_tx_addr[2] = rx_tx_addr[3];	// Model match
	rx_tx_addr[3] = 0xBB;
	ESKY_init();
	ESKY_init2();
	return 50000;
}

#endif

# 1 "src/E01X_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with E012 and E015

#if defined(E01X_NRF24L01_INO)

#include "iface_nrf24l01.h"

//Protocols constants
#define E01X_BIND_COUNT			500
#define E01X_INITIAL_WAIT		500
#define E01X_ADDRESS_LENGTH		5

#define E012_PACKET_PERIOD		4525
#define E012_RF_BIND_CHANNEL	0x3c
#define E012_NUM_RF_CHANNELS	4
#define E012_PACKET_SIZE		15

#define E015_PACKET_PERIOD		4500	// stock Tx=9000, but let's send more packets ...
#define E015_RF_CHANNEL			0x2d	// 2445 MHz
#define E015_PACKET_SIZE		10
#define E015_BIND_PACKET_SIZE	9

//Channels
#define E01X_ARM_SW      CH5_SW
#define E01X_FLIP_SW     CH6_SW
#define E01X_LED_SW      CH7_SW
#define E01X_HEADLESS_SW CH8_SW
#define E01X_RTH_SW      CH9_SW

// E012 flags packet[1]
#define E012_FLAG_FLIP       0x40
#define E012_FLAG_HEADLESS   0x10
#define E012_FLAG_RTH        0x04
// E012 flags packet[7]
#define E012_FLAG_EXPERT     0x02

// E015 flags packet[6]
#define E015_FLAG_DISARM     0x80
#define E015_FLAG_ARM        0x40
// E015 flags packet[7]
#define E015_FLAG_FLIP       0x80
#define E015_FLAG_HEADLESS   0x10
#define E015_FLAG_RTH        0x08
#define E015_FLAG_LED        0x04
#define E015_FLAG_EXPERT     0x02
#define E015_FLAG_INTERMEDIATE 0x01

static void __attribute__((unused)) E015_check_arming()
{
	uint8_t arm_channel = E01X_ARM_SW;

	if (arm_channel != arm_channel_previous)
	{
		arm_channel_previous = arm_channel;
		if (arm_channel)
		{
			armed = 1;
			arm_flags ^= E015_FLAG_ARM;
		}
		else
		{
			armed = 0;
			arm_flags ^= E015_FLAG_DISARM;
		}
	}
}

static void __attribute__((unused)) E01X_send_packet(uint8_t bind)
{
	if(sub_protocol==E012)
	{
		packet_length=E012_PACKET_SIZE;
		packet[0] = rx_tx_addr[1];
		if(bind)
		{
			packet[1] = 0xaa;
			memcpy(&packet[2], hopping_frequency, E012_NUM_RF_CHANNELS);
			memcpy(&packet[6], rx_tx_addr, E01X_ADDRESS_LENGTH);
			rf_ch_num=E012_RF_BIND_CHANNEL;
		}
		else
		{
			packet[1] = 0x01
				| GET_FLAG(E01X_RTH_SW,			E012_FLAG_RTH)
				| GET_FLAG(E01X_HEADLESS_SW,	E012_FLAG_HEADLESS)
				| GET_FLAG(E01X_FLIP_SW,		E012_FLAG_FLIP);
			packet[2] = convert_channel_16b_limit(AILERON,  0xc8, 0x00); // aileron
			packet[3] = convert_channel_16b_limit(ELEVATOR, 0x00, 0xc8); // elevator
			packet[4] = convert_channel_16b_limit(RUDDER,   0xc8, 0x00); // rudder
			packet[5] = convert_channel_16b_limit(THROTTLE, 0x00, 0xc8); // throttle
			packet[6] = 0xaa;
			packet[7] = E012_FLAG_EXPERT;	// rate (0-2)
			packet[8] = 0x00;
			packet[9] = 0x00;
			packet[10]= 0x00;
			rf_ch_num=hopping_frequency[hopping_frequency_no++];
			hopping_frequency_no %= E012_NUM_RF_CHANNELS;
		}
		packet[11] = 0x00;
		packet[12] = 0x00;
		packet[13] = 0x56; 
		packet[14] = rx_tx_addr[2];
	}
	else
	{ // E015
		if(bind)
		{
			packet[0] = 0x18;
			packet[1] = 0x04;
			packet[2] = 0x06;
			// data phase address
			memcpy(&packet[3], rx_tx_addr, E01X_ADDRESS_LENGTH);
			// checksum
			packet[8] = packet[3];
			for(uint8_t i=4; i<8; i++)
				packet[8] += packet[i];
			packet_length=E015_BIND_PACKET_SIZE;
		}
		else
		{
			E015_check_arming();
			packet[0] = convert_channel_16b_limit(THROTTLE,	0, 225); // throttle
			packet[1] = convert_channel_16b_limit(RUDDER,   225, 0); // rudder
			packet[2] = convert_channel_16b_limit(AILERON,  0, 225); // aileron
			packet[3] = convert_channel_16b_limit(ELEVATOR, 225, 0); // elevator
			packet[4] = 0x20; // elevator trim
			packet[5] = 0x20; // aileron trim
			packet[6] = arm_flags;
			packet[7] = E015_FLAG_EXPERT
				| GET_FLAG(E01X_FLIP_SW,	E015_FLAG_FLIP)
				| GET_FLAG(E01X_LED_SW,		E015_FLAG_LED)
				| GET_FLAG(E01X_HEADLESS_SW,E015_FLAG_HEADLESS)
				| GET_FLAG(E01X_RTH_SW,		E015_FLAG_RTH);
			packet[8] = 0;
			// checksum
			packet[9] = packet[0];
			for(uint8_t i=1; i<9; i++)
				packet[9] += packet[i];
			packet_length=E015_PACKET_SIZE;
		}
	}

	// Power on, TX mode, CRC enabled
	HS6200_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	HS6200_WritePayload(packet, packet_length);

	// Check and adjust transmission power. We do this after
	// transmission to not bother with timeout after power
	// settings change -  we have plenty of time until next
	// packet.
	NRF24L01_SetPower();
}

static void __attribute__((unused)) E01X_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	if(sub_protocol==E012)
		HS6200_SetTXAddr((uint8_t *)"\x55\x42\x9C\x8F\xC9", E01X_ADDRESS_LENGTH);
	else // E015
		HS6200_SetTXAddr((uint8_t *)"\x62\x54\x79\x38\x53", E01X_ADDRESS_LENGTH);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
	NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1 Mbps
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);                          // Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);     // Set feature bits on
	NRF24L01_Activate(0x73);
}

uint16_t E01X_callback()
{
	if(IS_BIND_IN_PROGRESS)
	{
		if (bind_counter == 0)
		{
			HS6200_SetTXAddr(rx_tx_addr, 5);
			BIND_DONE;
		}
		else
		{
			E01X_send_packet(1);
			bind_counter--;
		}
	}
	else
		E01X_send_packet(0);
	return packet_period;
}

static void __attribute__((unused)) E012_initialize_txid()
{
    // rf channels
    uint32_t lfsr=random(0xfefefefe);
    for(uint8_t i=0; i<E012_NUM_RF_CHANNELS; i++)
        hopping_frequency[i] = 0x10 + (((lfsr >> (i*8)) & 0xff) % 0x32); 
}

uint16_t initE01X()
{
	BIND_IN_PROGRESS;
	if(sub_protocol==E012)
	{
		E012_initialize_txid();
		packet_period=E012_PACKET_PERIOD;
	}
	else
	{ // E015
		packet_period=E015_PACKET_PERIOD;
		rf_ch_num=E015_RF_CHANNEL;
		armed = 0;
		arm_flags = 0;
		arm_channel_previous = E01X_ARM_SW;
	}
	E01X_init();
	bind_counter = E01X_BIND_COUNT;
	hopping_frequency_no = 0;
	return E01X_INITIAL_WAIT;
}

#endif


# 1 "src/J6Pro_cyrf6936.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(J6PRO_CYRF6936_INO)

#include "iface_cyrf6936.h"

enum PktState {
    J6PRO_BIND,
    J6PRO_BIND_01,
    J6PRO_BIND_03_START,
    J6PRO_BIND_03_CHECK,
    J6PRO_BIND_05_1,
    J6PRO_BIND_05_2,
    J6PRO_BIND_05_3,
    J6PRO_BIND_05_4,
    J6PRO_BIND_05_5,
    J6PRO_BIND_05_6,
    J6PRO_CHANSEL,
    J6PRO_CHAN_1,
    J6PRO_CHAN_2,
    J6PRO_CHAN_3,
    J6PRO_CHAN_4,
};

const uint8_t PROGMEM j6pro_bind_sop_code[] = {0x62, 0xdf, 0xc1, 0x49, 0xdf, 0xb1, 0xc0, 0x49};
const uint8_t j6pro_data_code[] = {0x02, 0xf9, 0x93, 0x97, 0x02, 0xfa, 0x5c, 0xe3, 0x01, 0x2b, 0xf1, 0xdb, 0x01, 0x32, 0xbe, 0x6f};

static void __attribute__((unused)) j6pro_build_bind_packet()
{
    packet[0] = 0x01;  //Packet type
    packet[1] = 0x01;  //FIXME: What is this? Model number maybe?
    packet[2] = 0x56;  //FIXME: What is this?
    packet[3] = cyrfmfg_id[0];
    packet[4] = cyrfmfg_id[1];
    packet[5] = cyrfmfg_id[2];
    packet[6] = cyrfmfg_id[3];
    packet[7] = cyrfmfg_id[4];
    packet[8] = cyrfmfg_id[5];
}

static void __attribute__((unused)) j6pro_build_data_packet()
{
    uint8_t i;
    uint32_t upperbits = 0;
    uint16_t value;
    packet[0] = 0xaa; //FIXME what is this?
    for (i = 0; i < 12; i++)
    {
        value = convert_channel_10b(CH_AETR[i]);
        packet[i+1] = value & 0xff;
        upperbits |= (value >> 8) << (i * 2);
    }
    packet[13] = upperbits & 0xff;
    packet[14] = (upperbits >> 8) & 0xff;
    packet[15] = (upperbits >> 16) & 0xff;
}

static void __attribute__((unused)) j6pro_cyrf_init()
{
    /* Initialise CYRF chip */
    CYRF_WriteRegister(CYRF_28_CLK_EN, 0x02);
    CYRF_WriteRegister(CYRF_32_AUTO_CAL_TIME, 0x3c);
    CYRF_WriteRegister(CYRF_35_AUTOCAL_OFFSET, 0x14);
    CYRF_WriteRegister(CYRF_1C_TX_OFFSET_MSB, 0x05);
    CYRF_WriteRegister(CYRF_1B_TX_OFFSET_LSB, 0x55);
    //CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24);
    //CYRF_SetPower(0x05);
    CYRF_WriteRegister(CYRF_06_RX_CFG, 0x4a);
    CYRF_SetPower(0x28);
    CYRF_WriteRegister(CYRF_12_DATA64_THOLD, 0x0e);
    CYRF_WriteRegister(CYRF_10_FRAMING_CFG, 0xee);
    CYRF_WriteRegister(CYRF_1F_TX_OVERRIDE, 0x00);
    CYRF_WriteRegister(CYRF_1E_RX_OVERRIDE, 0x00);
    CYRF_ConfigDataCode(j6pro_data_code, 16);
    CYRF_WritePreamble(0x333302);

    CYRF_GetMfgData(cyrfmfg_id);
	//Model match
	cyrfmfg_id[3]+=RX_num;
}

static void __attribute__((unused)) cyrf_bindinit()
{
    /* Use when binding */
    CYRF_SetPower(0x28); //Deviation using max power, replaced by bind power...
    //CYRF_ConfigRFChannel(0x52);
    CYRF_PROGMEM_ConfigSOPCode(j6pro_bind_sop_code);
    CYRF_ConfigCRCSeed(0x0000);
    //CYRF_WriteRegister(CYRF_06_RX_CFG, 0x4a);
    //CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x80);
    //CYRF_ConfigRFChannel(0x52);
    //CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24);
    //CYRF_WriteRegister(CYRF_02_TX_CTRL, 0x40);
    j6pro_build_bind_packet();
}

static void __attribute__((unused)) cyrf_datainit()
{
    /* Use when already bound */
    uint8_t sop_idx = (0xff & (cyrfmfg_id[0] + cyrfmfg_id[1] + cyrfmfg_id[2] + cyrfmfg_id[3] - cyrfmfg_id[5])) % 19;
    uint16_t crc =  (0xff & (cyrfmfg_id[1] - cyrfmfg_id[4] + cyrfmfg_id[5])) |
                   ((0xff & (cyrfmfg_id[2] + cyrfmfg_id[3] - cyrfmfg_id[4] + cyrfmfg_id[5])) << 8);
    //CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24);
    CYRF_PROGMEM_ConfigSOPCode(DEVO_j6pro_sopcodes[sop_idx]);
    CYRF_ConfigCRCSeed(crc);
}

static void __attribute__((unused)) j6pro_set_radio_channels()
{
    //FIXME: Query free channels
    //lowest channel is 0x08, upper channel is 0x4d?
    CYRF_FindBestChannels(hopping_frequency, 3, 5, 8, 77);
    hopping_frequency[3] = hopping_frequency[0];
}

uint16_t ReadJ6Pro()
{
    uint16_t start;

    switch(phase)
    {
        case J6PRO_BIND:
            cyrf_bindinit();
            phase = J6PRO_BIND_01;
            //no break because we want to send the 1st bind packet now
        case J6PRO_BIND_01:
            CYRF_ConfigRFChannel(0x52);
            CYRF_SetTxRxMode(TX_EN);
            //CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24);
            CYRF_WriteDataPacketLen(packet, 0x09);
            phase = J6PRO_BIND_03_START;
            return 3000; //3msec
        case J6PRO_BIND_03_START:
            start=(uint16_t)micros();
            while ((uint16_t)((uint16_t)micros()-(uint16_t)start) < 500)				// Wait max 500µs
				if((CYRF_ReadRegister(CYRF_02_TX_CTRL) & 0x80) == 0x00)
					break;										// Packet transmission complete
			CYRF_ConfigRFChannel(0x53);
            CYRF_SetTxRxMode(RX_EN);
            //CYRF_WriteRegister(CYRF_06_RX_CFG, 0x4a);
            CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x80);
            phase = J6PRO_BIND_03_CHECK;
            return 30000; //30msec
        case J6PRO_BIND_03_CHECK:
            {
            uint8_t rx = CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
            if((rx & 0x1a) == 0x1a) {
                rx = CYRF_ReadRegister(CYRF_0A_RX_LENGTH);
                if(rx == 0x0f) {
                    rx = CYRF_ReadRegister(CYRF_09_RX_COUNT);
                    if(rx == 0x0f) {
                        //Expected and actual length are both 15
                        CYRF_ReadDataPacketLen(packet, rx);
                        if (packet[0] == 0x03 &&
                            packet[3] == cyrfmfg_id[0] &&
                            packet[4] == cyrfmfg_id[1] &&
                            packet[5] == cyrfmfg_id[2] &&
                            packet[6] == cyrfmfg_id[3] &&
                            packet[7] == cyrfmfg_id[4] &&
                            packet[8] == cyrfmfg_id[5])
                        {
                            //Send back Ack
                            packet[0] = 0x05;
                            CYRF_ConfigRFChannel(0x54);
                            CYRF_SetTxRxMode(TX_EN);
                            phase = J6PRO_BIND_05_1;
                            return 2000; //2msec
                         }
                    }
                }
            }
            phase = J6PRO_BIND_01;
            return 500;
            }
        case J6PRO_BIND_05_1:
        case J6PRO_BIND_05_2:
        case J6PRO_BIND_05_3:
        case J6PRO_BIND_05_4:
        case J6PRO_BIND_05_5:
        case J6PRO_BIND_05_6:
            //CYRF_WriteRegister(CYRF_0F_XACT_CFG, 0x24);
            CYRF_WriteDataPacketLen(packet, 0x0f);
            phase = phase + 1;
            return 4600; //4.6msec
        case J6PRO_CHANSEL:
            BIND_DONE;
            j6pro_set_radio_channels();
            cyrf_datainit();
            phase = J6PRO_CHAN_1;
        case J6PRO_CHAN_1:
            //Keep transmit power updated
            CYRF_SetPower(0x28);
            j6pro_build_data_packet();
            //return 3400;
        case J6PRO_CHAN_2:
            //return 3500;
        case J6PRO_CHAN_3:
            //return 3750
        case J6PRO_CHAN_4:
            CYRF_ConfigRFChannel(hopping_frequency[phase - J6PRO_CHAN_1]);
            CYRF_SetTxRxMode(TX_EN);
            CYRF_WriteDataPacket(packet);
            if (phase == J6PRO_CHAN_4) {
                phase = J6PRO_CHAN_1;
                return 13900;
            }
            phase = phase + 1;
            return 3550;
    }
    return 0;
}

uint16_t initJ6Pro()
{
    j6pro_cyrf_init();

	if(IS_BIND_IN_PROGRESS)
        phase = J6PRO_BIND;
    else
        phase = J6PRO_CHANSEL;
    return 2400;
}

#endif


# 1 "src/SHENQI_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(SHENQI_NRF24L01_INO)

#include "iface_nrf24l01.h"

const uint8_t PROGMEM SHENQI_Freq[] = {
			50,50,20,60,30,40,
			10,30,40,20,60,10,
			50,20,50,40,10,60,
			30,30,60,10,40,50,
			20,10,60,20,50,30,
			40,40,30,50,20,60,
			10,10,20,30,40,50,
			60,60,50,40,30,20,
			10,60,10,50,30,40,
			20,10,40,30,60,20 };

void SHENQI_init()
{
    NRF24L01_Initialize();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);		// Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknowldgement on all data pipes
	NRF24L01_SetBitrate(NRF24L01_BR_1M);          // 1Mbps
    NRF24L01_SetPower();

    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);		// 5 bytes rx/tx address

	LT8900_Config(4, 8, _BV(LT8900_CRC_ON)|_BV(LT8900_PACKET_LENGTH_EN), 0xAA);
	LT8900_SetChannel(2);
	LT8900_SetAddress((uint8_t *)"\x9A\x9A\x9A\x9A",4);
	LT8900_SetTxRxMode(RX_EN);
}

void SHENQI_send_packet()
{
	packet[0]=0x00;
	if(packet_count==0)
	{
		uint8_t bind_addr[4];
		bind_addr[0]=rx_tx_addr[0];
		bind_addr[1]=rx_tx_addr[1];
		bind_addr[2]=0x9A;
		bind_addr[3]=0x9A;
		LT8900_SetAddress(bind_addr,4);
		LT8900_SetChannel(2);
		packet[1]=rx_tx_addr[2];
		packet[2]=rx_tx_addr[3];
		packet_period=2508;
	}
	else
	{
		LT8900_SetAddress(rx_tx_addr,4);
		packet[1]=255-convert_channel_8b(RUDDER);
		packet[2]=255-convert_channel_16b_limit(THROTTLE,0x60,0xA0);
		uint8_t freq=pgm_read_byte_near(&SHENQI_Freq[hopping_frequency_no])+(rx_tx_addr[2]&0x0F);
		LT8900_SetChannel(freq);
		hopping_frequency_no++;
		if(hopping_frequency_no==60)
			hopping_frequency_no=0;
		packet_period=1750;
	}
	// Send packet + 1 retransmit - not sure why but needed (not present on original TX...)
	LT8900_WritePayload(packet,3);
	while(NRF24L01_packet_ack()!=PKT_ACKED);
	LT8900_WritePayload(packet,3);
	
	packet_count++;
	if(packet_count==7)
	{
		packet_count=0;
		packet_period=3000;
	}
	// Set power
	NRF24L01_SetPower();
}

uint16_t SHENQI_callback()
{
	if(IS_BIND_DONE)
		SHENQI_send_packet();
	else
	{
		if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
		{
			if(LT8900_ReadPayload(packet, 3))
			{
				BIND_DONE;
				rx_tx_addr[0]=packet[1];
				rx_tx_addr[1]=packet[2];
				LT8900_SetTxRxMode(TX_EN);
				packet_period=14000;
			}
			NRF24L01_FlushRx();
		}
	}
    return packet_period;
}

uint16_t initSHENQI()
{
	BIND_IN_PROGRESS;	// autobind protocol
	SHENQI_init();
	hopping_frequency_no = 0;
	packet_count=0;
	packet_period=500;
	return 1000;
}

#endif

# 1 "src/Bugs_a7105.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifdef BUGS_A7105_INO

//////////// rxid -> radioid algorithm //////////////////////////////
// Hex digit 1 is periodic with length 2, and hex digit 2 is periodic
// with length 16. However, storing the byte of those 2 digits
// instead of manipulating bits results simpler code and smaller binary.
const uint8_t PROGMEM BUGS_most_popular_67_cycle[]= {
	0x34, 0xc5, 0x6a, 0xb4, 0x29, 0xd5, 0x2c, 0xd3, 0x91, 0xb3, 0x6c, 0x49,
	0x52, 0x9c, 0x4d, 0x65, 0xc3, 0x4a, 0x5b, 0xd6, 0x92, 0x6d, 0x94, 0xa6,
	0x55, 0xcd, 0x2b, 0x9a, 0x36, 0x95, 0x4b, 0xd4, 0x35, 0x8d, 0x96, 0xb2,
	0xa3 };

static uint8_t __attribute__((unused)) BUGS_most_popular_67(uint8_t i)
{
	uint8_t ii;
	if (i == 0)
		return 0xd2;
	else if (i == 1)
		return 0xda;
	else if (i % 16 < 2)
	{
		ii = 2 * (i / 16) + i % 16 - 2;
		if (ii % 2 == 0)
			ii += 7;
	}
	else
		ii=2 * (i / 16) + (i % 16 - 2) % 7;
	return pgm_read_byte_near( &BUGS_most_popular_67_cycle[ii]);
}

static uint8_t __attribute__((unused)) BUGS_most_popular_45(uint8_t i)
{
	if (i == 0)
		return 0xa3;
	else if (i == 1)
		return 0x86;
	else
	{
		if (i % 8 == 1)
			i -= 8;
		else
			i--;
		return BUGS_most_popular_67(i);
	}
}

static uint8_t __attribute__((unused)) BUGS_most_popular_23(uint8_t i)
{
	if (i == 0)
		return 0xb2;
	else if (i == 1)
		return 0xcb;
	else
	{
		if (i % 8 == 1)
			i -= 8;
		else
			i--;
		return BUGS_most_popular_45(i);
	}
}

const uint8_t PROGMEM BUGS_most_popular_01[] = {
	0x52, 0xac, 0x59, 0xa4, 0x53, 0xab, 0x57, 0xa9,
    0x56, 0xa5, 0x5b, 0xa7, 0x5d, 0xa6, 0x58, 0xad};

static uint32_t __attribute__((unused)) BUGS_most_popular(uint8_t i)
{
	i += !(i <= 127);
	uint8_t mp01=pgm_read_byte_near( &BUGS_most_popular_01[i % 16] );
	return (uint32_t) mp01 << 24 |
		(uint32_t) BUGS_most_popular_23(i) << 16 |
		(uint32_t) BUGS_most_popular_45(i) << 8 |
		BUGS_most_popular_67(i);
}

static uint32_t __attribute__((unused)) BUGS_second_most_popular(uint8_t i)
{
	if (i < 127)
		return BUGS_most_popular(i + 1);
	else if (i > 128)
		return BUGS_most_popular(i - 1);
	else
		return 0x52d6926d;
}

// The 22 irregular values do not match the above periodicities. They might be
// errors from the readout, but let us try them here as long as it is not
// proven.
#define BUGS_NBR_IRREGULAR 22
const uint16_t PROGMEM BUGS_irregular_keys[BUGS_NBR_IRREGULAR] = {
	1131, 1287, 2842, 4668, 5311, 11594, 13122, 13813,
	20655, 22975, 25007, 25068, 28252, 33309, 35364, 35765,
	37731, 40296, 43668, 46540, 49868, 65535 };

const uint32_t PROGMEM BUGS_irregular_values[BUGS_NBR_IRREGULAR] = {
	0x52d6926d, 0xa586da34, 0x5329d52c, 0xa66c4952,
	0x536c4952, 0x524a5bd6, 0x534d65c3, 0xa9d391b3,
	0x5249529c, 0xa555cd2b, 0xac9a3695, 0x58d391b3,
	0xa791b36c, 0x53926d94, 0xa7926d94, 0xa72cd391,
	0xa9b429d5, 0x5629d52c, 0xad2b9a36, 0xa74d65c3,
	0x526d94a6, 0xad96b2a3 };

static uint32_t __attribute__((unused)) BUGS_is_irregular(uint16_t i)
{
	for (uint8_t j = 0; j < BUGS_NBR_IRREGULAR; ++j)
		if (pgm_read_word_near( &BUGS_irregular_keys[j]) == i)
			return pgm_read_dword_near( &BUGS_irregular_values[j]);
	return 0;
}

static uint32_t __attribute__((unused)) BUGS_rxid_to_radioid(uint16_t rxid)
{
	uint8_t block = rxid / 256;
	uint8_t second_seq_size;
	bool use_most_popular;

	if (rxid < 32768)
	{
		second_seq_size = 128 - block;
		use_most_popular = rxid % 256 >= second_seq_size;
	}
	else
	{
		second_seq_size = block - 127;
		use_most_popular = 255 - rxid % 256 >= second_seq_size;
	}
	uint32_t v = BUGS_is_irregular(rxid);
	if (!v)
	{
		if (use_most_popular)
			v = BUGS_most_popular(rxid % 255);
		else
			v = BUGS_second_most_popular(rxid % 255);
	}
	return v;
}
//////////// rxid -> radioid algorithm //////////////////////////////

// For code readability
#define BUGS_CH_SW_ARM		CH5_SW
#define BUGS_CH_SW_ANGLE	CH6_SW
#define BUGS_CH_SW_FLIP		CH7_SW
#define BUGS_CH_SW_PICTURE	CH8_SW
#define BUGS_CH_SW_VIDEO	CH9_SW
#define BUGS_CH_SW_LED		CH10_SW

// flags packet byte 4
#define BUGS_FLAG_FLIP		0x08    // automatic flip
#define BUGS_FLAG_MODE		0x04    // low/high speed select (set is high speed)
#define BUGS_FLAG_VIDEO		0x02    // toggle video
#define BUGS_FLAG_PICTURE	0x01    // toggle picture

// flags packet byte 5
#define BUGS_FLAG_LED		0x80    // enable LEDs
#define BUGS_FLAG_ARM		0x40    // arm (toggle to turn on motors)
#define BUGS_FLAG_DISARM	0x20    // disarm (toggle to turn off motors)
#define BUGS_FLAG_ANGLE		0x04    // angle/acro mode (set is angle mode)

#define BUGS_PACKET_SIZE	22
#define BUGS_NUM_RFCHAN		16

enum {
	BUGS_BIND_1,
	BUGS_BIND_2,
	BUGS_BIND_3,
	BUGS_DATA_1,
	BUGS_DATA_2,
	BUGS_DATA_3,
};

static void __attribute__((unused)) BUGS_check_arming()
{
	uint8_t arm_channel = BUGS_CH_SW_ARM;

	if (arm_channel != arm_channel_previous)
	{
		arm_channel_previous = arm_channel;
		if (arm_channel)
		{
			armed = 1;
			arm_flags ^= BUGS_FLAG_ARM;
		}
		else
		{
			armed = 0;
			arm_flags ^= BUGS_FLAG_DISARM;
		}
	}
}

static void __attribute__((unused)) BUGS_build_packet(uint8_t bind)
{
	uint8_t force_values = bind | !armed;
	uint8_t change_channel = ((packet_count & 0x1) << 6);
	uint16_t aileron  = convert_channel_16b_limit(AILERON,800,0);
	uint16_t elevator = convert_channel_16b_limit(ELEVATOR,800,0);
	uint16_t throttle = convert_channel_16b_limit(THROTTLE,0,800);
	uint16_t rudder   = convert_channel_16b_limit(RUDDER,800,0);

	memset(packet, 0, BUGS_PACKET_SIZE);
	packet[1] = 0x76;		// txid (rx uses to know hopping frequencies)
	packet[2] = 0x71;
	packet[3] = 0x94;

	BUGS_check_arming();	// sets globals arm_flags and armed
	if(bind)
	{
		packet[4] = change_channel | 0x80;
		packet[5] = 0x02 | arm_flags
		| GET_FLAG(BUGS_CH_SW_ANGLE, BUGS_FLAG_ANGLE);
	}
	else
	{
		packet[4] = change_channel | BUGS_FLAG_MODE
		| GET_FLAG(BUGS_CH_SW_FLIP, BUGS_FLAG_FLIP)
		| GET_FLAG(BUGS_CH_SW_PICTURE, BUGS_FLAG_PICTURE)
		| GET_FLAG(BUGS_CH_SW_VIDEO, BUGS_FLAG_VIDEO);
		packet[5] = 0x02 | arm_flags
		| GET_FLAG(BUGS_CH_SW_ANGLE, BUGS_FLAG_ANGLE)
		| GET_FLAG(BUGS_CH_SW_LED, BUGS_FLAG_LED);
	}

	packet[6] = force_values ? 100 : (aileron  >> 2);
	packet[7] = force_values ? 100 : (elevator >> 2);
	packet[8] = force_values ?   0 : (throttle >> 2);
	packet[9] = force_values ? 100 : (rudder   >> 2);
	packet[10] = 100;
	packet[11] = 100;
	packet[12] = 100;
	packet[13] = 100;

	packet[14] = ((aileron  << 6) & 0xc0)
	| ((elevator << 4) & 0x30)
	| ((throttle << 2) & 0x0c)
	| ((rudder       ) & 0x03);

	//    packet[15] = 0;

	// driven trims
	packet[16] = aileron / 8 + 14;
	packet[17] = elevator / 8 + 14;
	packet[18] = 64;
	packet[19] = rudder / 8 + 14;

	//    packet[20] = 0;
	//    packet[21] = 0;

    uint8_t check = 0x6d;
    for (uint8_t i=1; i < BUGS_PACKET_SIZE; i++)
        check ^= packet[i];
	packet[0] = check;
}

const uint8_t PROGMEM BUGS_hop []= {
		0x1d, 0x3b, 0x4d, 0x29, 0x11, 0x2d, 0x0b, 0x3d, 0x59, 0x48, 0x17, 0x41, 0x23, 0x4e, 0x2a, 0x63,	// bind phase ID=0xac59a453
		0x4b, 0x19, 0x35, 0x1e, 0x63, 0x0f, 0x45, 0x21, 0x51, 0x3a, 0x5d, 0x25, 0x0a, 0x44, 0x61, 0x27,	// data phase ID=0xA4C56AB4 for txid 767194 if rx responds C6 BB 57 7F 00 00 00 00 00 00 FF 87 40 00 00 00
	};

static void  __attribute__((unused))BUGS_set_radio_data()
{	// captured radio data for bugs rx/tx version A2
	// it appears that the hopping frequencies are determined by the txid
	// and the data phase radio id is determined by the first 2 bytes of the
	// rx bind packet
	uint8_t offset=0;
	uint32_t radio_id=0xac59a453;	// bind phase ID=0xac59a453

	if(IS_BIND_DONE)
	{
		offset=BUGS_NUM_RFCHAN;
		// Read radio_id from EEPROM
		radio_id=0;
		uint8_t base_adr=BUGS_EEPROM_OFFSET+RX_num*4;
		for(uint8_t i=0; i<4; i++)
			radio_id|=eeprom_read_byte((EE_ADDR)(base_adr+i))<<(i*8);
	}
	A7105_WriteID(radio_id);

	for(uint8_t i=0; i<BUGS_NUM_RFCHAN;i++)
		hopping_frequency[i]=pgm_read_byte_near( &BUGS_hop[i+offset] );
}

static void __attribute__((unused)) BUGS_increment_counts()
{	// this logic works with the use of packet_count in BUGS_build_packet
	// to properly indicate channel changes to rx
	packet_count += 1;
	if ((packet_count & 1) == 0)
	{
		hopping_frequency_no += 1;
		hopping_frequency_no %= BUGS_NUM_RFCHAN;
	}
}

#define BUGS_PACKET_PERIOD   6100
#define BUGS_DELAY_TX        2000
#define BUGS_DELAY_POST_RX   1500
#define BUGS_DELAY_BIND_RST   200

// FIFO config is one less than desired value
#define BUGS_FIFO_SIZE_RX      15
#define BUGS_FIFO_SIZE_TX      21
uint16_t ReadBUGS(void)
{
	uint8_t mode, base_adr;
	uint16_t rxid;
	uint32_t radio_id;
	uint16_t start;

	// keep frequency tuning updated
	#ifndef FORCE_FLYSKY_TUNING
		A7105_AdjustLOBaseFreq(1);
	#endif

	switch(phase)
	{
		case BUGS_BIND_1:
			BUGS_build_packet(1);
			A7105_Strobe(A7105_STANDBY);
			A7105_WriteReg(A7105_03_FIFOI, BUGS_FIFO_SIZE_TX);
			A7105_WriteData(BUGS_PACKET_SIZE, hopping_frequency[hopping_frequency_no]);
			phase = BUGS_BIND_2;
			packet_period = BUGS_DELAY_TX;
			break;

		case BUGS_BIND_2:
			//Wait for TX completion
			start=micros();
			while ((uint16_t)micros()-start < 500)			// Wait max 500µs, using serial+telemetry exit in about 60µs
				if(!(A7105_ReadReg(A7105_00_MODE) & 0x01))
					break;
			A7105_SetTxRxMode(RX_EN);
			A7105_WriteReg(A7105_0F_PLL_I, hopping_frequency[hopping_frequency_no] - 2);
			A7105_WriteReg(A7105_03_FIFOI, BUGS_FIFO_SIZE_RX);
			A7105_Strobe(A7105_RX);

			BUGS_increment_counts();
			phase = BUGS_BIND_3;
			packet_period = BUGS_PACKET_PERIOD-BUGS_DELAY_TX-BUGS_DELAY_POST_RX;
			break;

		case BUGS_BIND_3:
			mode = A7105_ReadReg(A7105_00_MODE);
			A7105_Strobe(A7105_STANDBY);
			A7105_SetTxRxMode(TX_EN);
			if (mode & 0x01)
			{
				phase = BUGS_BIND_1;
				packet_period = BUGS_DELAY_BIND_RST;         // No received data so restart binding procedure.
				break;
			}
			A7105_ReadData(16);
			if ((packet[0] + packet[1] + packet[2] + packet[3]) == 0)
			{
				phase = BUGS_BIND_1;
				packet_period = BUGS_DELAY_BIND_RST;         // No received data so restart binding procedure.
				break;
			}
			A7105_Strobe(A7105_STANDBY);
			BIND_DONE;
			// set radio_id
			rxid = (packet[1] << 8) + packet[2];
			radio_id = BUGS_rxid_to_radioid(rxid);
			base_adr=BUGS_EEPROM_OFFSET+RX_num*4;
			for(uint8_t i=0; i<4; i++)
				eeprom_write_byte((EE_ADDR)(base_adr+i),radio_id>>(i*8));	// Save radio_id in EEPROM
			BUGS_set_radio_data();
			phase = BUGS_DATA_1;
			packet_count = 0;
			hopping_frequency_no = 0;
			packet_period = BUGS_DELAY_POST_RX;
			break;

		case BUGS_DATA_1:
			A7105_SetPower();
			BUGS_build_packet(0);
			A7105_WriteReg(A7105_03_FIFOI, BUGS_FIFO_SIZE_TX);
			A7105_WriteData(BUGS_PACKET_SIZE, hopping_frequency[hopping_frequency_no]);
			phase = BUGS_DATA_2;
			packet_period = BUGS_DELAY_TX;
			break;

		case BUGS_DATA_2:
			//Wait for TX completion
			start=micros();
			while ((uint16_t)micros()-start < 500)			// Wait max 500µs, using serial+telemetry exit in about 60µs
				if(!(A7105_ReadReg(A7105_00_MODE) & 0x01))
					break;
			A7105_SetTxRxMode(RX_EN);
			A7105_WriteReg(A7105_0F_PLL_I, hopping_frequency[hopping_frequency_no] - 2);
			A7105_WriteReg(A7105_03_FIFOI, BUGS_FIFO_SIZE_RX);
			A7105_Strobe(A7105_RX);

			BUGS_increment_counts();
			phase = BUGS_DATA_3;
			packet_period = BUGS_PACKET_PERIOD-BUGS_DELAY_TX-BUGS_DELAY_POST_RX;
			break;

		case BUGS_DATA_3:
			mode = A7105_ReadReg(A7105_00_MODE);
			A7105_Strobe(A7105_STANDBY);
			A7105_SetTxRxMode(TX_EN);
			if (!(mode & 0x01))
			{
				A7105_ReadData(16);
				#if defined(BUGS_HUB_TELEMETRY)
					v_lipo1=packet[10] == 0xff ? 0xff : 0x00;					// Voltage in this case is only an alert on level good or bad.
					RX_RSSI=packet[3];
					// Read TX RSSI
					int16_t temp=256-(A7105_ReadReg(A7105_1D_RSSI_THOLD)*8)/5;	// Value from A7105 is between 8 for maximum signal strength to 160 or less
					if(temp<0) temp=0;
					else if(temp>255) temp=255;
					TX_RSSI=temp;
					telemetry_link=1;
				#endif
			}
			phase = BUGS_DATA_1;
			packet_period = BUGS_DELAY_POST_RX;
			break;
	}
	return packet_period;
}

uint16_t initBUGS(void)
{
	uint32_t radio_id=0;
	uint8_t base_adr=BUGS_EEPROM_OFFSET+RX_num*4;
	for(uint8_t i=0; i<4; i++)
		radio_id|=eeprom_read_byte((EE_ADDR)(base_adr+i))<<(i*8);
	if(radio_id==0xffffffff)
		BIND_IN_PROGRESS;

	BUGS_set_radio_data();
	if (IS_BIND_IN_PROGRESS)
		phase = BUGS_BIND_1;
	else
		phase = BUGS_DATA_1;

	A7105_Init();

	hopping_frequency_no = 0;
	packet_count = 0;
	armed = 0;
	arm_flags = BUGS_FLAG_DISARM;		// initial value from captures
	arm_channel_previous = BUGS_CH_SW_ARM;
	#ifdef BUGS_HUB_TELEMETRY
		init_frskyd_link_telemetry();
	#endif

	return 10000;
}

#endif


# 1 "src/CG023_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with EAchine 3D X4, CG023/CG031, Attop YD-822/YD-829/YD-829C and H8_3D/JJRC H20/H22

#if defined(CG023_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CG023_PACKET_PERIOD		8200 // Timeout for callback in uSec
#define CG023_INITIAL_WAIT		500
#define CG023_PACKET_SIZE		15   // packets have 15-byte payload
#define CG023_RF_BIND_CHANNEL	0x2D
#define CG023_BIND_COUNT		500  // 4 seconds
#define YD829_PACKET_PERIOD		4100 // Timeout for callback in uSec


enum CG023_FLAGS {
    // flags going to packet[13]
    CG023_FLAG_FLIP     = 0x01, 
    CG023_FLAG_EASY     = 0x02, 
    CG023_FLAG_VIDEO    = 0x04, 
    CG023_FLAG_STILL    = 0x08, 
    CG023_FLAG_LED_OFF  = 0x10,
    CG023_FLAG_RATE_LOW = 0x00,
    CG023_FLAG_RATE_MID = 0x20,
    CG023_FLAG_RATE_HIGH= 0x40,
};

enum YD829_FLAGS {
    // flags going to packet[13] (YD-829)
    YD829_FLAG_FLIP     = 0x01,
    YD829_MASK_RATE     = 0x0C,
    YD829_FLAG_RATE_MID = 0x04,
    YD829_FLAG_RATE_HIGH= 0x08,
    YD829_FLAG_HEADLESS = 0x20,
    YD829_FLAG_VIDEO    = 0x40, 
    YD829_FLAG_STILL    = 0x80,
};

static void __attribute__((unused)) CG023_send_packet(uint8_t bind)
{
	// throttle : 0x00 - 0xFF
	throttle=convert_channel_8b(THROTTLE);
	// rudder
	rudder = convert_channel_16b_limit(RUDDER,0x44,0xBC);	// yaw right : 0x80 (neutral) - 0xBC (right)
	if (rudder<=0x80)
		rudder=0x80-rudder;							// yaw left : 0x00 (neutral) - 0x3C (left)
	// elevator : 0xBB - 0x7F - 0x43
	elevator = convert_channel_16b_limit(ELEVATOR, 0x43, 0xBB); 
	// aileron : 0x43 - 0x7F - 0xBB
	aileron = convert_channel_16b_limit(AILERON, 0x43, 0xBB); 
	
	if (bind)
		packet[0]= 0xaa;
	else
		packet[0]= 0x55;
	// transmitter id
	packet[1] = rx_tx_addr[0]; 
	packet[2] = rx_tx_addr[1];
	// unknown
	packet[3] = 0x00;
	packet[4] = 0x00;
	packet[5] = throttle;
	packet[6] = rudder;
	packet[7] = elevator;
	packet[8] = aileron;
	// throttle trim : 0x30 - 0x20 - 0x10
	packet[9] = 0x20; // neutral
	// neutral trims
	packet[10] = 0x20;
	packet[11] = 0x40;
	packet[12] = 0x40;
	if(sub_protocol==CG023)
	{
		// rate
		packet[13] =					  CG023_FLAG_RATE_HIGH
					| GET_FLAG(CH5_SW,CG023_FLAG_FLIP)
					| GET_FLAG(CH6_SW,CG023_FLAG_LED_OFF)
					| GET_FLAG(CH7_SW,CG023_FLAG_STILL)
					| GET_FLAG(CH8_SW,CG023_FLAG_VIDEO)
					| GET_FLAG(CH9_SW,CG023_FLAG_EASY);
	}
	else
	{// YD829
		// rate
		packet[13] =					  YD829_FLAG_RATE_HIGH
					| GET_FLAG(CH5_SW,YD829_FLAG_FLIP)
					| GET_FLAG(CH7_SW,YD829_FLAG_STILL)
					| GET_FLAG(CH8_SW,YD829_FLAG_VIDEO)
					| GET_FLAG(CH9_SW,YD829_FLAG_HEADLESS);
	}
	packet[14] = 0;
	
	// Power on, TX mode, 2byte CRC
	// Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if (bind)
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, CG023_RF_BIND_CHANNEL);
	else
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency_no);

	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, CG023_PACKET_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) CG023_init()
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t *)"\x26\xA8\x67\x35\xCC", 5);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower();
}

uint16_t CG023_callback()
{
	if(IS_BIND_DONE)
		CG023_send_packet(0);
	else
	{
		if (bind_counter == 0)
			BIND_DONE;
		else
		{
			CG023_send_packet(1);
			bind_counter--;
		}
	}
	return	packet_period;
}

static void __attribute__((unused)) CG023_initialize_txid()
{
	rx_tx_addr[0]= 0x80 | (rx_tx_addr[0] % 0x40);
	if( rx_tx_addr[0] == 0xAA)			// avoid using same freq for bind and data channel
		rx_tx_addr[0] ++;
	hopping_frequency_no = rx_tx_addr[0] - 0x7D;	// rf channel for data packets
}

uint16_t initCG023(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    bind_counter = CG023_BIND_COUNT;
	CG023_initialize_txid();
	CG023_init();
	if(sub_protocol==CG023)
		packet_period=CG023_PACKET_PERIOD;
	else // YD829
		packet_period=YD829_PACKET_PERIOD;
	return	CG023_INITIAL_WAIT+YD829_PACKET_PERIOD;
}

#endif


# 1 "src/H8_3D_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with EAchine 3D X4, CG023/CG031, Attop YD-822/YD-829/YD-829C and H8_3D/JJRC H20/H22
// Merged CG023 and H8_3D protocols
// Last sync with hexfet new_protocols/cg023_nrf24l01.c dated 2015-10-03
// Last sync with hexfet new_protocols/h8_3d_nrf24l01.c dated 2015-11-18

#if defined(H8_3D_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define H8_3D_PACKET_PERIOD		1800
#define H20H_PACKET_PERIOD		9340
#define H20MINI_PACKET_PERIOD	3100
#define H8_3D_INITIAL_WAIT		500
#define H8_3D_PACKET_SIZE		20
#define H8_3D_RF_NUM_CHANNELS	4
#define H20H_BIND_RF			0x49
#define H8_3D_BIND_COUNT		1000

enum H8_3D_FLAGS {
    // flags going to packet[17]
    H8_3D_FLAG_FLIP      = 0x01,
    H8_3D_FLAG_RATE_MID  = 0x02,
    H8_3D_FLAG_RATE_HIGH = 0x04,
    H8_3D_FLAG_LIGTH	 = 0x08, // Light on H22
    H8_3D_FLAG_HEADLESS  = 0x10, // RTH + headless on H8, headless on JJRC H20, RTH on H22
    H8_3D_FLAG_RTH		 = 0x20, // 360 flip mode on H8 3D and H22, RTH on JJRC H20
};

enum H8_3D_FLAGS_2 {
    // flags going to packet[18]
    H8_3D_FLAG_VIDEO      = 0x80,
    H8_3D_FLAG_PICTURE    = 0x40,
    H8_3D_FLAG_CALIBRATE1 = 0x20,  // H8 3D acc calibration, H20,H20H headless calib
    H8_3D_FLAG_CALIBRATE2 = 0x10,  // H11D, H20, H20H acc calibration
    H8_3D_FLAG_CAM_DN     = 0x08,
    H8_3D_FLAG_CAM_UP     = 0x04,
};

static void __attribute__((unused)) H8_3D_send_packet(uint8_t bind)
{
	if(sub_protocol==H20H)
		packet[0] = 0x14;
	else // H8_3D, H20MINI, H30MINI
		packet[0] = 0x13;

	packet[1] = rx_tx_addr[0]; 
	packet[2] = rx_tx_addr[1];
	packet[3] = rx_tx_addr[2];
	packet[4] = rx_tx_addr[3];
	packet[8] = rx_tx_addr[0]+rx_tx_addr[1]+rx_tx_addr[2]+rx_tx_addr[3]; // txid checksum
	memset(&packet[9], 0, 10);
	if (bind)
	{    
		packet[5] = 0x00;
		packet[6] = 0x00;
		packet[7] = 0x01;
	}
	else
	{
		packet[5] = hopping_frequency_no;
		packet[7] = 0x03;

		rudder = convert_channel_16b_limit(RUDDER,0x44,0xBC);			// yaw right : 0x80 (neutral) - 0xBC (right)
		if(sub_protocol!=H20H)
		{ // H8_3D, H20MINI, H30MINI
			packet[6] = 0x08;
			packet[9] = convert_channel_8b(THROTTLE);					// throttle  : 0x00 - 0xFF
			packet[15] = 0x20;	// trims
			packet[16] = 0x20;	// trims
			if (rudder<=0x80)
				rudder=0x80-rudder;										// yaw left  : 0x00 (neutral) - 0x3C (left)
			if(rudder==0x01 || rudder==0x81)
				rudder=0x00;	// Small deadband
		}
		else
		{ //H20H
			packet[6] = hopping_frequency_no == 0 ? 8 - packet_count : 16 - packet_count;
			packet[9] = convert_channel_16b_limit(THROTTLE, 0x43, 0xBB);	// throttle : 0x43 - 0x7F - 0xBB
			packet[15]= 0x40;	// trims
			packet[16]= 0x40;	// trims
			rudder--;													// rudder : 0x43 - 0x7F - 0xBB
			if (rudder>=0x7F-1 && rudder<=0x7F+1)
				rudder=0x7F;	// Small deadband
		}
		packet[10] = rudder;
		packet[11] = convert_channel_16b_limit(ELEVATOR, 0x43, 0xBB);	// elevator : 0x43 - 0x7F - 0xBB
		packet[12] = convert_channel_16b_limit(AILERON,  0x43, 0xBB);	// aileron  : 0x43 - 0x7F - 0xBB
		// neutral trims
		packet[13] = 0x20;
		packet[14] = 0x20;
		// flags
		packet[17] = 					  H8_3D_FLAG_RATE_HIGH
					| GET_FLAG(CH5_SW,H8_3D_FLAG_FLIP)
					| GET_FLAG(CH6_SW,H8_3D_FLAG_LIGTH) //H22 light
					| GET_FLAG(CH9_SW,H8_3D_FLAG_HEADLESS)
					| GET_FLAG(CH10_SW,H8_3D_FLAG_RTH); // 180/360 flip mode on H8 3D
		packet[18] =  GET_FLAG(CH7_SW,H8_3D_FLAG_PICTURE)
					| GET_FLAG(CH8_SW,H8_3D_FLAG_VIDEO)
					| GET_FLAG(CH11_SW,H8_3D_FLAG_CALIBRATE1)
					| GET_FLAG(CH12_SW,H8_3D_FLAG_CALIBRATE2);
		if(Channel_data[CH13]<CHANNEL_MIN_COMMAND)
			packet[18] |= H8_3D_FLAG_CAM_DN;
		if(CH13_SW)
			packet[18] |= H8_3D_FLAG_CAM_UP;
	}
	uint8_t  sum = packet[9];
	for (uint8_t i=10; i < H8_3D_PACKET_SIZE-1; i++)
		sum += packet[i];
	packet[19] = sum; // data checksum
	
	// Power on, TX mode, 2byte CRC
	// Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if(sub_protocol!=H20H)
	{ // H8_3D, H20MINI, H30MINI
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? hopping_frequency[0] : hopping_frequency[hopping_frequency_no++]);
		hopping_frequency_no %= H8_3D_RF_NUM_CHANNELS;
	}
	else
	{ //H20H
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? H20H_BIND_RF : hopping_frequency[packet_count>>3]);  
		if(!bind)
		{
			packet_count++;
			if(packet_count>15)
			{
				packet_count = 0;
				hopping_frequency_no = 0;
			}
			else
				if(packet_count > 7)
					hopping_frequency_no = 1;
		}
	}
	
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, H8_3D_PACKET_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) H8_3D_init()
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
	if(sub_protocol==H20H)
		XN297_SetTXAddr((uint8_t *)"\xEE\xDD\xCC\xBB\x11", 5);
	else // H8_3D, H20MINI, H30MINI
		XN297_SetTXAddr((uint8_t *)"\xC4\x57\x09\x65\x21", 5);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower();
}

uint16_t H8_3D_callback()
{
	if(IS_BIND_DONE)
		H8_3D_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			BIND_DONE;
			packet_count=0;
		}
		else
		{
			H8_3D_send_packet(1);
			bind_counter--;
		}
	}
	return	packet_period;
}

// captured from H20H stock transmitters
const uint8_t PROGMEM h20h_tx_rf_map[3][6] = {{/*ID*/0x83, 0x3c, 0x60, 0x00, /*RF*/0x47, 0x3e},
											  {/*ID*/0x5c, 0x2b, 0x60, 0x00, /*RF*/0x4a, 0x3c},
											  {/*ID*/0x57, 0x07, 0x00, 0x00, /*RF*/0x41, 0x48} };
// captured from H20 Mini / H30 Mini stock transmitters
const uint8_t PROGMEM h20mini_tx_rf_map[4][8] =  {{/*ID*/0xb4, 0xbb, 0x09, 0x00, /*RF*/0x3e, 0x45, 0x47, 0x4a},
												  {/*ID*/0x94, 0x9d, 0x0b, 0x00, /*RF*/0x3e, 0x43, 0x49, 0x4a},
												  {/*ID*/0xd1, 0xd0, 0x00, 0x00, /*RF*/0x3f, 0x42, 0x46, 0x4a},
												  {/*ID*/0xcb, 0xcd, 0x04, 0x00, /*RF*/0x41, 0x43, 0x46, 0x4a}};
static void __attribute__((unused)) H8_3D_initialize_txid()
{
	uint8_t id_num=rx_tx_addr[4];
	switch(sub_protocol)
	{
		case H8_3D:
            for(uint8_t i=0; i<4; i++)
                hopping_frequency[i] = 6 + (0x0f*i) + (((rx_tx_addr[i] >> 4) + (rx_tx_addr[i] & 0x0f)) % 0x0f);
			break;
		case H20H:
            id_num%=3; // 3 different IDs
			for(uint8_t i=0; i<4; i++)
			{
				rx_tx_addr[i] = pgm_read_byte_near(&h20h_tx_rf_map[id_num][i]);
				if(i<2)
					hopping_frequency[i] = pgm_read_byte_near(&h20h_tx_rf_map[id_num][i+4]);
			}
			break;
		case H20MINI:
		case H30MINI:
            id_num%=4; // 4 different IDs
			for(uint8_t i=0; i<4; i++)
			{
				rx_tx_addr[i] = pgm_read_byte_near(&h20mini_tx_rf_map[id_num][i]);
				hopping_frequency[i] = pgm_read_byte_near(&h20mini_tx_rf_map[id_num][i+4]);
			}
			break;
	}
}

uint16_t initH8_3D(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    bind_counter = H8_3D_BIND_COUNT;
	H8_3D_initialize_txid();
	H8_3D_init();
	switch(sub_protocol)
	{
        case H8_3D:
			packet_period=H8_3D_PACKET_PERIOD;
			break;
		case H20H:
			packet_period=H20H_PACKET_PERIOD;
			break;
		case H20MINI:
		case H30MINI:
			packet_period=H20MINI_PACKET_PERIOD;
			break;
	}
	return	H8_3D_INITIAL_WAIT;
}

#endif


# 1 "src/FY326_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/fy326_nrf24l01.c dated 2015-07-29

#if defined(FY326_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define FY326_INITIAL_WAIT		500
#define FY326_PACKET_PERIOD		1500
#define FY326_PACKET_CHKTIME	300
#define FY326_PACKET_SIZE		15
#define FY326_BIND_COUNT		16
#define FY326_RF_BIND_CHANNEL	0x17
#define FY326_NUM_RF_CHANNELS	5

enum {
    FY326_BIND1=0,
    FY326_BIND2,
    FY326_DATA,
    FY319_BIND1,
    FY319_BIND2,
};

#define rxid channel

#define CHAN_TO_TRIM(chanval) ((chanval/10)-10)
static void __attribute__((unused)) FY326_send_packet(uint8_t bind)
{
	packet[0] = rx_tx_addr[3];
	if(bind)
		packet[1] = 0x55;
	else
		packet[1] =	  GET_FLAG(CH7_SW,	0x80)	// Headless
					| GET_FLAG(CH6_SW,	0x40)	// RTH
					| GET_FLAG(CH5_SW,	0x02)	// Flip
					| GET_FLAG(CH9_SW,	0x01)	// Calibrate
					| GET_FLAG(CH8_SW,	0x04);	// Expert
	packet[2]  = convert_channel_16b_limit(AILERON, 0, 200);	// aileron
	packet[3]  = convert_channel_16b_limit(ELEVATOR, 0, 200);		// elevator
	packet[4]  = convert_channel_16b_limit(RUDDER, 0, 200);	// rudder
	packet[5]  = convert_channel_16b_limit(THROTTLE, 0, 200);		// throttle
	if(sub_protocol==FY319)
	{
		packet[6] = convert_channel_8b(AILERON);
		packet[7] = convert_channel_8b(ELEVATOR);
		packet[8] = convert_channel_8b(RUDDER);
	}
	else
	{
		packet[6]  = rx_tx_addr[0];
		packet[7]  = rx_tx_addr[1];
		packet[8]  = rx_tx_addr[2];
	}
	packet[9]  = CHAN_TO_TRIM(packet[2]);	// aileron_trim;
	packet[10] = CHAN_TO_TRIM(packet[3]);	// elevator_trim;
	packet[11] = CHAN_TO_TRIM(packet[4]);	// rudder_trim;
	packet[12] = 0;							// throttle_trim;
	packet[13] = rxid;
	packet[14] = rx_tx_addr[4];

	if (bind)
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, FY326_RF_BIND_CHANNEL);
	else
	{
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++]);
		hopping_frequency_no %= FY326_NUM_RF_CHANNELS;
	}

	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	NRF24L01_WritePayload(packet, FY326_PACKET_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) FY326_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	if(sub_protocol==FY319)
		NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // Five-byte rx/tx address
	else
		NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x01);   // Three-byte rx/tx address
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    (uint8_t *)"\x15\x59\x23\xc6\x29", 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t *)"\x15\x59\x23\xc6\x29", 5);
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, FY326_PACKET_SIZE);
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, FY326_RF_BIND_CHANNEL);
    NRF24L01_SetBitrate(NRF24L01_BR_250K);
    NRF24L01_SetPower();

	NRF24L01_Activate(0x73);
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3f);
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);
	NRF24L01_Activate(0x73);

	//Switch to RX
	NRF24L01_SetTxRxMode(TXRX_OFF);
	NRF24L01_FlushRx();
	NRF24L01_SetTxRxMode(RX_EN);
}

uint16_t FY326_callback()
{
	switch (phase)
	{
		case FY319_BIND1:
			if(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{
				NRF24L01_ReadPayload(packet, FY326_PACKET_SIZE);
				rxid = packet[13];
				packet[0] = rx_tx_addr[3];
				packet[1] = 0x80;
				packet[14]= rx_tx_addr[4];
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
				NRF24L01_FlushTx();
				bind_counter = 255;
				for(uint8_t i=2; i<6; i++)
					packet[i] = hopping_frequency[0];
				phase = FY319_BIND2;
			}
			return FY326_PACKET_CHKTIME;
			break;
		case FY319_BIND2:
			NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
			NRF24L01_FlushTx();
			NRF24L01_WritePayload(packet, FY326_PACKET_SIZE);
			if(bind_counter == 250)
				packet[1] = 0x40;
			if(--bind_counter == 0)
			{
				BIND_DONE;
				phase = FY326_DATA;
			}
			break;
		case FY326_BIND1:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				NRF24L01_ReadPayload(packet, FY326_PACKET_SIZE);
				rxid = packet[13];
				rx_tx_addr[0] = 0xAA;
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				BIND_DONE;
				phase = FY326_DATA;
			}
			else
				if (bind_counter-- == 0)
				{
					bind_counter = FY326_BIND_COUNT;
					NRF24L01_SetTxRxMode(TXRX_OFF);
					NRF24L01_SetTxRxMode(TX_EN);
					FY326_send_packet(1);
					phase = FY326_BIND2;
					return FY326_PACKET_CHKTIME;
				}
			break;
		case FY326_BIND2:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_TX_DS))
			{ // TX data sent -> switch to RX mode
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_FlushRx();
				NRF24L01_SetTxRxMode(RX_EN);
				phase = FY326_BIND1;
			}
			else
				return FY326_PACKET_CHKTIME;
			break;
		case FY326_DATA:
			FY326_send_packet(0);
			break;
	}
	return FY326_PACKET_PERIOD;
}

static void __attribute__((unused)) FY326_initialize_txid()
{
	hopping_frequency[0] = 		  (rx_tx_addr[0]&0x0f);
	hopping_frequency[1] = 0x10 + (rx_tx_addr[0] >> 4);
	hopping_frequency[2] = 0x20 + (rx_tx_addr[1]&0x0f);
	hopping_frequency[3] = 0x30 + (rx_tx_addr[1] >> 4);
	hopping_frequency[4] = 0x40 + (rx_tx_addr[2] >> 4);
	if(sub_protocol==FY319)
		for(uint8_t i=0;i<5;i++)
			hopping_frequency[i]=rx_tx_addr[0] & ~0x80;
}

uint16_t initFY326(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    rxid = 0xAA;
	bind_counter = FY326_BIND_COUNT;
	FY326_initialize_txid();
	FY326_init();
	if(sub_protocol==FY319)
	{
		phase=FY319_BIND1;
	}
	else
		phase=FY326_BIND1;
	return	FY326_INITIAL_WAIT;
}

#endif


# 1 "src/CABELL_nrf224l01.ino" // Helps debugging !
/*
 Protocol by Dennis Cabell, 2017
 KE8FZX
  
 To use this software, you must adhere to the license terms described below, and assume all responsibility for the use
 of the software.  The user is responsible for all consequences or damage that may result from using this software.
 The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that 
 any radio signal generated or received from use of this software is legal for that user to generate.  The author(s) of this software 
 assume no liability whatsoever.  The author(s) of this software is not responsible for legal or civil consequences of 
 using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software.  
 If this software is copied or modified, this disclaimer must accompany all copies.
 
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
 
// The Receiver for this protocol is available at: https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS

#if defined(CABELL_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CABELL_BIND_COUNT		2000		// At least 2000 so that if TX toggles the serial bind flag then bind mode is never exited
#define CABELL_PACKET_PERIOD	3000		// Do not set too low or else next packet may not be finished transmitting before the channel is changed next time around

#define CABELL_NUM_CHANNELS		16			// The maximum number of RC channels that can be sent in one packet
#define CABELL_MIN_CHANNELS		4			// The minimum number of channels that must be included in a packet, the number of channels cannot be reduced any further than this
#define CABELL_PAYLOAD_BYTES	24			// 12 bits per value * 16 channels

#define CABELL_RADIO_CHANNELS			9	// This is 1/5 of the total number of radio channels used for FHSS
#define CABELL_RADIO_MIN_CHANNEL_NUM	3	// Channel 0 is right on the boarder of allowed frequency range, so move up to avoid bleeding over
#define CABELL_TELEMETRY_PACKET_LENGTH	4

#define CABELL_BIND_RADIO_ADDR	0xA4B7C123F7LL

#define CABELL_OPTION_MASK_CHANNEL_REDUCTION		0x0F
#define CABELL_OPTION_MASK_RECIEVER_OUTPUT_MODE		0x30
#define CABELL_OPTION_SHIFT_RECIEVER_OUTPUT_MODE	4
#define CABELL_OPTION_MASK_MAX_POWER_OVERRIDE		0x40

typedef struct
{
   enum RxMode_t : uint8_t
   {   // Note bit 8 is used to indicate if the packet is the first of 2 on the channel.  Mask out this bit before using the enum
         normal                 = 0,
         bind                   = 1,
         setFailSafe            = 2,
         normalWithTelemetry    = 3,
         telemetryResponse      = 4,
         unBind                 = 127
   } RxMode;
   uint8_t reserved = 0;
   uint8_t option;
                          /*   mask 0x0F    : Channel reduction.  The number of channels to not send (subtracted from the 16 max channels) at least 4 are always sent
                           *   mask 0x30>>4 : Receiver output mode
                           *                  0 (00) = Single PPM on individual pins for each channel 
                           *                  1 (01) = SUM PPM on channel 1 pin
                           *                  2 (10) = Future use.  Reserved for SBUS output
                           *                  3 (11) = Unused
                           *   mask 0x40>>6   Contains max power override flag for Multi-protocol TX module. Also sent to RX
                           *   mask 0x80>>7   Unused 
                           */  
   uint8_t modelNum;
   uint8_t checkSum_LSB; 
   uint8_t checkSum_MSB; 
   uint8_t payloadValue [CABELL_PAYLOAD_BYTES] = {0}; //12 bits per channel value, unsigned
} CABELL_RxTxPacket_t;   

//-----------------------------------------------------------------------------------------
static uint8_t __attribute__((unused)) CABELL_getNextChannel (uint8_t seqArray[], uint8_t seqArraySize, uint8_t prevChannel)
{
	/* Possible channels are in 5 bands, each band comprised of seqArraySize channels
	* seqArray contains seqArraySize elements in the relative order in which we should progress through the band 
	* 
	* Each time the channel is changes, bands change in a way so that the next channel will be in a
	* different non-adjacent band. Both the band changes and the index in seqArray is incremented.
	*/
	prevChannel -= CABELL_RADIO_MIN_CHANNEL_NUM;				// Subtract CABELL_RADIO_MIN_CHANNEL_NUM because it was added to the return value
	if(prevChannel>(seqArraySize * 5))
		prevChannel=seqArraySize * 5;							// Constrain the values just in case something bogus was sent in.

	uint8_t currBand = prevChannel / seqArraySize;             
	uint8_t nextBand = (currBand + 3) % 5;

	uint8_t prevChannalSeqArrayValue = prevChannel % seqArraySize;
	uint8_t prevChannalSeqArrayPosition = 0;
	for (int x = 0; x < seqArraySize; x++)
	{	// Find the position of the previous channel in the array
		if (seqArray[x] == prevChannalSeqArrayValue)
			prevChannalSeqArrayPosition = x;
	}
	uint8_t nextChannalSeqArrayPosition = prevChannalSeqArrayPosition + 1;
	if (nextChannalSeqArrayPosition >= seqArraySize)
		nextChannalSeqArrayPosition = 0;

	return (seqArraySize * nextBand) + seqArray[nextChannalSeqArrayPosition] + CABELL_RADIO_MIN_CHANNEL_NUM;	// Add CABELL_RADIO_MIN_CHANNEL_NUM so we dont use channel 0 as it may bleed below 2.400 GHz
}

//-----------------------------------------------------------------------------------------
#if defined CABELL_HUB_TELEMETRY 
static void __attribute__((unused)) CABELL_get_telemetry()
{
	// calculate TX rssi based on past 250 expected telemetry packets.  Cannot use full second count because telemetry_counter is not large enough
	state++;
	if (state > 250)
	{
		TX_RSSI = telemetry_counter;
		telemetry_counter = 0;
		state = 0;
		telemetry_lost=0;
	}

	// Process incoming telemetry packet of it was received
	if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) 
	{ // data received from model
		NRF24L01_ReadPayload(packet, CABELL_TELEMETRY_PACKET_LENGTH);
		if ((packet[0] & 0x7F) == CABELL_RxTxPacket_t::telemetryResponse)	// ignore high order bit in compare because it toggles with each packet
		{
			RX_RSSI = packet[1];	// Packet rate 0 to 255 where 255 is 100% packet rate
			v_lipo1 = packet[2];	// Directly from analog input of receiver, but reduced to 8-bit depth (0 to 255).  Scaling depends on the input to the analog pin of the receiver.
			v_lipo2 = packet[3];	// Directly from analog input of receiver, but reduced to 8-bit depth (0 to 255).  Scaling depends on the input to the analog pin of the receiver.
			telemetry_counter++;      
			if(telemetry_lost==0)
				telemetry_link=1;
		}
	}
	else
	{
		// If no telemetry packet was received then delay by the typical telemetry packet processing time
		// This is done to try to keep the sendPacket process timing more consistent. Since the SPI payload read takes some time
		delayMicroseconds(50);
	}
	NRF24L01_SetTxRxMode(TX_EN);  
	NRF24L01_FlushRx(); 
}
#endif

//-----------------------------------------------------------------------------------------
static void __attribute__((unused)) CABELL_send_packet(uint8_t bindMode)
{  
	#if defined CABELL_HUB_TELEMETRY  
		if (!bindMode && (sub_protocol == CABELL_V3_TELEMETRY))		// check for incoming packet and switch radio back to TX mode if we were listening for telemetry      
			CABELL_get_telemetry();
	#endif

	CABELL_RxTxPacket_t TxPacket;

	uint8_t channelReduction = constrain((option & CABELL_OPTION_MASK_CHANNEL_REDUCTION),0,CABELL_NUM_CHANNELS-CABELL_MIN_CHANNELS);	// Max 12 - cannot reduce below 4 channels
	if (bindMode)
		channelReduction = 0;	// Send full packet to bind as higher channels will contain bind info

	uint8_t packetSize = sizeof(TxPacket) - ((((channelReduction - (channelReduction%2))/ 2)) * 3);		// reduce 3 bytes per 2 channels, but not last channel if it is odd
	uint8_t maxPayloadValueIndex = sizeof(TxPacket.payloadValue) - (sizeof(TxPacket) - packetSize);

	if ((sub_protocol == CABELL_UNBIND) && !bindMode)
	{
		TxPacket.RxMode = CABELL_RxTxPacket_t::unBind;
		TxPacket.option = option;
	}
	else
	{
		if (sub_protocol == CABELL_SET_FAIL_SAFE && !bindMode)
			TxPacket.RxMode = CABELL_RxTxPacket_t::setFailSafe;
		else
		{
			if (bindMode)
				TxPacket.RxMode = CABELL_RxTxPacket_t::bind;        
			else
			{
				switch (sub_protocol)
				{
					case CABELL_V3_TELEMETRY:
						TxPacket.RxMode = CABELL_RxTxPacket_t::normalWithTelemetry;
						break;
					default:
						TxPacket.RxMode = CABELL_RxTxPacket_t::normal;  
						break;
				}      
			}
		}
		TxPacket.option = (bindMode) ? (option & (~CABELL_OPTION_MASK_CHANNEL_REDUCTION)) : option;		//remove channel reduction if in bind mode
	}
	TxPacket.reserved = 0;
	TxPacket.modelNum = RX_num;
	uint16_t checkSum = TxPacket.modelNum + TxPacket.option + TxPacket.RxMode  + TxPacket.reserved;		// Start Calculate checksum

	int adjusted_x;
	int payloadIndex = 0;
	uint16_t holdValue;

	for (int x = 0;(x < CABELL_NUM_CHANNELS - channelReduction); x++)
	{
		switch (x)
		{
			case 0	: adjusted_x = ELEVATOR;	break;
			case 1	: adjusted_x = AILERON;		break;
			case 2	: adjusted_x = RUDDER;		break;
			case 3	: adjusted_x = THROTTLE;	break;
			default	: adjusted_x = x;			break;
		}
		holdValue = convert_channel_16b_limit(adjusted_x,1000,2000);				// valid channel values are 1000 to 2000
		if (bindMode)
		{
			switch (adjusted_x)
			{
				case THROTTLE	: holdValue = 1000;					break;      // always set throttle to off when binding for safety
				//tx address sent for bind
				case 11			: holdValue = 1000 + rx_tx_addr[0];	break;
				case 12			: holdValue = 1000 + rx_tx_addr[1];	break;
				case 13			: holdValue = 1000 + rx_tx_addr[2];	break;
				case 14			: holdValue = 1000 + rx_tx_addr[3];	break;
				case 15			: holdValue = 1000 + rx_tx_addr[4];	break;
			}
		}

		// use 12 bits per value
		if (x % 2)
		{ //output channel number is ODD
			holdValue = holdValue<<4;
			payloadIndex--;     
		}
		else
			holdValue &= 0x0FFF;
		TxPacket.payloadValue[payloadIndex] |=  (uint8_t)(holdValue & 0x00FF);
		payloadIndex++;
		TxPacket.payloadValue[payloadIndex] |=  (uint8_t)((holdValue>>8) & 0x00FF);   
		payloadIndex++;
	}

	for(int x = 0; x < maxPayloadValueIndex ; x++)
		checkSum += TxPacket.payloadValue[x];  // Finish Calculate checksum 

	TxPacket.checkSum_MSB = checkSum >> 8;
	TxPacket.checkSum_LSB = checkSum & 0x00FF;

	// Set channel for next transmission
	rf_ch_num = CABELL_getNextChannel (hopping_frequency,CABELL_RADIO_CHANNELS, rf_ch_num);
	NRF24L01_WriteReg(NRF24L01_05_RF_CH,rf_ch_num); 

	//NRF24L01_FlushTx();   //just in case things got hung up
	//NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);

	uint8_t* p = reinterpret_cast<uint8_t*>(&TxPacket.RxMode);
	*p &= 0x7F;                  // Make sure 8th bit is clear
	*p |= (packet_count++)<<7;   // This causes the 8th bit of the first byte to toggle with each xmit so consecutive payloads are not identical.
	// This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.

	CABELL_SetPower();
	NRF24L01_WritePayload((uint8_t*)&TxPacket, packetSize);

	#if defined CABELL_HUB_TELEMETRY 
		if (!bindMode && (sub_protocol == CABELL_V3_TELEMETRY))
		{ // switch radio to rx as soon as packet is sent  
			// calculate transmit time based on packet size and data rate of 1MB per sec
			// This is done because polling the status register during xmit caused issues.
			// bits = packst_size * 8  +  73 bits overhead
			// at 250 Kbs per sec, one bit is 4 uS
			// then add 140 uS which is 130 uS to begin the xmit and 10 uS fudge factor
			delayMicroseconds(((((unsigned long)packetSize * 8ul)  +  73ul) * 4ul) + 140ul)   ;
			packet_period = CABELL_PACKET_PERIOD + (constrain(((int16_t)(CABELL_NUM_CHANNELS - channelReduction) - (int16_t)6 ),(int16_t)0 ,(int16_t)10 ) * (int16_t)100);  // increase packet period by 100 us for each channel over 6
			NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);  // RX mode with 16 bit CRC
		}
		else 
	#endif
			packet_period = CABELL_PACKET_PERIOD;   // Standard packet period when not in telemetry mode.
}

//-----------------------------------------------------------------------------------------
static void __attribute__((unused)) CABELL_getChannelSequence (uint8_t outArray[], uint8_t numChannels, uint64_t permutation)
{
	/* This procedure initializes an array with the sequence progression of channels.
	* This is not the actual channels itself, but the sequence base to be used within bands of 
	* channels.
	* 
	* There are numChannels! permutations for arranging the channels
	* one of these permutations will be calculated based on the permutation input
	* permutation should be between 1 and numChannels! but the routine will constrain it
	* if these bounds are exceeded.  Typically the radio's unique TX ID should be used.
	* 
	* The maximum numChannels is 20.  Anything larger than this will cause the uint64_t
	* variables to overflow, yielding unknown results (possibly infinite loop?).  Therefor
	* this routine constrains the value.
	*/  
	uint8_t i;   //iterator counts numChannels
	uint64_t indexOfNextSequenceValue;
	uint64_t numChannelsFactorial=1;
	uint8_t  sequenceValue;

	numChannels = constrain(numChannels,1,20);

	for (i = 1; i <= numChannels;i++)
	{
		numChannelsFactorial *= i;      //  Calculate n!
		outArray[i-1] = i-1;            //  Initialize array with the sequence
	}

	permutation = (permutation % numChannelsFactorial) + 1;    // permutation must be between 1 and n! or this algorithm will infinite loop

	//Rearrange the array elements based on the permutation selected
	for (i=0, permutation--; i<numChannels; i++ )
	{
		numChannelsFactorial /= ((uint64_t)numChannels)-i;
		indexOfNextSequenceValue = i+(permutation/numChannelsFactorial);
		permutation %= numChannelsFactorial;

		//Copy the value in the selected array position
		sequenceValue = outArray[indexOfNextSequenceValue];

		//Shift the unused elements in the array to make room to move in the one just selected
		for( ; indexOfNextSequenceValue > i; indexOfNextSequenceValue--)
			outArray[indexOfNextSequenceValue] = outArray[indexOfNextSequenceValue-1];

		// Copy the selected value into it's new array slot
		outArray[i] = sequenceValue;
	}
}

//-----------------------------------------------------------------------------------------
static void __attribute__((unused)) CABELL_setAddress()
{
	uint64_t CABELL_addr;

	//  Serial.print("NORM ID: ");Serial.print((uint32_t)(CABELL_normal_addr>>32)); Serial.print("    ");Serial.println((uint32_t)((CABELL_normal_addr<<32)>>32));

	if (IS_BIND_DONE)
	{
		CABELL_addr = (((uint64_t)rx_tx_addr[0]) << 32) + 
			(((uint64_t)rx_tx_addr[1]) << 24) + 
			(((uint64_t)rx_tx_addr[2]) << 16) + 
			(((uint64_t)rx_tx_addr[3]) << 8) + 
			(((uint64_t)rx_tx_addr[4]));					// Address to use after binding
	}
	else
		CABELL_addr = CABELL_BIND_RADIO_ADDR;				// Static addr for binding

	CABELL_getChannelSequence(hopping_frequency,CABELL_RADIO_CHANNELS,CABELL_addr);		// Get the sequence for hopping through channels
	rf_ch_num = CABELL_RADIO_MIN_CHANNEL_NUM;				// Initialize the channel sequence

	packet_count=0;  

	uint64_t CABELL_Telemetry_addr = ~CABELL_addr;			// Invert bits for reading so that telemetry packets have a different address.

	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, reinterpret_cast<uint8_t*>(&CABELL_Telemetry_addr), 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, reinterpret_cast<uint8_t*>(&CABELL_Telemetry_addr), 5);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR,    reinterpret_cast<uint8_t*>(&CABELL_addr), 5);
}

//-----------------------------------------------------------------------------------------
static void __attribute__((unused)) CABELL_init()
{
	NRF24L01_Initialize();
	CABELL_SetPower();
	NRF24L01_SetBitrate(NRF24L01_BR_250K);				// slower data rate gives better range/reliability
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);			// No Auto Acknowledgment on all data pipes  
	NRF24L01_SetTxRxMode(TX_EN);						//Power up and 16 bit CRC

	CABELL_setAddress();

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);		// 32 byte packet length
	NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, 0x20);		// 32 byte packet length
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x5F);	// no retransmits
	NRF24L01_Activate(0x73);							// Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);			// Enable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x04);		// Enable dynamic Payload Length
	NRF24L01_Activate(0x73);
	prev_power = NRF_POWER_0;
}

//-----------------------------------------------------------------------------------------
static void CABELL_SetPower()    // This over-ride the standard Set Power to allow an flag in option to indicate max power setting
                          // Note that on many modules max power may actually be worse than the normal high power setting
                          // test and only use max if it helps the range
{
	if(IS_BIND_DONE && !IS_RANGE_FLAG_on && ((option & CABELL_OPTION_MASK_MAX_POWER_OVERRIDE) != 0))
	{   // If we are not in range or bind mode and power setting override is in effect, then set max power, else standard power logic
		if(prev_power != NRF_POWER_3)   // prev_power is global variable for NRF24L01; NRF_POWER_3 is max power
		{
			uint8_t rf_setup = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP);
			rf_setup = (rf_setup & 0xF9) | (NRF_POWER_3 << 1);
			NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
			prev_power=NRF_POWER_3;
		}    
	}
	else
		NRF24L01_SetPower();
}

//-----------------------------------------------------------------------------------------
uint16_t CABELL_callback()
{
	if (IS_BIND_DONE)
	{
		CABELL_send_packet(0);  // packet_period is set/adjusted in CABELL_send_packet
		return packet_period;
	}
	if (bind_counter == 0)
	{
		BIND_DONE;
		CABELL_init();   // non-bind address 
	}
	else
	{
		CABELL_send_packet(1);
		bind_counter--;
	}
	return CABELL_PACKET_PERIOD;
}

//-----------------------------------------------------------------------------------------
uint16_t initCABELL(void)
{
	if (IS_BIND_DONE)
		bind_counter = 0;
	else  
		bind_counter = CABELL_BIND_COUNT;
	CABELL_init();
	#if defined CABELL_HUB_TELEMETRY 
		init_frskyd_link_telemetry();
		telemetry_lost=1;				// do not send telemetry to TX right away until we have a TX_RSSI value to prevent warning message...
	#endif

	packet_period = CABELL_PACKET_PERIOD;

	return packet_period;
}

#endif

# 1 "src/FrSkyV_cc2500.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(FRSKYV_CC2500_INO)

#define FRSKYV_BIND_COUNT 200

enum {
	FRSKYV_DATA1=0,
	FRSKYV_DATA2,
	FRSKYV_DATA3,
	FRSKYV_DATA4,
	FRSKYV_DATA5
};


#include "iface_cc2500.h"
static uint8_t __attribute__((unused)) FRSKYV_crc8(uint8_t result, uint8_t *data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		result = result ^ data[i];
		for(uint8_t j = 0; j < 8; j++)
			if(result & 0x80)
				result = (result << 1) ^ 0x07;
			else
				result = result << 1;
	}
	return result;
}

static uint8_t __attribute__((unused)) FRSKYV_crc8_le(uint8_t *data, uint8_t len)
{
	uint8_t result = 0xD6;

	for(uint8_t i = 0; i < len; i++)
	{
		result = result ^ data[i];
		for(uint8_t j = 0; j < 8; j++)
			if(result & 0x01)
				result = (result >> 1) ^ 0x83;
			else
				result = result >> 1;
	}
	return result;
}

static void __attribute__((unused)) FRSKYV_build_bind_packet()
{
    //0e 03 01 57 12 00 06 0b 10 15 1a 00 00 00 61
    packet[0] = 0x0e;                //Length
    packet[1] = 0x03;                //Packet type
    packet[2] = 0x01;                //Packet type
    packet[3] = rx_tx_addr[3];
    packet[4] = rx_tx_addr[2];
    packet[5] = (binding_idx % 10) * 5;
    packet[6] = packet[5] * 5 + 6;
    packet[7] = packet[5] * 5 + 11;
    packet[8] = packet[5] * 5 + 16;
    packet[9] = packet[5] * 5 + 21;
    packet[10] = packet[5] * 5 + 26;
    packet[11] = 0x00;
    packet[12] = 0x00;
    packet[13] = 0x00;
    packet[14] = FRSKYV_crc8(0x93, packet, 14);
}

static uint8_t __attribute__((unused)) FRSKYV_calc_channel()
{
	uint32_t temp=seed;
	temp = (temp * 0xaa) % 0x7673;
	seed = temp;
	return (seed & 0xff) % 0x32;
}

static void __attribute__((unused)) FRSKYV_build_data_packet()
{
	uint8_t idx = 0;			// transmit lower channels
	
	packet[0] = 0x0e;
	packet[1] = rx_tx_addr[3];
	packet[2] = rx_tx_addr[2];
	packet[3] = seed & 0xff;
	packet[4] = seed >> 8;
	if (phase == FRSKYV_DATA1 || phase == FRSKYV_DATA3)
		packet[5] = 0x0f;
	else
		if(phase == FRSKYV_DATA2 || phase == FRSKYV_DATA4)
		{
			packet[5] = 0xf0;
			idx=4;				// transmit upper channels
		}
		else
			packet[5] = 0x00;
	for(uint8_t i = 0; i < 4; i++)
	{
		uint16_t value = convert_channel_frsky(i+idx);
		packet[2*i + 6] = value & 0xff;
		packet[2*i + 7] = value >> 8;
	}
	packet[14] = FRSKYV_crc8(crc8, packet, 14);
}

uint16_t ReadFRSKYV()
{
	if(IS_BIND_DONE)
	{	// Normal operation
		uint8_t chan = FRSKYV_calc_channel();
		CC2500_Strobe(CC2500_SIDLE);
		if (option != prev_option)
		{
			CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
			prev_option=option;
		}
		CC2500_WriteReg(CC2500_0A_CHANNR, chan * 5 + 6);
		FRSKYV_build_data_packet();

		if (phase == FRSKYV_DATA5)
		{
			CC2500_SetPower();
			phase = FRSKYV_DATA1;
		}
		else
			phase++;

		CC2500_WriteData(packet, packet[0]+1);
		return 9006;
	}
	// Bind mode
	FRSKYV_build_bind_packet();
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_WriteReg(CC2500_0A_CHANNR, 0x00);
	CC2500_WriteData(packet, packet[0]+1);
	binding_idx++;
	if(binding_idx>=FRSKYV_BIND_COUNT)
		BIND_DONE;
	return 53460;
}

uint16_t initFRSKYV()
{
	//ID is 15 bits. Using rx_tx_addr[2] and rx_tx_addr[3] since we want to use RX_Num for model match
	rx_tx_addr[2]&=0x7F;
	crc8 = FRSKYV_crc8_le(rx_tx_addr+2, 2);

	FRSKY_init_cc2500(FRSKYV_cc2500_conf);
	seed = 1;
	binding_idx=0;
	phase = FRSKYV_DATA1;
	return 10000;
}

#endif


# 1 "src/FQ777_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with bikemike FQ777-124.ino

#if defined(FQ777_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define FQ777_INITIAL_WAIT		500
#define FQ777_PACKET_PERIOD		2000
#define FQ777_PACKET_SIZE		8
#define FQ777_BIND_COUNT		1000
#define FQ777_NUM_RF_CHANNELS	4

enum {
	FQ777_FLAG_RETURN     = 0x40,  // 0x40 when not off, !0x40 when one key return
	FQ777_FLAG_HEADLESS   = 0x04,
	FQ777_FLAG_EXPERT     = 0x01,
	FQ777_FLAG_FLIP       = 0x80,
};

const uint8_t ssv_xor[] = {0x80,0x44,0x64,0x75,0x6C,0x71,0x2A,0x36,0x7C,0xF1,0x6E,0x52,0x9,0x9D,0x1F,0x78,0x3F,0xE1,0xEE,0x16,0x6D,0xE8,0x73,0x9,0x15,0xD7,0x92,0xE7,0x3,0xBA};
uint8_t FQ777_bind_addr []   = {0xe7,0xe7,0xe7,0xe7,0x67};

static void __attribute__((unused)) ssv_pack_dpl(uint8_t addr[], uint8_t pid, uint8_t* len, uint8_t* payload, uint8_t* packed_payload)
{
	uint8_t i = 0;

	uint16_t pcf = (*len & 0x3f) << 3;
	pcf |= (pid & 0x3) << 1;
	pcf |= 0x00; // noack field
	
	uint8_t header[7] = {0};
	header[6] = pcf;
	header[5] = (pcf >> 7) | (addr[0] << 1);
	header[4] = (addr[0] >> 7) | (addr[1] << 1);
	header[3] = (addr[1] >> 7) | (addr[2] << 1);
	header[2] = (addr[2] >> 7) | (addr[3] << 1);
	header[1] = (addr[3] >> 7) | (addr[4] << 1);
	header[0] = (addr[4] >> 7);

	// calculate the crc
	union 
	{
		uint8_t bytes[2];
		uint16_t val;
	} crc;

	crc.val=0x3c18;
	for (i = 0; i < 7; ++i)
		crc.val=crc16_update(crc.val,header[i],8);
	for (i = 0; i < *len; ++i)
		crc.val=crc16_update(crc.val,payload[i],8);

	// encode payload and crc
	// xor with this:
	for (i = 0; i < *len; ++i)
		payload[i] ^= ssv_xor[i];
	crc.bytes[1] ^= ssv_xor[i++];
	crc.bytes[0] ^= ssv_xor[i++];

	// pack the pcf, payload, and crc into packed_payload
	packed_payload[0] = pcf >> 1;
	packed_payload[1] = (pcf << 7) | (payload[0] >> 1);
	
	for (i = 0; i < *len - 1; ++i)
		packed_payload[i+2] = (payload[i] << 7) | (payload[i+1] >> 1);

	packed_payload[i+2] = (payload[i] << 7) | (crc.val >> 9);
	++i;
	packed_payload[i+2] = (crc.val >> 1 & 0x80 ) | (crc.val >> 1 & 0x7F);
	++i;
	packed_payload[i+2] = (crc.val << 7);

	*len += 4;
}

static void __attribute__((unused)) FQ777_send_packet(uint8_t bind)
{
	uint8_t packet_len = FQ777_PACKET_SIZE;
	uint8_t packet_ori[8];
	if (bind)
	{
		// 4,5,6 = address fields
		// last field is checksum of address fields
		packet_ori[0] = 0x20;
		packet_ori[1] = 0x15;
		packet_ori[2] = 0x05;
		packet_ori[3] = 0x06;
		packet_ori[4] = rx_tx_addr[0];
		packet_ori[5] = rx_tx_addr[1];
		packet_ori[6] = rx_tx_addr[2];
		packet_ori[7] = packet_ori[4] + packet_ori[5] + packet_ori[6];
	}
	else
	{
		// throt, yaw, pitch, roll, trims, flags/left button,00,right button
		//0-3 0x00-0x64
		//4 roll/pitch/yaw trims. cycles through one trim at a time - 0-40 trim1, 40-80 trim2, 80-C0 trim3 (center:  A0 20 60)
		//5 flags for throttle button, two buttons above throttle - def: 0x40
		//6 00 ??
		//7 checksum - add values in other fields 

		
		// Trims are usually done through the radio configuration but leaving the code here just in case...
		uint8_t trim_mod  = packet_count % 144;
		uint8_t trim_val  = 0;
		if (36 <= trim_mod && trim_mod < 72) // yaw
			trim_val  = 0x20; // don't modify yaw trim
		else
			if (108 < trim_mod && trim_mod) // pitch
				trim_val = 0xA0;
			else // roll
				trim_val = 0x60;

		packet_ori[0] = convert_channel_16b_limit(THROTTLE,0,0x64);
		packet_ori[1] = convert_channel_16b_limit(RUDDER,0,0x64);
		packet_ori[2] = convert_channel_16b_limit(ELEVATOR,0,0x64);
		packet_ori[3] = convert_channel_16b_limit(AILERON,0,0x64);
		packet_ori[4] = trim_val; // calculated above
		packet_ori[5] = GET_FLAG(CH5_SW, FQ777_FLAG_FLIP)
				  | GET_FLAG(CH7_SW, FQ777_FLAG_HEADLESS)
				  | GET_FLAG(!CH6_SW, FQ777_FLAG_RETURN)
				  | GET_FLAG(CH8_SW,FQ777_FLAG_EXPERT);
		packet_ori[6] = 0x00;
		// calculate checksum
		uint8_t checksum = 0;
		for (int i = 0; i < 7; ++i)
			checksum += packet_ori[i];
		packet_ori[7] = checksum;

		packet_count++;
	}

	ssv_pack_dpl( (0 == bind) ? rx_tx_addr : FQ777_bind_addr, hopping_frequency_no, &packet_len, packet_ori, packet);
	
	NRF24L01_WriteReg(NRF24L01_00_CONFIG,_BV(NRF24L01_00_PWR_UP));
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++]);
	hopping_frequency_no %= FQ777_NUM_RF_CHANNELS;
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	NRF24L01_WritePayload(packet, packet_len);
	NRF24L01_WritePayload(packet, packet_len);
	NRF24L01_WritePayload(packet, packet_len);
}

static void __attribute__((unused)) FQ777_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, FQ777_bind_addr, 5);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowledgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x00);
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
	NRF24L01_SetBitrate(NRF24L01_BR_250K);
	NRF24L01_SetPower();
    NRF24L01_Activate(0x73);                         // Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
}

uint16_t FQ777_callback()
{
	if(bind_counter!=0)
	{
		FQ777_send_packet(1);
		bind_counter--;
		if (bind_counter == 0)
		{
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
			BIND_DONE;
		}
	}
	else
		FQ777_send_packet(0);
	return FQ777_PACKET_PERIOD;
}

uint16_t initFQ777(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
	bind_counter = FQ777_BIND_COUNT;
	packet_count=0;
	hopping_frequency[0] = 0x4D;
	hopping_frequency[1] = 0x43;
	hopping_frequency[2] = 0x27;
	hopping_frequency[3] = 0x07;
	hopping_frequency_no=0;
	rx_tx_addr[2] = 0x00;
	rx_tx_addr[3] = 0xe7;
	rx_tx_addr[4] = 0x67;
	FQ777_init();
	return	FQ777_INITIAL_WAIT;
}

#endif


# 1 "src/GW008_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Compatible with Global Drone GW008 protocol.
// There are 3 versions of this small quad, this protocol is for the one with a XNS104 IC in the stock Tx and PAN159CY IC in the quad (SOCs with built-in xn297 compatible RF).
// The xn297 version is compatible with the CX10 protocol (green pcb).
// The LT8910 version is not supported yet.

#if defined(GW008_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define GW008_INITIAL_WAIT    500
#define GW008_PACKET_PERIOD   2400
#define GW008_RF_BIND_CHANNEL 2
#define GW008_PAYLOAD_SIZE    15

enum {
	GW008_BIND1,
	GW008_BIND2,
	GW008_DATA
};

static void __attribute__((unused)) GW008_send_packet(uint8_t bind)
{
	packet[0] = rx_tx_addr[0];
	if(bind)
	{
		packet[1] = 0x55;
		packet[2] = hopping_frequency[0];
		packet[3] = hopping_frequency[1];
		packet[4] = hopping_frequency[2];
		packet[5] = hopping_frequency[3];
		memset(&packet[6], 0, 7);
		packet[13] = 0xaa;
	}
	else
	{
		packet[1] = 0x01 | GET_FLAG(CH5, 0x40); // flip
		packet[2] = convert_channel_16b_limit(AILERON , 200, 0); // aileron
		packet[3] = convert_channel_16b_limit(ELEVATOR, 0, 200); // elevator
		packet[4] = convert_channel_16b_limit(RUDDER  , 200, 0); // rudder
		packet[5] = convert_channel_16b_limit(THROTTLE, 0, 200); // throttle
		packet[6] = 0xaa;
		packet[7] = 0x02; // max rate
		packet[8] = 0x00;
		packet[9] = 0x00;
		packet[10]= 0x00;
		packet[11]= 0x00;
		packet[12]= 0x00;
		packet[13]= rx_tx_addr[2];
	}
	packet[14] = rx_tx_addr[1];

	// Power on, TX mode, CRC enabled
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? GW008_RF_BIND_CHANNEL : hopping_frequency[(hopping_frequency_no++)/2]);
	hopping_frequency_no %= 8;

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WriteEnhancedPayload(packet, GW008_PAYLOAD_SIZE, 0, 0x3c7d);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) GW008_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t*)"\xcc\xcc\xcc\xcc\xcc", 5);
	XN297_SetRXAddr((uint8_t*)"\xcc\xcc\xcc\xcc\xcc", 5);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, GW008_PAYLOAD_SIZE+2); // payload + 2 bytes for pcf
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); // no retransmits
	NRF24L01_SetBitrate(NRF24L01_BR_1M);
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);                         // Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      // Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);    // Set feature bits on
	NRF24L01_Activate(0x73);
}

static void __attribute__((unused)) GW008_initialize_txid()
{
	uint32_t lfsr = random(0xfefefefe) + ((uint32_t)random(0xfefefefe) << 16);
    for(uint8_t i=0; i<4; i++)
        hopping_frequency[i] = 0x10 + ((lfsr >> (i*8)) % 0x37);
}

uint16_t GW008_callback()
{
	switch(phase)
	{
		case GW008_BIND1:
			if((NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR)) &&	// RX fifo data ready
				XN297_ReadEnhancedPayload(packet, GW008_PAYLOAD_SIZE) == GW008_PAYLOAD_SIZE &&	// check payload size
				packet[0] == rx_tx_addr[0] && packet[14] == rx_tx_addr[1])			// check tx id
			{
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				rx_tx_addr[2] = packet[13];
				BIND_DONE;
				phase = GW008_DATA;
			}
			else
			{
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				GW008_send_packet(1);
				phase = GW008_BIND2;
				return 300;
			}
			break;
		case GW008_BIND2:
			// switch to RX mode
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_FlushRx();
			NRF24L01_SetTxRxMode(RX_EN);
			XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) 
						| _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX)); 
			phase = GW008_BIND1;
			return 5000;
			break;
		case GW008_DATA:
			GW008_send_packet(0);
			break;
	}
	return GW008_PACKET_PERIOD;
}

uint16_t initGW008()
{
	BIND_IN_PROGRESS;	// autobind protocol
	GW008_initialize_txid();
	phase = GW008_BIND1;
	GW008_init();
	hopping_frequency_no = 0;
	return GW008_INITIAL_WAIT;
}

#endif


# 1 "src/Telemetry.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
//**************************
// Telemetry serial code   *
//**************************
#if defined TELEMETRY

uint8_t RetrySequence ;

#if ( defined(MULTI_TELEMETRY) || defined(MULTI_STATUS) )
	#define MULTI_TIME				500	//in ms
	#define INPUT_SYNC_TIME			100	//in ms
	#define INPUT_ADDITIONAL_DELAY	100	// in 10µs, 100 => 1000 µs
	uint32_t lastMulti = 0;
#endif // MULTI_TELEMETRY/MULTI_STATUS

#if defined SPORT_TELEMETRY	
	#define SPORT_TIME 12000	//12ms
	#define FRSKY_SPORT_PACKET_SIZE   8
	#define FX_BUFFERS	4
	uint32_t last = 0;
	uint8_t sport_counter=0;
	uint8_t RxBt = 0;
	uint8_t sport = 0;
	uint8_t pktx1[FRSKY_SPORT_PACKET_SIZE*FX_BUFFERS];

	// Store for out of sequence packet
	uint8_t FrskyxRxTelemetryValidSequence ;
	struct t_fx_rx_frame
	{
		uint8_t valid ;
		uint8_t count ;
		uint8_t payload[6] ;
	} ;

	// Store for FrskyX telemetry
	struct t_fx_rx_frame FrskyxRxFrames[4] ;
	uint8_t NextFxFrameToForward ;
	#ifdef SPORT_POLLING
		uint8_t sport_rx_index[28] ;
		uint8_t ukindex ;
		uint8_t kindex ;
		uint8_t TxData[2];
		uint8_t SportIndexPolling;
		uint8_t RxData[16] ;
		volatile uint8_t RxIndex=0 ;
		uint8_t sport_bytes=0;
		uint8_t skipped_id;
		uint8_t rx_counter=0;
	#endif
#endif // SPORT_TELEMETRY

#if defined HUB_TELEMETRY
	#define USER_MAX_BYTES 6
	uint8_t prev_index;
#endif // HUB_TELEMETRY

#define START_STOP	0x7e
#define BYTESTUFF	0x7d
#define STUFF_MASK	0x20
#define MAX_PKTX	10
uint8_t pktx[MAX_PKTX];
uint8_t indx;
uint8_t frame[18];

#if ( defined(MULTI_TELEMETRY) || defined(MULTI_STATUS) )
static void multi_send_header(uint8_t type, uint8_t len)
{
	Serial_write('M');
	#ifdef MULTI_TELEMETRY
		Serial_write('P');
		Serial_write(type);
	#else
		(void)type;
	#endif
	Serial_write(len);
}

static void multi_send_status()
{
	#ifdef SPORT_POLLING
	#ifdef INVERT_SERIAL
		USART3_BASE->CR1 &= ~USART_CR1_TE ;
		TX_INV_on;	//activate inverter for both serial TX and RX signals
		USART3_BASE->CR1 |= USART_CR1_TE ;
	#endif
		rx_pause();
	#endif
	multi_send_header(MULTI_TELEMETRY_STATUS, 5);

	// Build flags
	uint8_t flags=0;
	if (IS_INPUT_SIGNAL_on)
		flags |= 0x01;
	if (mode_select==MODE_SERIAL)
		flags |= 0x02;
	if (remote_callback != 0)
	{
		flags |= 0x04;
		if (IS_WAIT_BIND_on)
			flags |= 0x10;
		else
			if (IS_BIND_IN_PROGRESS)
				flags |= 0x08;
		#ifdef FAILSAFE_ENABLE
			//Is failsafe supported?
			switch (protocol)
			{
				case PROTO_HISKY:
					if(sub_protocol!=HK310)
						break;
				case PROTO_AFHDS2A:
				case PROTO_DEVO:
				case PROTO_SFHSS:
				case PROTO_WK2x01:
				case PROTO_FRSKYX:
					flags |= 0x20;	//Yes
				default:
					break;
			}
		#endif
	}
	Serial_write(flags);

	// Version number example: 1.1.6.1
	Serial_write(VERSION_MAJOR);
	Serial_write(VERSION_MINOR);
	Serial_write(VERSION_REVISION);
	Serial_write(VERSION_PATCH_LEVEL);
}
#endif

#ifdef DSM_TELEMETRY
	#ifdef MULTI_TELEMETRY
		void DSM_frame()
		{
			if (pkt[0] == 0x80)
			{
				multi_send_header(MULTI_TELEMETRY_DSMBIND, 10);
				for (uint8_t i = 1; i < 11; i++) 	// 10 bytes of DSM bind response
					Serial_write(pkt[i]);

			}
			else
			{
				multi_send_header(MULTI_TELEMETRY_DSM, 17);
				for (uint8_t i = 0; i < 17; i++)	// RSSI value followed by 16 bytes of telemetry data
					Serial_write(pkt[i]);
			}
		}
	#else
		void DSM_frame()
		{
			Serial_write(0xAA);						// Telemetry packet
			for (uint8_t i = 0; i < 17; i++)		// RSSI value followed by 16 bytes of telemetry data
				Serial_write(pkt[i]);
		}
	#endif
#endif

#ifdef AFHDS2A_FW_TELEMETRY
	void AFHDSA_short_frame()
	{
		#if defined MULTI_TELEMETRY
			multi_send_header(MULTI_TELEMETRY_AFHDS2A, 29);
		#else
			Serial_write(0xAA);						// Telemetry packet
		#endif
		for (uint8_t i = 0; i < 29; i++)			// RSSI value followed by 4*7 bytes of telemetry data
			Serial_write(pkt[i]);
	}
#endif

#ifdef HITEC_FW_TELEMETRY
	void HITEC_short_frame()
	{
		#if defined MULTI_TELEMETRY
			multi_send_header(MULTI_TELEMETRY_HITEC, 8);
		#else
			Serial_write(0xAA);					// Telemetry packet
		#endif
		for (uint8_t i = 0; i < 8; i++)			// TX RSSI and TX LQI values followed by frame number and 5 bytes of telemetry data
			Serial_write(pkt[i]);
	}
#endif

#ifdef MULTI_TELEMETRY
static void multi_send_frskyhub()
{
	multi_send_header(MULTI_TELEMETRY_HUB, 9);
	for (uint8_t i = 0; i < 9; i++)
		Serial_write(frame[i]);
}
#endif

void frskySendStuffed()
{
	Serial_write(START_STOP);
	for (uint8_t i = 0; i < 9; i++)
	{
		if ((frame[i] == START_STOP) || (frame[i] == BYTESTUFF))
		{
			Serial_write(BYTESTUFF);
			frame[i] ^= STUFF_MASK;
		}
		Serial_write(frame[i]);
	}
	Serial_write(START_STOP);
}

void frsky_check_telemetry(uint8_t *pkt,uint8_t len)
{
	uint8_t clen = pkt[0] + 3 ;
	if(pkt[1] == rx_tx_addr[3] && pkt[2] == rx_tx_addr[2] && len == clen )
	{
		telemetry_link|=1;								// Telemetry data is available
		TX_RSSI = pkt[len-2];
		if(TX_RSSI >=128)
			TX_RSSI -= 128;
		else
			TX_RSSI += 128;
		TX_LQI = pkt[len-1]&0x7F;
		for (uint8_t i=3;i<len-2;i++)
			pktt[i]=pkt[i];								// Buffer telemetry values to be sent 
		
		if(pktt[6]>0 && pktt[6]<=10)
		{
			if (protocol==PROTO_FRSKYD)
			{
				if ( ( pktt[7] & 0x1F ) == (telemetry_counter & 0x1F) )
				{
					uint8_t topBit = 0 ;
					if ( telemetry_counter & 0x80 )
						if ( ( telemetry_counter & 0x1F ) != RetrySequence )
							topBit = 0x80 ;
					telemetry_counter = ( (telemetry_counter+1)%32 ) | topBit ;	// Request next telemetry frame
				}
				else
				{
					// incorrect sequence
					RetrySequence = pktt[7] & 0x1F ;
					telemetry_counter |= 0x80 ;
					pktt[6]=0 ;							// Discard current packet and wait for retransmit
				}
			}
		}
		else
			pktt[6]=0; 									// Discard packet
		//
#if defined SPORT_TELEMETRY && defined FRSKYX_CC2500_INO
		telemetry_lost=0;
		if (protocol==PROTO_FRSKYX)
		{
			uint16_t lcrc = frskyX_crc_x(&pkt[3], len-7 ) ;

			if ( ( (lcrc >> 8) == pkt[len-4]) && ( (lcrc & 0x00FF ) == pkt[len-3]) )
			{
				// Check if in sequence
				if ( (pkt[5] & 0x0F) == 0x08 )
				{
					FrX_receive_seq = 0x08 ;
					NextFxFrameToForward = 0 ;
					FrskyxRxFrames[0].valid = 0 ;
					FrskyxRxFrames[1].valid = 0 ;
					FrskyxRxFrames[2].valid = 0 ;
					FrskyxRxFrames[3].valid = 0 ;
				}
				else if ( (pkt[5] & 0x03) == (FrX_receive_seq & 0x03 ) )
				{
					// OK to process
					struct t_fx_rx_frame *p ;
					uint8_t count ;
					p = &FrskyxRxFrames[FrX_receive_seq & 3] ;
					count = pkt[6] ;
					if ( count <= 6 )
					{
						p->count = count ;
						for ( uint8_t i = 0 ; i < count ; i += 1 )
							p->payload[i] = pkt[i+7] ;
					}
					else
						p->count = 0 ;
					p->valid = 1 ;
		
					FrX_receive_seq = ( FrX_receive_seq + 1 ) & 0x03 ;

					if ( FrskyxRxTelemetryValidSequence & 0x80 )
					{
						FrX_receive_seq = ( FrskyxRxTelemetryValidSequence + 1 ) & 3 ;
						FrskyxRxTelemetryValidSequence &= 0x7F ;
					}

				}
				else
				{
					// Save and request correct packet
					struct t_fx_rx_frame *q ;
					uint8_t count ;
					// pkt[4] RSSI
					// pkt[5] sequence control
					// pkt[6] payload count
					// pkt[7-12] payload			
					pktt[6] = 0 ; // Don't process
					if ( (pkt[5] & 0x03) == ( ( FrX_receive_seq +1 ) & 3 ) )
					{
						q = &FrskyxRxFrames[(pkt[5] & 0x03)] ;
						count = pkt[6] ;
						if ( count <= 6 )
						{
							q->count = count ;
							for ( uint8_t i = 0 ; i < count ; i += 1 )
							{
								q->payload[i] = pkt[i+7] ;
							}
						}
						else
							q->count = 0 ;
						q->valid = 1 ;
					
						FrskyxRxTelemetryValidSequence = 0x80 | ( pkt[5] & 0x03 ) ;
					}
					 
					 FrX_receive_seq = ( FrX_receive_seq & 0x03 ) | 0x04 ;	// Request re-transmission
				}

				if (((pktt[5] >> 4) & 0x0f) == 0x08)
					FrX_send_seq = 0 ;
			}
		}
#endif
	}
}

void init_frskyd_link_telemetry()
{
	telemetry_link=0;
	telemetry_counter=0;
	v_lipo1=0;
	v_lipo2=0;
	RX_RSSI=0;
	TX_RSSI=0;
	RX_LQI=0;
	TX_LQI=0;
}

void frsky_link_frame()
{
	frame[0] = 0xFE;			// Link frame
	if (protocol==PROTO_FRSKYD)
	{		
		frame[1] = pktt[3];		// A1
		frame[2] = pktt[4];		// A2
		frame[3] = pktt[5];		// RX_RSSI
		telemetry_link &= ~1 ;		// Sent
		telemetry_link |= 2 ;		// Send hub if available
	}
	else
		if (protocol==PROTO_HUBSAN||protocol==PROTO_AFHDS2A||protocol==PROTO_BAYANG||protocol==PROTO_NCC1701||protocol==PROTO_CABELL||protocol==PROTO_HITEC||protocol==PROTO_BUGS||protocol==PROTO_BUGSMINI)
		{	
			frame[1] = v_lipo1;
			frame[2] = v_lipo2;
			frame[3] = RX_RSSI;
			telemetry_link=0;
		}
	frame[4] = TX_RSSI;
	frame[5] = RX_LQI;
	frame[6] = TX_LQI;
	frame[7] = frame[8] = 0;
	#if defined MULTI_TELEMETRY
		multi_send_frskyhub();
	#else
		frskySendStuffed();
	#endif
}

#if defined HUB_TELEMETRY
void frsky_user_frame()
{
	if(pktt[6])
	{//only send valid hub frames
		frame[0] = 0xFD;				// user frame
		if(pktt[6]>USER_MAX_BYTES)
		{
			frame[1]=USER_MAX_BYTES;	// packet size
			pktt[6]-=USER_MAX_BYTES;
			telemetry_link |= 2 ;			// 2 packets need to be sent
		}
		else
		{
			frame[1]=pktt[6];			// packet size
			telemetry_link=0;			// only 1 packet or processing second packet
		}
		frame[2] = pktt[7];
		for(uint8_t i=0;i<USER_MAX_BYTES;i++)
			frame[i+3]=pktt[i+8];
		if(telemetry_link & 2)				// prepare the content of second packet
			for(uint8_t i=8;i<USER_MAX_BYTES+8;i++)
				pktt[i]=pktt[i+USER_MAX_BYTES];
		#if defined MULTI_TELEMETRY
			multi_send_frskyhub();
		#else
			frskySendStuffed();
		#endif
	}
	else
		telemetry_link=0;
}
/*
HuB RX packets.
pkt[6]|(counter++)|00 01 02 03 04 05 06 07 08 09 
        %32        
01     08          5E 28 12 00 5E 5E 3A 06 00 5E
0A     09          28 12 00 5E 5E 3A 06 00 5E 5E  
09     0A          3B 09 00 5E 5E 06 36 7D 5E 5E 
03     0B          5E 28 11 00 5E 5E 06 06 6C 5E
0A     0C          00 5E 5E 3A 06 00 5E 5E 3B 09
07     0D          00 5E 5E 06 06 6C 5E 16 72 5E
05     0E          5E 28 11 00 5E 5E 3A 06 00 5E
0A     0F          5E 3A 06 00 5E 5E 3B 09 00 5E
05     10          5E 06 16 72 5E 5E 3A 06 00 5E
*/
#endif


#if defined SPORT_TELEMETRY
/* SPORT details serial
			100K 8E2 normal-multiprotocol
			-every 12ms-or multiple of 12; %36
			1  2  3  4  5  6  7  8  9  CRC DESCR
			7E 98 10 05 F1 20 23 0F 00 A6 SWR_ID 
			7E 98 10 01 F1 33 00 00 00 C9 RSSI_ID 
			7E 98 10 04 F1 58 00 00 00 A1 BATT_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 
			7E BA 10 03 F1 E2 00 00 00 18 ADC2_ID 	
			
			
			Telemetry frames(RF) SPORT info 
			15 bytes payload
			SPORT frame valid 6+3 bytes
			[00] PKLEN  0E 0E 0E 0E 
			[01] TXID1  DD DD DD DD 
			[02] TXID2  6D 6D 6D 6D 
			[03] CONST  02 02 02 02 
			[04] RS/RB  2C D0 2C CE	//D0;CE=2*RSSI;....2C = RX battery voltage(5V from Bec)
			[05] HD-SK  03 10 21 32	//TX/RX telemetry hand-shake bytes
			[06] NO.BT  00 00 06 03	//No.of valid SPORT frame bytes in the frame		
			[07] STRM1  00 00 7E 00 
			[08] STRM2  00 00 1A 00 
			[09] STRM3  00 00 10 00 
			[10] STRM4  03 03 03 03  
			[11] STRM5  F1 F1 F1 F1 
			[12] STRM6  D1 D1 D0 D0
			[13] CHKSUM1 --|2 CRC bytes sent by RX (calculated on RX side crc16/table)
			[14] CHKSUM2 --|
			+2	appended bytes automatically  RSSI and LQI/CRC bytes(len=0x0E+3);
			
0x06	0x06	0x06	0x06	0x06

0x7E	0x00	0x03	0x7E	0x00
0x1A	0x00	0xF1	0x1A	0x00
0x10	0x00	0xD7	0x10	0x00
0x03	0x7E	0x00	0x03	0x7E
0xF1	0x1A	0x00	0xF1	0x1A
0xD7	0x10	0x00	0xD7	0x10

0xE1	0x1C	0xD0	0xEE	0x33
0x34	0x0A	0xC3	0x56	0xF3
				
		*/
#if defined SPORT_POLLING || defined MULTI_TELEMETRY
const uint8_t PROGMEM Indices[] = {	0x00, 0xA1, 0x22, 0x83, 0xE4, 0x45,
									0xC6, 0x67, 0x48, 0xE9, 0x6A, 0xCB,
									0xAC, 0x0D, 0x8E, 0x2F, 0xD0, 0x71,
									0xF2, 0x53, 0x34, 0x95, 0x16, 0xB7,
									0x98, 0x39, 0xBA, 0x1B } ;
#endif

#ifdef MULTI_TELEMETRY
	void sportSend(uint8_t *p)
	{
	#ifdef SPORT_POLLING
	#ifdef INVERT_SERIAL
		USART3_BASE->CR1 &= ~USART_CR1_TE ;
		TX_INV_on;	//activate inverter for both serial TX and RX signals
		USART3_BASE->CR1 |= USART_CR1_TE ;
	#endif
	#endif
		multi_send_header(MULTI_TELEMETRY_SPORT, 9);
		uint16_t crc_s = 0;
		uint8_t x = p[0] ;
		if ( x <= 0x1B )
			x = pgm_read_byte_near( &Indices[x] ) ;
		Serial_write(x) ;
		for (uint8_t i = 1; i < 9; i++)
		{
			if (i == 8)
				p[i] = 0xff - crc_s;
				Serial_write(p[i]);

			if (i>0)
			{
				crc_s += p[i];			//0-1FF
				crc_s += crc_s >> 8;	//0-100
				crc_s &= 0x00ff;
			}
		}
	}
#else
	void sportSend(uint8_t *p)
	{
		uint16_t crc_s = 0;
	#ifdef SPORT_POLLING
	#ifdef INVERT_SERIAL
		USART3_BASE->CR1 &= ~USART_CR1_TE ;
		TX_INV_on;	//activate inverter for both serial TX and RX signals
		USART3_BASE->CR1 |= USART_CR1_TE ;
	#endif
	#endif
		Serial_write(START_STOP);//+9
		Serial_write(p[0]) ;
		for (uint8_t i = 1; i < 9; i++)
		{
			if (i == 8)
				p[i] = 0xff - crc_s;
			
			if ((p[i] == START_STOP) || (p[i] == BYTESTUFF))
			{
				Serial_write(BYTESTUFF);//stuff again
				Serial_write(STUFF_MASK ^ p[i]);
			} 
			else			
				Serial_write(p[i]);					
			
			if (i>0)
			{
				crc_s += p[i]; //0-1FF
				crc_s += crc_s >> 8; //0-100
				crc_s &= 0x00ff;
			}
		}
	}
	
#endif

#if defined  SPORT_POLLING
uint8_t nextID()
{
	uint8_t i ;
	uint8_t poll_idx ; 
	if (phase)
	{
		poll_idx = 99 ;
		for ( i = 0 ; i < 28 ; i++ )
		{
			if ( sport_rx_index[kindex] )
			{
				poll_idx = kindex ;
			}
			kindex++ ;
			if ( kindex>= 28 )
			{
				kindex = 0 ;
				phase = 0 ;
				break ;
			}
			if ( poll_idx != 99 )
			{
				break ;
			}
		}
		if ( poll_idx != 99 )
		{
			return poll_idx ;
		}
	}
	if ( phase == 0 )
	{
		for ( i = 0 ; i < 28 ; i++ )
		{
			if ( sport_rx_index[ukindex] == 0 )
			{
				poll_idx = ukindex ;
				phase = 1 ;
			}
			ukindex++;
			if (ukindex >= 28 )
			{
				ukindex = 0 ;
			}
			if ( poll_idx != 99 )
			{
				return poll_idx ;
			}
		}
		if ( poll_idx == 99 )
		{
			phase = 1 ;
			return 0 ;
		}
	}
	return poll_idx ;
}

#ifdef INVERT_SERIAL
void start_timer4()
{
	TIMER4_BASE->PSC = 71;								// 72-1;for 72 MHZ / 1.0sec/(71+1)
	TIMER4_BASE->CCER = 0 ;
	TIMER4_BASE->DIER = 0 ;
	TIMER4_BASE->CCMR1 = 0 ;
	TIMER4_BASE->CCMR1 = TIMER_CCMR1_OC1M ;
	HWTimer4.attachInterrupt(TIMER_CH1, __irq_timer4);		// Assign function to Timer2/Comp2 interrupt
	nvic_irq_set_priority( NVIC_TIMER4, 14 ) ;
}

void stop_timer4()
{
	TIMER5_BASE->CR1 = 0 ;
	nvic_irq_disable( NVIC_TIMER4 ) ;
}

void __irq_timer4(void)			
{
	TIMER4_BASE->DIER = 0 ;
	TIMER4_BASE->CR1 = 0 ;
	TX_INV_on;	//activate inverter for both serial TX and RX signals
}

#endif

void pollSport()
{
	uint8_t pindex = nextID() ;
	TxData[0]  = START_STOP;
	TxData[1] = pgm_read_byte_near(&Indices[pindex]) ;
	if(!telemetry_lost && ((TxData[1] &0x1F)== skipped_id ||TxData[1]==0x98))
	{//98 ID(RSSI/RxBat and SWR ) and ID's from sport telemetry
		pindex = nextID() ;	
		TxData[1] = pgm_read_byte_near(&Indices[pindex]);
	}		
	SportIndexPolling = pindex ;
	RxIndex = 0;
	#ifdef INVERT_SERIAL
		USART3_BASE->CR1 &= ~USART_CR1_TE ;
		TX_INV_on;	//activate inverter for both serial TX and RX signals
		USART3_BASE->CR1 |= USART_CR1_TE ;
	#endif
#ifdef MULTI_TELEMETRY
	multi_send_header(MULTI_TELEMETRY_SPORT_POLLING, 1);
#else
    Serial_write(TxData[0]);
#endif
	RxIndex=0;
	Serial_write(TxData[1]);
	USART3_BASE->CR1 |= USART_CR1_TCIE ;
#ifdef INVERT_SERIAL
	TIMER4_BASE->CNT = 0 ;
	TIMER4_BASE->CCR1 = 3000 ;
	TIMER4_BASE->DIER = TIMER_DIER_CC1IE ;
	TIMER4_BASE->CR1 = TIMER_CR1_CEN ;
#endif
}

bool checkSportPacket()
{
	uint8_t *packet = RxData ;
	uint16_t crc = 0 ;
	if ( RxIndex < 8 )
		return 0 ;
	for ( uint8_t i = 0 ; i<8 ; i += 1 )
	{
		crc += packet[i]; 
		crc += crc >> 8; 
		crc &= 0x00ff;
	}
	return (crc == 0x00ff) ;
}

uint8_t unstuff()
{
	uint8_t i ;
	uint8_t j ;
	j = 0 ;
	for ( i = 0 ; i < RxIndex ; i += 1 )
	{
		if ( RxData[i] == BYTESTUFF )
		{
			i += 1 ;
			RxData[j] = RxData[i] ^ STUFF_MASK ; ;
		}
		else
			RxData[j] = RxData[i] ;
		j += 1 ;
	}
	return j ;
}

void processSportData(uint8_t *p)
{	

	RxIndex = unstuff() ;
	uint8_t x=checkSportPacket() ;
	if (x)
	{
		SportData[sport_idx]=0x7E;
		sport_idx =(sport_idx+1) & (MAX_SPORT_BUFFER-1);
		SportData[sport_idx]=TxData[1]&0x1F;
		sport_idx =(sport_idx+1) & (MAX_SPORT_BUFFER-1);	
		
		for(uint8_t i=0;i<(RxIndex-1);i++)
		{//no crc		
			if(p[i]==START_STOP || p[i]==BYTESTUFF)
			{//stuff back
				SportData[sport_idx]=BYTESTUFF;
				sport_idx =(sport_idx+1) & (MAX_SPORT_BUFFER-1);
				SportData[sport_idx]=p[i]^STUFF_MASK;
			}
			else
				SportData[sport_idx]=p[i];
			sport_idx =(sport_idx+1) & (MAX_SPORT_BUFFER-1);
		}
		sport_rx_index[SportIndexPolling] = 1 ;	
		ok_to_send=true;
		RxIndex =0 ; 
	}
}

inline void rx_pause()
{
	USART3_BASE->CR1 &= ~ USART_CR1_RXNEIE;	//disable rx interrupt on USART3	
}
inline void rx_resume()
{
	USART3_BASE->CR1 |= USART_CR1_RXNEIE;	//enable rx interrupt on USART3
}	
#endif//end SPORT_POLLING

void sportIdle()
{
	#if !defined MULTI_TELEMETRY
		Serial_write(START_STOP);
	#endif
}	

void sportSendFrame()
{
	#if defined SPORT_POLLING
		rx_pause();
	#endif
	uint8_t i;
	sport_counter = (sport_counter + 1) %36;
	if(telemetry_lost)
	{
		#ifdef SPORT_POLLING
			pollSport();
		#else
			sportIdle();
         #endif
		return;
	}
	if(sport_counter<6)
	{
		frame[0] = 0x98;
		frame[1] = 0x10;
		for (i=5;i<8;i++)
			frame[i]=0;
	}
	switch (sport_counter)
	{
		case 0:
			frame[2] = 0x05;
			frame[3] = 0xf1;
			frame[4] = 0x02 ;//dummy values if swr 20230f00
			frame[5] = 0x23;
			frame[6] = 0x0F;
			break;
		case 2: // RSSI
			frame[2] = 0x01;
			frame[3] = 0xf1;
			frame[4] = RX_RSSI;
			frame[5] = TX_RSSI;
			frame[6] = RX_LQI;
			frame[7] = TX_LQI;
			break;
		case 4: //BATT
			frame[2] = 0x04;
			frame[3] = 0xf1;
			frame[4] = RxBt;//a1;
			break;								
		default:
			if(sport)
			{	
				for (i=0;i<FRSKY_SPORT_PACKET_SIZE;i++)
				frame[i]=pktx1[i];
				sport -= 1 ;
				#ifdef SPORT_POLLING
					skipped_id=frame[0];
				#endif
				if ( sport )
				{
					uint8_t j = sport * FRSKY_SPORT_PACKET_SIZE ;
					for (i=0;i<j;i++)
					pktx1[i] = pktx1[i+FRSKY_SPORT_PACKET_SIZE] ;
				}
				break;
			}
			else
			{
				#ifdef SPORT_POLLING
					pollSport();
				#else
					sportIdle();
				#endif
				return;
			}		
	}
	sportSend(frame);
}	

void proces_sport_data(uint8_t data)
{
	switch (pass)
	{
		case 0:
			if (data == START_STOP)
			{//waiting for 0x7e
				indx = 0;
				pass = 1;
			}
			break;		
		case 1:
			if (data == START_STOP)	// Happens if missed packet
			{//waiting for 0x7e
				indx = 0;
				pass = 1;
				break;		
			}
			if(data == BYTESTUFF)	//if they are stuffed
				pass=2;
			else
				if (indx < MAX_PKTX)		
					pktx[indx++] = data;		
			break;
		case 2:	
			if (indx < MAX_PKTX)	
				pktx[indx++] = data ^ STUFF_MASK;	//unstuff bytes	
			pass=1;
			break;	
	} // end switch
	if (indx >= FRSKY_SPORT_PACKET_SIZE)
	{//8 bytes no crc 
		if ( sport < FX_BUFFERS )
		{
			uint8_t dest = sport * FRSKY_SPORT_PACKET_SIZE ;
			uint8_t i ;
			for ( i = 0 ; i < FRSKY_SPORT_PACKET_SIZE ; i += 1 )
				pktx1[dest++] = pktx[i] ;	// Triple buffer
			sport += 1 ;//ok to send
		}
//		else
//		{
//			// Overrun
//		}
		pass = 0;//reset
	}
}

#endif

void TelemetryUpdate()
{
	// check for space in tx buffer
	#ifdef BASH_SERIAL
		uint8_t h ;
		uint8_t t ;
		h = SerialControl.head ;
		t = SerialControl.tail ;
		if ( h >= t )
			t += TXBUFFER_SIZE - h ;
		else
			t -= h ;
		if ( t < 64 )
		{
			return ;
		}
	#else
		uint8_t h ;
		uint8_t t ;
		h = tx_head ;
		t = tx_tail ;
		if ( h >= t )
			t += TXBUFFER_SIZE - h ;
		else
			t -= h ;
		if ( t < 32 )
		{
			return ;
		}
	#endif
	#if ( defined(MULTI_TELEMETRY) || defined(MULTI_STATUS) )
		{
			uint32_t now = millis();
			if ((now - lastMulti) > MULTI_TIME)
			{
				multi_send_status();
				lastMulti = now;
				return;
			}
		}
	#endif
	 
	#if defined SPORT_TELEMETRY
		if (protocol==PROTO_FRSKYX)
		{	// FrSkyX
			for(;;)
			{
				struct t_fx_rx_frame *p ;
				uint8_t count ;
				p = &FrskyxRxFrames[NextFxFrameToForward] ;
				if ( p->valid )
				{
					count = p->count ;
					for (uint8_t i=0; i < count ; i++)
						proces_sport_data(p->payload[i]) ;
					p->valid = 0 ;	// Sent on
					NextFxFrameToForward = ( NextFxFrameToForward + 1 ) & 3 ;
				}
				else
				{
					break ;
				}
			}
			 
			if(telemetry_link)
			{		
				if(pktt[4] & 0x80)
					RX_RSSI=pktt[4] & 0x7F ;
				else 
					RxBt = (pktt[4]<<1) + 1 ;
				telemetry_link=0;
			}
			uint32_t now = micros();
			if ((now - last) > SPORT_TIME)
			{
				#if defined SPORT_POLLING
					processSportData(RxData);	//process arrived data before polling
				#endif
				sportSendFrame();
				#ifdef STM32_BOARD
					last=now;
				#else
					last += SPORT_TIME ;
				#endif
			}
		}
	#endif // SPORT_TELEMETRY

	#if defined DSM_TELEMETRY
		if(telemetry_link && protocol == PROTO_DSM)
		{	// DSM
			DSM_frame();
			telemetry_link=0;
			return;
		}
	#endif
	#if defined AFHDS2A_FW_TELEMETRY
		if(telemetry_link == 2 && protocol == PROTO_AFHDS2A)
		{
			AFHDSA_short_frame();
			telemetry_link=0;
			return;
		}
	#endif
	#if defined HITEC_FW_TELEMETRY
		if(telemetry_link == 2 && protocol == PROTO_HITEC)
		{
			HITEC_short_frame();
			telemetry_link=0;
			return;
		}
	#endif

		if((telemetry_link & 1 )&& protocol != PROTO_FRSKYX)
		{	// FrSkyD + Hubsan + AFHDS2A + Bayang + Cabell + Hitec + Bugs + BugsMini + NCC1701
			frsky_link_frame();
			return;
		}
	#if defined HUB_TELEMETRY
		if((telemetry_link & 2) && protocol == PROTO_FRSKYD)
		{	// FrSkyD
			frsky_user_frame();
			return;
		}
	#endif
}


/**************************/
/**************************/
/**  Serial TX routines  **/
/**************************/
/**************************/

#ifndef BASH_SERIAL
	// Routines for normal serial output
	void Serial_write(uint8_t data)
	{
		uint8_t nextHead ;
		nextHead = tx_head + 1 ;
		if ( nextHead >= TXBUFFER_SIZE )
			nextHead = 0 ;
		tx_buff[nextHead]=data;
		tx_head = nextHead ;
		tx_resume();
	}

	void initTXSerial( uint8_t speed)
	{
		#ifdef ENABLE_PPM
			if(speed==SPEED_9600)
			{ // 9600
				#ifdef ORANGE_TX
					USARTC0.BAUDCTRLA = 207 ;
					USARTC0.BAUDCTRLB = 0 ;
					USARTC0.CTRLB = 0x18 ;
					USARTC0.CTRLA = (USARTC0.CTRLA & 0xCF) | 0x10 ;
					USARTC0.CTRLC = 0x03 ;
				#else
					#ifdef STM32_BOARD
						usart3_begin(9600,SERIAL_8N1);		//USART3 
						USART3_BASE->CR1 &= ~ USART_CR1_RE;	//disable RX leave TX enabled
					#else
						UBRR0H = 0x00;
						UBRR0L = 0x67;
						UCSR0A = 0 ;						// Clear X2 bit
						//Set frame format to 8 data bits, none, 1 stop bit
						UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
					#endif
				#endif
			}
			else if(speed==SPEED_57600)
			{ // 57600
				#ifdef ORANGE_TX
					/*USARTC0.BAUDCTRLA = 207 ;
					USARTC0.BAUDCTRLB = 0 ;
					USARTC0.CTRLB = 0x18 ;
					USARTC0.CTRLA = (USARTC0.CTRLA & 0xCF) | 0x10 ;
					USARTC0.CTRLC = 0x03 ;*/
				#else
					#ifdef STM32_BOARD
						usart3_begin(57600,SERIAL_8N1);		//USART3 
						USART3_BASE->CR1 &= ~ USART_CR1_RE;	//disable RX leave TX enabled
					#else
						UBRR0H = 0x00;
						UBRR0L = 0x22;
						UCSR0A = 0x02 ;	// Set X2 bit
						//Set frame format to 8 data bits, none, 1 stop bit
						UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
					#endif
				#endif
			}
			else if(speed==SPEED_125K)
			{ // 125000
				#ifdef ORANGE_TX
					/*USARTC0.BAUDCTRLA = 207 ;
					USARTC0.BAUDCTRLB = 0 ;
					USARTC0.CTRLB = 0x18 ;
					USARTC0.CTRLA = (USARTC0.CTRLA & 0xCF) | 0x10 ;
					USARTC0.CTRLC = 0x03 ;*/
				#else
					#ifdef STM32_BOARD
						usart3_begin(125000,SERIAL_8N1);	//USART3 
						USART3_BASE->CR1 &= ~ USART_CR1_RE;	//disable RX leave TX enabled
					#else
						UBRR0H = 0x00;
						UBRR0L = 0x07;
						UCSR0A = 0x00 ;	// Clear X2 bit
						//Set frame format to 8 data bits, none, 1 stop bit
						UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
					#endif
				#endif
			}
		#else
			(void)speed;
		#endif
		#ifndef ORANGE_TX
			#ifndef STM32_BOARD
				UCSR0B |= (1<<TXEN0);//tx enable
			#endif
		#endif
	}

	//Serial TX
	#ifdef ORANGE_TX
		ISR(USARTC0_DRE_vect)
	#else
		#ifdef STM32_BOARD
			void __irq_usart3()			
		#else
			ISR(USART_UDRE_vect)
		#endif
	#endif
	{	// Transmit interrupt
		#ifdef STM32_BOARD
			#ifdef SPORT_POLLING		
				if(USART3_BASE->SR & USART_SR_TC) 
				{
					if ( USART3_BASE->CR1 & USART_CR1_TCIE )
					{
						USART3_BASE->CR1 &= ~USART_CR1_TCIE ;
						TX_INV_off;
					}
				}

				if(USART3_BASE->SR & USART_SR_RXNE) 
				{
					USART3_BASE->SR &= ~USART_SR_RXNE;
					if (RxIndex < 16 )
					{
						if(RxData[0]==TxData[0] && RxData[1]==TxData[1])
							RxIndex=0;
						RxData[RxIndex++] = USART3_BASE->DR & 0xFF ;					
					}
				}
			#endif
			if(USART3_BASE->SR & USART_SR_TXE)
			{
		#endif
				if(tx_head!=tx_tail)
				{
					if(++tx_tail>=TXBUFFER_SIZE)//head 
						tx_tail=0;
					#ifdef STM32_BOARD	
						USART3_BASE->DR=tx_buff[tx_tail];//clears TXE bit				
					#else
						UDR0=tx_buff[tx_tail];
					#endif
				}
				if (tx_tail == tx_head)
				{
					tx_pause(); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
					#ifdef  SPORT_POLLING
						rx_resume();
					#endif
				}
		#ifdef STM32_BOARD	
			}
		#endif		
	}
#else	//BASH_SERIAL
// Routines for bit-bashed serial output

// Speed is 0 for 100K and 1 for 9600
void initTXSerial( uint8_t speed)
{
	TIMSK0 = 0 ;	// Stop all timer 0 interrupts
	#ifdef INVERT_SERIAL
		SERIAL_TX_off;
	#else
		SERIAL_TX_on;
	#endif
	UCSR0B &= ~(1<<TXEN0) ;

	SerialControl.speed = speed ;
	if ( speed == SPEED_9600 )
	{
		OCR0A = 207 ;	// 104uS period
		TCCR0A = 3 ;
		TCCR0B = 0x0A ; // Fast PMM, 2MHz
	}
	else	// 100K
	{
		TCCR0A = 0 ;
		TCCR0B = 2 ;	// Clock/8 (0.5uS)
	}
}

void Serial_write( uint8_t byte )
{
	uint8_t temp ;
	uint8_t temp1 ;
	uint8_t byteLo ;

	#ifdef INVERT_SERIAL
		byte = ~byte ;
	#endif

	byteLo = byte ;
	byteLo >>= 7 ;		// Top bit
	if ( SerialControl.speed == SPEED_100K )
	{
		#ifdef INVERT_SERIAL
				byteLo |= 0x02 ;	// Parity bit
		#else
				byteLo |= 0xFC ;	// Stop bits
		#endif
		// calc parity
		temp = byte ;
		temp >>= 4 ;
		temp = byte ^ temp ;
		temp1 = temp ;
		temp1 >>= 2 ;
		temp = temp ^ temp1 ;
		temp1 = temp ;
		temp1 <<= 1 ;
		temp ^= temp1 ;
		temp &= 0x02 ;
		#ifdef INVERT_SERIAL
				byteLo ^= temp ;
		#else	
				byteLo |= temp ;
		#endif
	}
	else
	{
		byteLo |= 0xFE ;	// Stop bit
	}
	byte <<= 1 ;
	#ifdef INVERT_SERIAL
		byte |= 1 ;		// Start bit
	#endif
	uint8_t next = SerialControl.head + 2;
	if(next>=TXBUFFER_SIZE)
		next=0;
	if ( next != SerialControl.tail )
	{
		SerialControl.data[SerialControl.head] = byte ;
		SerialControl.data[SerialControl.head+1] = byteLo ;
		SerialControl.head = next ;
	}
	if(!IS_TX_PAUSE_on)
		tx_resume();
}

void resumeBashSerial()
{
	cli() ;
	if ( SerialControl.busy == 0 )
	{
		sei() ;
		// Start the transmission here
		#ifdef INVERT_SERIAL
			GPIOR2 = 0 ;
		#else
			GPIOR2 = 0x01 ;
		#endif
		if ( SerialControl.speed == SPEED_100K )
		{
			GPIOR1 = 1 ;
			OCR0B = TCNT0 + 40 ;
			OCR0A = OCR0B + 210 ;
			TIFR0 = (1<<OCF0A) | (1<<OCF0B) ;
			TIMSK0 |= (1<<OCIE0B) ;
			SerialControl.busy = 1 ;
		}
		else
		{
			GPIOR1 = 1 ;
			TIFR0 = (1<<TOV0) ;
			TIMSK0 |= (1<<TOIE0) ;
			SerialControl.busy = 1 ;
		}
	}
	else
	{
		sei() ;
	}
}

// Assume timer0 at 0.5uS clock

ISR(TIMER0_COMPA_vect)
{
	uint8_t byte ;
	byte = GPIOR0 ;
	if ( byte & 0x01 )
		SERIAL_TX_on;
	else
		SERIAL_TX_off;
	byte /= 2 ;		// Generates shorter code than byte >>= 1
	GPIOR0 = byte ;
	if ( --GPIOR1 == 0 )
	{
		TIMSK0 &= ~(1<<OCIE0A) ;
		GPIOR1 = 3 ;
	}
	else
		OCR0A += 20 ;
}

ISR(TIMER0_COMPB_vect)
{
	uint8_t byte ;
	byte = GPIOR2 ;
	if ( byte & 0x01 )
		SERIAL_TX_on;
	else
		SERIAL_TX_off;
	byte /= 2 ;		// Generates shorter code than byte >>= 1
	GPIOR2 = byte ;
	if ( --GPIOR1 == 0 )
	{
		if ( IS_TX_PAUSE_on )
		{
			SerialControl.busy = 0 ;
			TIMSK0 &= ~(1<<OCIE0B) ;
		}
		else
		{
			// prepare next byte and allow for 2 stop bits
			volatile struct t_serial_bash *ptr = &SerialControl ;
			if ( ptr->head != ptr->tail )
			{
				GPIOR0 = ptr->data[ptr->tail] ;
				GPIOR2 = ptr->data[ptr->tail+1] ;
				uint8_t nextTail = ptr->tail + 2 ;
				if ( nextTail >= TXBUFFER_SIZE )
					nextTail = 0 ;
				ptr->tail = nextTail ;
				GPIOR1 = 8 ;
				OCR0A = OCR0B + 40 ;
				OCR0B = OCR0A + 8 * 20 ;
				TIMSK0 |= (1<<OCIE0A) ;
			}
			else
			{
				SerialControl.busy = 0 ;
				TIMSK0 &= ~(1<<OCIE0B) ;
			}
		}
	}
	else
		OCR0B += 20 ;
}

ISR(TIMER0_OVF_vect)
{
	uint8_t byte ;
	if ( GPIOR1 > 2 )
		byte = GPIOR0 ;
	else
		byte = GPIOR2 ;
	if ( byte & 0x01 )
		SERIAL_TX_on;
	else
		SERIAL_TX_off;
	byte /= 2 ;		// Generates shorter code than byte >>= 1
	if ( GPIOR1 > 2 )
		GPIOR0 = byte ;
	else
		GPIOR2 = byte ;
	if ( --GPIOR1 == 0 )
	{	// prepare next byte
		volatile struct t_serial_bash *ptr = &SerialControl ;
		if ( ptr->head != ptr->tail )
		{
			GPIOR0 = ptr->data[ptr->tail] ;
			GPIOR2 = ptr->data[ptr->tail+1] ;
			uint8_t nextTail = ptr->tail + 2 ;
			if ( nextTail >= TXBUFFER_SIZE )
				nextTail = 0 ;
			ptr->tail = nextTail ;
			GPIOR1 = 10 ;
		}
		else
		{
			SerialControl.busy = 0 ;
			TIMSK0 &= ~(1<<TOIE0) ;
		}
	}
}


#endif // BASH_SERIAL

#endif // TELEMETRY


# 1 "src/DM002_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with DM002

#if defined(DM002_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define DM002_PACKET_PERIOD		6100 // Timeout for callback in uSec
#define DM002_INITIAL_WAIT		500
#define DM002_PACKET_SIZE		12   // packets have 12-byte payload
#define DM002_RF_BIND_CHANNEL	0x27
#define DM002_BIND_COUNT		655  // 4 seconds


enum DM002_FLAGS {
    // flags going to packet[9]
    DM002_FLAG_FLIP		= 0x01, 
    DM002_FLAG_LED		= 0x02, 
    DM002_FLAG_MEDIUM	= 0x04, 
    DM002_FLAG_HIGH		= 0x08, 
    DM002_FLAG_RTH		= 0x10,
    DM002_FLAG_HEADLESS	= 0x20,
    DM002_FLAG_CAMERA1	= 0x40,
    DM002_FLAG_CAMERA2	= 0x80,
};

static void __attribute__((unused)) DM002_send_packet(uint8_t bind)
{
	memcpy(packet+5,(uint8_t *)"\x00\x7F\x7F\x7F\x00\x00\x00",7);
	if(bind)
	{
		packet[0] = 0xAA;
		packet[1] = rx_tx_addr[0]; 
		packet[2] = rx_tx_addr[1];
		packet[3] = rx_tx_addr[2];
		packet[4] = rx_tx_addr[3];
	}
	else
	{
		packet[0]=0x55;
		// Throttle : 0 .. 200
		packet[1]=convert_channel_16b_limit(THROTTLE,0,200);
		// Other channels min 0x57, mid 0x7F, max 0xA7
		packet[2] = convert_channel_16b_limit(RUDDER,0x57,0xA7);
		packet[3] = convert_channel_16b_limit(AILERON, 0x57,0xA7);
		packet[4] = convert_channel_16b_limit(ELEVATOR, 0xA7, 0x57);
		// Features
		packet[9] =   GET_FLAG(CH5_SW,DM002_FLAG_FLIP)
					| GET_FLAG(!CH6_SW,DM002_FLAG_LED)
					| GET_FLAG(CH7_SW,DM002_FLAG_CAMERA1)
					| GET_FLAG(CH8_SW,DM002_FLAG_CAMERA2)
					| GET_FLAG(CH9_SW,DM002_FLAG_HEADLESS)
					| GET_FLAG(CH10_SW,DM002_FLAG_RTH)
					| GET_FLAG(!CH11_SW,DM002_FLAG_HIGH);
		// Packet counter
		if(packet_count&0x03)
		{
			packet_count++;
			hopping_frequency_no++;
			hopping_frequency_no&=4;
		}
		packet_count&=0x0F;
		packet[10] = packet_count;
		packet_count++;
	}
	//CRC
	for(uint8_t i=0;i<DM002_PACKET_SIZE-1;i++)
		packet[11]+=packet[i];
	
	// Power on, TX mode, 2byte CRC
	// Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if (bind)
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, DM002_RF_BIND_CHANNEL);
	else
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, DM002_PACKET_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) DM002_init()
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t *)"\x26\xA8\x67\x35\xCC", 5);

    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             // 1Mbps
    NRF24L01_SetPower();
}

uint16_t DM002_callback()
{
	if(IS_BIND_DONE)
		DM002_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			BIND_DONE;
			XN297_SetTXAddr(rx_tx_addr, 5);
		}
		else
		{
			DM002_send_packet(1);
			bind_counter--;
		}
	}
	return	DM002_PACKET_PERIOD;
}

static void __attribute__((unused)) DM002_initialize_txid()
{
	// Only 3 IDs/RFs are available, RX_NUM is used to switch between them
	switch(rx_tx_addr[3]%3)
	{
		case 0:
			memcpy(hopping_frequency,(uint8_t *)"\x34\x39\x43\x48",4);
			memcpy(rx_tx_addr,(uint8_t *)"\x47\x93\x00\x00\xD5",5);
			break;
		case 1:
			memcpy(hopping_frequency,(uint8_t *)"\x35\x39\x3B\x3D",4);
			memcpy(rx_tx_addr,(uint8_t *)"\xAC\xA1\x00\x00\xD5",5);
			break;
		case 2:
			memcpy(hopping_frequency,(uint8_t *)"\x32\x37\x41\x46",4);
			memcpy(rx_tx_addr,(uint8_t *)"\x92\x45\x01\x00\xD5",5);
			break;
	}
}

uint16_t initDM002(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    bind_counter = DM002_BIND_COUNT;
	DM002_initialize_txid();
	DM002_init();
	return	DM002_INITIAL_WAIT;
}

#endif


# 1 "src/Arduino.ino" // Helps debugging !


/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
/************************************/
/************************************/
/**  Arduino replacement routines  **/
/************************************/
// replacement map()
int16_t map16b( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	long y ;
	x -= in_min ;
	y = out_max - out_min ;
	y *= x ;
	x = y / (in_max - in_min) ;
	return x  + out_min ;
}

#ifndef STM32_BOARD
int16_t map( int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
//  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	long y ;
	x -= in_min ;
	y = out_max - out_min ;
	y *= x ;
	x = y / (in_max - in_min) ;
	return x  + out_min ;
}

// replacement millis() and micros()
// These work polled, no interrupts
// micros() MUST be called at least once every 32 milliseconds
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
   uint16_t time = TCNT1 ;   // Read timer 1
   SREG = oldSREG ;

   elapsed = time - lastTimerValue ;
   elapsed += Correction ;
   Correction = elapsed & 0x01 ;
   elapsed >>= 1 ;
   
   uint32_t ltime = TotalMicros ;
   ltime += elapsed ;
   cli() ;
   TotalMicros = ltime ;   // Done this way for RPM to work correctly
   lastTimerValue = time ;
   SREG = oldSREG ;   // Still valid from above
   
   elapsed += MillisPrecount;
   millisToAdd = 0 ;
   
   if ( elapsed  > 15999 )
   {
      millisToAdd = 16 ;
      elapsed -= 16000 ;
   }
   if ( elapsed  > 7999 )
   {
      millisToAdd += 8 ;
      elapsed -= 8000 ;
   }
   if ( elapsed  > 3999 )
   {
      millisToAdd += 4 ;      
      elapsed -= 4000 ;
   }
   if ( elapsed  > 1999 )
   {
      millisToAdd += 2 ;
      elapsed -= 2000 ;
   }
   if ( elapsed  > 999 )
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

void delayMilliseconds(unsigned long ms)
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

/* Important notes:
	- Max value is 16000µs
	- delay is not accurate due to interrupts happening */
void delayMicroseconds(unsigned int us)
{
   if (--us == 0)
      return;
   us <<= 2;	// * 4
   us -= 2;		// - 2
#ifdef ORANGE_TX
	 __asm__ __volatile__ (
      "1: sbiw %0,1" "\n\t" // 2 cycles
			"nop \n"
			"nop \n"
			"nop \n"
			"nop \n"
      "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
   );
#else   
	 __asm__ __volatile__ (
      "1: sbiw %0,1" "\n\t" // 2 cycles
      "brne 1b" : "=w" (us) : "0" (us) // 2 cycles
   );
#endif
}

#ifndef ORANGE_TX
	void init()
	{
	   // this needs to be called before setup() or some functions won't work there
	   sei();
	}
#endif //ORANGE_TX

#endif //STM32_BOARD


# 1 "src/KN_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/KN_nrf24l01.c dated 2015-11-09

#if defined(KN_NRF24L01_INO)

#include "iface_nrf24l01.h"

// Wait for RX chip stable - 10ms
#define KN_INIT_WAIT_MS  10000

//Payload(16 bytes) plus overhead(10 bytes) is 208 bits, takes about 0.4ms or 0.2ms
//to send for the rate of 500kb/s and 1Mb/s respectively.

// Callback timeout period for sending bind packets, minimum 250
#define KN_BINDING_PACKET_PERIOD  1000

// Timeout for sending data packets, in uSec, KN protocol requires 2ms
#define KN_WL_SENDING_PACKET_PERIOD  2000
// Timeout for sending data packets, in uSec, KNFX protocol requires 1.2 ms
#define KN_FX_SENDING_PACKET_PERIOD  1200

// packets to be sent during binding, last 0.5 seconds in WL Toys and 0.2 seconds in Feilun
#define KN_WL_BIND_COUNT 500
#define KN_FX_BIND_COUNT 200 

#define KN_PAYLOADSIZE 16

//24L01 has 126 RF channels, can we use all of them?
#define KN_MAX_RF_CHANNEL 73

//KN protocol for WL Toys changes RF frequency every 10 ms, repeats with only 4 channels.
//Feilun variant uses only 2 channels, so we will just repeat the hopping channels later
#define KN_RF_CH_COUNT 4

//KN protocol for WL Toys sends 4 data packets every 2ms per frequency, plus a 2ms gap.
#define KN_WL_PACKET_SEND_COUNT 5
//KN protocol for Feilun sends 8 data packets every 1.2ms per frequency, plus a 0.3ms gap.
#define KN_FX_PACKET_SEND_COUNT 8
 
#define KN_TX_ADDRESS_SIZE 5

enum {
    KN_PHASE_PRE_BIND,
    KN_PHASE_BINDING,
    KN_PHASE_PRE_SEND,
    KN_PHASE_SENDING,
};

enum {
	KN_FLAG_DR     = 0x01, // Dual Rate: 1 - full range
	KN_FLAG_TH     = 0x02, // Throttle Hold: 1 - hold
	KN_FLAG_IDLEUP = 0x04, // Idle up: 1 - 3D
	KN_FLAG_RES1   = 0x08,
	KN_FLAG_RES2   = 0x10,
	KN_FLAG_RES3   = 0x20,
	KN_FLAG_GYRO3  = 0x40, // 0 - 6G mode, 1 - 3G mode
	KN_FLAG_GYROR  = 0x80  // Always 0 so far
};

//-------------------------------------------------------------------------------------------------
// This function init 24L01 regs and packet data for binding
// Send tx address, hopping table (for Wl Toys), and data rate to the KN receiver during binding.
// It seems that KN can remember these parameters, no binding needed after power up.
// Bind uses fixed TX address "KNDZK", 1 Mbps data rate and channel 83
//-------------------------------------------------------------------------------------------------
static void __attribute__((unused)) kn_bind_init()
{
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)"KNDZK", 5);
	packet[0]  = 'K';
	packet[1]  = 'N';
	packet[2]  = 'D';
	packet[3]  = 'Z';
	//Use first four bytes of tx_addr
	packet[4]  = rx_tx_addr[0];
	packet[5]  = rx_tx_addr[1];
	packet[6]  = rx_tx_addr[2];
	packet[7]  = rx_tx_addr[3];

	if(sub_protocol==WLTOYS)
	{
		packet[8]  = hopping_frequency[0];
		packet[9]  = hopping_frequency[1];
		packet[10] = hopping_frequency[2];
		packet[11] = hopping_frequency[3];
	}
	else
	{
		packet[8]  = 0x00;
		packet[9]  = 0x00;
		packet[10] = 0x00;
		packet[11] = 0x00;
	}
	packet[12] = 0x00;
	packet[13] = 0x00;
	packet[14] = 0x00;
	packet[15] = 0x01;	//(USE1MBPS_YES) ? 0x01 : 0x00;

	//Set RF channel
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 83);
}

//-------------------------------------------------------------------------------------------------
// Update control data to be sent
// Do it once per frequency, so the same values will be sent 4 times
// KN uses 4 10-bit data channels plus a 8-bit switch channel
//
// The packet[0] is used for pitch/throttle, the relation is hard coded, not changeable.
// We can change the throttle/pitch range though.
//
// How to use trim? V977 stock controller can trim 6-axis mode to eliminate the drift.
//-------------------------------------------------------------------------------------------------
static void __attribute__((unused)) kn_update_packet_control_data()
{
	uint16_t value;
	value = convert_channel_10b(THROTTLE);
	packet[0]  = (value >> 8) & 0xFF;
	packet[1]  = value & 0xFF;
	value = convert_channel_10b(AILERON);
	packet[2]  = (value >> 8) & 0xFF;
	packet[3]  = value & 0xFF;
	value = convert_channel_10b(ELEVATOR);
	packet[4]  = (value >> 8) & 0xFF;
	packet[5]  = value & 0xFF;
	value = convert_channel_10b(RUDDER);
	packet[6]  = (value >> 8) & 0xFF;
	packet[7]  = value & 0xFF;
	// Trims, middle is 0x64 (100) range 0-200
	packet[8]  = convert_channel_16b_limit(CH9,0,200); // 0x64; // T
	packet[9]  = convert_channel_16b_limit(CH10,0,200); // 0x64; // A
	packet[10] = convert_channel_16b_limit(CH11,0,200); // 0x64; // E
	packet[11] = 0x64; // R

	flags=0;
	if (CH5_SW)
		flags = KN_FLAG_DR;
	if (CH6_SW)
		flags |= KN_FLAG_TH;
	if (CH7_SW)
		flags |= KN_FLAG_IDLEUP;
	if (CH8_SW)
		flags |= KN_FLAG_GYRO3;

	packet[12] = flags;

	packet[13] = 0x00;
	if(sub_protocol==WLTOYS)
		packet[13] = (packet_sent << 5) | (hopping_frequency_no << 2);

	packet[14] = 0x00;
	packet[15] = 0x00;

	NRF24L01_SetPower();
}


//-------------------------------------------------------------------------------------------------
// This function generate RF TX packet address
// V977 can remember the binding parameters; we do not need rebind when power up.
// This requires the address must be repeatable for a specific RF ID at power up.
//-------------------------------------------------------------------------------------------------
static void __attribute__((unused)) kn_calculate_tx_addr()
{
	if(sub_protocol==FEILUN)
	{
		uint8_t addr2;
		// Generate TXID with sum of minimum 256 and maximum 256+MAX_RF_CHANNEL-32
		rx_tx_addr[1] = 1 + rx_tx_addr[0] % (KN_MAX_RF_CHANNEL-33);
		addr2 = 1 + rx_tx_addr[2] % (KN_MAX_RF_CHANNEL-33);
		if ((uint16_t)(rx_tx_addr[0] + rx_tx_addr[1]) < 256)
			rx_tx_addr[2] = addr2;
		else
			rx_tx_addr[2] = 0x00;
		rx_tx_addr[3] = 0x00;
		while((uint16_t)(rx_tx_addr[0] + rx_tx_addr[1] + rx_tx_addr[2] + rx_tx_addr[3]) < 257)
			rx_tx_addr[3] += addr2;
	}
    //The 5th byte is a constant, must be 'K'
    rx_tx_addr[4] = 'K';
}

//-------------------------------------------------------------------------------------------------
// This function generates "random" RF hopping frequency channel numbers.
// These numbers must be repeatable for a specific seed
// The generated number range is from 0 to MAX_RF_CHANNEL. No repeat or adjacent numbers
//
// For Feilun variant, the channels are calculated from TXID, and since only 2 channels are used
// we copy them to fill up to MAX_RF_CHANNEL
//-------------------------------------------------------------------------------------------------
static void __attribute__((unused)) kn_calculate_freqency_hopping_channels()
{
	if(sub_protocol==WLTOYS)
	{
		uint8_t idx = 0;
		uint32_t rnd = MProtocol_id;
		while (idx < KN_RF_CH_COUNT)
		{
			uint8_t i;
			rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization

			// Use least-significant byte. 73 is prime, so channels 76..77 are unused
			uint8_t next_ch = ((rnd >> 8) % KN_MAX_RF_CHANNEL) + 2;
			// Keep the distance 2 between the channels - either odd or even
			if (((next_ch ^ MProtocol_id) & 0x01 )== 0)
				continue;
			// Check that it's not duplicate and spread uniformly
			for (i = 0; i < idx; i++)
				if(hopping_frequency[i] == next_ch)
					break;
			if (i != idx)
				continue;
			hopping_frequency[idx++] = next_ch;
		}
	}
	else
	{//FEILUN
		hopping_frequency[0] = rx_tx_addr[0] + rx_tx_addr[1] + rx_tx_addr[2] + rx_tx_addr[3]; // - 256; ???
		hopping_frequency[1] = hopping_frequency[0] + 32;
		hopping_frequency[2] = hopping_frequency[0];
		hopping_frequency[3] = hopping_frequency[1];
	}
}

//-------------------------------------------------------------------------------------------------
// This function setup 24L01
// V977 uses one way communication, receiving only. 24L01 RX is never enabled.
// V977 needs payload length in the packet. We should configure 24L01 to enable Packet Control Field(PCF)
//   Some RX reg settings are actually for enable PCF
//-------------------------------------------------------------------------------------------------
static void __attribute__((unused)) kn_init()
{
	kn_calculate_tx_addr();
	kn_calculate_freqency_hopping_channels();

	NRF24L01_Initialize();

	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);    // Disable retransmit
	NRF24L01_SetPower();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20);   // bytes of data payload for pipe 0


	NRF24L01_Activate(0x73);
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1); // Dynamic payload for data pipe 0
	// Enable: Dynamic Payload Length to enable PCF
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL));

	NRF24L01_SetPower();

	NRF24L01_FlushTx();
	// Turn radio power on
	NRF24L01_SetTxRxMode(TX_EN);
	NRF24L01_SetBitrate(NRF24L01_BR_1M);	//USE1MBPS_YES ? NRF24L01_BR_1M : NRF24L01_BR_250K;
}
  
//================================================================================================
// Private Functions
//================================================================================================
uint16_t initKN()
{
	if(sub_protocol==WLTOYS)
	{
		packet_period = KN_WL_SENDING_PACKET_PERIOD;
		bind_counter  = KN_WL_BIND_COUNT;
		packet_count  = KN_WL_PACKET_SEND_COUNT;
	}
	else
	{
		packet_period = KN_FX_SENDING_PACKET_PERIOD;
		bind_counter  = KN_FX_BIND_COUNT;
		packet_count  = KN_FX_PACKET_SEND_COUNT;
	}
	kn_init();
	phase = IS_BIND_IN_PROGRESS ? KN_PHASE_PRE_BIND : KN_PHASE_PRE_SEND;

	return KN_INIT_WAIT_MS;
}

uint16_t kn_callback()
{
	switch (phase)
	{
		case KN_PHASE_PRE_BIND:
			kn_bind_init();
			phase=KN_PHASE_BINDING;
			//Do once, no break needed
		case KN_PHASE_BINDING:
			if(bind_counter>0)
			{
				bind_counter--;
				NRF24L01_WritePayload(packet, KN_PAYLOADSIZE);
				return KN_BINDING_PACKET_PERIOD;
			}
			BIND_DONE;
			//Continue
		case KN_PHASE_PRE_SEND:
			packet_sent = 0;
			hopping_frequency_no = 0;
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, KN_TX_ADDRESS_SIZE);
			phase = KN_PHASE_SENDING;
			//Do once, no break needed
		case KN_PHASE_SENDING:
			if(packet_sent >= packet_count)
			{
				packet_sent = 0;
				hopping_frequency_no++;
				if(hopping_frequency_no >= KN_RF_CH_COUNT) hopping_frequency_no = 0;
				kn_update_packet_control_data();
				NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
			}
			else
			{
				// Update sending count and RF channel index.
				// The protocol sends 4 data packets every 2ms per frequency, plus a 2ms gap.
				// Each data packet need a packet number and RF channel index
				packet[13] = 0x00;
				if(sub_protocol==WLTOYS)
					packet[13] = (packet_sent << 5) | (hopping_frequency_no << 2);
			}
			NRF24L01_WritePayload(packet, KN_PAYLOADSIZE);
			packet_sent++;
			return packet_period;
	} //switch
 
    //Bad things happened, reset
    packet_sent = 0;
    phase = KN_PHASE_PRE_SEND;
    return packet_period;
}

#endif


# 1 "src/YD717_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with hexfet new_protocols/yd717_nrf24l01.c dated 2015-09-28

#if defined(YD717_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define YD717_BIND_COUNT		120
#define YD717_PACKET_PERIOD		8000	// Timeout for callback in uSec, 8ms=8000us for YD717
#define YD717_INITIAL_WAIT		50000	// Initial wait before starting callbacks

// Stock tx fixed frequency is 0x3C. Receiver only binds on this freq.
#define YD717_RF_CHANNEL 0x3C

#define YD717_FLAG_FLIP     0x0F
#define YD717_FLAG_LIGHT    0x80
#define YD717_FLAG_PICTURE  0x40
#define YD717_FLAG_VIDEO    0x20
#define YD717_FLAG_HEADLESS 0x10

#define YD717_PAYLOADSIZE 8				// receive data pipes set to this size, but unused

static void __attribute__((unused)) yd717_send_packet(uint8_t bind)
{
	uint8_t rudder_trim, elevator_trim, aileron_trim;
	if (bind)
	{
		packet[0]= rx_tx_addr[0]; // send data phase address in first 4 bytes
		packet[1]= rx_tx_addr[1];
		packet[2]= rx_tx_addr[2];
		packet[3]= rx_tx_addr[3];
		packet[4] = 0x56;
		packet[5] = 0xAA;
		packet[6] = (sub_protocol == NIHUI) ? 0x00 : 0x32;
		packet[7] = 0x00;
	}
	else
	{
		// Throttle
		packet[0] = convert_channel_8b(THROTTLE);
		// Rudder
		if( sub_protocol==XINXUN )
		{
			rudder = convert_channel_8b(RUDDER);
			rudder_trim = (0xff - rudder) >> 1;
		}
		else
		{
			rudder = 0xff - convert_channel_8b(RUDDER);
			rudder_trim = rudder >> 1;
		}
		packet[1] = rudder;
		// Elevator
		elevator = convert_channel_8b(ELEVATOR);
		elevator_trim = elevator >> 1;
		packet[3] = elevator;
		// Aileron
		aileron = 0xff - convert_channel_8b(AILERON);
		aileron_trim = aileron >> 1;
		packet[4] = aileron;
		// Trims
		if( sub_protocol == YD717 )
		{
			packet[2] = elevator_trim;
			packet[5] = aileron_trim;
			packet[6] = rudder_trim;
		}
		else
		{
			packet[2] = rudder_trim;
			packet[5] = elevator_trim;
			packet[6] = aileron_trim;
		}
		// Flags
		flags=0;
		// Channel 5
		if (CH5_SW)	flags = YD717_FLAG_FLIP;
		// Channel 6
		if (CH6_SW)	flags |= YD717_FLAG_LIGHT;
		// Channel 7
		if (CH7_SW)	flags |= YD717_FLAG_PICTURE;
		// Channel 8
		if (CH8_SW)	flags |= YD717_FLAG_VIDEO;
		// Channel 9
		if (CH9_SW)	flags |= YD717_FLAG_HEADLESS;
		packet[7] = flags;
	}

    // clear packet status bits and TX FIFO
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();

	if( sub_protocol == YD717 )
		NRF24L01_WritePayload(packet, 8);
	else
	{
		packet[8] = packet[0];  // checksum
		for(uint8_t i=1; i < 8; i++)
			packet[8] += packet[i];
		packet[8] = ~packet[8];
		NRF24L01_WritePayload(packet, 9);
	}

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) yd717_init()
{
	NRF24L01_Initialize();

	// CRC, radio on
	NRF24L01_SetTxRxMode(TX_EN);
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_PWR_UP)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x3F);				// Enable Acknowledgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);			// Enable all data pipes
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);			// 5-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x1A);		// 500uS retransmit t/o, 10 tries
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, YD717_RF_CHANNEL);	// Channel 3C
	NRF24L01_SetBitrate(NRF24L01_BR_1M);					// 1Mbps
	NRF24L01_SetPower();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent and retransmit

	NRF24L01_Activate(0x73);								// Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F);				// Enable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x07);			// Set feature bits on
	NRF24L01_Activate(0x73);

	// for bind packets set address to prearranged value known to receiver
	uint8_t bind_rx_tx_addr[5];
	uint8_t offset=5;
	if( sub_protocol==SYMAX4 )
		offset=0;
	else
		if( sub_protocol==NIHUI )
			offset=4;
	for(uint8_t i=0; i < 5; i++)
		bind_rx_tx_addr[i]  = 0x60 + offset;
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, bind_rx_tx_addr, 5);
    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, bind_rx_tx_addr, 5);
}

uint16_t yd717_callback()
{
	if(IS_BIND_DONE)
		yd717_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);	// set address
			NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, 5);
			yd717_send_packet(0);
			BIND_DONE;							// bind complete
		}
		else
		{
			yd717_send_packet(1);
			bind_counter--;
		}
	}
	return YD717_PACKET_PERIOD;						// Packet every 8ms
}

uint16_t initYD717()
{
	BIND_IN_PROGRESS;			// autobind protocol
	rx_tx_addr[4] = 0xC1;		// always uses first data port
	yd717_init();
	bind_counter = YD717_BIND_COUNT;

	// Call callback in 50ms
	return YD717_INITIAL_WAIT;
}

#endif

# 1 "src/BUGSMINI_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with MJX Bugs 3 Mini and Bugs 3H

#if defined(BUGSMINI_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define BUGSMINI_INITIAL_WAIT    500
#define BUGSMINI_PACKET_INTERVAL 6840
#define BUGSMINI_WRITE_WAIT      2000
#define BUGSMINI_TX_PAYLOAD_SIZE 24
#define BUGSMINI_RX_PAYLOAD_SIZE 16
#define BUGSMINI_NUM_RF_CHANNELS 15
#define BUGSMINI_ADDRESS_SIZE    5

static uint8_t BUGSMINI_txid[3];
static uint8_t BUGSMINI_txhash;

enum {
    BUGSMINI_BIND1,
    BUGSMINI_BIND2,
    BUGSMINI_DATA1,
    BUGSMINI_DATA2
};

#define BUGSMINI_CH_SW_ARM		CH5_SW
#define BUGSMINI_CH_SW_ANGLE	CH6_SW
#define BUGSMINI_CH_SW_FLIP		CH7_SW
#define BUGSMINI_CH_SW_PICTURE	CH8_SW
#define BUGSMINI_CH_SW_VIDEO	CH9_SW
#define BUGSMINI_CH_SW_LED		CH10_SW

// flags packet[12]
#define BUGSMINI_FLAG_FLIP    0x08    // automatic flip
#define BUGSMINI_FLAG_MODE    0x04    // low/high speed select (set is high speed)
#define BUGSMINI_FLAG_VIDEO   0x02    // toggle video
#define BUGSMINI_FLAG_PICTURE 0x01    // toggle picture

// flags packet[13]
#define BUGSMINI_FLAG_LED     0x80    // enable LEDs
#define BUGSMINI_FLAG_ARM     0x40    // arm (toggle to turn on motors)
#define BUGSMINI_FLAG_DISARM  0x20    // disarm (toggle to turn off motors)
#define BUGSMINI_FLAG_ANGLE   0x02    // angle/acro mode (set is angle mode)

static void __attribute__((unused)) BUGSMINI_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BUGSMINI_RX_PAYLOAD_SIZE); // bytes of data payload for rx pipe 1
	NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x07);
	NRF24L01_SetBitrate(NRF24L01_BR_1M);
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);                          // Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);       // Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x00);     // Set feature bits on
}

static void __attribute__((unused)) BUGSMINI_check_arming()
{
	uint8_t arm_channel = BUGSMINI_CH_SW_ARM;

	if (arm_channel != arm_channel_previous)
	{
		arm_channel_previous = arm_channel;
		if (arm_channel)
		{
			armed = 1;
			arm_flags ^= BUGSMINI_FLAG_ARM;
		}
		else
		{
			armed = 0;
			arm_flags ^= BUGSMINI_FLAG_DISARM;
		}
	}
}

static void __attribute__((unused)) BUGSMINI_send_packet(uint8_t bind)
{
	BUGSMINI_check_arming();  // sets globals arm_flags and armed

	uint16_t aileron  = convert_channel_16b_limit(AILERON,500,0);
	uint16_t elevator = convert_channel_16b_limit(ELEVATOR,0,500);
	uint16_t throttle = armed ? convert_channel_16b_limit(THROTTLE,0,500) : 0;
	uint16_t rudder   = convert_channel_16b_limit(RUDDER,500,0);

	packet[1] = BUGSMINI_txid[0];
	packet[2] = BUGSMINI_txid[1];
	packet[3] = BUGSMINI_txid[2];
	if(bind)
	{
		packet[4] = 0x00;
		packet[5] = 0x7d;
		packet[6] = 0x7d;
		packet[7] = 0x7d;
		packet[8] = 0x20;
		packet[9] = 0x20;
		packet[10]= 0x20;
		packet[11]= 0x40;
		packet[12]^= 0x40;	 // alternating freq hopping flag
		packet[13]= 0x60;
		packet[14]= 0x00;
		packet[15]= 0x00;
	}
	else
	{
		packet[4] = throttle >> 1;
		packet[5] = rudder >> 1;
		packet[6] = elevator >> 1;
		packet[7] = aileron >> 1;
		packet[8] = 0x20 | (aileron << 7);
		packet[9] = 0x20 | (elevator << 7);
		packet[10]= 0x20 | (rudder << 7);
		packet[11]= 0x40 | (throttle << 7);
		packet[12]= 0x80 | (packet[12] ^ 0x40) // bugs 3 H doesn't have 0x80 ?
			| BUGSMINI_FLAG_MODE
			| GET_FLAG(BUGSMINI_CH_SW_PICTURE, BUGSMINI_FLAG_PICTURE)
			| GET_FLAG(BUGSMINI_CH_SW_VIDEO, BUGSMINI_FLAG_VIDEO);
		if(armed)
			packet[12] |= GET_FLAG(BUGSMINI_CH_SW_FLIP, BUGSMINI_FLAG_FLIP);
		packet[13] = arm_flags
			| GET_FLAG(BUGSMINI_CH_SW_LED, BUGSMINI_FLAG_LED)
			| GET_FLAG(BUGSMINI_CH_SW_ANGLE, BUGSMINI_FLAG_ANGLE);

		packet[14] = 0;
		packet[15] = 0; // 0x53 on bugs 3 H ?
	}
	uint8_t checksum = 0x6d;
	for(uint8_t i=1; i < BUGSMINI_TX_PAYLOAD_SIZE; i++)
	checksum ^= packet[i];
	packet[0] = checksum;

	if(!(packet[12]&0x40))
	{
		hopping_frequency_no++;
		if(hopping_frequency_no >= BUGSMINI_NUM_RF_CHANNELS)
			hopping_frequency_no = 0;
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? hopping_frequency[hopping_frequency_no+BUGSMINI_NUM_RF_CHANNELS] : hopping_frequency[hopping_frequency_no]);
	}

	// Power on, TX mode, 2byte CRC
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, BUGSMINI_TX_PAYLOAD_SIZE);
	NRF24L01_SetPower();
}

// compute final address for the rxid received during bind
// thanks to Pascal for the function!
const uint8_t PROGMEM BUGSMINI_end []= {
	0x2d,0x9e ,0x95,0xa4 ,0x9c,0x5c ,0xb4,0xa6 ,0xa9,0xce ,0x56,0x2b ,0x3e,0x73 ,0xb8,0x95 ,0x6a,0x82,
	0x94,0x37 ,0x3d,0x5a ,0x4b,0xb2 ,0x69,0x49 ,0xc2,0x24 ,0x6b,0x3d ,0x23,0xc6 ,0x9e,0xa3 ,0xa4,0x98,
	0x5c,0x9e ,0xa6,0x52 ,0xce,0x76 ,0x2b,0x4b ,0x73,0x3a };
static void __attribute__((unused)) BUGSMINI_make_address()
{
    uint8_t start, length, index;

	//read rxid
	uint8_t base_adr=BUGSMINI_EEPROM_OFFSET+RX_num*2;
    uint8_t rxid_high = eeprom_read_byte((EE_ADDR)(base_adr+0));
    uint8_t rxid_low  = eeprom_read_byte((EE_ADDR)(base_adr+1));
    
    if(rxid_high==0x00 || rxid_high==0xFF)
        rx_tx_addr[0]=0x52;
    else
        rx_tx_addr[0]=rxid_high;
    
    rx_tx_addr[1]=BUGSMINI_txhash;
    
    if(rxid_low==0x00 || rxid_low==0xFF)
        rx_tx_addr[2]=0x66;
    else
        rx_tx_addr[2]=rxid_low;
    
    for(uint8_t end_idx=0;end_idx<23;end_idx++)
    {
        //calculate sequence start
        if(end_idx<=7)
            start=end_idx;
        else
            start=(end_idx-7)*16+7;
        //calculate sequence length
        if(end_idx>6)
        {
            if(end_idx>15)
                length=(23-end_idx)<<1;
            else
                length=16;
        }
        else
            length=(end_idx+1)<<1;
        //calculate first index
        index=start-rxid_high;
        //scan for a possible match using the current end
        for(uint8_t i=0;i<length;i++)
        {
            if(index==rxid_low)
            { //match found
                rx_tx_addr[3]=pgm_read_byte_near( &BUGSMINI_end[end_idx<<1] );
                rx_tx_addr[4]=pgm_read_byte_near( &BUGSMINI_end[(end_idx<<1)+1] );
                return;
            }
            index+=i&1?7:8;	//increment index
        }
    }
    // Something wrong happened if we arrive here....
}

static void __attribute__((unused)) BUGSMINI_update_telemetry()
{
#if defined(BUGS_HUB_TELEMETRY)
	uint8_t checksum = 0x6d;
	for(uint8_t i=1; i<12; i++)
		checksum += packet[i];
	if(packet[0] == checksum)
	{
		RX_RSSI = packet[3];
		if(packet[11] & 0x80)
			v_lipo1 = 0xff; // Ok
		else if(packet[11] & 0x40)
			v_lipo1 = 0x80; // Warning
		else
			v_lipo1 = 0x00; // Critical
	}
#endif
}

uint16_t BUGSMINI_callback()
{
	uint8_t base_adr;
	switch(phase)
	{
		case BUGSMINI_BIND1:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				XN297_ReadPayload(packet, BUGSMINI_RX_PAYLOAD_SIZE);
				base_adr=BUGSMINI_EEPROM_OFFSET+RX_num*2;
				eeprom_write_byte((EE_ADDR)(base_adr+0),packet[1]);	// Save rxid in EEPROM
				eeprom_write_byte((EE_ADDR)(base_adr+1),packet[2]);	// Save rxid in EEPROM
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				BUGSMINI_make_address();
				XN297_SetTXAddr(rx_tx_addr, 5);
				XN297_SetRXAddr(rx_tx_addr, 5);
				phase = BUGSMINI_DATA1;
				BIND_DONE;
				return BUGSMINI_PACKET_INTERVAL;
			}
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(TX_EN);
			BUGSMINI_send_packet(1);
			phase = BUGSMINI_BIND2;
			return BUGSMINI_WRITE_WAIT;
		case BUGSMINI_BIND2:
			// switch to RX mode
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(RX_EN);
			NRF24L01_FlushRx();
			XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) 
						  | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
			phase = BUGSMINI_BIND1;
			return BUGSMINI_PACKET_INTERVAL - BUGSMINI_WRITE_WAIT;
		case BUGSMINI_DATA1:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready => read only 12 bytes to not overwrite channel change flag
				XN297_ReadPayload(packet, 12);
				BUGSMINI_update_telemetry();
			}
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(TX_EN);
			BUGSMINI_send_packet(0);
			phase = BUGSMINI_DATA2;
			return BUGSMINI_WRITE_WAIT;
		case BUGSMINI_DATA2:
			// switch to RX mode
			NRF24L01_SetTxRxMode(TXRX_OFF);
			NRF24L01_SetTxRxMode(RX_EN);
			NRF24L01_FlushRx();
			XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) 
							| _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
			phase = BUGSMINI_DATA1;
			return BUGSMINI_PACKET_INTERVAL - BUGSMINI_WRITE_WAIT;
	}
	return BUGSMINI_PACKET_INTERVAL;
}

#define BUGSMINI_NUM_TX_RF_MAPS 4
// haven't figured out BUGSMINI_txid<-->rf channel mapping yet
const uint8_t PROGMEM BUGSMINI_RF_chans[BUGSMINI_NUM_TX_RF_MAPS][BUGSMINI_NUM_RF_CHANNELS] = {
	{0x22,0x2f,0x3a,0x14,0x20,0x2d,0x38,0x18,0x26,0x32,0x11,0x1d,0x29,0x35,0x17},
	{0x3d,0x34,0x2b,0x22,0x19,0x40,0x37,0x2e,0x25,0x1c,0x3a,0x31,0x28,0x1f,0x16},
	{0x12,0x20,0x2f,0x1a,0x28,0x38,0x14,0x23,0x32,0x1c,0x2c,0x3b,0x17,0x26,0x34},
	{0x13,0x25,0x37,0x1F,0x31,0x17,0x28,0x3A,0x1C,0x2E,0x22,0x33,0x19,0x2B,0x3D} };
const uint8_t PROGMEM BUGSMINI_bind_chans[BUGSMINI_NUM_RF_CHANNELS] = {
	0x1A,0x23,0x2C,0x35,0x3E,0x17,0x20,0x29,0x32,0x3B,0x14,0x1D,0x26,0x2F,0x38}; // bugs 3 mini bind channels
const uint8_t PROGMEM BUGSMINI_tx_id[BUGSMINI_NUM_TX_RF_MAPS][3] = {
	{0xA8,0xE6,0x32},
	{0xdd,0xab,0xfd},
	{0x90,0x9e,0x4a},
	{0x20,0x28,0xBA} };
const uint8_t PROGMEM BUGSMINI_tx_hash[BUGSMINI_NUM_TX_RF_MAPS] = { // 2nd byte of final address
	0x6c,0x9e,0x3d,0xb3};
	
static void __attribute__((unused)) BUGSMINI_initialize_txid()
{
	// load hopping_frequency with tx channels in low part and bind channels in high part
	for(uint8_t i=0; i<BUGSMINI_NUM_RF_CHANNELS;i++)
	{
		hopping_frequency[i]=pgm_read_byte_near( &BUGSMINI_RF_chans[rx_tx_addr[3]%BUGSMINI_NUM_TX_RF_MAPS][i] );
		hopping_frequency[i+BUGSMINI_NUM_RF_CHANNELS]=pgm_read_byte_near( &BUGSMINI_bind_chans[i] );
	}
	// load txid
	for(uint8_t i=0; i<sizeof(BUGSMINI_txid);i++)
		BUGSMINI_txid[i]=pgm_read_byte_near( &BUGSMINI_tx_id[rx_tx_addr[3]%BUGSMINI_NUM_TX_RF_MAPS][i] );
	//load tx_hash
	BUGSMINI_txhash = pgm_read_byte_near( &BUGSMINI_tx_hash[rx_tx_addr[3]%BUGSMINI_NUM_TX_RF_MAPS] );
}

uint16_t initBUGSMINI()
{
	BUGSMINI_initialize_txid();
	memset(packet, (uint8_t)0, BUGSMINI_TX_PAYLOAD_SIZE);
	BUGSMINI_init();
	if(IS_BIND_IN_PROGRESS)
	{
		XN297_SetTXAddr((const uint8_t*)"mjxRC", 5);
		XN297_SetRXAddr((const uint8_t*)"mjxRC", 5);
		phase = BUGSMINI_BIND1;
	}
	else
	{
		BUGSMINI_make_address();
		XN297_SetTXAddr(rx_tx_addr, 5);
		XN297_SetRXAddr(rx_tx_addr, 5);
		phase = BUGSMINI_DATA1;
	}
	armed = 0;
	arm_flags = BUGSMINI_FLAG_DISARM;    // initial value from captures
	arm_channel_previous = BUGSMINI_CH_SW_ARM;
	#ifdef BUGS_HUB_TELEMETRY
		init_frskyd_link_telemetry();
	#endif
	return BUGSMINI_INITIAL_WAIT;
}

#endif


# 1 "src/Bayang_nrf24l01.ino" // Helps debugging !


/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Compatible with EAchine H8 mini, H10, BayangToys X6/X7/X9, JJRC JJ850 ...
// Last sync with hexfet new_protocols/bayang_nrf24l01.c dated 2015-12-22

#if defined(BAYANG_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define BAYANG_BIND_COUNT		1000
#define BAYANG_PACKET_PERIOD	1000
#define BAYANG_INITIAL_WAIT		500
#define BAYANG_PACKET_SIZE		15
#define BAYANG_RF_NUM_CHANNELS	4
#define BAYANG_RF_BIND_CHANNEL	0
#define BAYANG_RF_BIND_CHANNEL_X16_AH 10
#define BAYANG_ADDRESS_LENGTH	5

enum BAYANG_FLAGS {
	// flags going to packet[2]
	BAYANG_FLAG_RTH			= 0x01,
	BAYANG_FLAG_HEADLESS	= 0x02, 
	BAYANG_FLAG_FLIP		= 0x08,
	BAYANG_FLAG_VIDEO		= 0x10, 
	BAYANG_FLAG_PICTURE		= 0x20, 
	// flags going to packet[3]
	BAYANG_FLAG_INVERTED	= 0x80,			// inverted flight on Floureon H101
	BAYANG_FLAG_TAKE_OFF	= 0x20,			// take off / landing on X16 AH
	BAYANG_FLAG_EMG_STOP	= 0x04|0x08,	// 0x08 for VISUO XS809H-W-HD-G
};

static void __attribute__((unused)) BAYANG_send_packet(uint8_t bind)
{
	uint8_t i;
	if (bind)
	{
	#ifdef BAYANG_HUB_TELEMETRY
		if(option)
			packet[0]= 0xA3;	// telemetry is enabled
		else
	#endif
			packet[0]= 0xA4;
		for(i=0;i<5;i++)
			packet[i+1]=rx_tx_addr[i];
		for(i=0;i<4;i++)
			packet[i+6]=hopping_frequency[i];
		switch (sub_protocol)
		{
			case X16_AH:
				packet[10] = 0x00;
				packet[11] = 0x00;
				break;
			case IRDRONE:
				packet[10] = 0x30;
				packet[11] = 0x01;
				break;
			default:
				packet[10] = rx_tx_addr[0];	// txid[0]
				packet[11] = rx_tx_addr[1];	// txid[1]
				break;
		}
	}
	else
	{
		uint16_t val;
		uint8_t dyntrim = 1;
		switch (sub_protocol)
		{
			case X16_AH:
			case IRDRONE:
				packet[0] = 0xA6;
				break;
			default:
				packet[0] = 0xA5;
				break;
		}
		packet[1] = 0xFA;		// normal mode is 0xf7, expert 0xfa

		//Flags packet[2]
		packet[2] = 0x00;
		if(CH5_SW)
			packet[2] = BAYANG_FLAG_FLIP;
		if(CH6_SW)
			packet[2] |= BAYANG_FLAG_RTH;
		if(CH7_SW)
			packet[2] |= BAYANG_FLAG_PICTURE;
		if(CH8_SW)
			packet[2] |= BAYANG_FLAG_VIDEO;
		if(CH9_SW)
		{
			packet[2] |= BAYANG_FLAG_HEADLESS;
			dyntrim = 0;
		}
		//Flags packet[3]
		packet[3] = 0x00;
		if(CH10_SW)
			packet[3] = BAYANG_FLAG_INVERTED;
		if(CH11_SW)
			dyntrim = 0;
		if(CH12_SW)
			packet[3] |= BAYANG_FLAG_TAKE_OFF;
		if(CH13_SW)
			packet[3] |= BAYANG_FLAG_EMG_STOP;
		//Aileron
		val = convert_channel_10b(AILERON);
		packet[4] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[5] = val & 0xFF;
		//Elevator
		val = convert_channel_10b(ELEVATOR);
		packet[6] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[7] = val & 0xFF;
		//Throttle
		val = convert_channel_10b(THROTTLE);
		packet[8] = (val>>8) + 0x7C;
		packet[9] = val & 0xFF;
		//Rudder
		val = convert_channel_10b(RUDDER);
		packet[10] = (val>>8) + (dyntrim ? ((val>>2) & 0xFC) : 0x7C);
		packet[11] = val & 0xFF;
	}
	switch (sub_protocol)
	{
		case H8S3D:
			packet[12] = rx_tx_addr[2];	// txid[2]
			packet[13] = 0x34;
			break;
		case X16_AH:
			packet[12] = 0;
			packet[13] = 0;
			break;
		case IRDRONE:
			packet[12] = 0xE0;
			packet[13] = 0x2E;
			break;
		default:
			packet[12] = rx_tx_addr[2];	// txid[2]
			packet[13] = 0x0A;
			break;
	}
	packet[14] = 0;
	for (uint8_t i=0; i < BAYANG_PACKET_SIZE-1; i++)
		packet[14] += packet[i];

	NRF24L01_WriteReg(NRF24L01_05_RF_CH, bind ? rf_ch_num:hopping_frequency[hopping_frequency_no++]);
	hopping_frequency_no%=BAYANG_RF_NUM_CHANNELS;

	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	XN297_WritePayload(packet, BAYANG_PACKET_SIZE);

	NRF24L01_SetTxRxMode(TXRX_OFF);
	NRF24L01_SetTxRxMode(TX_EN);

	// Power on, TX mode, 2byte CRC
	// Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));

	#ifdef BAYANG_HUB_TELEMETRY
	if (option)
	{	// switch radio to rx as soon as packet is sent
		while (!(NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_TX_DS)));
		NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x03);
    }
	#endif

	NRF24L01_SetPower();	// Set tx_power
}

#ifdef BAYANG_HUB_TELEMETRY
static void __attribute__((unused)) BAYANG_check_rx(void)
{
	if (NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
	{ // data received from model
		XN297_ReadPayload(packet, BAYANG_PACKET_SIZE);
		NRF24L01_WriteReg(NRF24L01_07_STATUS, 255);

		NRF24L01_FlushRx();
		uint8_t check = packet[0];
		for (uint8_t i=1; i < BAYANG_PACKET_SIZE-1; i++)
			check += packet[i];
		// decode data , check sum is ok as well, since there is no crc
		if (packet[0] == 0x85 && packet[14] == check)
		{
			// uncompensated battery volts*100/2
			v_lipo1 = (packet[3]<<7) + (packet[4]>>2);
			// compensated battery volts*100/2
			v_lipo2 = (packet[5]<<7) + (packet[6]>>2);
			// reception in packets / sec
			RX_RSSI = packet[7];
			//Flags
			//uint8_t flags = packet[3] >> 3;
			// battery low: flags & 1
			telemetry_counter++;
			if(telemetry_lost==0)
				telemetry_link=1;
		}
	}
}
#endif

static void __attribute__((unused)) BAYANG_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);

	XN297_SetTXAddr((uint8_t *)"\x00\x00\x00\x00\x00", BAYANG_ADDRESS_LENGTH);

	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     	// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      	// No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  	// Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, BAYANG_PACKET_SIZE);
	NRF24L01_SetBitrate(NRF24L01_BR_1M);             	// 1Mbps
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00);	// No retransmits
	NRF24L01_SetPower();
	NRF24L01_Activate(0x73);							// Activate feature register
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);			// Disable dynamic payload length on all pipes
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
	NRF24L01_Activate(0x73);
	
	switch (sub_protocol)
	{
		case X16_AH:
		case IRDRONE:
			rf_ch_num = BAYANG_RF_BIND_CHANNEL_X16_AH;
			break;
		default:
			rf_ch_num = BAYANG_RF_BIND_CHANNEL;
			break;
	}
}

uint16_t BAYANG_callback()
{
	if(IS_BIND_DONE)
	{
		if(packet_count==0)
			BAYANG_send_packet(0);
		packet_count++;
		#ifdef BAYANG_HUB_TELEMETRY
			if (option)
			{	// telemetry is enabled 
				state++;
				if (state > 1000)
				{
					//calculate telemetry reception packet rate - packets per 1000ms
					TX_RSSI = telemetry_counter;
					telemetry_counter = 0;
					state = 0;
					telemetry_lost=0;
				}

				if (packet_count > 1)
					BAYANG_check_rx();

				packet_count %= 5;
			}
			else
		#endif
				packet_count%=2;
	}
	else
	{
		if (bind_counter == 0)
		{
			XN297_SetTXAddr(rx_tx_addr, BAYANG_ADDRESS_LENGTH);
			#ifdef BAYANG_HUB_TELEMETRY
				XN297_SetRXAddr(rx_tx_addr, BAYANG_ADDRESS_LENGTH);
			#endif
			BIND_DONE;
		}
		else
		{
			if(packet_count==0)
				BAYANG_send_packet(1);
			packet_count++;
			packet_count%=4;
			bind_counter--;
		}
	}
	return BAYANG_PACKET_PERIOD;
}

static void __attribute__((unused)) BAYANG_initialize_txid()
{
	//Could be using txid[0..2] but using rx_tx_addr everywhere instead...
	hopping_frequency[0]=0;
	hopping_frequency[1]=(rx_tx_addr[3]&0x1F)+0x10;
	hopping_frequency[2]=hopping_frequency[1]+0x20;
	hopping_frequency[3]=hopping_frequency[2]+0x20;
	hopping_frequency_no=0;
}

uint16_t initBAYANG(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
    bind_counter = BAYANG_BIND_COUNT;
	BAYANG_initialize_txid();
	BAYANG_init();
	packet_count=0;
	#ifdef BAYANG_HUB_TELEMETRY
		init_frskyd_link_telemetry();
		telemetry_lost=1;	// do not send telemetry to TX right away until we have a TX_RSSI value to prevent warning message...
	#endif
	return BAYANG_INITIAL_WAIT+BAYANG_PACKET_PERIOD;
}

#endif


# 1 "src/SFHSS_cc2500.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Last sync with main deviation/sfhss_cc2500.c dated 2016-03-23

#if defined(SFHSS_CC2500_INO)

#include "iface_cc2500.h"

#define SFHSS_COARSE	0

#define SFHSS_PACKET_LEN 13
#define SFHSS_TX_ID_LEN   2

uint8_t	fhss_code=0; // 0-27

enum {
    SFHSS_START = 0x00,
    SFHSS_CAL   = 0x01,
    SFHSS_DATA1 = 0x02,
    SFHSS_DATA2 = 0x03,
    SFHSS_TUNE  = 0x04
};

#define SFHSS_FREQ0_VAL 0xC4

// Some important initialization parameters, all others are either default,
// or not important in the context of transmitter
// IOCFG2   2F - GDO2_INV=0 GDO2_CFG=2F - HW0
// IOCFG1   2E - GDO1_INV=0 GDO1_CFG=2E - High Impedance
// IOCFG0   2F - GDO0 same as GDO2, TEMP_SENSOR_ENABLE=off
// FIFOTHR  07 - 33 decimal TX threshold
// SYNC1    D3
// SYNC0    91
// PKTLEN   0D - Packet length, 0D bytes
// PKTCTRL1 04 - APPEND_STATUS on, all other are receive parameters - irrelevant
// PKTCTRL0 0C - No whitening, use FIFO, CC2400 compatibility on, use CRC, fixed packet length
// ADDR     29
// CHANNR   10
// FSCTRL1  06 - IF 152343.75Hz, see page 65
// FSCTRL0  00 - zero freq offset
// FREQ2    5C - synthesizer frequency 2399999633Hz for 26MHz crystal, ibid
// FREQ1    4E
// FREQ0    C4
// MDMCFG4  7C - CHANBW_E - 01, CHANBW_M - 03, DRATE_E - 0C. Filter bandwidth = 232142Hz
// MDMCFG3  43 - DRATE_M - 43. Data rate = 128143bps
// MDMCFG2  83 - disable DC blocking, 2-FSK, no Manchester code, 15/16 sync bits detected (irrelevant for TX)
// MDMCFG1  23 - no FEC, 4 preamble bytes, CHANSPC_E - 03
// MDMCFG0  3B - CHANSPC_M - 3B. Channel spacing = 249938Hz (each 6th channel used, resulting in spacing of 1499628Hz)
// DEVIATN  44 - DEVIATION_E - 04, DEVIATION_M - 04. Deviation = 38085.9Hz
// MCSM2    07 - receive parameters, default, irrelevant
// MCSM1    0C - no CCA (transmit always), when packet received stay in RX, when sent go to IDLE
// MCSM0    08 - no autocalibration, PO_TIMEOUT - 64, no pin radio control, no forcing XTAL to stay in SLEEP
// FOCCFG   1D - not interesting, Frequency Offset Compensation
// FREND0   10 - PA_POWER = 0
const PROGMEM uint8_t SFHSS_init_values[] = {
  /* 00 */ 0x2F, 0x2E, 0x2F, 0x07, 0xD3, 0x91, 0x0D, 0x04,
  /* 08 */ 0x0C, 0x29, 0x10, 0x06, 0x00, 0x5C, 0x4E, SFHSS_FREQ0_VAL + SFHSS_COARSE,
  /* 10 */ 0x7C, 0x43, 0x83, 0x23, 0x3B, 0x44, 0x07, 0x0C,
  /* 18 */ 0x08, 0x1D, 0x1C, 0x43, 0x40, 0x91, 0x57, 0x6B,
  /* 20 */ 0xF8, 0xB6, 0x10, 0xEA, 0x0A, 0x11, 0x11
};

static void __attribute__((unused)) SFHSS_rf_init()
{
	CC2500_Strobe(CC2500_SIDLE);

	for (uint8_t i = 0; i < 39; ++i)
		CC2500_WriteReg(i, pgm_read_byte_near(&SFHSS_init_values[i]));

	prev_option = option;
	CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
	
	CC2500_SetTxRxMode(TX_EN);
	CC2500_SetPower();
}

static void __attribute__((unused)) SFHSS_tune_chan()
{
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_WriteReg(CC2500_0A_CHANNR, rf_ch_num*6+16);
	CC2500_Strobe(CC2500_SCAL);
}

static void __attribute__((unused)) SFHSS_tune_chan_fast()
{
	CC2500_Strobe(CC2500_SIDLE);
	CC2500_WriteReg(CC2500_0A_CHANNR, rf_ch_num*6+16);
	CC2500_WriteReg(CC2500_25_FSCAL1, calData[rf_ch_num]);
}

static void __attribute__((unused)) SFHSS_tune_freq()
{
	if ( prev_option != option )
	{
		CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
		CC2500_WriteReg(CC2500_0F_FREQ0, SFHSS_FREQ0_VAL + SFHSS_COARSE);
		prev_option = option ;
		phase = SFHSS_START;	// Restart the tune process if option is changed to get good tuned values
	}
}

static void __attribute__((unused)) SFHSS_calc_next_chan()
{
    rf_ch_num += fhss_code + 2;
    if (rf_ch_num > 29)
	{
        if (rf_ch_num < 31)
			rf_ch_num += fhss_code + 2;
        rf_ch_num -= 31;
    }
}

// Channel values are 12-bit values between 1020 and 2020, 1520 is the middle.
// Futaba @140% is 2070...1520...970
// Values grow down and to the right.
static void __attribute__((unused)) SFHSS_build_data_packet()
{
	uint16_t ch[4];
	// command.bit0 is the packet number indicator: =0 -> SFHSS_DATA1, =1 -> SFHSS_DATA2
	// command.bit1 is unknown but seems to be linked to the payload[0].bit0 but more dumps are needed: payload[0]=0x82 -> =0, payload[0]=0x81 -> =1
	// command.bit2 is the failsafe transmission indicator: =0 -> normal data, =1->failsafe data
	// command.bit3 is the channels indicator: =0 -> CH1-4, =1 -> CH5-8
	
	//Coding below matches the Futaba T8J transmission scheme DATA1->CH1-4, DATA2->CH5-8, DATA1->CH5-8, DATA2->CH1-4,...
	// XK, T10J and TM-FH are different with a classic DATA1->CH1-4, DATA2->CH5-8,...
	//Failsafe is sent twice every couple of seconds (unknown but >5s) 
	
	uint8_t command= (phase == SFHSS_DATA1) ? 0 : 1;	// Building packet for Data1 or Data2
	counter+=command;
	#ifdef FAILSAFE_ENABLE
		if( (counter&0x3FC) == 0x3FC && IS_FAILSAFE_VALUES_on)
		{	// Transmit failsafe data twice every 7s
			if( ((counter&1)^(command&1)) == 0 )
				command|=0x04;							// Failsafe
		}
		else
	#endif
			command|=0x02;								// Assuming packet[0] == 0x81
	counter&=0x3FF;										// Reset failsafe counter
	if(counter&1) command|=0x08;						// Transmit lower and upper channels twice in a row

	uint8_t ch_offset = (command&0x08) >> 1;			// CH1..CH4 or CH5..CH8

	#ifdef FAILSAFE_ENABLE
		if(command&0x04)
		{	//Failsafe data are:
			// 0 to 1023 -> no output on channel
			// 1024-2047 -> hold output on channel
			// 2048-4095 -> channel_output=(data&0x3FF)*5/4+880 in µs
			// Notes:
			//    2048-2559 -> does not look valid since it only covers the range from 1520µs to 2160µs 
			//    2560-3583 -> valid for any channel values from 880µs to 2160µs
			//    3584-4095 -> looks to be used for the throttle channel with values ranging from 880µs to 1520µs
			for(uint8_t i=0;i<4;i++)
			{
				ch[i]=Failsafe_data[CH_AETR[ch_offset+i]];
				if(ch[i]==FAILSAFE_CHANNEL_HOLD)
					ch[i]=1024;
				else if(ch[i]==FAILSAFE_CHANNEL_NOPULSES)
					ch[i]=0;
				else
				{ //Use channel value
					ch[i]=(ch[i]>>1)+2560;
					if(CH_AETR[ch_offset+i]==THROTTLE && ch[i]<3072)		// Throttle
						ch[i]+=1024;
				}
			}
		}
		else
	#endif
		{	//Normal data
			for(uint8_t i=0;i<4;i++)
				ch[i] = convert_channel_16b_nolimit(CH_AETR[ch_offset+i],2020,1020);
		}

	
	// XK		[0]=0x81 [3]=0x00 [4]=0x00
	// T8J		[0]=0x81 [3]=0x42 [4]=0x07
	// T10J		[0]=0x81 [3]=0x0F [4]=0x09
	// TM-FH	[0]=0x82 [3]=0x9A [4]=0x06
	packet[0] = 0x81;	// can be 80 or 81 for Orange, only 81 for XK
	packet[1] = rx_tx_addr[0];
	packet[2] = rx_tx_addr[1];
	packet[3] = 0x00;	// unknown but prevents some receivers to bind if not 0
	packet[4] = 0x00;	// unknown but prevents some receivers to bind if not 0
	packet[5] = (rf_ch_num << 3) | ((ch[0] >> 9) & 0x07);
	packet[6] = (ch[0] >> 1);
	packet[7] = (ch[0] << 7) | ((ch[1] >> 5) & 0x7F );
	packet[8] = (ch[1] << 3) | ((ch[2] >> 9) & 0x07 );
	packet[9] = (ch[2] >> 1);
	packet[10] = (ch[2] << 7) | ((ch[3] >> 5) & 0x7F );
	packet[11] = (ch[3] << 3) | ((fhss_code >> 2) & 0x07 );
	packet[12] = (fhss_code << 6) | command;
}

static void __attribute__((unused)) SFHSS_send_packet()
{
    CC2500_WriteData(packet, SFHSS_PACKET_LEN);
}

uint16_t ReadSFHSS()
{
	switch(phase)
	{
		case SFHSS_START:
			rf_ch_num = 0;
			SFHSS_tune_chan();
			phase = SFHSS_CAL;
			return 2000;
		case SFHSS_CAL:
			calData[rf_ch_num]=CC2500_ReadReg(CC2500_25_FSCAL1);
			if (++rf_ch_num < 30)
				SFHSS_tune_chan();
			else
			{
				rf_ch_num = 0;
				counter = 0;
				phase = SFHSS_DATA1;
			}
			return 2000;

		/* Work cycle: 6.8ms */
#define SFHSS_PACKET_PERIOD	6800
#define SFHSS_DATA2_TIMING	1625	// Adjust this value between 1600 and 1650 if your RX(s) are not operating properly
		case SFHSS_DATA1:
			SFHSS_build_data_packet();
			SFHSS_send_packet();
			phase = SFHSS_DATA2;
			return SFHSS_DATA2_TIMING;								// original 1650
		case SFHSS_DATA2:
			SFHSS_build_data_packet();
			SFHSS_send_packet();
			SFHSS_calc_next_chan();
			phase = SFHSS_TUNE;
			return (SFHSS_PACKET_PERIOD -2000 -SFHSS_DATA2_TIMING);	// original 2000
		case SFHSS_TUNE:
			phase = SFHSS_DATA1;
			SFHSS_tune_freq();
			SFHSS_tune_chan_fast();
			CC2500_SetPower();
			return 2000;											// original 3150
	}
	return 0;
}

// Generate internal id
static void __attribute__((unused)) SFHSS_get_tx_id()
{
	// Some receivers (Orange) behaves better if they tuned to id that has
	//  no more than 6 consecutive zeros and ones
	uint32_t fixed_id;
	uint8_t run_count = 0;
	// add guard for bit count
	fixed_id = 1 ^ (MProtocol_id & 1);
	for (uint8_t i = 0; i < 16; ++i)
	{
		fixed_id = (fixed_id << 1) | (MProtocol_id & 1);
		MProtocol_id >>= 1;
		// If two LS bits are the same
		if ((fixed_id & 3) == 0 || (fixed_id & 3) == 3)
		{
			if (++run_count > 6)
			{
				fixed_id ^= 1;
				run_count = 0;
			}
		}
		else
			run_count = 0;
	}
	//    fixed_id = 0xBC11;
	rx_tx_addr[0] = fixed_id >> 8;
	rx_tx_addr[1] = fixed_id >> 0;
}

uint16_t initSFHSS()
{
	BIND_DONE;	// Not a TX bind protocol
	SFHSS_get_tx_id();

	fhss_code=random(0xfefefefe)%28; // Initialize it to random 0-27 inclusive

	SFHSS_rf_init();
	phase = SFHSS_START;
	return 10000;
}

#endif

# 1 "src/ESky150_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
//  ESky protocol for small models since 2014 (150, 300, 150X, ...)

#if defined(ESKY150_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define ESKY150_PAYLOADSIZE 15
#define ESKY150_TX_ADDRESS_SIZE 4
#define ESKY150_BINDING_PACKET_PERIOD	2000
#define ESKY150_SENDING_PACKET_PERIOD	4800

static void __attribute__((unused)) ESKY150_init()
{
	//Original TX always sets for channelx 0x22 and 0x4a
	// Use channels 2..79
	hopping_frequency[0] = rx_tx_addr[3]%37+2;
	hopping_frequency[1] = hopping_frequency[0] + 40;

	NRF24L01_Initialize();
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, (_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO))); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  // Enable data pipe 0
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);   // 4-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0);    // Disable retransmit
	NRF24L01_SetPower();
	NRF24L01_SetBitrate(NRF24L01_BR_2M);
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, ESKY150_PAYLOADSIZE);   // bytes of data payload for pipe 0
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, ESKY150_TX_ADDRESS_SIZE);

	NRF24L01_Activate(0x73);
	NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 1); // Dynamic payload for data pipe 0
	// Enable: Dynamic Payload Length, Payload with ACK , W_TX_PAYLOAD_NOACK
	NRF24L01_WriteReg(NRF24L01_1D_FEATURE, _BV(NRF2401_1D_EN_DPL) | _BV(NRF2401_1D_EN_ACK_PAY) | _BV(NRF2401_1D_EN_DYN_ACK));
	NRF24L01_Activate(0x73);
	NRF24L01_FlushTx();
	// Turn radio power on
	NRF24L01_SetTxRxMode(TX_EN);
}

static void __attribute__((unused)) ESKY150_bind_init()
{
	uint8_t ESKY150_addr[ESKY150_TX_ADDRESS_SIZE] = { 0x73, 0x73, 0x74, 0x63 }; //This RX address "sstc" is fixed for ESky2

	// Build packet
	packet[0]  = rx_tx_addr[0];
	packet[1]  = rx_tx_addr[1];
	packet[2]  = rx_tx_addr[2];
	packet[3]  = rx_tx_addr[3]; 
	packet[4]  = ESKY150_addr[0];
	packet[5]  = ESKY150_addr[1];
	packet[6]  = ESKY150_addr[2];
	packet[7]  = ESKY150_addr[3];
	packet[8]  = rx_tx_addr[0];
	packet[9]  = rx_tx_addr[1];
	packet[10] = rx_tx_addr[2];
	packet[11] = rx_tx_addr[3]; 
	packet[12] = 0;
	packet[13] = 0;
	packet[14] = 0;

	// Bind address
	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, ESKY150_addr, ESKY150_TX_ADDRESS_SIZE);
	NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, ESKY150_addr, ESKY150_TX_ADDRESS_SIZE);
	
	// Bind Channel 1
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 1);
}

static void __attribute__((unused)) ESKY150_send_packet()
{
	// Build packet
	uint16_t throttle=convert_channel_16b_limit(THROTTLE,1000,2000);
	uint16_t aileron=convert_channel_16b_limit(AILERON,1000,2000);
	uint16_t elevator=convert_channel_16b_limit(ELEVATOR,1000,2000);
	uint16_t rudder=convert_channel_16b_limit(RUDDER,1000,2000);
	//set unused channels to zero, for compatibility with older 4 channel models
	uint8_t flight_mode=0;
	uint16_t aux_ch6=0;
	uint8_t aux_ch7=0;
	if(option==1)
	{
		flight_mode=ESKY150_convert_2bit_channel(CH5);
		aux_ch6=convert_channel_16b_limit(CH6,1000,2000);
		aux_ch7=ESKY150_convert_2bit_channel(CH7);
	}
	packet[0]  = hopping_frequency[0];
	packet[1]  = hopping_frequency[1];
	packet[2]  = ((flight_mode << 6) & 0xC0) | ((aux_ch7 << 4) & 0x30) | ((throttle >> 8) & 0xFF);
	packet[3]  = throttle & 0xFF;
	packet[4]  = ((aux_ch6 >> 4) & 0xF0) | ((aileron >> 8) & 0xFF); //and 0xFF works as values are anyways not bigger than 12 bits, but faster code like that
	packet[5]  = aileron  & 0xFF;
	packet[6]  = (aux_ch6 & 0xF0) | ((elevator >> 8) & 0xFF); //and 0xFF works as values are anyways not bigger than 12 bits, but faster code like that
	packet[7]  = elevator & 0xFF;
	packet[8]  = ((aux_ch6 << 4) & 0xF0) | ((rudder >> 8) & 0xFF); //and 0xFF works as values are anyways not bigger than 12 bits, but faster code like that
	packet[9]  = rudder & 0xFF;
	// The next 4 Bytes are sint8 trim values (TAER). As trims are already included within normal outputs, these values are set to zero.
	packet[10] = 0x00;
	packet[11] = 0x00;
	packet[12] = 0x00;
	packet[13] = 0x00;
	// Calculate checksum:
	uint8_t sum = 0;
	for (uint8_t i = 0; i < 14; ++i)
		sum += packet[i];
	packet[14] = sum;

	// Hop on 2 channels
	hopping_frequency_no++;
	hopping_frequency_no&=0x01;
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);

	// Clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	// Send packet
	NRF24L01_WritePayload(packet, ESKY150_PAYLOADSIZE);

	//Keep transmit power updated
	NRF24L01_SetPower();
}

uint8_t ESKY150_convert_2bit_channel(uint8_t num)
{
	if(Channel_data[num] > CHANNEL_MAX_COMMAND)
		return 0x03;
	else
		if(Channel_data[num] < CHANNEL_MIN_COMMAND)
			return 0x00;
		else
			if(Channel_data[num] > CHANNEL_SWITCH)
				return 0x02;
	return 0x01;	
}

uint16_t ESKY150_callback()
{
	if(IS_BIND_DONE)
		ESKY150_send_packet();
	else
	{
		NRF24L01_WritePayload(packet, ESKY150_PAYLOADSIZE);
		if (--bind_counter == 0)
		{
			BIND_DONE;
			// Change TX address from bind to normal mode
			NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, ESKY150_TX_ADDRESS_SIZE);
		}
		return ESKY150_BINDING_PACKET_PERIOD;
	}
	return ESKY150_SENDING_PACKET_PERIOD;
}

uint16_t initESKY150(void)
{
	ESKY150_init();
	if(IS_BIND_IN_PROGRESS)
	{
		bind_counter=3000;
		ESKY150_bind_init();
	}
	hopping_frequency_no=0;
	return 10000;
}

#endif

# 1 "src/Hubsan_a7105.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with Hubsan H102D, H107/L/C/D and H107P/C+/D+
// Last sync with hexfet new_protocols/hubsan_a7105.c dated 2015-12-11

#if defined(HUBSAN_A7105_INO)

#include "iface_a7105.h"

enum{
	// flags going to packet[9] (H107)
	HUBSAN_FLAG_VIDEO= 0x01,   // record video
	HUBSAN_FLAG_FLIP = 0x08,   // enable flips
	HUBSAN_FLAG_LED  = 0x04    // enable LEDs
};

enum{
	// flags going to packet[9] (H107 Plus series)
	HUBSAN_FLAG_HEADLESS = 0x08, // headless mode
};

enum{
	// flags going to packet[9] (H301)
	FLAG_H301_VIDEO = 0x01,
	FLAG_H301_STAB  = 0x02,
	FLAG_H301_LED   = 0x10,
	FLAG_H301_RTH   = 0x40,
};

enum{
	// flags going to packet[13] (H107 Plus series)
	HUBSAN_FLAG_SNAPSHOT  = 0x01,
	HUBSAN_FLAG_FLIP_PLUS = 0x80,
};

enum{
	// flags going to packet[9] (H501S)
	FLAG_H501_VIDEO     = 0x01,
	FLAG_H501_LED       = 0x04,
	FLAG_H122D_FLIP     = 0x08,	//H122D
	FLAG_H501_RTH       = 0x20,
	FLAG_H501_HEADLESS1 = 0x40,
	FLAG_H501_GPS_HOLD  = 0x80,
	};

enum{
	// flags going to packet[11] (H122D & H123D)
	FLAG_H123D_FMODES   = 0x03,	//H123D 3 FMODES: Sport mode 1, Sport mode 2, Acro
	FLAG_H122D_OSD	    = 0x20,	//H122D OSD
};

enum{
	// flags going to packet[13] (H501S)
	FLAG_H501_SNAPSHOT  = 0x01,
	FLAG_H501_HEADLESS2 = 0x02,
	FLAG_H501_ALT_HOLD  = 0x08,
};

uint32_t hubsan_id_data;

enum {
	BIND_1,
	BIND_2,
	BIND_3,
	BIND_4,
	BIND_5,
	BIND_6,
	BIND_7,
	BIND_8,
	DATA_1,
	DATA_2,
	DATA_3,
	DATA_4,
	DATA_5,
};
#define HUBSAN_WAIT_WRITE 0x80

static void __attribute__((unused)) hubsan_update_crc()
{
	uint8_t sum = 0;
	for(uint8_t i = 0; i < 15; i++)
		sum += packet[i];
	packet[15] = (256 - (sum % 256)) & 0xFF;
}

static void __attribute__((unused)) hubsan_build_bind_packet(uint8_t bind_state)
{
	static uint8_t handshake_counter;
	if(phase < BIND_7)
		handshake_counter = 0;
	memset(packet, 0, 16);
	packet[0] = bind_state;
	packet[1] = channel;
	packet[2] = (MProtocol_id >> 24) & 0xFF;
	packet[3] = (MProtocol_id >> 16) & 0xFF;
	packet[4] = (MProtocol_id >>  8) & 0xFF;
	packet[5] = (MProtocol_id >>  0) & 0xFF;
	if(hubsan_id_data == ID_NORMAL && sub_protocol != H501)
	{
		packet[6] = 0x08;
		packet[7] = 0xe4;
		packet[8] = 0xea;
		packet[9] = 0x9e;
		packet[10] = 0x50;
		//const uint32_t txid = 0xdb042679; 
		packet[11] = 0xDB;
		packet[12] = 0x04;
		packet[13] = 0x26;
		packet[14] = 0x79;
	}
	else
	{ //ID_PLUS
		if(phase >= BIND_3)
		{
			packet[7] = 0x62;
			packet[8] = 0x16;
		}
		if(phase == BIND_7)
			packet[2] = handshake_counter++;
	}
	hubsan_update_crc();
}

//cc : throttle  observed range: 0x00 - 0xFF (smaller is down)
//ee : rudder    observed range: 0x34 - 0xcc (smaller is right)52-204-60%
//gg : elevator  observed range: 0x3e - 0xbc (smaller is up)62-188 -50%
//ii : aileron   observed range: 0x45 - 0xc3 (smaller is right)69-195-50%
static void __attribute__((unused)) hubsan_build_packet()
{
	static uint8_t vtx_freq = 0, h501_packet = 0; 
	memset(packet, 0, 16);
	if(vtx_freq != option || packet_count==100) // set vTX frequency (H107D)
	{
		vtx_freq = option;
		packet[0] = 0x40;	// vtx data packet
		packet[1] = (vtx_freq>0xF2)?0x17:0x16;
		packet[2] = vtx_freq+0x0D;	// 5645 - 5900 MHz
		packet[3] = 0x82;
		packet_count++;      
	}
	else //20 00 00 00 80 00 7d 00 84 02 64 db 04 26 79 7b
	{
		packet[0] = 0x20;	// normal data packet
		packet[2] = convert_channel_8b(THROTTLE);		//Throtle
	}
	packet[4] = 0xFF - convert_channel_8b(RUDDER);		//Rudder is reversed
	packet[6] = 0xFF - convert_channel_8b(ELEVATOR);	//Elevator is reversed
	packet[8] = convert_channel_8b(AILERON);			//Aileron
	if(hubsan_id_data == ID_NORMAL && sub_protocol==H107)
	{// H107/L/C/D, H102D
		if( packet_count < 100)
		{
			packet[9] = 0x02 | HUBSAN_FLAG_LED | HUBSAN_FLAG_FLIP; // sends default value for the 100 first packets
			packet_count++;
		}
		else
		{
			packet[9] = 0x02;
			// Channel 5
			if(CH5_SW)	packet[9] |= HUBSAN_FLAG_FLIP;
			// Channel 6
			if(CH6_SW)	packet[9] |= HUBSAN_FLAG_LED;
			// Channel 8
			if(CH8_SW)	packet[9] |= HUBSAN_FLAG_VIDEO; // H102D
		}
		packet[10] = 0x64;
		//const uint32_t txid = 0xdb042679; 
		packet[11] = 0xDB;
		packet[12] = 0x04;
		packet[13] = 0x26;
		packet[14] = 0x79;
	} else 	if(sub_protocol==H301)
	{// H301
		if( packet_count < 100)
		{
			packet[9] = FLAG_H301_STAB; // sends default value for the 100 first packets
			packet_count++;
		}
		else
		{
            packet[9] = GET_FLAG(CH6_SW, FLAG_H301_LED)
                      | GET_FLAG(CH7_SW, FLAG_H301_STAB)
                      | GET_FLAG(CH8_SW, FLAG_H301_VIDEO)
                      | GET_FLAG(CH5_SW, FLAG_H301_RTH);
		}
		packet[10] = 0x18; // ?
		packet[12] = 0x5c; // ?
		packet[14] = 0xf6; // ?
	}
	else
	{ //ID_PLUS && H501
		packet[3] = sub_protocol==H501 ? 0x00:0x64;
		packet[5] = sub_protocol==H501 ? 0x00:0x64;
		packet[7] = sub_protocol==H501 ? 0x00:0x64;

		if(sub_protocol==H501)
		{ // H501S
			packet[9] = 0x02
					   | GET_FLAG(CH6_SW, FLAG_H501_LED)
					   | GET_FLAG(CH8_SW, FLAG_H501_VIDEO)
					   | GET_FLAG(CH12_SW, FLAG_H122D_FLIP)	// H122D specific -> flip
					   | GET_FLAG(CH5_SW, FLAG_H501_RTH)
					   | GET_FLAG(CH10_SW, FLAG_H501_GPS_HOLD)
					   | GET_FLAG(CH9_SW, FLAG_H501_HEADLESS1);
			//packet[10]= 0x1A;

			//packet[11] content 0x00 is default
			//H123D specific -> Flight modes
			packet[11] = 0x41;	// Sport mode 1
			if(Channel_data[CH13]>CHANNEL_MAX_COMMAND)
				packet[11]=0x43;	// Acro
			else if(Channel_data[CH13]>CHANNEL_MIN_COMMAND)
				packet[11]=0x42;	// Sport mode 2
			//H122D specific -> OSD but useless...
			//packet[11]|= 0x80
			//		  | GET_FLAG(CHXX_SW,FLAG_H122D_OSD); 

			packet[13] = GET_FLAG(CH9_SW, FLAG_H501_HEADLESS2)
					   | GET_FLAG(CH11_SW, FLAG_H501_ALT_HOLD)
					   | GET_FLAG(CH7_SW, FLAG_H501_SNAPSHOT);
		}
		else
		{ // H107P/C+/D+
			packet[9] = 0x06;
			//FLIP|LIGHT|PICTURE|VIDEO|HEADLESS
			if(CH8_SW)	packet[9] |= HUBSAN_FLAG_VIDEO;
			if(CH9_SW)	packet[9] |= HUBSAN_FLAG_HEADLESS;
			packet[10]= 0x19;
			packet[12]= 0x5C; // ghost channel ?
			packet[13] = 0;
			if(CH7_SW)	packet[13]  = HUBSAN_FLAG_SNAPSHOT;
			if(CH5_SW)	packet[13] |= HUBSAN_FLAG_FLIP_PLUS;
			packet[14]= 0x49; // ghost channel ?
		}
		if(packet_count < 100)
		{ // set channels to neutral for first 100 packets
			packet[2] = 0x80; // throttle neutral is at mid stick on plus series
			packet[4] = 0x80;
			packet[6] = 0x80;
			packet[8] = 0x80;
			packet[9] = 0x06;
			packet[13]= 0x00;
			packet_count++;
		}
		if(sub_protocol==H501)
		{ // H501S
			h501_packet++;
			if(h501_packet == 10)
			{
				memset(packet, 0, 16);
				packet[0] = 0xe8;
			}
			else if(h501_packet == 20)
			{
				memset(packet, 0, 16);
				packet[0] = 0xe9;
			}
			if(h501_packet >= 20) h501_packet = 0;
		}
	}
	hubsan_update_crc();
}

#ifdef HUBSAN_HUB_TELEMETRY
static uint8_t __attribute__((unused)) hubsan_check_integrity() 
{
    if( (packet[0]&0xFE) != 0xE0 )
		return 0;
	uint8_t sum = 0;
    for(uint8_t i = 0; i < 15; i++)
        sum += packet[i];
	return ( packet[15] == (uint8_t)(-sum) );
}
#endif

uint16_t ReadHubsan() 
{
#ifdef HUBSAN_HUB_TELEMETRY
	static uint8_t rfMode=0;
#endif
	static uint8_t txState=0;
	uint16_t delay;
	uint8_t i;

	#ifndef FORCE_HUBSAN_TUNING
		A7105_AdjustLOBaseFreq(1);
	#endif
	switch(phase)
	{
		case BIND_1:
			bind_phase++;
			if(bind_phase >= 20 && sub_protocol != H501)
			{
				if(hubsan_id_data == ID_NORMAL)
					hubsan_id_data = ID_PLUS;
				else
					hubsan_id_data = ID_NORMAL;
				A7105_WriteID(hubsan_id_data);    
				bind_phase = 0;
			}
		case BIND_3:
		case BIND_5:
		case BIND_7:
			hubsan_build_bind_packet(phase == BIND_7 ? 9 : (phase == BIND_5 ? 1 : phase + 1 - BIND_1));
			A7105_Strobe(A7105_STANDBY);
			A7105_WriteData(16, channel);
			phase |= HUBSAN_WAIT_WRITE;
			return 3000;
		case BIND_1 | HUBSAN_WAIT_WRITE:
		case BIND_3 | HUBSAN_WAIT_WRITE:
		case BIND_5 | HUBSAN_WAIT_WRITE:
		case BIND_7 | HUBSAN_WAIT_WRITE:
			//wait for completion
			for(i = 0; i< 20; i++)
				if(! (A7105_ReadReg(A7105_00_MODE) & 0x01))
					break;
			A7105_SetTxRxMode(RX_EN);
			A7105_Strobe(A7105_RX);
			phase &= ~HUBSAN_WAIT_WRITE;
			if(hubsan_id_data == ID_PLUS)
			{
				if(phase == BIND_7 && packet[2] == 9)
				{
					phase = DATA_1;
					A7105_WriteReg(A7105_1F_CODE_I, 0x0F);
					BIND_DONE;
					return 4500;
				}
			}
			phase++;
			return 4500; //7.5msec elapsed since last write
		case BIND_2:
		case BIND_4:
		case BIND_6:
			A7105_SetTxRxMode(TX_EN);
			if(A7105_ReadReg(A7105_00_MODE) & 0x01) {
				phase = BIND_1;
				return 4500; //No signal, restart binding procedure.  12msec elapsed since last write
			}
			A7105_ReadData(16);
			phase++;
			if (phase == BIND_5)
				A7105_WriteID(((uint32_t)packet[2] << 24) | ((uint32_t)packet[3] << 16) | ((uint32_t)packet[4] << 8) | packet[5]);
			return 500;  //8msec elapsed time since last write;
		case BIND_8:
			A7105_SetTxRxMode(TX_EN);
			if(A7105_ReadReg(A7105_00_MODE) & 0x01) {
				phase = BIND_7;
				return 15000; //22.5msec elapsed since last write
			}
			A7105_ReadData(16);
			if(packet[1] == 9 && hubsan_id_data == ID_NORMAL) {
				phase = DATA_1;
				A7105_WriteReg(A7105_1F_CODE_I, 0x0F);
				BIND_DONE;
				return 28000; //35.5msec elapsed since last write
			} else {
				phase = BIND_7;
				return 15000; //22.5 msec elapsed since last write
			}
		case DATA_1:
		case DATA_2:
		case DATA_3:
		case DATA_4:
		case DATA_5:
			if( txState == 0) { // send packet
#ifdef HUBSAN_HUB_TELEMETRY
				rfMode = A7105_TX;
#endif
				if( phase == DATA_1)
						A7105_SetPower(); //Keep transmit power in sync
				hubsan_build_packet();
				A7105_Strobe(A7105_STANDBY);
				uint8_t ch;
				if((phase == DATA_5 && hubsan_id_data == ID_NORMAL) && sub_protocol == H107)
					ch = channel + 0x23;
				else
					ch = channel;
				A7105_WriteData(16, ch);
				if (phase == DATA_5)
					phase = DATA_1;
				else
					phase++;
				delay=3000;
			}
			else {
#ifdef HUBSAN_HUB_TELEMETRY
				if( rfMode == A7105_TX)
				{// switch to rx mode 3ms after packet sent
					for( i=0; i<10; i++)
					{
						if( !(A7105_ReadReg(A7105_00_MODE) & 0x01)) {// wait for tx completion
							A7105_SetTxRxMode(RX_EN);
							A7105_Strobe(A7105_RX); 
							rfMode = A7105_RX;
							break;
						}
					}
				}
				if( rfMode == A7105_RX)
				{ // check for telemetry frame
					for( i=0; i<10; i++)
					{
						if( !(A7105_ReadReg(A7105_00_MODE) & 0x01))
						{ // data received
							A7105_ReadData(16);
							if( hubsan_check_integrity() )
							{
								v_lipo1=packet[13]*2;// hubsan lipo voltage 8bits the real value is h_lipo/10(0x2A=42 -> 4.2V)
								telemetry_link=1;
							}	
							A7105_Strobe(A7105_RX);
							// Read TX RSSI
							int16_t temp=256-(A7105_ReadReg(A7105_1D_RSSI_THOLD)*8)/5;		// value from A7105 is between 8 for maximum signal strength to 160 or less
							if(temp<0) temp=0;
							else if(temp>255) temp=255;
							TX_RSSI=temp;
							break;
						}
					}
				}
#endif
				delay=1000;
			}
			if (++txState == 8) { // 3ms + 7*1ms
				A7105_SetTxRxMode(TX_EN);
				txState = 0;
			}
			return delay;
	}
	return 0;
}

uint16_t initHubsan()
{
	const uint8_t allowed_ch[] = {0x14, 0x1e, 0x28, 0x32, 0x3c, 0x46, 0x50, 0x5a, 0x64, 0x6e, 0x78, 0x82};
	A7105_Init();

	channel = allowed_ch[MProtocol_id % sizeof(allowed_ch)];
	hubsan_id_data=ID_NORMAL;

	if(IS_BIND_IN_PROGRESS || sub_protocol==H107)
	{
		BIND_IN_PROGRESS;	// autobind protocol
		phase = BIND_1;
	}
	else 
	{
		phase = DATA_1;
		A7105_WriteID(MProtocol_id);
		A7105_WriteReg(A7105_1F_CODE_I, 0x0F);
	}
	packet_count=0;
	bind_phase=0;
	#ifdef HUBSAN_HUB_TELEMETRY
		init_frskyd_link_telemetry();
	#endif
	return 10000;
}

#endif


# 1 "src/DSM_cyrf6936.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(DSM_CYRF6936_INO)

#include "iface_cyrf6936.h"

#define DSM_BIND_CHANNEL 0x0d //13 This can be any odd channel

//During binding we will send BIND_COUNT/2 packets
//One packet each 10msec
#define DSM_BIND_COUNT 300

enum {
	DSM_BIND_WRITE=0,
	DSM_BIND_CHECK,
	DSM_BIND_READ,
	DSM_CHANSEL,
	DSM_CH1_WRITE_A,
	DSM_CH1_CHECK_A,
	DSM_CH2_WRITE_A,
	DSM_CH2_CHECK_A,
	DSM_CH2_READ_A,
	DSM_CH1_WRITE_B,
	DSM_CH1_CHECK_B,
	DSM_CH2_WRITE_B,
	DSM_CH2_CHECK_B,
	DSM_CH2_READ_B,
};

//
uint8_t sop_col;
uint8_t DSM_num_ch=0;
uint8_t ch_map[14];
const uint8_t PROGMEM DSM_ch_map_progmem[][14] = {
//22+11ms for 4..7 channels
	{1, 0, 2, 3, 0xff, 0xff, 0xff, 1,    0,    2,    3, 0xff, 0xff,    0xff}, //4ch  - Guess
	{1, 0, 2, 3, 4,    0xff, 0xff, 1,    0,    2,    3,    4, 0xff,    0xff}, //5ch  - Guess
	{1, 5, 2, 3, 0,    4,    0xff, 1,    5,    2,    3,    0,    4,    0xff}, //6ch  - HP6DSM
	{1, 5, 2, 4, 3,    6,    0,    1,    5,    2,    4,    3,    6,    0   }, //7ch  - DX6i
//22ms for 8..12 channels
	{1, 5, 2, 3, 6,    0xff, 0xff, 4,    0,    7,    0xff, 0xff, 0xff, 0xff}, //8ch  - DX8/DX7
	{1, 5, 2, 3, 6,    0xff, 0xff, 4,    0,    7,    8,    0xff, 0xff, 0xff}, //9ch  - Guess
	{1, 5, 2, 3, 6,    0xff, 0xff, 4,    0,    7,    8,    9,    0xff, 0xff}, //10ch - Guess
	{1, 5, 2, 3, 6,    10,   0xff, 4,    0,    7,    8,    9,    0xff, 0xff}, //11ch - Guess
	{1, 5, 2, 4, 6,    10,   0xff, 0,    7,    3,    8,    9   , 11  , 0xff}, //12ch - DX18
//11ms for 8..12 channels
	{1, 5, 2, 3, 6,    7,    0xff, 1,    5,    2,    4,    0,    0xff, 0xff}, //8ch  - DX7
	{1, 5, 2, 3, 6,    7,    0xff, 1,    5,    2,    4,    0,    8,    0xff}, //9ch  - Guess
	{1, 5, 2, 3, 4,    8,    9,    1,    5,    2,    3,     0,   7,    6   }, //10ch - DX18
};

const uint8_t PROGMEM DSM_pncodes[5][8][8] = {
	/* Note these are in order transmitted (LSB 1st) */
	{ /* Row 0 */
		/* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
		/* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
		/* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
		/* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
		/* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
		/* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
		/* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
		/* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
		/* Col 8    {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}*/
	},
	{ /* Row 1 */
		/* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
		/* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
		/* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
		/* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
		/* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
		/* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
		/* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
		/* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
		/* Col 8    {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97} */
	},
	{ /* Row 2 */
		/* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
		/* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
		/* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
		/* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
		/* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
		/* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
		/* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
		/* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
		/* Col 8    {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E} */
	},
	{ /* Row 3 */
		/* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
		/* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
		/* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
		/* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
		/* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
		/* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
		/* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
		/* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
		/* Col 8    {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93} */
// Wrong values used by Orange TX/RX
//		/* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
	},
	{ /* Row 4 */
		/* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
		/* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
		/* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
		/* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
		/* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
		/* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
		/* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
		/* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
		/* Col 8    {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8} */
	},
};

static void __attribute__((unused)) DSM_read_code(uint8_t *buf, uint8_t row, uint8_t col, uint8_t len)
{
	for(uint8_t i=0;i<len;i++)
		buf[i]=pgm_read_byte_near( &DSM_pncodes[row][col][i] );
}

static uint8_t __attribute__((unused)) DSM_get_pn_row(uint8_t channel)
{
	return ((sub_protocol == DSMX_11 || sub_protocol == DSMX_22 )? (channel - 2) % 5 : channel % 5);	
}

const uint8_t PROGMEM DSM_init_vals[][2] = {
	{CYRF_02_TX_CTRL, 0x00},				// All TX interrupt disabled
	{CYRF_05_RX_CTRL, 0x00},				// All RX interrupt disabled
	{CYRF_28_CLK_EN, 0x02},					// Force receive clock enable
	{CYRF_32_AUTO_CAL_TIME, 0x3c},			// Default init value
	{CYRF_35_AUTOCAL_OFFSET, 0x14},			// Default init value
	{CYRF_06_RX_CFG, 0x4A},					// LNA enabled, RX override enabled, Fast turn mode enabled, RX is 1MHz below TX
	{CYRF_1B_TX_OFFSET_LSB, 0x55},			// Default init value
	{CYRF_1C_TX_OFFSET_MSB, 0x05},			// Default init value
	{CYRF_39_ANALOG_CTRL, 0x01},			// All slow for synth setting time
	{CYRF_01_TX_LENGTH, 0x10},				// 16 bytes packet
	{CYRF_14_EOP_CTRL, 0x02},				// Set EOP Symbol Count to 2
	{CYRF_12_DATA64_THOLD, 0x0a},			// 64 Chip Data PN corelator threshold, default datasheet value is 0x0E
	//Below is for bind only
	{CYRF_03_TX_CFG, 0x38 | CYRF_BIND_POWER}, //64 chip codes, SDR mode
	{CYRF_10_FRAMING_CFG, 0x4a},			// SOP disabled, no LEN field and SOP correlator of 0x0a but since SOP is disabled...
	{CYRF_1F_TX_OVERRIDE, 0x04},			// Disable TX CRC, no ACK, use TX synthesizer
	{CYRF_1E_RX_OVERRIDE, 0x14},			// Disable RX CRC, Force receive data rate, use RX synthesizer
};

const uint8_t PROGMEM DSM_data_vals[][2] = {
	{CYRF_29_RX_ABORT, 0x20},				// Abort RX operation in case we are coming from bind
	{CYRF_0F_XACT_CFG, 0x24},				// Force Idle
	{CYRF_29_RX_ABORT, 0x00},				// Clear abort RX
	{CYRF_03_TX_CFG, 0x28 | CYRF_HIGH_POWER}, // 64 chip codes, 8DR mode
	{CYRF_10_FRAMING_CFG, 0xea},			// SOP enabled, SOP_CODE_ADR 64 chips, Packet len enabled, SOP correlator 0x0A
	{CYRF_1F_TX_OVERRIDE, 0x00},			// CRC16 enabled, no ACK
	{CYRF_1E_RX_OVERRIDE, 0x00},			// CRC16 enabled, no ACK
};

static void __attribute__((unused)) DSM_cyrf_config()
{
	for(uint8_t i = 0; i < sizeof(DSM_init_vals) / 2; i++)	
		CYRF_WriteRegister(pgm_read_byte_near(&DSM_init_vals[i][0]), pgm_read_byte_near(&DSM_init_vals[i][1]));
	CYRF_WritePreamble(0x333304);
	CYRF_ConfigRFChannel(0x61);
}

static void __attribute__((unused)) DSM_build_bind_packet()
{
	uint8_t i;
	uint16_t sum = 384 - 0x10;//
	packet[0] = 0xff ^ cyrfmfg_id[0];
	packet[1] = 0xff ^ cyrfmfg_id[1];
	packet[2] = 0xff ^ cyrfmfg_id[2];
	packet[3] = 0xff ^ cyrfmfg_id[3];
	packet[4] = packet[0];
	packet[5] = packet[1];
	packet[6] = packet[2];
	packet[7] = packet[3];
	for(i = 0; i < 8; i++)
		sum += packet[i];
	packet[8] = sum >> 8;
	packet[9] = sum & 0xff;
	packet[10] = 0x01; //???
	packet[11] = DSM_num_ch;

	if (sub_protocol==DSM2_22)
		packet[12]=DSM_num_ch<8?0x01:0x02;	// DSM2/1024 1 or 2 packets depending on the number of channels
	if(sub_protocol==DSM2_11)
		packet[12]=0x12;					// DSM2/2048 2 packets
	if(sub_protocol==DSMX_22)
		#if defined DSM_TELEMETRY
			packet[12] = 0xb2;				// DSMX/2048 2 packets
		#else
			packet[12] = DSM_num_ch<8? 0xa2 : 0xb2;	// DSMX/2048 1 or 2 packets depending on the number of channels
		#endif
	if(sub_protocol==DSMX_11 || sub_protocol==DSM_AUTO) // Force DSMX/1024 in mode Auto
		packet[12]=0xb2;					// DSMX/1024 2 packets
	
	packet[13] = 0x00; //???
	for(i = 8; i < 14; i++)
		sum += packet[i];
	packet[14] = sum >> 8;
	packet[15] = sum & 0xff;
}

static void __attribute__((unused)) DSM_initialize_bind_phase()
{
	CYRF_ConfigRFChannel(DSM_BIND_CHANNEL); //This seems to be random?
	//64 SDR Mode is configured so only the 8 first values are needed but need to write 16 values...
	CYRF_ConfigDataCode((const uint8_t*)"\xD7\xA1\x54\xB1\x5E\x89\xAE\x86\xc6\x94\x22\xfe\x48\xe6\x57\x4e", 16);
	DSM_build_bind_packet();
}

static void __attribute__((unused)) DSM_cyrf_configdata()
{
	for(uint8_t i = 0; i < sizeof(DSM_data_vals) / 2; i++)
		CYRF_WriteRegister(pgm_read_byte_near(&DSM_data_vals[i][0]), pgm_read_byte_near(&DSM_data_vals[i][1]));
}

static void __attribute__((unused)) DSM_update_channels()
{
	prev_option=option;
	if(sub_protocol==DSM_AUTO)
		DSM_num_ch=12;						// Force 12 channels in mode Auto
	else
		DSM_num_ch=option;
	if(DSM_num_ch<4 || DSM_num_ch>12)
		DSM_num_ch=6;						// Default to 6 channels if invalid choice...

	// Create channel map based on number of channels and refresh rate
	uint8_t idx=DSM_num_ch-4;
	if(DSM_num_ch>7 && DSM_num_ch<11 && (sub_protocol==DSM2_11 || sub_protocol==DSMX_11))
		idx+=5;								// In 11ms mode change index only for channels 8..10
	for(uint8_t i=0;i<14;i++)
		ch_map[i]=pgm_read_byte_near(&DSM_ch_map_progmem[idx][i]);
}

static void __attribute__((unused)) DSM_build_data_packet(uint8_t upper)
{
	uint8_t bits = 11;

	if(prev_option!=option)
		DSM_update_channels();

	if (sub_protocol==DSMX_11 || sub_protocol==DSMX_22 )
	{
		packet[0] = cyrfmfg_id[2];
		packet[1] = cyrfmfg_id[3];
	}
	else
	{
		packet[0] = (0xff ^ cyrfmfg_id[2]);
		packet[1] = (0xff ^ cyrfmfg_id[3]);
		if(sub_protocol==DSM2_22)
			bits=10;						// Only DSM_22 is using a resolution of 1024
	}
	#ifdef DSM_THROTTLE_KILL_CH
		uint16_t kill_ch=Channel_data[DSM_THROTTLE_KILL_CH-1];
	#endif
	for (uint8_t i = 0; i < 7; i++)
	{	
		uint8_t idx = ch_map[(upper?7:0) + i];//1,5,2,3,0,4	   
		uint16_t value = 0xffff;;	
		if (idx != 0xff)
		{
			/* Spektrum own remotes transmit normal values during bind and actually use this (e.g. Nano CP X) to
			   select the transmitter mode (e.g. computer vs non-computer radio), so always send normal output */
			#ifdef DSM_THROTTLE_KILL_CH
				if(CH_TAER[idx]==THROTTLE && kill_ch<=604)
				{//Activate throttle kill only if DSM_THROTTLE_KILL_CH below -50%
					if(kill_ch<CHANNEL_MIN_100)					// restrict val to 0...400
						kill_ch=0;
					else
						kill_ch-=CHANNEL_MIN_100;
					value=(kill_ch*21)/25;			// kill channel -100%->904us ... -50%->1100us *0x150/400
				}
				else
			#endif
				#ifdef DSM_MAX_THROW
					value=Channel_data[CH_TAER[idx]];								// -100%..+100% => 1024..1976us and -125%..+125% => 904..2096us based on Redcon 6 channel DSM2 RX
				#else
					value=convert_channel_16b_nolimit(CH_TAER[idx],0x150,0x6B0);	// -100%..+100% => 1100..1900us and -125%..+125% => 1000..2000us based on Redcon 6 channel DSM2 RX
				#endif
			if(bits==10) value>>=1;
			value |= (upper && i==0 ? 0x8000 : 0) | (idx << bits);
		}	  
		packet[i*2+2] = (value >> 8) & 0xff;
		packet[i*2+3] = (value >> 0) & 0xff;
	}
}

static void __attribute__((unused)) DSM_set_sop_data_crc()
{
	//The crc for channel '1' is NOT(mfgid[0] << 8 + mfgid[1])
	//The crc for channel '2' is (mfgid[0] << 8 + mfgid[1])
	uint16_t crc = (cyrfmfg_id[0] << 8) + cyrfmfg_id[1];
	if(phase==DSM_CH1_CHECK_A||phase==DSM_CH1_CHECK_B)
		CYRF_ConfigCRCSeed(crc);	//CH2
	else
		CYRF_ConfigCRCSeed(~crc);	//CH1

	uint8_t pn_row = DSM_get_pn_row(hopping_frequency[hopping_frequency_no]);
	uint8_t code[16];
	DSM_read_code(code,pn_row,sop_col,8);					// pn_row between 0 and 4, sop_col between 1 and 7
	CYRF_ConfigSOPCode(code);
	DSM_read_code(code,pn_row,7 - sop_col,8);				// 7-sop_col between 0 and 6
	DSM_read_code(code+8,pn_row,7 - sop_col + 1,8);			// 7-sop_col+1 between 1 and 7
	CYRF_ConfigDataCode(code, 16);

	CYRF_ConfigRFChannel(hopping_frequency[hopping_frequency_no]);
	hopping_frequency_no++;
	if(sub_protocol == DSMX_11 || sub_protocol == DSMX_22)
		hopping_frequency_no %=23;
	else
		hopping_frequency_no %=2;
}

static void __attribute__((unused)) DSM_calc_dsmx_channel()
{
	uint8_t idx = 0;
	uint32_t id = ~(((uint32_t)cyrfmfg_id[0] << 24) | ((uint32_t)cyrfmfg_id[1] << 16) | ((uint32_t)cyrfmfg_id[2] << 8) | (cyrfmfg_id[3] << 0));
	uint32_t id_tmp = id;
	while(idx < 23)
	{
		uint8_t i;
		uint8_t count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;
		id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F;		// Randomization
		uint8_t next_ch = ((id_tmp >> 8) % 0x49) + 3;	// Use least-significant byte and must be larger than 3
		if ( (next_ch ^ cyrfmfg_id[3]) & 0x01 )
			continue;
		for (i = 0; i < idx; i++)
		{
			if(hopping_frequency[i] == next_ch)
				break;
			if(hopping_frequency[i] <= 27)
				count_3_27++;
			else
				if (hopping_frequency[i] <= 51)
					count_28_51++;
				else
					count_52_76++;
		}
		if (i != idx)
			continue;
		if ((next_ch < 28 && count_3_27 < 8)
			||(next_ch >= 28 && next_ch < 52 && count_28_51 < 7)
			||(next_ch >= 52 && count_52_76 < 8))
			hopping_frequency[idx++] = next_ch;
	}
}

static uint8_t __attribute__((unused)) DSM_Check_RX_packet()
{
	uint8_t result=1;						// assume good packet
	
	uint16_t sum = 384 - 0x10;
	for(uint8_t i = 1; i < 9; i++)
	{
		sum += pkt[i];
		if(i<5)
			if(pkt[i] != (0xff ^ cyrfmfg_id[i-1]))
				result=0; 					// bad packet
	}
	if( pkt[9] != (sum>>8)  && pkt[10] != (uint8_t)sum )
		result=0;
	return result;
}

uint16_t ReadDsm()
{
#define DSM_CH1_CH2_DELAY	4010			// Time between write of channel 1 and channel 2
#define DSM_WRITE_DELAY		1950			// Time after write to verify write complete
#define DSM_READ_DELAY		600				// Time before write to check read phase, and switch channels. Was 400 but 600 seems what the 328p needs to read a packet
	#if defined DSM_TELEMETRY
		uint8_t rx_phase;
		uint8_t len;
	#endif
	uint8_t start;
	
	switch(phase)
	{
		case DSM_BIND_WRITE:
			if(bind_counter--==0)
			#if defined DSM_TELEMETRY
				phase=DSM_BIND_CHECK;						//Check RX answer
			#else
				phase=DSM_CHANSEL;							//Switch to normal mode
			#endif
			CYRF_WriteDataPacket(packet);
			return 10000;
	#if defined DSM_TELEMETRY
		case DSM_BIND_CHECK:
			//64 SDR Mode is configured so only the 8 first values are needed but we need to write 16 values...
			CYRF_ConfigDataCode((const uint8_t *)"\x98\x88\x1B\xE4\x30\x79\x03\x84\xC9\x2C\x06\x93\x86\xB9\x9E\xD7", 16);
			CYRF_SetTxRxMode(RX_EN);						//Receive mode
			CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x87);		//Prepare to receive
			bind_counter=2*DSM_BIND_COUNT;					//Timeout of 4.2s if no packet received
			phase++;										// change from BIND_CHECK to BIND_READ
			return 2000;
		case DSM_BIND_READ:
			//Read data from RX
			rx_phase = CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			if((rx_phase & 0x03) == 0x02)  					// RXC=1, RXE=0 then 2nd check is required (debouncing)
				rx_phase |= CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			if((rx_phase & 0x07) == 0x02)
			{ // data received with no errors
				CYRF_WriteRegister(CYRF_07_RX_IRQ_STATUS, 0x80);	// need to set RXOW before data read
				len=CYRF_ReadRegister(CYRF_09_RX_COUNT);
				if(len>MAX_PKT-2)
					len=MAX_PKT-2;
				CYRF_ReadDataPacketLen(pkt+1, len);
				if(len==10 && DSM_Check_RX_packet())
				{
					pkt[0]=0x80;
					telemetry_link=1;						// send received data on serial
					phase++;
					return 2000;
				}
			}
			else
				if((rx_phase & 0x02) != 0x02)
				{ // data received with errors
					CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x20);	// Abort RX operation
					CYRF_SetTxRxMode(RX_EN);					// Force end state read
					CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x00);	// Clear abort RX operation
					CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x83);	// Prepare to receive
				}
			if( --bind_counter == 0 )
			{ // Exit if no answer has been received for some time
				phase++;									// DSM_CHANSEL
				return 7000 ;
			}
			return 7000;
	#endif
		case DSM_CHANSEL:
			BIND_DONE;
			DSM_cyrf_configdata();
			CYRF_SetTxRxMode(TX_EN);
			hopping_frequency_no = 0;
			phase = DSM_CH1_WRITE_A;						// in fact phase++
			DSM_set_sop_data_crc();
			return 10000;
		case DSM_CH1_WRITE_A:
		case DSM_CH1_WRITE_B:
		case DSM_CH2_WRITE_A:
		case DSM_CH2_WRITE_B:
			DSM_build_data_packet(phase == DSM_CH1_WRITE_B||phase == DSM_CH2_WRITE_B);	// build lower or upper channels
			CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS);		// clear IRQ flags
			CYRF_WriteDataPacket(packet);
			phase++;										// change from WRITE to CHECK mode
			return DSM_WRITE_DELAY;
		case DSM_CH1_CHECK_A:
		case DSM_CH1_CHECK_B:
		case DSM_CH2_CHECK_A:
		case DSM_CH2_CHECK_B:
			start=(uint8_t)micros();
			while ((uint8_t)((uint8_t)micros()-(uint8_t)start) < 100)			// Wait max 100µs, max I've seen is 50µs
				if((CYRF_ReadRegister(CYRF_02_TX_CTRL) & 0x80) == 0x00)
					break;
			if(phase==DSM_CH1_CHECK_A || phase==DSM_CH1_CHECK_B)
			{
				#if defined DSM_TELEMETRY
					// reset cyrf6936 if freezed after switching from TX to RX
					if (((CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x22) == 0x20) || (CYRF_ReadRegister(CYRF_02_TX_CTRL) & 0x80))
					{
						CYRF_Reset();
						DSM_cyrf_config();
						DSM_cyrf_configdata();
						CYRF_SetTxRxMode(TX_EN);
					}
				#endif
				DSM_set_sop_data_crc();
				phase++;										// change from CH1_CHECK to CH2_WRITE
				return DSM_CH1_CH2_DELAY - DSM_WRITE_DELAY;
			}
			if (phase == DSM_CH2_CHECK_A)
				CYRF_SetPower(0x28);						//Keep transmit power in sync
#if defined DSM_TELEMETRY
			phase++;										// change from CH2_CHECK to CH2_READ
			CYRF_SetTxRxMode(RX_EN);						//Receive mode
			CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x87);		//0x80??? //Prepare to receive
			return 11000 - DSM_CH1_CH2_DELAY - DSM_WRITE_DELAY - DSM_READ_DELAY;
		case DSM_CH2_READ_A:
		case DSM_CH2_READ_B:
			//Read telemetry
			rx_phase = CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			if((rx_phase & 0x03) == 0x02)  					// RXC=1, RXE=0 then 2nd check is required (debouncing)
				rx_phase |= CYRF_ReadRegister(CYRF_07_RX_IRQ_STATUS);
			if((rx_phase & 0x07) == 0x02)
			{ // good data (complete with no errors)
				CYRF_WriteRegister(CYRF_07_RX_IRQ_STATUS, 0x80);	// need to set RXOW before data read
				len=CYRF_ReadRegister(CYRF_09_RX_COUNT);
				if(len>MAX_PKT-2)
					len=MAX_PKT-2;
				CYRF_ReadDataPacketLen(pkt+1, len);
				pkt[0]=CYRF_ReadRegister(CYRF_13_RSSI)&0x1F;// store RSSI of the received telemetry signal
				telemetry_link=1;
			}
			CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x20);		// Abort RX operation
			if (phase == DSM_CH2_READ_A && (sub_protocol==DSM2_22 || sub_protocol==DSMX_22) && DSM_num_ch < 8)	// 22ms mode
			{
				CYRF_SetTxRxMode(RX_EN);					// Force end state read
				CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x00);	// Clear abort RX operation
				CYRF_WriteRegister(CYRF_05_RX_CTRL, 0x87);	//0x80???	//Prepare to receive
				phase = DSM_CH2_READ_B;
				return 11000;
			}
			if (phase == DSM_CH2_READ_A)
				phase = DSM_CH1_WRITE_B;					//Transmit upper
			else
				phase = DSM_CH1_WRITE_A;					//Transmit lower
			CYRF_SetTxRxMode(TX_EN);						//TX mode
			CYRF_WriteRegister(CYRF_29_RX_ABORT, 0x00);		//Clear abort RX operation
			DSM_set_sop_data_crc();
			return DSM_READ_DELAY;
#else
			// No telemetry
			DSM_set_sop_data_crc();
			if (phase == DSM_CH2_CHECK_A)
			{
				if(DSM_num_ch > 7 || sub_protocol==DSM2_11 || sub_protocol==DSMX_11)
					phase = DSM_CH1_WRITE_B;				//11ms mode or upper to transmit change from CH2_CHECK_A to CH1_WRITE_A
				else										
				{											//Normal mode 22ms
					phase = DSM_CH1_WRITE_A;				// change from CH2_CHECK_A to CH1_WRITE_A (ie no upper)
					return 22000 - DSM_CH1_CH2_DELAY - DSM_WRITE_DELAY ;
				}
			}
			else
				phase = DSM_CH1_WRITE_A;					// change from CH2_CHECK_B to CH1_WRITE_A (upper already transmitted so transmit lower)
			return 11000 - DSM_CH1_CH2_DELAY - DSM_WRITE_DELAY;
#endif
	}
	return 0;		
}

uint16_t initDsm()
{ 
	CYRF_GetMfgData(cyrfmfg_id);
	//Model match
	cyrfmfg_id[3]^=RX_num;
	//Calc sop_col
	sop_col = (cyrfmfg_id[0] + cyrfmfg_id[1] + cyrfmfg_id[2] + 2) & 0x07;
	//Fix for OrangeRX using wrong DSM_pncodes by preventing access to "Col 8"
	if(sop_col==0)
	{
	   cyrfmfg_id[rx_tx_addr[0]%3]^=0x01;					//Change a bit so sop_col will be different from 0
	   sop_col = (cyrfmfg_id[0] + cyrfmfg_id[1] + cyrfmfg_id[2] + 2) & 0x07;
	}
	//Hopping frequencies
	if (sub_protocol == DSMX_11 || sub_protocol == DSMX_22)
		DSM_calc_dsmx_channel();
	else
	{ 
		uint8_t tmpch[10];
		CYRF_FindBestChannels(tmpch, 10, 5, 3, 75);
		//
		uint8_t idx = random(0xfefefefe) % 10;
		hopping_frequency[0] = tmpch[idx];
		while(1)
		{
			idx = random(0xfefefefe) % 10;
			if (tmpch[idx] != hopping_frequency[0])
				break;
		}
		hopping_frequency[1] = tmpch[idx];
	}
	//
	DSM_cyrf_config();
	CYRF_SetTxRxMode(TX_EN);
	//
	DSM_update_channels();
	//
	if(IS_BIND_IN_PROGRESS)
	{
		DSM_initialize_bind_phase();		
		phase = DSM_BIND_WRITE;
		bind_counter=DSM_BIND_COUNT;
	}
	else
		phase = DSM_CHANSEL;//
	return 10000;
}

#endif


# 1 "src/Devo_cyrf6936.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(DEVO_CYRF6936_INO) 
 
#include "iface_cyrf6936.h"

#define DEVO_NUM_CHANNELS 8

//For Debug
//#define NO_SCRAMBLE

#define DEVO_PKTS_PER_CHANNEL	4
#define DEVO_BIND_COUNT			0x1388

#define DEVO_NUM_WAIT_LOOPS (100 / 5) //each loop is ~5us.  Do not wait more than 100us

enum {
	DEVO_BIND,
	DEVO_BIND_SENDCH,
	DEVO_BOUND,
	DEVO_BOUND_1,
	DEVO_BOUND_2,
	DEVO_BOUND_3,
	DEVO_BOUND_4,
	DEVO_BOUND_5,
	DEVO_BOUND_6,
	DEVO_BOUND_7,
	DEVO_BOUND_8,
	DEVO_BOUND_9,
	DEVO_BOUND_10,
};

static void __attribute__((unused)) DEVO_scramble_pkt()
{
#ifdef NO_SCRAMBLE
	return;
#else
	for(uint8_t i = 0; i < 15; i++)
		packet[i + 1] ^= cyrfmfg_id[i % 4];
#endif
}

static void __attribute__((unused)) DEVO_add_pkt_suffix()
{
    uint8_t bind_state;
	#ifdef ENABLE_PPM
	if(mode_select && option==0 && IS_BIND_DONE) 			//PPM mode and option not already set and bind is finished
	{
		BIND_SET_INPUT;
		BIND_SET_PULLUP;										// set pullup
		if(IS_BIND_BUTTON_on)
		{
			eeprom_write_byte((EE_ADDR)(MODELMODE_EEPROM_OFFSET+RX_num),0x01);	// Set fixed id mode for the current model
			option=1;
		}
		BIND_SET_OUTPUT;
	}
	#endif //ENABLE_PPM
    if(prev_option!=option && IS_BIND_DONE)
	{
		MProtocol_id = RX_num + MProtocol_id_master;
		bind_counter=DEVO_BIND_COUNT;
	}
	if (option)
	{
        if (bind_counter > 0)
            bind_state = 0xc0;
        else
            bind_state = 0x80;
    }
	else
        bind_state = 0x00;
	packet[10] = bind_state | (DEVO_PKTS_PER_CHANNEL - packet_count - 1);
	packet[11] = *(hopping_frequency_ptr + 1);
	packet[12] = *(hopping_frequency_ptr + 2);
	packet[13] = MProtocol_id  & 0xff;
	packet[14] = (MProtocol_id >> 8) & 0xff;
	packet[15] = (MProtocol_id >> 16) & 0xff;
}

static void __attribute__((unused)) DEVO_build_beacon_pkt(uint8_t upper)
{
	packet[0] = (DEVO_NUM_CHANNELS << 4) | 0x07;
	uint8_t max = 8, offset = 0, enable = 0;
	if (upper)
	{
		packet[0] += 1;
		max = 4;
		offset = 8;
	}
	for(uint8_t i = 0; i < max; i++)
	{
		#ifdef FAILSAFE_ENABLE
			uint16_t failsafe=Failsafe_data[CH_EATR[i+offset]];
			if(i + offset < DEVO_NUM_CHANNELS && failsafe!=FAILSAFE_CHANNEL_HOLD && IS_FAILSAFE_VALUES_on)
			{
				enable |= 0x80 >> i;
				packet[i+1] = ((failsafe*25)>>8)-100;
			}
			else
		#else
			(void)offset;
		#endif
				packet[i+1] = 0;
	}
	packet[9] = enable;
	DEVO_add_pkt_suffix();
}

static void __attribute__((unused)) DEVO_build_bind_pkt()
{
	packet[0] = (DEVO_NUM_CHANNELS << 4) | 0x0a;
	packet[1] = bind_counter & 0xff;
	packet[2] = (bind_counter >> 8);
	packet[3] = *hopping_frequency_ptr;
	packet[4] = *(hopping_frequency_ptr + 1);
	packet[5] = *(hopping_frequency_ptr + 2);
	packet[6] = cyrfmfg_id[0];
	packet[7] = cyrfmfg_id[1];
	packet[8] = cyrfmfg_id[2];
	packet[9] = cyrfmfg_id[3];
	DEVO_add_pkt_suffix();
	//The fixed-id portion is scrambled in the bind packet
	//I assume it is ignored
	packet[13] ^= cyrfmfg_id[0];
	packet[14] ^= cyrfmfg_id[1];
	packet[15] ^= cyrfmfg_id[2];
}

static void __attribute__((unused)) DEVO_build_data_pkt()
{
	static uint8_t ch_idx=0;

	packet[0] = (DEVO_NUM_CHANNELS << 4) | (0x0b + ch_idx);
	uint8_t sign = 0x0b;
	for (uint8_t i = 0; i < 4; i++)
	{
		int16_t value=convert_channel_16b_nolimit(CH_EATR[ch_idx * 4 + i],-1600,1600);//range -1600..+1600
		if(value < 0)
		{
			value = -value;
			sign |= 1 << (7 - i);
		}
		packet[2 * i + 1] = value & 0xff;
		packet[2 * i + 2] = (value >> 8) & 0xff;
	}
	packet[9] = sign;
	ch_idx++;
	if (ch_idx * 4 >= DEVO_NUM_CHANNELS)
		ch_idx = 0;
	DEVO_add_pkt_suffix();
}

static void __attribute__((unused)) DEVO_cyrf_set_bound_sop_code()
{
	/* crc == 0 isn't allowed, so use 1 if the math results in 0 */
	uint8_t crc = (cyrfmfg_id[0] + (cyrfmfg_id[1] >> 6) + cyrfmfg_id[2]);
	if(! crc)
		crc = 1;
	uint8_t sopidx = (0xff &((cyrfmfg_id[0] << 2) + cyrfmfg_id[1] + cyrfmfg_id[2])) % 10;
	CYRF_SetTxRxMode(TX_EN);
	CYRF_ConfigCRCSeed((crc << 8) + crc);
	CYRF_PROGMEM_ConfigSOPCode(DEVO_j6pro_sopcodes[sopidx]);
	CYRF_SetPower(0x08);
}

const uint8_t PROGMEM DEVO_init_vals[][2] = {
	{ CYRF_1D_MODE_OVERRIDE, 0x38 },
	{ CYRF_03_TX_CFG, 0x08 },
	{ CYRF_06_RX_CFG, 0x4A },
	{ CYRF_0B_PWR_CTRL, 0x00 },
	{ CYRF_10_FRAMING_CFG, 0xA4 },
	{ CYRF_11_DATA32_THOLD, 0x05 },
	{ CYRF_12_DATA64_THOLD, 0x0E },
	{ CYRF_1B_TX_OFFSET_LSB, 0x55 },
	{ CYRF_1C_TX_OFFSET_MSB, 0x05 },
	{ CYRF_32_AUTO_CAL_TIME, 0x3C },
	{ CYRF_35_AUTOCAL_OFFSET, 0x14 },
	{ CYRF_39_ANALOG_CTRL, 0x01 },
	{ CYRF_1E_RX_OVERRIDE, 0x10 },
	{ CYRF_1F_TX_OVERRIDE, 0x00 },
	{ CYRF_01_TX_LENGTH, 0x10 },
	{ CYRF_0F_XACT_CFG, 0x10 },
	{ CYRF_27_CLK_OVERRIDE, 0x02 },
	{ CYRF_28_CLK_EN, 0x02 },
	{ CYRF_0F_XACT_CFG, 0x28 }
};

static void __attribute__((unused)) DEVO_cyrf_init()
{
	/* Initialise CYRF chip */
	for(uint8_t i = 0; i < sizeof(DEVO_init_vals) / 2; i++)	
		CYRF_WriteRegister(pgm_read_byte( &DEVO_init_vals[i][0]), pgm_read_byte( &DEVO_init_vals[i][1]) );
}

static void __attribute__((unused)) DEVO_set_radio_channels()
{
	CYRF_FindBestChannels(hopping_frequency, 3, 4, 4, 80);
	hopping_frequency[3] = hopping_frequency[0];
	hopping_frequency[4] = hopping_frequency[1];
}

static void __attribute__((unused)) DEVO_BuildPacket()
{
	static uint8_t failsafe_pkt=0;
	switch(phase)
	{
		case DEVO_BIND:
			if(bind_counter)
				bind_counter--;
			DEVO_build_bind_pkt();
			phase = DEVO_BIND_SENDCH;
			break;
		case DEVO_BIND_SENDCH:
			if(bind_counter)
				bind_counter--;
			DEVO_build_data_pkt();
			DEVO_scramble_pkt();
			if (bind_counter == 0)
			{
				phase = DEVO_BOUND;
				BIND_DONE;
			}
			else
				phase = DEVO_BIND;
			break;
		case DEVO_BOUND:
		case DEVO_BOUND_1:
		case DEVO_BOUND_2:
		case DEVO_BOUND_3:
		case DEVO_BOUND_4:
		case DEVO_BOUND_5:
		case DEVO_BOUND_6:
		case DEVO_BOUND_7:
		case DEVO_BOUND_8:
		case DEVO_BOUND_9:
			DEVO_build_data_pkt();
			DEVO_scramble_pkt();
			phase++;
			if (bind_counter)
			{
				bind_counter--;
				if (bind_counter == 0)
					BIND_DONE;
			}
			break;
		case DEVO_BOUND_10:
			DEVO_build_beacon_pkt(DEVO_NUM_CHANNELS > 8 ? failsafe_pkt : 0);
			failsafe_pkt = failsafe_pkt ? 0 : 1;
			DEVO_scramble_pkt();
			phase = DEVO_BOUND_1;
			break;
	}
	packet_count++;
	if(packet_count == DEVO_PKTS_PER_CHANNEL)
		packet_count = 0;
}

uint16_t devo_callback()
{
	static uint8_t txState=0;
	if (txState == 0)
	{
		txState = 1;
		DEVO_BuildPacket();
		CYRF_WriteDataPacket(packet);
		return 1200;
	}
	txState = 0;
	uint8_t i = 0;
	while (! (CYRF_ReadRegister(CYRF_04_TX_IRQ_STATUS) & 0x02))
		if(++i > DEVO_NUM_WAIT_LOOPS)
			return 1200;
	if (phase == DEVO_BOUND)
	{
		/* exit binding state */
		phase = DEVO_BOUND_3;
		DEVO_cyrf_set_bound_sop_code();
	}   
	if(packet_count == 0)
	{
		CYRF_SetPower(0x08);		//Keep tx power updated
		hopping_frequency_ptr = hopping_frequency_ptr == &hopping_frequency[2] ? hopping_frequency : hopping_frequency_ptr + 1;
		CYRF_ConfigRFChannel(*hopping_frequency_ptr);
	}
	return 1200;
}

uint16_t DevoInit()
{	
	DEVO_cyrf_init();
	CYRF_GetMfgData(cyrfmfg_id);
	CYRF_SetTxRxMode(TX_EN);
	CYRF_ConfigCRCSeed(0x0000);
	CYRF_PROGMEM_ConfigSOPCode(DEVO_j6pro_sopcodes[0]);
	DEVO_set_radio_channels();

	hopping_frequency_ptr = hopping_frequency;
	CYRF_ConfigRFChannel(*hopping_frequency_ptr);

	packet_count = 0;

	prev_option=option;
	if(option==0)
	{
		MProtocol_id = ((uint32_t)(hopping_frequency[0] ^ cyrfmfg_id[0] ^ cyrfmfg_id[3]) << 16)
					 | ((uint32_t)(hopping_frequency[1] ^ cyrfmfg_id[1] ^ cyrfmfg_id[4]) << 8)
					 | ((uint32_t)(hopping_frequency[2] ^ cyrfmfg_id[2] ^ cyrfmfg_id[5]) << 0);
		MProtocol_id %= 1000000;
		bind_counter = DEVO_BIND_COUNT;
		phase = DEVO_BIND;
		BIND_IN_PROGRESS;
	}
	else
	{
		phase = DEVO_BOUND_1;
		bind_counter = 0;
		DEVO_cyrf_set_bound_sop_code();
	}  
	return 2400;
}

#endif


# 1 "src/Hitec_cc2500.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

#if defined(HITEC_CC2500_INO)

#include "iface_cc2500.h"

//#define HITEC_FORCE_ID	//Use the ID and hopping table from the original dump

#define HITEC_COARSE			0

#define HITEC_PACKET_LEN		13
#define HITEC_TX_ID_LEN			2
#define HITEC_BIND_COUNT		444	// 10sec
#define HITEC_NUM_FREQUENCE		21
#define HITEC_BIND_NUM_FREQUENCE 14

enum {
    HITEC_START = 0x00,
    HITEC_CALIB = 0x01,
    HITEC_PREP  = 0x02,
    HITEC_DATA1 = 0x03,
    HITEC_DATA2 = 0x04,
    HITEC_DATA3 = 0x05,
    HITEC_DATA4	= 0x06,
    HITEC_RX1	= 0x07,
    HITEC_RX2	= 0x08,
};

const PROGMEM uint8_t HITEC_init_values[] = {
  /* 00 */ 0x2F, 0x2E, 0x2F, 0x07, 0xD3, 0x91, 0xFF, 0x04,
  /* 08 */ 0x45, 0x00, 0x00, 0x12, 0x00, 0x5C, 0x85, 0xE8 + HITEC_COARSE,
  /* 10 */ 0x3D, 0x3B, 0x73, 0x73, 0x7A, 0x01, 0x07, 0x30,
  /* 18 */ 0x08, 0x1D, 0x1C, 0xC7, 0x40, 0xB0, 0x87, 0x6B,
  /* 20 */ 0xF8, 0xB6, 0x10, 0xEA, 0x0A, 0x00, 0x11
};

static void __attribute__((unused)) HITEC_CC2500_init()
{
	CC2500_Strobe(CC2500_SIDLE);

	for (uint8_t i = 0; i < 39; ++i)
		CC2500_WriteReg(i, pgm_read_byte_near(&HITEC_init_values[i]));

	prev_option = option;
	CC2500_WriteReg(CC2500_0C_FSCTRL0, option);
	
	CC2500_SetTxRxMode(TX_EN);
	CC2500_SetPower();
}

// Generate RF channels
static void __attribute__((unused)) HITEC_RF_channels()
{
	//Normal hopping
	uint8_t idx = 0;
	uint32_t rnd = MProtocol_id;

	while (idx < HITEC_NUM_FREQUENCE)
	{
		uint8_t i;
		uint8_t count_0_47 = 0, count_48_93 = 0, count_94_140 = 0;

		rnd = rnd * 0x0019660D + 0x3C6EF35F; // Randomization
		// Use least-significant byte and make sure it's pair.
		uint8_t next_ch = ((rnd >> 8) % 141) & 0xFE;
		// Check that it's not duplicated and spread uniformly
		for (i = 0; i < idx; i++) {
			if(hopping_frequency[i] == next_ch)
				break;
			if(hopping_frequency[i] <= 47)
				count_0_47++;
			else if (hopping_frequency[i] <= 93)
				count_48_93++;
			else
				count_94_140++;
		}
		if (i != idx)
			continue;
		if ( (next_ch <= 47 && count_0_47 < 8) || (next_ch >= 48 && next_ch <= 93 && count_48_93 < 8) || (next_ch >= 94 && count_94_140 < 8) )
			hopping_frequency[idx++] = next_ch;//find hopping frequency
	}
}

static void __attribute__((unused)) HITEC_tune_chan()
{
	CC2500_Strobe(CC2500_SIDLE);
	if(IS_BIND_IN_PROGRESS)
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency_no*10);
	else
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[hopping_frequency_no]);
	CC2500_Strobe(CC2500_SFTX);
	CC2500_Strobe(CC2500_SCAL);
	CC2500_Strobe(CC2500_STX);
}

static void __attribute__((unused)) HITEC_change_chan_fast()
{
	CC2500_Strobe(CC2500_SIDLE);
	if(IS_BIND_IN_PROGRESS)
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency_no*10);
	else
		CC2500_WriteReg(CC2500_0A_CHANNR, hopping_frequency[hopping_frequency_no]);
	CC2500_WriteReg(CC2500_25_FSCAL1, calData[hopping_frequency_no]);
}

static void __attribute__((unused)) HITEC_build_packet()
{
	static boolean F5_frame=false;
	static uint8_t F5_counter=0;
	uint8_t offset;
	
	packet[1] = rx_tx_addr[1];
	packet[2] = rx_tx_addr[2];
	packet[3] = rx_tx_addr[3];
	packet[22] = 0xEE;			// unknown always 0xEE
	if(IS_BIND_IN_PROGRESS)
	{
		packet[0] = 0x16;		// 22 bytes to follow
		memset(packet+5,0x00,14);
		switch(bind_phase)
		{
			case 0x72:			// first part of the hopping table
				for(uint8_t i=0;i<14;i++)
					packet[5+i]=hopping_frequency[i]>>1;
				break;
			case 0x73:			// second part of the hopping table
				for(uint8_t i=0;i<7;i++)
					packet[5+i]=hopping_frequency[i+14]>>1;
				break;
			case 0x74:
				packet[7]=0x55;	// unknown but bind does not complete if not there
				packet[8]=0x55;	// unknown but bind does not complete if not there
				break;
			case 0x7B:
				packet[5]=hopping_frequency[13]>>1;	// if not there the Optima link is jerky...
				break;
		}
		if(sub_protocol==MINIMA)
			packet[4] = bind_phase+0x10;
		else
			packet[4] = bind_phase;	// Optima: increments based on RX answer
		packet[19] = 0x08;		// packet sequence
		offset=20;				// packet[20] and [21]
	}
	else
	{
		packet[0] = 0x1A;		// 26 bytes to follow
		for(uint8_t i=0;i<9;i++)
		{
			uint16_t ch = convert_channel_16b_nolimit(i,0x1B87,0x3905);
			packet[4+2*i] = ch >> 8;
			packet[5+2*i] = ch & 0xFF;
		}
		packet[23] = 0x80;		// packet sequence
		offset=24;				// packet[24] and [25]
		packet[26] = 0x00;		// unknown always 0 and the RX doesn't seem to care about the value?
	}

	if(F5_frame)
	{// No idea what it is but Minima RXs are expecting these frames to work to work
		packet[offset] = 0xF5;
		packet[offset+1] = 0xDF;
		if((F5_counter%9)==0)
			packet[offset+1] -= 0x04;	// every 8 packets send 0xDB
		F5_counter++;
		F5_counter%=59;					// every 6 0xDB packets wait only 4 to resend instead of 8
		F5_frame=false;					// alternate
		if(IS_BIND_IN_PROGRESS)
			packet[offset+1]++;			// when binding the values are 0xE0 and 0xDC
	}
	else
	{
		packet[offset] = 0x00;
		packet[offset+1] = 0x00;
		F5_frame=true;					// alternate
	}
/*	debug("P:");
	for(uint8_t i=0;i<packet[0]+1;i++)
		debug("%02X,",packet[i]);
	debugln("");
*/
}

static void __attribute__((unused)) HITEC_send_packet()
{
	CC2500_WriteData(packet, packet[0]+1);
	if(IS_BIND_IN_PROGRESS)
	{
		packet[19] >>= 1;	// packet sequence
		if( (packet[4] & 0xFE) ==0x82 )
		{ // Minima
			packet[4] ^= 1;					// alternate 0x82 and 0x83
			if( packet[4] & 0x01 )
				for(uint8_t i=0;i<7;i++)	// 0x83
					packet[5+i]=hopping_frequency[i+14]>>1;
			else
				for(uint8_t i=0;i<14;i++)	// 0x82
					packet[5+i]=hopping_frequency[i]>>1;
		}
	}
	else
		packet[23] >>= 1;	// packet sequence
}

uint16_t ReadHITEC()
{
	switch(phase)
	{
		case HITEC_START:
			HITEC_CC2500_init();
			bind_phase=0x72;
			if(IS_BIND_IN_PROGRESS)
			{
				bind_counter = HITEC_BIND_COUNT;
				rf_ch_num=HITEC_BIND_NUM_FREQUENCE;
			}
			else
			{
				bind_counter=0;
				rf_ch_num=HITEC_NUM_FREQUENCE;
				//Set TXID
				CC2500_WriteReg(CC2500_05_SYNC0,rx_tx_addr[2]);
				CC2500_WriteReg(CC2500_04_SYNC1,rx_tx_addr[3]);
			}
			hopping_frequency_no=0;
			HITEC_tune_chan();
			phase = HITEC_CALIB;
			return 2000;
		case HITEC_CALIB:
			calData[hopping_frequency_no]=CC2500_ReadReg(CC2500_25_FSCAL1);
			hopping_frequency_no++;
			if (hopping_frequency_no < rf_ch_num)
				HITEC_tune_chan();
			else
			{
				hopping_frequency_no = 0;
				phase = HITEC_PREP;
			}
			return 2000;

		/* Work cycle: 22.5ms */
#define HITEC_PACKET_PERIOD	22500
#define HITEC_PREP_TIMING	462
#define HITEC_DATA_TIMING	2736
#define HITEC_RX1_TIMING	4636
		case HITEC_PREP:
			if ( prev_option == option )
			{	// No user frequency change
				HITEC_change_chan_fast();
				hopping_frequency_no++;
				if(hopping_frequency_no>=rf_ch_num)
					hopping_frequency_no=0;
				CC2500_SetPower();
				CC2500_SetTxRxMode(TX_EN);
				HITEC_build_packet();
				phase++;
			}
			else
				phase = HITEC_START;	// Restart the tune process if option is changed to get good tuned values
			return HITEC_PREP_TIMING;
		case HITEC_DATA1:
		case HITEC_DATA2:
		case HITEC_DATA3:
		case HITEC_DATA4:
			HITEC_send_packet();
			phase++;
			return HITEC_DATA_TIMING;
		case HITEC_RX1:
			CC2500_SetTxRxMode(RX_EN);
			CC2500_Strobe(CC2500_SRX);	// Turn RX ON
			phase++;
			return HITEC_RX1_TIMING;
		case HITEC_RX2:
			uint8_t len=CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
			if(len && len<MAX_PKT)
			{ // Something has been received
				CC2500_ReadData(pkt, len);
				if( (pkt[len-1] & 0x80) && pkt[0]==len-3 && pkt[1]==rx_tx_addr[1] && pkt[2]==rx_tx_addr[2] && pkt[3]==rx_tx_addr[3])
				{ //valid crc && length ok && tx_id ok
					debug("RX:l=%d",len);
					for(uint8_t i=0;i<len;i++)
						debug(",%02X",pkt[i]);
					if(IS_BIND_IN_PROGRESS)
					{
						if(len==13)	// Bind packets have a length of 13
						{ // bind packet: 0A,00,E5,F2,7X,05,06,07,08,09,00
							debug(",bind");
							boolean check=true;
							for(uint8_t i=5;i<=10;i++)
								if(pkt[i]!=i%10) check=false;
							if((pkt[4]&0xF0)==0x70 && check)
							{
								bind_phase=pkt[4]+1;
								if(bind_phase==0x7B)
									bind_counter=164;	// in dumps the RX stops to reply at 0x7B so wait a little and exit
							}
						}
					}
					else
						if( len==15 && pkt[4]==0 && pkt[12]==0 )
						{	// Valid telemetry packets
							// no station:
							//		0C,1C,A1,2B,00,00,00,00,00,00,00,8D,00,64,8E	-> 00 8D=>RX battery voltage 0x008D/28=5.03V
							// with HTS-SS:
							//		0C,1C,A1,2B,00,11,AF,00,2D,00,8D,11,00,4D,96	-> 00 8D=>RX battery voltage 0x008D/28=5.03V
							//		0C,1C,A1,2B,00,12,00,00,00,00,00,12,00,52,93
							//		0C,1C,A1,2B,00,13,00,00,00,00,46,13,00,52,8B	-> 46=>temperature2 0x46-0x28=30°C
							//		0C,1C,A1,2B,00,14,00,00,00,00,41,14,00,2C,93	-> 41=>temperature1 0x41-0x28=25°C
							//		0C,1C,A1,2B,00,15,00,2A,00,0E,00,15,00,44,96	-> 2A 00=>rpm1=420, 0E 00=>rpm2=140 
							//		0C,1C,A1,2B,00,16,00,00,00,00,00,16,00,2C,8E
							//		0C,1C,A1,2B,00,17,00,00,00,42,44,17,00,48,8D	-> 42=>temperature3 0x42-0x28=26°C,44=>temperature4 0x44-0x28=28°C
							//		0C,1C,A1,2B,00,18,00,00,00,00,00,18,00,50,92
							debug(",telem,%02x",pkt[14]&0x7F);
							#if defined(HITEC_FW_TELEMETRY)
								if(sub_protocol==OPT_FW)
								{
									// 8 bytes telemetry packets => see at the end of this file how to fully decode it
									pkt[0]=pkt[13];				// TX RSSI
									pkt[1]=pkt[14]&0x7F;		// TX LQI
									uint8_t offset=pkt[5]==0?1:0;
									for(uint8_t i=5;i < 11; i++)
										pkt[i-3]=pkt[i+offset];	// frame number followed by 5 bytes of data
									telemetry_link=2;			// telemetry forward available
								}
							#endif
							#if defined(HITEC_HUB_TELEMETRY)
								if(sub_protocol==OPT_HUB)
								{
									switch(pkt[5])		// telemetry frame number
									{
										case 0x00:
											v_lipo1 = (pkt[10])<<5 | (pkt[11])>>3;	// calculation in float is volt=(pkt[10]<<8+pkt[11])/28
											break;
										case 0x11:
											v_lipo1 = (pkt[9])<<5 | (pkt[10])>>3;	// calculation in float is volt=(pkt[9]<<8+pkt[10])/28
											break;
										case 0x18:
											v_lipo2 =  (pkt[6])<<5 | (pkt[7])>>3;	// calculation in float is volt=(pkt[6]<<8+pkt[7])/10
											break;
									}
									TX_RSSI = pkt[13];
									if(TX_RSSI >=128)
										TX_RSSI -= 128;
									else
										TX_RSSI += 128;
									TX_LQI = pkt[14]&0x7F;
									telemetry_link=1;			// telemetry hub available
								}
							#endif
						}
					debugln("");
				}
			}
			CC2500_Strobe(CC2500_SFRX);	// Flush the RX FIFO buffer
			phase = HITEC_PREP;
			if(bind_counter)
			{
				bind_counter--;
				if(!bind_counter)
				{
					BIND_DONE;
					phase=HITEC_START;
				}
			}
			return (HITEC_PACKET_PERIOD -HITEC_PREP_TIMING -4*HITEC_DATA_TIMING -HITEC_RX1_TIMING);
	}
	return 0;
}

uint16_t initHITEC()
{
	HITEC_RF_channels();
	#ifdef HITEC_FORCE_ID	// ID and channels taken from dump
		rx_tx_addr[1]=0x00;
		rx_tx_addr[2]=0x03;
		rx_tx_addr[3]=0x6A;
		memcpy((void *)hopping_frequency,(void *)"\x00\x3A\x4A\x32\x0C\x58\x2A\x10\x26\x20\x08\x60\x68\x70\x78\x80\x88\x56\x5E\x66\x6E",HITEC_NUM_FREQUENCE);
	#endif
	#if defined(HITEC_HUB_TELEMETRY)
		if(sub_protocol==OPT_HUB)
			init_frskyd_link_telemetry();
	#endif
	phase = HITEC_START;
	return 10000;
}

/* Full telemetry 
packet[0] = TX RSSI value
packet[1] = TX LQI value
packet[2] = frame number
packet[3-7] telemetry data

The frame number takes the following values: 0x00, 0x11, 0x12, ..., 0x18. The frames can be present or not, they also do not have to follow each others.
Here is a description of the telemetry data for each frame number:
- frame 0x00
data byte 0 -> 0x00				unknown
data byte 1 -> 0x00				unknown
data byte 2 -> 0x00				unknown
data byte 3 -> RX Batt Volt_H
data byte 4 -> RX Batt Volt_L => RX Batt=(Volt_H*256+Volt_L)/28
- frame 0x11
data byte 0 -> 0xAF				start of frame
data byte 1 -> 0x00				unknown
data byte 2 -> 0x2D				frame type but constant here
data byte 3 -> Volt1_H
data byte 4 -> Volt1_L			RX Batt=(Volt1_H*256+Volt1_L)/28 V
- frame 0x12
data byte 0 -> Lat_sec_H		GPS : latitude second
data byte 1 -> Lat_sec_L		signed int : 1/100 of second
data byte 2 -> Lat_deg_min_H	GPS : latitude degree.minute
data byte 3 -> Lat_deg_min_L	signed int : +=North, - = south
data byte 4 -> Time_second		GPS Time
- frame 0x13
data byte 0 -> 					GPS Longitude second
data byte 1 -> 					signed int : 1/100 of second
data byte 2 -> 					GPS Longitude degree.minute
data byte 3 -> 					signed int : +=Est, - = west
data byte 4 -> Temp2			Temperature2=Temp2-40°C
- frame 0x14
data byte 0 -> Speed_H
data byte 1 -> Speed_L			Speed=Speed_H*256+Speed_L km/h
data byte 2 -> Alti_sea_H
data byte 3 -> Alti_sea_L		Altitude sea=Alti_sea_H*256+Alti_sea_L m
data byte 4 -> Temp1			Temperature1=Temp1-40°C
- frame 0x15
data byte 0 -> FUEL
data byte 1 -> RPM1_L
data byte 2 -> RPM1_H			RPM1=RPM1_H*256+RPM1_L
data byte 3 -> RPM2_L
data byte 4 -> RPM2_H			RPM2=RPM2_H*256+RPM2_L
- frame 0x16
data byte 0 -> Date_year		GPS Date
data byte 1 -> Date_month
data byte 2 -> Date_day
data byte 3 -> Time_hour		GPS Time
data byte 4 -> Time_min
- frame 0x17
data byte 0 -> 0x00	COURSEH
data byte 1 -> 0x00	COURSEL		GPS Course = COURSEH*256+COURSEL
data byte 2 -> 0x00				GPS count
data byte 3 -> Temp3			Temperature3=Temp2-40°C
data byte 4 -> Temp4			Temperature4=Temp3-40°C
- frame 0x18
data byte 1 -> Volt2_H
data byte 2 -> Volt2_L			Volt2=(Volt2_H*256+Volt2_L)/10 V
data byte 3 -> AMP1_L
data byte 4 -> AMP1_H			Amp=(AMP1_H*256+AMP1_L -180)/14 in signed A
*/
#endif

# 1 "src/GD00X_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// Compatible with GD005 C-17 and GD006 DA62 planes.

#if defined(GD00X_NRF24L01_INO)

#include "iface_nrf24l01.h"

//#define FORCE_GD00X_ORIGINAL_ID

#define GD00X_INITIAL_WAIT    500
#define GD00X_PACKET_PERIOD   3500
#define GD00X_RF_BIND_CHANNEL 2
#define GD00X_PAYLOAD_SIZE    15
#define GD00X_BIND_COUNT	  857	//3sec

// flags going to packet[11]
#define	GD00X_FLAG_DR		0x08
#define	GD00X_FLAG_LIGHT	0x04

static void __attribute__((unused)) GD00X_send_packet()
{
	packet[0] = IS_BIND_IN_PROGRESS?0xAA:0x55;
	memcpy(packet+1,rx_tx_addr,4);
	uint16_t channel=convert_channel_ppm(AILERON);
	packet[5 ] = channel;
	packet[6 ] = channel>>8;
	channel=convert_channel_ppm(THROTTLE);
	packet[7 ] = channel;
	packet[8 ] = channel>>8;
	channel=convert_channel_ppm(CH5);		// TRIM
	packet[9 ] = channel;
	packet[10] = channel>>8;
	packet[11] = GD00X_FLAG_DR						// Force high rate
			   | GET_FLAG(CH6_SW, GD00X_FLAG_LIGHT);
	packet[12] = 0x00;
	packet[13] = 0x00;
	packet[14] = 0x00;

	// Power on, TX mode, CRC enabled
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if(IS_BIND_DONE)
	{
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++]);
		hopping_frequency_no &= 3;	// 4 RF channels
	}

	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, GD00X_PAYLOAD_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) GD00X_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t*)"\xcc\xcc\xcc\xcc\xcc", 5);
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, GD00X_RF_BIND_CHANNEL);	// Bind channel
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);	// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);		// No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);	// Enable data pipe 0 only
	NRF24L01_SetBitrate(NRF24L01_BR_250K);			// 250Kbps
	NRF24L01_SetPower();
}

static void __attribute__((unused)) GD00X_initialize_txid()
{
	uint8_t start=76+(rx_tx_addr[0]&0x03);
	for(uint8_t i=0; i<4;i++)
		hopping_frequency[i]=start-(i<<1);
	#ifdef FORCE_GD00X_ORIGINAL_ID
		rx_tx_addr[0]=0x1F;					// or 0xA5 or 0x26
		rx_tx_addr[1]=0x39;					// or 0x37 or 0x35
		rx_tx_addr[2]=0x12;					// Constant on 3 TXs
		rx_tx_addr[3]=0x13;					// Constant on 3 TXs
		for(uint8_t i=0; i<4;i++)
			hopping_frequency[i]=79-(i<<1);	// or 77 or 78
	#endif
}

uint16_t GD00X_callback()
{
	if(IS_BIND_IN_PROGRESS)
		if(--bind_counter==0)
			BIND_DONE;
	GD00X_send_packet();
	return GD00X_PACKET_PERIOD;
}

uint16_t initGD00X()
{
	BIND_IN_PROGRESS;	// autobind protocol
	GD00X_initialize_txid();
	GD00X_init();
	hopping_frequency_no = 0;
	bind_counter=GD00X_BIND_COUNT;
	return GD00X_INITIAL_WAIT;
}

#endif


# 1 "src/CX10_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with Cheerson CX-10 blue & newer red pcb, CX-10A, CX11, CX-10 green pcb, DM007, Floureon FX-10, JXD 509 (Q282), Q222, Q242 and Q282
// Last sync with hexfet new_protocols/cx10_nrf24l01.c dated 2015-11-26

#if defined(CX10_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CX10_BIND_COUNT		4360   // 6 seconds
#define CX10_PACKET_SIZE	15
#define CX10A_PACKET_SIZE	19       // CX10 blue board packets have 19-byte payload
#define Q2X2_PACKET_SIZE	21
#define CX10_PACKET_PERIOD	1316  // Timeout for callback in uSec
#define CX10A_PACKET_PERIOD	6000

#define CX10_INITIAL_WAIT     500

// flags
#define CX10_FLAG_FLIP       0x10 // goes to rudder channel
#define CX10_FLAG_MODE_MASK  0x03
#define CX10_FLAG_HEADLESS   0x04
// flags2
#define CX10_FLAG_VIDEO      0x02
#define CX10_FLAG_SNAPSHOT   0x04

// frequency channel management
#define CX10_RF_BIND_CHANNEL 0x02
#define CX10_NUM_RF_CHANNELS    4

enum {
    CX10_BIND1 = 0,
    CX10_BIND2,
    CX10_DATA
};

static void __attribute__((unused)) CX10_Write_Packet(uint8_t bind)
{
	uint8_t offset = 0;
	if(sub_protocol == CX10_BLUE)
		offset = 4;
	packet[0] = bind ? 0xAA : 0x55;
	packet[1] = rx_tx_addr[0];
	packet[2] = rx_tx_addr[1];
	packet[3] = rx_tx_addr[2];
	packet[4] = rx_tx_addr[3];
	// packet[5] to [8] (aircraft id) is filled during bind for blue board
	uint16_t aileron= convert_channel_16b_limit(AILERON ,1000,2000);
	uint16_t elevator=convert_channel_16b_limit(ELEVATOR,2000,1000);
	uint16_t throttle=convert_channel_16b_limit(THROTTLE,1000,2000);
	uint16_t rudder=  convert_channel_16b_limit(RUDDER  ,2000,1000);
    // Channel 5 - flip flag
	packet[12+offset] = GET_FLAG(CH5_SW,CX10_FLAG_FLIP); // flip flag applied on rudder

	// Channel 6 - rate mode is 2 lsb of packet 13
	if(CH6_SW)		// rate 3 / headless on CX-10A
		flags = 0x02;
	else
		if(Channel_data[CH6] < CHANNEL_MIN_COMMAND)
			flags = 0x00;			// rate 1
		else
			flags = 0x01;			// rate 2
	uint8_t flags2=0;	// packet 14

	uint8_t video_state=packet[14] & 0x21;
	switch(sub_protocol)
	{
		case CX10_BLUE:
			flags |= GET_FLAG(!CH7_SW, 0x10)	// Channel 7 - picture
					|GET_FLAG( CH8_SW, 0x08);	// Channel 8 - video
			break;
		case F_Q282:
		case F_Q242:
		case F_Q222:
			memcpy(&packet[15], "\x10\x10\xaa\xaa\x00\x00", 6);
			//FLIP|LED|PICTURE|VIDEO|HEADLESS|RTH|XCAL|YCAL
			flags2 = GET_FLAG(CH5_SW, 0x80)		// Channel 5 - FLIP
					|GET_FLAG(!CH6_SW, 0x40)	// Channel 6 - LED
					|GET_FLAG(CH9_SW, 0x08)		// Channel 9 - HEADLESS
					|GET_FLAG(CH11_SW, 0x04)		// Channel 11 - XCAL
					|GET_FLAG(CH12_SW, 0x02);	// Channel 12 - YCAL or Start/Stop motors on JXD 509
	
			if(sub_protocol==F_Q242)
			{
				flags=2;
				flags2|= GET_FLAG(CH7_SW,0x01)	// Channel 7 - picture
						|GET_FLAG(CH8_SW,0x10);	// Channel 8 - video
				packet[17]=0x00;
				packet[18]=0x00;
			}
			else
			{ // F_Q282 & F_Q222
				flags=3;							// expert
				if(CH8_SW)						// Channel 8 - F_Q282 video / F_Q222 Module 1
				{
					if (!(video_state & 0x20)) video_state ^= 0x21;
				}
				else
					if (video_state & 0x20) video_state &= 0x01;
				flags2 |= video_state
						|GET_FLAG(CH7_SW,0x10);	// Channel 7 - F_Q282 picture / F_Q222 Module 2
			}
			if(CH10_SW)	flags |=0x80;			// Channel 10 - RTH
			break;
		case DM007:
			aileron = 3000 - aileron;
			//FLIP|MODE|PICTURE|VIDEO|HEADLESS
			flags2=  GET_FLAG(CH7_SW,CX10_FLAG_SNAPSHOT)	// Channel 7 - picture
					|GET_FLAG(CH8_SW,CX10_FLAG_VIDEO);		// Channel 8 - video
			if(CH9_SW)	flags |= CX10_FLAG_HEADLESS;		// Channel 9 - headless
			break;
		case JC3015_2:
			aileron = 3000 - aileron;
			elevator = 3000 - elevator;
			//FLIP|MODE|LED|DFLIP
			if(CH8_SW)	packet[12] &= ~CX10_FLAG_FLIP;
		case JC3015_1:
			//FLIP|MODE|PICTURE|VIDEO
			flags2=	 GET_FLAG(CH7_SW,_BV(3))	// Channel 7
					|GET_FLAG(CH8_SW,_BV(4));	// Channel 8
			break;
		case MK33041:
			elevator = 3000 - elevator;
			//FLIP|MODE|PICTURE|VIDEO|HEADLESS|RTH
			flags|=GET_FLAG(CH7_SW,_BV(7))	// Channel 7 - picture
				  |GET_FLAG(CH10_SW,_BV(2));	// Channel 10 - rth
			flags2=GET_FLAG(CH8_SW,_BV(0))	// Channel 8 - video
				  |GET_FLAG(CH9_SW,_BV(5));	// Channel 9 - headless
			break;
	}
	packet[5+offset] = lowByte(aileron);
	packet[6+offset] = highByte(aileron);
	packet[7+offset] = lowByte(elevator);
	packet[8+offset] = highByte(elevator);
	packet[9+offset] = lowByte(throttle);
	packet[10+offset]= highByte(throttle);
	packet[11+offset]= lowByte(rudder);
	packet[12+offset]|= highByte(rudder);
	packet[13+offset]=flags;
	packet[14+offset]=flags2;
	
	// Power on, TX mode, 2byte CRC
	// Why CRC0? xn297 does not interpret it - either 16-bit CRC or nothing
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if (bind)
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_RF_BIND_CHANNEL);
	else
	{
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no++]);
		hopping_frequency_no %= CX10_NUM_RF_CHANNELS;
	}
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();

	XN297_WritePayload(packet, packet_length);
	NRF24L01_SetPower();
}

static void __attribute__((unused)) CX10_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t *)"\xcc\xcc\xcc\xcc\xcc",5);
	XN297_SetRXAddr((uint8_t *)"\xcc\xcc\xcc\xcc\xcc",5);
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);			// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);				// No Auto Acknowledgment on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);			// Enable data pipe 0 only
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, packet_length);	// rx pipe 0 (used only for blue board)
    NRF24L01_WriteReg(NRF24L01_05_RF_CH, CX10_RF_BIND_CHANNEL);
	NRF24L01_SetBitrate(NRF24L01_BR_1M);					// 1Mbps
	NRF24L01_SetPower();
}

uint16_t CX10_callback()
{
	switch (phase) {
		case CX10_BIND1:
			if (bind_counter == 0)
			{
				phase = CX10_DATA;
				BIND_DONE;
			}
			else
			{
				CX10_Write_Packet(1);
				bind_counter--;
			}
			break;
		case CX10_BIND2:
			if( NRF24L01_ReadReg(NRF24L01_07_STATUS) & _BV(NRF24L01_07_RX_DR))
			{ // RX fifo data ready
				XN297_ReadPayload(packet, packet_length);
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_SetTxRxMode(TX_EN);
				if(packet[9] == 1)
				{
					BIND_DONE;
					phase = CX10_DATA;
				}
			}
			else
			{
				// switch to TX mode
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_FlushTx();
				NRF24L01_SetTxRxMode(TX_EN);
				CX10_Write_Packet(1);
				delayMicroseconds(400);
				// switch to RX mode
				NRF24L01_SetTxRxMode(TXRX_OFF);
				NRF24L01_FlushRx();
				NRF24L01_SetTxRxMode(RX_EN);
				XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP) | _BV(NRF24L01_00_PRIM_RX));
			}
			break;
		case CX10_DATA:
			CX10_Write_Packet(0);
			break;
	}
	return packet_period;
}

static void __attribute__((unused)) CX10_initialize_txid()
{
	rx_tx_addr[1]%= 0x30;
	if(sub_protocol&0x08)	//F_Q2X2 protocols
	{
		uint8_t offset=0;	//F_Q282
		if(sub_protocol==F_Q242)
			offset=2;
		if(sub_protocol==F_Q222)
			offset=3;
		for(uint8_t i=0;i<4;i++)
			hopping_frequency[i]=0x46+2*i+offset;
	}
	else
	{
		hopping_frequency[0] = 0x03 + (rx_tx_addr[0] & 0x0F);
		hopping_frequency[1] = 0x16 + (rx_tx_addr[0] >> 4);
		hopping_frequency[2] = 0x2D + (rx_tx_addr[1] & 0x0F);
		hopping_frequency[3] = 0x40 + (rx_tx_addr[1] >> 4);
	}
}

uint16_t initCX10(void)
{
	BIND_IN_PROGRESS;	// autobind protocol
	if(sub_protocol==CX10_BLUE)
	{
		packet_length = CX10A_PACKET_SIZE;
		packet_period = CX10A_PACKET_PERIOD;

		phase = CX10_BIND2;

		for(uint8_t i=0; i<4; i++)
			packet[5+i] = 0xff; // clear aircraft id
		packet[9] = 0;
	}
	else
	{
		if(sub_protocol&0x08)	//F_Q2X2 protocols
			packet_length = Q2X2_PACKET_SIZE;
		else
		    packet_length = CX10_PACKET_SIZE;
		packet_period = CX10_PACKET_PERIOD;
		phase = CX10_BIND1;
		bind_counter = CX10_BIND_COUNT;
	}
	CX10_initialize_txid();
	CX10_init();
	return CX10_INITIAL_WAIT+packet_period;
}

#endif


# 1 "src/CFlie_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */

// Most of this code was ported from theseankelly's related DeviationTX work.

#if defined(CFLIE_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define CFLIE_BIND_COUNT 60

//=============================================================================
// CRTP (Crazy RealTime Protocol) Implementation
//=============================================================================

// Port IDs
enum {
    CRTP_PORT_CONSOLE = 0x00,
    CRTP_PORT_PARAM = 0x02,
    CRTP_PORT_SETPOINT = 0x03,
    CRTP_PORT_MEM = 0x04,
    CRTP_PORT_LOG = 0x05,
    CRTP_PORT_POSITION = 0x06,
    CRTP_PORT_SETPOINT_GENERIC = 0x07,
    CRTP_PORT_PLATFORM = 0x0D,
    CRTP_PORT_LINK = 0x0F,
};

// Channel definitions for the LOG port
enum {
    CRTP_LOG_CHAN_TOC = 0x00,
    CRTP_LOG_CHAN_SETTINGS = 0x01,
    CRTP_LOG_CHAN_LOGDATA = 0x02,
};

// Command definitions for the LOG port's TOC channel
enum {
    CRTP_LOG_TOC_CMD_ELEMENT = 0x00,
    CRTP_LOG_TOC_CMD_INFO = 0x01,
};

// Command definitions for the LOG port's CMD channel
enum {
    CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK = 0x00,
    CRTP_LOG_SETTINGS_CMD_APPEND_BLOCK = 0x01,
    CRTP_LOG_SETTINGS_CMD_DELETE_BLOCK = 0x02,
    CRTP_LOG_SETTINGS_CMD_START_LOGGING = 0x03,
    CRTP_LOG_SETTINGS_CMD_STOP_LOGGING = 0x04,
    CRTP_LOG_SETTINGS_CMD_RESET_LOGGING = 0x05,
};

// Log variables types
enum {
    LOG_UINT8 = 0x01,
    LOG_UINT16 = 0x02,
    LOG_UINT32 = 0x03,
    LOG_INT8 = 0x04,
    LOG_INT16 = 0x05,
    LOG_INT32 = 0x06,
    LOG_FLOAT = 0x07,
    LOG_FP16 = 0x08,
};

#define CFLIE_TELEM_LOG_BLOCK_ID            0x01
#define CFLIE_TELEM_LOG_BLOCK_PERIOD_10MS   50 // 50*10 = 500ms

// Setpoint type definitions for the generic setpoint channel
enum {
    CRTP_SETPOINT_GENERIC_STOP_TYPE = 0x00,
    CRTP_SETPOINT_GENERIC_VELOCITY_WORLD_TYPE = 0x01,
    CRTP_SETPOINT_GENERIC_Z_DISTANCE_TYPE = 0x02,
    CRTP_SETPOINT_GENERIC_CPPM_EMU_TYPE = 0x03,
};

static inline uint8_t crtp_create_header(uint8_t port, uint8_t channel)
{
    return ((port)&0x0F)<<4 | (channel & 0x03);
}

//=============================================================================
// End CRTP implementation
//=============================================================================

// Address size
#define TX_ADDR_SIZE 5

// Timeout for callback in uSec, 10ms=10000us for Crazyflie
#define PACKET_PERIOD 10000

#define MAX_PACKET_SIZE 32  // CRTP is 32 bytes

// CPPM CRTP supports up to 10 aux channels but deviation only
// supports a total of 12 channels. R,P,Y,T leaves 8 aux channels left
#define MAX_CPPM_AUX_CHANNELS 8

static uint8_t tx_payload_len = 0; // Length of the packet stored in packet
static uint8_t rx_payload_len = 0; // Length of the packet stored in rx_packet
static uint8_t rx_packet[MAX_PACKET_SIZE]; // For reading in ACK payloads

static uint16_t cflie_counter;
static uint32_t packet_counter;
static uint8_t data_rate;

enum {
    CFLIE_INIT_SEARCH = 0,
    CFLIE_INIT_CRTP_LOG,
    CFLIE_INIT_DATA,
    CFLIE_SEARCH,
    CFLIE_DATA
};

static uint8_t crtp_log_setup_state;
enum {
    CFLIE_CRTP_LOG_SETUP_STATE_INIT = 0,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM,
    // It might be a good idea to add a state here
    // to send the command to reset the logging engine
    // to avoid log block ID conflicts. However, there
    // is not a conflict with the current defaults in
    // cfclient and I'd rather be able to log from the Tx
    // and cfclient simultaneously
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK,
    CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE,
};

// State variables for the crtp_log_setup_state_machine
static uint8_t toc_size;             // Size of the TOC read from the crazyflie
static uint8_t next_toc_variable;    // State variable keeping track of the next var to read
static uint8_t vbat_var_id;          // ID of the vbatMV variable
static uint8_t extvbat_var_id;       // ID of the extVbatMV variable
static uint8_t rssi_var_id;          // ID of the RSSI variable

// Constants used for finding var IDs from the toc
static const char* pm_group_name = "pm";
static const char* vbat_var_name = "vbatMV";
static const uint8_t vbat_var_type = LOG_UINT16;
static const char* extvbat_var_name = "extVbatMV";
static const uint8_t extvbat_var_type = LOG_UINT16;
static const char* radio_group_name = "radio";
static const char* rssi_var_name = "rssi";
static const uint8_t rssi_var_type = LOG_UINT8;

// Repurposing DSM Telemetry fields
#define TELEM_CFLIE_INTERNAL_VBAT   TELEM_DSM_FLOG_VOLT2    // Onboard voltage
#define TELEM_CFLIE_EXTERNAL_VBAT   TELEM_DSM_FLOG_VOLT1    // Voltage from external pin (BigQuad)
#define TELEM_CFLIE_RSSI            TELEM_DSM_FLOG_FADESA   // Repurpose FADESA for RSSI

enum {
    PROTOOPTS_TELEMETRY = 0,
    PROTOOPTS_CRTP_MODE = 1,
    LAST_PROTO_OPT,
};

#define TELEM_OFF 0
#define TELEM_ON_ACKPKT 1
#define TELEM_ON_CRTPLOG 2

#define CRTP_MODE_RPYT 0
#define CRTP_MODE_CPPM 1

// Bit vector from bit position
#define BV(bit) (1 << bit)

#define PACKET_CHKTIME 500      // time to wait if packet not yet acknowledged or timed out    

// Helper for sending a packet
// Assumes packet data has been put in packet
// and tx_payload_len has been set correctly
static void send_packet()
{
    // clear packet status bits and Tx/Rx FIFOs
    NRF24L01_WriteReg(NRF24L01_07_STATUS, (_BV(NRF24L01_07_TX_DS) | _BV(NRF24L01_07_MAX_RT)));
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();

    // Transmit the payload
    NRF24L01_WritePayload(packet, tx_payload_len);

    ++packet_counter;

    // // Check and adjust transmission power.
    NRF24L01_SetPower();
}

static uint16_t dbg_cnt = 0;
static uint8_t packet_ack()
{
	if (++dbg_cnt > 50)
	{
		// debugln("S: %02x\n", NRF24L01_ReadReg(NRF24L01_07_STATUS));
		dbg_cnt = 0;
	}
	switch (NRF24L01_ReadReg(NRF24L01_07_STATUS) & (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)))
	{
		case BV(NRF24L01_07_TX_DS):
			rx_payload_len = NRF24L01_GetDynamicPayloadSize();
			if (rx_payload_len > MAX_PACKET_SIZE)
				rx_payload_len = MAX_PACKET_SIZE;
			NRF24L01_ReadPayload(rx_packet, rx_payload_len);
			return PKT_ACKED;
		case BV(NRF24L01_07_MAX_RT):
			return PKT_TIMEOUT;
	}
	return PKT_PENDING;
}

static void set_rate_channel(uint8_t rate, uint8_t channel)
{
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, channel);		// Defined by model id
	NRF24L01_SetBitrate(rate);							// Defined by model id
}

static void send_search_packet()
{
	uint8_t buf[1];
	buf[0] = 0xff;
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, (BV(NRF24L01_07_TX_DS) | BV(NRF24L01_07_MAX_RT)));
	NRF24L01_FlushTx();

	if (rf_ch_num++ > 125)
	{
		rf_ch_num = 0;
		switch(data_rate)
		{
			case NRF24L01_BR_250K:
				data_rate = NRF24L01_BR_1M;
				break;
			case NRF24L01_BR_1M:
				data_rate = NRF24L01_BR_2M;
				break;
			case NRF24L01_BR_2M:
				data_rate = NRF24L01_BR_250K;
				break;
		}
	}
	set_rate_channel(data_rate, rf_ch_num);

	NRF24L01_WritePayload(buf, sizeof(buf));

	++packet_counter;
}

// Frac 16.16
#define FRAC_MANTISSA 16 // This means, not IEEE 754...
#define FRAC_SCALE (1 << FRAC_MANTISSA)

// Convert fractional 16.16 to float32
static void frac2float(int32_t n, float* res)
{
	if (n == 0)
	{
		*res = 0.0;
		return;
	}
	uint32_t m = n < 0 ? -n : n; // Figure out mantissa?
	int i;
	for (i = (31-FRAC_MANTISSA); (m & 0x80000000) == 0; i--, m <<= 1);
	m <<= 1; // Clear implicit leftmost 1
	m >>= 9;
	uint32_t e = 127 + i;
	if (n < 0) m |= 0x80000000;
	m |= e << 23;
	*((uint32_t *) res) = m;
}

static void send_crtp_rpyt_packet()
{
	int32_t f_roll;
	int32_t f_pitch;
	int32_t f_yaw;
	uint16_t thrust;

	uint16_t val;

	struct CommanderPacketRPYT
	{
		float roll;
		float pitch;
		float yaw;
		uint16_t thrust;
	}__attribute__((packed)) cpkt;

	// Channels in AETR order
	// Roll, aka aileron, float +- 50.0 in degrees
	// float roll  = -(float) Channels[0]*50.0/10000;
	val = convert_channel_16b_limit(AILERON, -10000, 10000);
	// f_roll = -Channels[0] * FRAC_SCALE / (10000 / 50);
	f_roll = val * FRAC_SCALE / (10000 / 50);

	frac2float(f_roll, &cpkt.roll); // TODO: Remove this and use the correct Mode switch below...
	// debugln("Roll: raw, converted:  %d, %d, %d, %0.2f", Channel_data[AILERON], val, f_roll, cpkt.roll);

	// Pitch, aka elevator, float +- 50.0 degrees
	//float pitch = -(float) Channels[1]*50.0/10000;
	val = convert_channel_16b_limit(ELEVATOR, -10000, 10000);
	// f_pitch = -Channels[1] * FRAC_SCALE / (10000 / 50);
	f_pitch = -val * FRAC_SCALE / (10000 / 50);

	frac2float(f_pitch, &cpkt.pitch); // TODO: Remove this and use the correct Mode switch below...
	// debugln("Pitch: raw, converted:  %d, %d, %d, %0.2f", Channel_data[ELEVATOR], val, f_pitch, cpkt.pitch);

	// Thrust, aka throttle 0..65535, working range 5535..65535
	// Android Crazyflie app puts out a throttle range of 0-80%: 0..52000
	thrust = convert_channel_16b_limit(THROTTLE, 0, 32767) * 2;

	// Crazyflie needs zero thrust to unlock
	if (thrust < 900)
		cpkt.thrust = 0;
	else
		cpkt.thrust = thrust;

	// debugln("Thrust: raw, converted:  %d, %u, %u", Channel_data[THROTTLE], thrust, cpkt.thrust);

	// Yaw, aka rudder, float +- 400.0 deg/s
	// float yaw   = -(float) Channels[3]*400.0/10000;
	val = convert_channel_16b_limit(RUDDER, -10000, 10000);
	// f_yaw = - Channels[3] * FRAC_SCALE / (10000 / 400);
	f_yaw = val * FRAC_SCALE / (10000 / 400);
	frac2float(f_yaw, &cpkt.yaw);

	// debugln("Yaw: raw, converted:  %d, %d, %d, %0.2f", Channel_data[RUDDER], val, f_yaw, cpkt.yaw);

	// Switch on/off?
	// TODO: Get X or + mode working again:
	// if (Channels[4] >= 0) {
	//     frac2float(f_roll, &cpkt.roll);
	//     frac2float(f_pitch, &cpkt.pitch);
	// } else {
	//     // Rotate 45 degrees going from X to + mode or opposite.
	//     // 181 / 256 = 0.70703125 ~= sqrt(2) / 2
	//     int32_t f_x_roll = (f_roll + f_pitch) * 181 / 256;
	//     frac2float(f_x_roll, &cpkt.roll);
	//     int32_t f_x_pitch = (f_pitch - f_roll) * 181 / 256;
	//     frac2float(f_x_pitch, &cpkt.pitch);
	// }

	// Construct and send packet
	packet[0] = crtp_create_header(CRTP_PORT_SETPOINT, 0); // Commander packet to channel 0
	memcpy(&packet[1], (char*) &cpkt, sizeof(cpkt));
	tx_payload_len = 1 + sizeof(cpkt);
	send_packet();
}

/*static void send_crtp_cppm_emu_packet()
{
    struct CommanderPacketCppmEmu {
        struct {
            uint8_t numAuxChannels : 4; // Set to 0 through MAX_AUX_RC_CHANNELS
            uint8_t reserved : 4;
        } hdr;
        uint16_t channelRoll;
        uint16_t channelPitch;
        uint16_t channelYaw;
        uint16_t channelThrust;
        uint16_t channelAux[10];
    } __attribute__((packed)) cpkt;

    // To emulate PWM RC signals, rescale channels from (-10000,10000) to (1000,2000)
    // This is done by dividing by 20 to get a total range of 1000 (-500,500)
    // and then adding 1500 to to rebase the offset
#define RESCALE_RC_CHANNEL_TO_PWM(chan) ((chan / 20) + 1500)

    // Make sure the number of aux channels in use is capped to MAX_CPPM_AUX_CHANNELS
    // uint8_t numAuxChannels = Model.num_channels - 4;
    uint8_t numAuxChannels = 2; // TODO: Figure this out correctly
    if(numAuxChannels > MAX_CPPM_AUX_CHANNELS)
    {
        numAuxChannels = MAX_CPPM_AUX_CHANNELS;
    }

    cpkt.hdr.numAuxChannels = numAuxChannels;

    // Remap AETR to AERT (RPYT)
    cpkt.channelRoll = convert_channel_16b_limit(AILERON,1000,2000);
    cpkt.channelPitch = convert_channel_16b_limit(ELEVATOR,1000,2000);
    // Note: T & R Swapped:
    cpkt.channelYaw = convert_channel_16b_limit(RUDDER, 1000, 2000);
    cpkt.channelThrust = convert_channel_16b_limit(THROTTLE, 1000, 2000);

    // Rescale the rest of the aux channels - RC channel 4 and up
    for (uint8_t i = 4; i < 14; i++)
    {
        cpkt.channelAux[i] = convert_channel_16b_limit(i, 1000, 2000);
    }

    // Total size of the commander packet is a 1-byte header, 4 2-byte channels and
    // a variable number of 2-byte auxiliary channels
    uint8_t commanderPacketSize = 1 + 8 + (2*numAuxChannels);

    // Construct and send packet
    packet[0] = crtp_create_header(CRTP_PORT_SETPOINT_GENERIC, 0); // Generic setpoint packet to channel 0
    packet[1] = CRTP_SETPOINT_GENERIC_CPPM_EMU_TYPE;

    // Copy the header (1) plus 4 2-byte channels (8) plus whatever number of 2-byte aux channels are in use
    memcpy(&packet[2], (char*)&cpkt, commanderPacketSize); // Why not use sizeof(cpkt) here??
    tx_payload_len = 2 + commanderPacketSize; // CRTP header, commander type, and packet
    send_packet();
}*/

static void send_cmd_packet()
{
    // TODO: Fix this so we can actually configure the packet type
    // switch(Model.proto_opts[PROTOOPTS_CRTP_MODE])
    // {
    // case CRTP_MODE_CPPM:
    //     send_crtp_cppm_emu_packet();
    //     break;
    // case CRTP_MODE_RPYT:
    //     send_crtp_rpyt_packet();
    //     break;
    // default:
    //     send_crtp_rpyt_packet();
    // }

    // send_crtp_cppm_emu_packet(); // oh maAAAn
    send_crtp_rpyt_packet();
}

// State machine for setting up CRTP logging
// returns 1 when the state machine has completed, 0 otherwise
static uint8_t crtp_log_setup_state_machine()
{
    uint8_t state_machine_completed = 0;
    // A note on the design of this state machine:
    //
    // Responses from the crazyflie come in the form of ACK payloads.
    // There is no retry logic associated with ACK payloads, so it is possible
    // to miss a response from the crazyflie. To avoid this, the request
    // packet must be re-sent until the expected response is received. However,
    // re-sending the same request generates another response in the crazyflie
    // Rx queue, which can produce large backlogs of duplicate responses.
    //
    // To avoid this backlog but still guard against dropped ACK payloads,
    // transmit cmd packets (which don't generate responses themselves)
    // until an empty ACK payload is received (the crazyflie alternates between
    // 0xF3 and 0xF7 for empty ACK payloads) which indicates the Rx queue on the
    // crazyflie has been drained. If the queue has been drained and the
    // desired ACK has still not been received, it was likely dropped and the
    // request should be re-transmit.

    switch (crtp_log_setup_state) {
    case CFLIE_CRTP_LOG_SETUP_STATE_INIT:
        toc_size = 0;
        next_toc_variable = 0;
        vbat_var_id = 0;
        extvbat_var_id = 0;
        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO;
        // fallthrough
    case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO:
        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO;
        packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC);
        packet[1] = CRTP_LOG_TOC_CMD_INFO;
        tx_payload_len = 2;
        send_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_INFO:
        if (packet_ack() == PKT_ACKED) {
            if (rx_payload_len >= 3
                    && rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC)
                    && rx_packet[1] == CRTP_LOG_TOC_CMD_INFO) {
                // Received the ACK payload. Save the toc_size
                // and advance to the next state
                toc_size = rx_packet[2];
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM;
                return state_machine_completed;
            } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                // "empty" ACK packet received - likely missed the ACK
                // payload we are waiting for.
                // return to the send state and retransmit the request
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO;
                return state_machine_completed;
            }
        }

        // Otherwise, send a cmd packet to get the next ACK in the Rx queue
        send_cmd_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM:
        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM;
        packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC);
        packet[1] = CRTP_LOG_TOC_CMD_ELEMENT;
        packet[2] = next_toc_variable;
        tx_payload_len = 3;
        send_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CMD_GET_ITEM:
        if (packet_ack() == PKT_ACKED) {
            if (rx_payload_len >= 3
                    && rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_TOC)
                    && rx_packet[1] == CRTP_LOG_TOC_CMD_ELEMENT
                    && rx_packet[2] == next_toc_variable) {
                // For every element in the TOC we must compare its
                // type (rx_packet[3]), group and name (back to back
                // null terminated strings starting with the fifth byte)
                // and see if it matches any of the variables we need
                // for logging
                //
                // Currently enabled for logging:
                //  - vbatMV (LOG_UINT16)
                //  - extVbatMV (LOG_UINT16)
                //  - rssi (LOG_UINT8)
                if(rx_packet[3] == vbat_var_type
                        && (0 == strcmp((char*)&rx_packet[4], pm_group_name))
                        && (0 == strcmp((char*)&rx_packet[4 + strlen(pm_group_name) + 1], vbat_var_name))) {
                    // Found the vbat element - save it for later
                    vbat_var_id = next_toc_variable;
                }

                if(rx_packet[3] == extvbat_var_type
                        && (0 == strcmp((char*)&rx_packet[4], pm_group_name))
                        && (0 == strcmp((char*)&rx_packet[4 + strlen(pm_group_name) + 1], extvbat_var_name))) {
                    // Found the extvbat element - save it for later
                    extvbat_var_id = next_toc_variable;
                }

                if(rx_packet[3] == rssi_var_type
                        && (0 == strcmp((char*)&rx_packet[4], radio_group_name))
                        && (0 == strcmp((char*)&rx_packet[4 + strlen(radio_group_name) + 1], rssi_var_name))) {
                    // Found the rssi element - save it for later
                    rssi_var_id = next_toc_variable;
                }

                // Advance the toc variable counter
                // If there are more variables, read them
                // If not, move on to the next state
                next_toc_variable += 1;
                if(next_toc_variable >= toc_size) {
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK;
                } else {
                    // There are more TOC elements to get
                    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_ITEM;
                }
                return state_machine_completed;
            } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                // "empty" ACK packet received - likely missed the ACK
                // payload we are waiting for.
                // return to the send state and retransmit the request
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CMD_GET_INFO;
                return state_machine_completed;
            }
        }

        // Otherwise, send a cmd packet to get the next ACK in the Rx queue
        send_cmd_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK:
        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK;
        packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS);
        packet[1] = CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK;
        packet[2] = CFLIE_TELEM_LOG_BLOCK_ID; // Log block ID
        packet[3] = vbat_var_type; // Variable type
        packet[4] = vbat_var_id; // ID of the VBAT variable
        packet[5] = extvbat_var_type; // Variable type
        packet[6] = extvbat_var_id; // ID of the ExtVBat variable
        packet[7] = rssi_var_type; // Variable type
        packet[8] = rssi_var_id; // ID of the RSSI variable
        tx_payload_len = 9;
        send_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_CREATE_BLOCK:
        if (packet_ack() == PKT_ACKED) {
            if (rx_payload_len >= 2
                    && rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS)
                    && rx_packet[1] == CRTP_LOG_SETTINGS_CMD_CREATE_BLOCK) {
                // Received the ACK payload. Advance to the next state
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK;
                return state_machine_completed;
            } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                // "empty" ACK packet received - likely missed the ACK
                // payload we are waiting for.
                // return to the send state and retransmit the request
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_CREATE_BLOCK;
                return state_machine_completed;
            }
        }

        // Otherwise, send a cmd packet to get the next ACK in the Rx queue
        send_cmd_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK:
        crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK;
        packet[0] = crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS);
        packet[1] = CRTP_LOG_SETTINGS_CMD_START_LOGGING;
        packet[2] = CFLIE_TELEM_LOG_BLOCK_ID; // Log block ID 1
        packet[3] = CFLIE_TELEM_LOG_BLOCK_PERIOD_10MS; // Log frequency in 10ms units
        tx_payload_len = 4;
        send_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_ACK_CONTROL_START_BLOCK:
        if (packet_ack() == PKT_ACKED) {
            if (rx_payload_len >= 2
                    && rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_SETTINGS)
                    && rx_packet[1] == CRTP_LOG_SETTINGS_CMD_START_LOGGING) {
                // Received the ACK payload. Advance to the next state
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE;
                return state_machine_completed;
            } else if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
                // "empty" ACK packet received - likely missed the ACK
                // payload we are waiting for.
                // return to the send state and retransmit the request
                crtp_log_setup_state =
                        CFLIE_CRTP_LOG_SETUP_STATE_SEND_CONTROL_START_BLOCK;
                return state_machine_completed;
            }
        }

        // Otherwise, send a cmd packet to get the next ACK in the Rx queue
        send_cmd_packet();
        break;

    case CFLIE_CRTP_LOG_SETUP_STATE_COMPLETE:
        state_machine_completed = 1;
        return state_machine_completed;
        break;
    }

    return state_machine_completed;
}

static int cflie_init()
{
    NRF24L01_Initialize();

    // CRC, radio on
    NRF24L01_SetTxRxMode(TX_EN);
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP)); 
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x01);              // Auto Acknowledgement for data pipe 0
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);          // Enable data pipe 0
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, TX_ADDR_SIZE-2); // 5-byte RX/TX address
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x13);         // 3 retransmits, 500us delay

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num);        // Defined in initialize_rx_tx_addr
    NRF24L01_SetBitrate(data_rate);                          // Defined in initialize_rx_tx_addr

    NRF24L01_SetPower();
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);             // Clear data ready, data sent, and retransmit

    NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rx_tx_addr, TX_ADDR_SIZE);
    NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, TX_ADDR_SIZE);

    // this sequence necessary for module from stock tx
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);
    NRF24L01_Activate(0x73);                          // Activate feature register
    NRF24L01_ReadReg(NRF24L01_1D_FEATURE);

    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x01);       // Enable Dynamic Payload Length on pipe 0
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x06);     // Enable Dynamic Payload Length, enable Payload with ACK

    // 50ms delay in callback
    return 50000;
}

// TODO: Fix telemetry

// Update telemetry using the CRTP logging framework
// static void update_telemetry_crtplog()
// {
//     static uint8_t frameloss = 0;

//     // Read and reset count of dropped packets
//     frameloss += NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
//     NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num); // reset packet loss counter
//     Telemetry.value[TELEM_DSM_FLOG_FRAMELOSS] = frameloss;
//     TELEMETRY_SetUpdated(TELEM_DSM_FLOG_FRAMELOSS);

//     if (packet_ack() == PKT_ACKED) {
//         // See if the ACK packet is a cflie log packet
//         // A log data packet is a minimum of 5 bytes. Ignore anything less.
//         if (rx_payload_len >= 5) {
//             // Port 5 = log, Channel 2 = data
//             if (rx_packet[0] == crtp_create_header(CRTP_PORT_LOG, CRTP_LOG_CHAN_LOGDATA)) {
//                 // The log block ID
//                 if (rx_packet[1] == CFLIE_TELEM_LOG_BLOCK_ID) {
//                     // Bytes 5 and 6 are the Vbat in mV units
//                     uint16_t vBat;
//                     memcpy(&vBat, &rx_packet[5], sizeof(uint16_t));
//                     Telemetry.value[TELEM_CFLIE_INTERNAL_VBAT] = (int32_t) (vBat / 10); // The log value expects centivolts
//                     TELEMETRY_SetUpdated(TELEM_CFLIE_INTERNAL_VBAT);

//                     // Bytes 7 and 8 are the ExtVbat in mV units
//                     uint16_t extVBat;
//                     memcpy(&extVBat, &rx_packet[7], sizeof(uint16_t));
//                     Telemetry.value[TELEM_CFLIE_EXTERNAL_VBAT] = (int32_t) (extVBat / 10); // The log value expects centivolts
//                     TELEMETRY_SetUpdated(TELEM_CFLIE_EXTERNAL_VBAT);

//                     // Byte 9 is the RSSI
//                     Telemetry.value[TELEM_CFLIE_RSSI] = rx_packet[9];
//                     TELEMETRY_SetUpdated(TELEM_CFLIE_RSSI);
//                 }
//             }
//         }
//     }
// }

// // Update telemetry using the ACK packet payload
// static void update_telemetry_ackpkt()
// {
//     static uint8_t frameloss = 0;

//     // Read and reset count of dropped packets
//     frameloss += NRF24L01_ReadReg(NRF24L01_08_OBSERVE_TX) >> 4;
//     NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_ch_num); // reset packet loss counter
//     Telemetry.value[TELEM_DSM_FLOG_FRAMELOSS] = frameloss;
//     TELEMETRY_SetUpdated(TELEM_DSM_FLOG_FRAMELOSS);

//     if (packet_ack() == PKT_ACKED) {
//         // Make sure this is an ACK packet (first byte will alternate between 0xF3 and 0xF7
//         if (rx_packet[0] == 0xF3 || rx_packet[0] == 0xF7) {
//             // If ACK packet contains RSSI (proper length and byte 1 is 0x01)
//             if(rx_payload_len >= 3 && rx_packet[1] == 0x01) {
//                 Telemetry.value[TELEM_CFLIE_RSSI] = rx_packet[2];
//                 TELEMETRY_SetUpdated(TELEM_CFLIE_RSSI);
//             }
//             // If ACK packet contains VBAT (proper length and byte 3 is 0x02)
//             if(rx_payload_len >= 8 && rx_packet[3] == 0x02) {
//                 uint32_t vBat = 0;
//                 memcpy(&vBat, &rx_packet[4], sizeof(uint32_t));
//                 Telemetry.value[TELEM_CFLIE_INTERNAL_VBAT] = (int32_t)(vBat / 10); // The log value expects centivolts
//                 TELEMETRY_SetUpdated(TELEM_CFLIE_INTERNAL_VBAT);
//             }
//         }
//     }
// }

static uint16_t cflie_callback()
{
    switch (phase) {
    case CFLIE_INIT_SEARCH:
        send_search_packet();
        phase = CFLIE_SEARCH;
        break;
    case CFLIE_INIT_CRTP_LOG:
        if (crtp_log_setup_state_machine()) {
            phase = CFLIE_INIT_DATA;
        }
        break;
    case CFLIE_INIT_DATA:
        send_cmd_packet();
        phase = CFLIE_DATA;
        break;
    case CFLIE_SEARCH:
        switch (packet_ack()) {
        case PKT_PENDING:
            return PACKET_CHKTIME;                 // packet send not yet complete
        case PKT_ACKED:
            phase = CFLIE_DATA;
            // PROTOCOL_SetBindState(0);
            // MUSIC_Play(MUSIC_DONE_BINDING);
            BIND_DONE;
            break;
        case PKT_TIMEOUT:
            send_search_packet();
            cflie_counter = CFLIE_BIND_COUNT;
        }
        break;

    case CFLIE_DATA:
        // if (Model.proto_opts[PROTOOPTS_TELEMETRY] == TELEM_ON_CRTPLOG) {
        //     update_telemetry_crtplog();
        // } else if (Model.proto_opts[PROTOOPTS_TELEMETRY] == TELEM_ON_ACKPKT) {
        //     update_telemetry_ackpkt();
        // }

        if (packet_ack() == PKT_PENDING)
            return PACKET_CHKTIME;         // packet send not yet complete
        send_cmd_packet();
        break;
    }
    return PACKET_PERIOD;                  // Packet at standard protocol interval
}

// Generate address to use from TX id and manufacturer id (STM32 unique id)
static uint8_t initialize_rx_tx_addr()
{
    rx_tx_addr[0] = 
    rx_tx_addr[1] = 
    rx_tx_addr[2] = 
    rx_tx_addr[3] = 
    rx_tx_addr[4] = 0xE7; // CFlie uses fixed address

    // if (Model.fixed_id) {
    //     rf_ch_num = Model.fixed_id % 100;
    //     switch (Model.fixed_id / 100) {
    //     case 0:
    //         data_rate = NRF24L01_BR_250K;
    //         break;
    //     case 1:
    //         data_rate = NRF24L01_BR_1M;
    //         break;
    //     case 2:
    //         data_rate = NRF24L01_BR_2M;
    //         break;
    //     default:
    //         break;
    //     }

    //     if (Model.proto_opts[PROTOOPTS_TELEMETRY] == TELEM_ON_CRTPLOG) {
    //         return CFLIE_INIT_CRTP_LOG;
    //     } else {
    //         return CFLIE_INIT_DATA;
    //     }
    // } else {
    //     data_rate = NRF24L01_BR_250K;
    //     rf_ch_num = 10;
    //     return CFLIE_INIT_SEARCH;
    // }

    // Default 1
    data_rate = NRF24L01_BR_1M;
    rf_ch_num = 10;

    // Default 2
    // data_rate = NRF24L01_BR_2M;
    // rf_ch_num = 110;
    return CFLIE_INIT_SEARCH;
}

uint16_t initCFlie(void)
{
	BIND_IN_PROGRESS;	// autobind protocol

    phase = initialize_rx_tx_addr();
    crtp_log_setup_state = CFLIE_CRTP_LOG_SETUP_STATE_INIT;
    packet_count=0;

    int delay = cflie_init();

    // debugln("CFlie init!");

	return delay;
}

#endif

# 1 "src/V911S_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with V911S

#if defined(V911S_NRF24L01_INO)

#include "iface_nrf24l01.h"

//#define V911S_ORIGINAL_ID

#define V911S_PACKET_PERIOD			5000
#define V911S_BIND_PACKET_PERIOD	3300
#define V911S_INITIAL_WAIT			500
#define V911S_PACKET_SIZE			16
#define V911S_RF_BIND_CHANNEL		35
#define V911S_NUM_RF_CHANNELS		8
#define V911S_BIND_COUNT			200

// flags going to packet[1]
#define	V911S_FLAG_EXPERT	0x04
// flags going to packet[2]
#define	V911S_FLAG_CALIB	0x01

static void __attribute__((unused)) V911S_send_packet(uint8_t bind)
{
	if(bind)
	{
		packet[0] = 0x42;
		packet[1] = 0x4E;
		packet[2] = 0x44;
		for(uint8_t i=0;i<5;i++)
			packet[i+3] = rx_tx_addr[i];
		for(uint8_t i=0;i<8;i++)
			packet[i+8] = hopping_frequency[i];
	}
	else
	{
		uint8_t channel=hopping_frequency_no;
		if(rf_ch_num&1)
		{
			if((hopping_frequency_no&1)==0)
				channel+=8;
			channel>>=1;
		}
		if(rf_ch_num&2)
			channel=7-channel;
		packet[ 0]=(rf_ch_num<<3)|channel;
		packet[ 1]=V911S_FLAG_EXPERT;					// short press on left button
		packet[ 2]=GET_FLAG(CH5_SW,V911S_FLAG_CALIB);	// long  press on right button
		memset(packet+3,0x00,14);
		//packet[3..6]=trims TAER signed
		uint16_t ch=convert_channel_16b_limit(THROTTLE ,0,0x7FF);
		packet[ 7] = ch;
		packet[ 8] = ch>>8;
		ch=convert_channel_16b_limit(AILERON ,0x7FF,0);
		packet[ 8]|= ch<<3;
		packet[ 9] = ch>>5;
		ch=convert_channel_16b_limit(ELEVATOR,0,0x7FF);
		packet[10] = ch;
		packet[11] = ch>>8;
		ch=convert_channel_16b_limit(RUDDER  ,0x7FF,0);
		packet[11]|= ch<<3;
		packet[12] = ch>>5;
	}
	
	// Power on, TX mode, 2byte CRC
	XN297_Configure(_BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO) | _BV(NRF24L01_00_PWR_UP));
	if (!bind)
	{
		NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[channel]);
		hopping_frequency_no++;
		hopping_frequency_no&=7;	// 8 RF channels
	}
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_FlushTx();
	XN297_WritePayload(packet, V911S_PACKET_SIZE);

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) V911S_init()
{
	NRF24L01_Initialize();
	NRF24L01_SetTxRxMode(TX_EN);
	XN297_SetTXAddr((uint8_t *)"\x4B\x4E\x42\x4E\x44", 5);			// Bind address
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, V911S_RF_BIND_CHANNEL);	// Bind channel
	NRF24L01_FlushTx();
	NRF24L01_FlushRx();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);	// Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);		// No Auto Acknowldgement on all data pipes
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);	// Enable data pipe 0 only
	NRF24L01_SetBitrate(NRF24L01_BR_250K);			// 250Kbps
	NRF24L01_SetPower();
}

static void __attribute__((unused)) V911S_initialize_txid()
{
	//channels
	uint8_t offset=rx_tx_addr[3]%5;				// 0-4
	for(uint8_t i=0;i<V911S_NUM_RF_CHANNELS;i++)
		hopping_frequency[i]=0x10+i*5+offset;
	if(!offset) hopping_frequency[0]++;
	
	// channels order
	rf_ch_num=random(0xfefefefe)&0x03;			// 0-3
}

uint16_t V911S_callback()
{
	if(IS_BIND_DONE)
		V911S_send_packet(0);
	else
	{
		if (bind_counter == 0)
		{
			BIND_DONE;
			XN297_SetTXAddr(rx_tx_addr, 5);
			packet_period=V911S_PACKET_PERIOD;
		}
		else
		{
			V911S_send_packet(1);
			bind_counter--;
			if(bind_counter==100)		// same as original TX...
				packet_period=V911S_BIND_PACKET_PERIOD*3;
		}
	}
	return	packet_period;
}

uint16_t initV911S(void)
{
	V911S_initialize_txid();
	#ifdef V911S_ORIGINAL_ID
		rx_tx_addr[0]=0xA5;
		rx_tx_addr[1]=0xFF;
		rx_tx_addr[2]=0x70;
		rx_tx_addr[3]=0x8D;
		rx_tx_addr[4]=0x76;
		for(uint8_t i=0;i<V911S_NUM_RF_CHANNELS;i++)
			hopping_frequency[i]=0x10+i*5;
		hopping_frequency[0]++;
		rf_ch_num=0;
	#endif

	V911S_init();

	if(IS_BIND_IN_PROGRESS)
	{
		bind_counter = V911S_BIND_COUNT;
		packet_period= V911S_BIND_PACKET_PERIOD;
	}
	else
	{
		XN297_SetTXAddr(rx_tx_addr, 5);
		packet_period= V911S_PACKET_PERIOD;
	}
	hopping_frequency_no=0;
	return	V911S_INITIAL_WAIT;
}

#endif


# 1 "src/Symax_nrf24l01.ino" // Helps debugging !
/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 */
// compatible with Syma X5C-1, X11, X11C, X12 and for sub protocol X5C Syma X5C (original), X2

#if defined(SYMAX_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define SYMAX_BIND_COUNT			345		// 1.5 seconds
#define SYMAX_FIRST_PACKET_DELAY	12000
#define SYMAX_PACKET_PERIOD			4000	// Timeout for callback in uSec
#define SYMAX_INITIAL_WAIT			500

#define SYMAX_MAX_RF_CHANNELS    	17

#define SYMAX_FLAG_FLIP				0x01
#define SYMAX_FLAG_VIDEO			0x02
#define SYMAX_FLAG_PICTURE			0x04
#define SYMAX_FLAG_HEADLESS			0x08
#define SYMAX_XTRM_RATES			0x10

#define SYMAX_PAYLOADSIZE			10		// receive data pipes set to this size, but unused
#define SYMAX_MAX_PACKET_LENGTH		16		// X11,X12,X5C-1 10-byte, X5C 16-byte

enum {
	SYMAX_INIT1 = 0,
	SYMAX_BIND2,
	SYMAX_BIND3,
	SYMAX_DATA
};

static uint8_t __attribute__((unused)) SYMAX_checksum(uint8_t *data)
{
	uint8_t sum = data[0];

	for (uint8_t i=1; i < packet_length-1; i++)
	if ( sub_protocol==SYMAX5C )
		sum += data[i];
	else
		sum ^= data[i];

	return sum + ( sub_protocol==SYMAX5C ? 0 : 0x55 );
}

static void __attribute__((unused)) SYMAX_read_controls()
{
	// Protocol is registered AETRF, that is
	// Aileron is channel 1, Elevator - 2, Throttle - 3, Rudder - 4, Flip control - 5
	// Extended (trim-added) Rates - 6, Photo - 7, Video - 8, Headless - 9
	aileron  = convert_channel_s8b(AILERON);
	elevator = convert_channel_s8b(ELEVATOR);
	throttle = convert_channel_8b(THROTTLE);
	rudder   = convert_channel_s8b(RUDDER);

	flags=0;
	// Channel 5
	if (CH5_SW)
		flags = SYMAX_FLAG_FLIP;
	// Channel 6
	if (CH6_SW)
		flags |= SYMAX_XTRM_RATES;
	// Channel 7
	if (CH7_SW)
		flags |= SYMAX_FLAG_PICTURE;
	// Channel 8
	if (CH8_SW)
		flags |= SYMAX_FLAG_VIDEO;
	// Channel 9
	if (CH9_SW)
	{
		flags |= SYMAX_FLAG_HEADLESS;
		flags &= ~SYMAX_XTRM_RATES;	// Extended rates & headless incompatible
	}
}

#define X5C_CHAN2TRIM(X) ((((X) & 0x80 ? 0xff - (X) : 0x80 + (X)) >> 2) + 0x20)

static void __attribute__((unused)) SYMAX_build_packet_x5c(uint8_t bind)
{
	if (bind)
	{
		memset(packet, 0, packet_length);
		packet[7] = 0xae;
		packet[8] = 0xa9;
		packet[14] = 0xc0;
		packet[15] = 0x17;
	}
	else
	{
		SYMAX_read_controls();

		packet[0] = throttle;
		packet[1] = rudder;
		packet[2] = elevator ^ 0x80;  // reversed from default
		packet[3] = aileron;
		if (flags & SYMAX_XTRM_RATES)
		{	// drive trims for extra control range
			packet[4] = X5C_CHAN2TRIM(rudder ^ 0x80);
			packet[5] = X5C_CHAN2TRIM(elevator);
			packet[6] = X5C_CHAN2TRIM(aileron ^ 0x80);
		}
		else
		{
			packet[4] = 0x00;
			packet[5] = 0x00;
			packet[6] = 0x00;
		}
		packet[7] = 0xae;
		packet[8] = 0xa9;
		packet[9] = 0x00;
		packet[10] = 0x00;
		packet[11] = 0x00;
		packet[12] = 0x00;
		packet[13] = 0x00;
		packet[14] =  (flags & SYMAX_FLAG_VIDEO   ? 0x10 : 0x00) 
					| (flags & SYMAX_FLAG_PICTURE ? 0x08 : 0x00)
					| (flags & SYMAX_FLAG_FLIP    ? 0x01 : 0x00)
					| 0x04;// always high rates (bit 3 is rate control)
		packet[15] = SYMAX_checksum(packet);
	}
}

static void __attribute__((unused)) SYMAX_build_packet(uint8_t bind)
{
	if (bind)
	{
		packet[0] = rx_tx_addr[4];
		packet[1] = rx_tx_addr[3];
		packet[2] = rx_tx_addr[2];
		packet[3] = rx_tx_addr[1];
		packet[4] = rx_tx_addr[0];
		packet[5] = 0xaa;
		packet[6] = 0xaa;
		packet[7] = 0xaa;
		packet[8] = 0x00;
	}
	else
	{
		SYMAX_read_controls();
		packet[0] = throttle;
		packet[1] = elevator;
		packet[2] = rudder;
		packet[3] = aileron;
		packet[4] = (flags & SYMAX_FLAG_VIDEO   ? 0x80 : 0x00) | (flags & SYMAX_FLAG_PICTURE ? 0x40 : 0x00);
		packet[5] = 0xc0;	//always high rates (bit 7 is rate control)
		packet[6] = flags & SYMAX_FLAG_FLIP ? 0x40 : 0x00;
		packet[7] = flags & SYMAX_FLAG_HEADLESS ? 0x80 : 0x00;
		if (flags & SYMAX_XTRM_RATES)
		{	// use trims to extend controls
			packet[5] |= elevator >> 2;
			packet[6] |= rudder >> 2;
			packet[7] |= aileron >> 2;
		}
		packet[8] = 0x00;
	}
	packet[9] = SYMAX_checksum(packet);
}

static void __attribute__((unused)) SYMAX_send_packet(uint8_t bind)
{
	if (sub_protocol==SYMAX5C)
		SYMAX_build_packet_x5c(bind);
	else
		SYMAX_build_packet(bind);
		
	// clear packet status bits and TX FIFO
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x2e);
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, hopping_frequency[hopping_frequency_no]);
	NRF24L01_FlushTx();

	NRF24L01_WritePayload(packet, packet_length);

	if (packet_count++ % 2)	// use each channel twice
		hopping_frequency_no = (hopping_frequency_no + 1) % rf_ch_num;

	NRF24L01_SetPower();	// Set tx_power
}

static void __attribute__((unused)) symax_init()
{
	NRF24L01_Initialize();
	//
	NRF24L01_SetTxRxMode(TX_EN);
	//	
	NRF24L01_ReadReg(NRF24L01_07_STATUS);
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_EN_CRC) | _BV(NRF24L01_00_CRCO)); 
	NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);      // No Auto Acknoledgement
	NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x3F);  // Enable all data pipes (even though not used?)
	NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03);   // 5-byte RX/TX address
	NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0xff); // 4mS retransmit t/o, 15 tries (retries w/o AA?)
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);

	if (sub_protocol==SYMAX5C)
	{
		NRF24L01_SetBitrate(NRF24L01_BR_1M);
		packet_length = 16;
	}
	else
	{
		NRF24L01_SetBitrate(NRF24L01_BR_250K);
		packet_length = 10;
	}
	//
	NRF24L01_SetPower();
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);     // Clear data ready, data sent, and retransmit
	NRF24L01_WriteReg(NRF24L01_08_OBSERVE_TX, 0x00);
	NRF24L01_WriteReg(NRF24L01_09_CD, 0x00);
	NRF24L01_WriteReg(NRF24L01_0C_RX_ADDR_P2, 0xC3); // LSB byte of pipe 2 receive address
	NRF24L01_WriteReg(NRF24L01_0D_RX_ADDR_P3, 0xC4);
	NRF24L01_WriteReg(NRF24L01_0E_RX_ADDR_P4, 0xC5);
	NRF24L01_WriteReg(NRF24L01_0F_RX_ADDR_P5, 0xC6);
	NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, SYMAX_PAYLOADSIZE);   // bytes of data payload for pipe 1
	NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, SYMAX_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_13_RX_PW_P2, SYMAX_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_14_RX_PW_P3, SYMAX_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_15_RX_PW_P4, SYMAX_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_16_RX_PW_P5, SYMAX_PAYLOADSIZE);
	NRF24L01_WriteReg(NRF24L01_17_FIFO_STATUS, 0x00); // Just in case, no real bits to write here

	NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR , sub_protocol==SYMAX5C ? (uint8_t *)"\x6D\x6A\x73\x73\x73" : (uint8_t *)"\xAB\xAC\xAD\xAE\xAF" ,5);

	NRF24L01_ReadReg(NRF24L01_07_STATUS);
	NRF24L01_FlushTx();
	NRF24L01_ReadReg(NRF24L01_07_STATUS);
	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x0e);
	NRF24L01_ReadReg(NRF24L01_00_CONFIG); 
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0c); 
	NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0e);  // power on
}

static void __attribute__((unused)) symax_init1()
{
	// duplicate stock tx sending strange packet (effect unknown)
	uint8_t first_packet[] = {0xf9, 0x96, 0x82, 0x1b, 0x20, 0x08, 0x08, 0xf2, 0x7d, 0xef, 0xff, 0x00, 0x00, 0x00, 0x00};
	uint8_t chans_bind[] = {0x4b, 0x30, 0x40, 0x20};
	uint8_t chans_bind_x5c[] = {0x27, 0x1b, 0x39, 0x28, 0x24, 0x22, 0x2e, 0x36,
								0x19, 0x21, 0x29, 0x14, 0x1e, 0x12, 0x2d, 0x18};

	NRF24L01_FlushTx();
	NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x08);
	NRF24L01_WritePayload(first_packet, 15);

	if (sub_protocol==SYMAX5C)
	{
		rf_ch_num = sizeof(chans_bind_x5c);
		memcpy(hopping_frequency, chans_bind_x5c, rf_ch_num);
	}
	else
	{
		rx_tx_addr[4] = 0xa2; // this is constant in ID
		rf_ch_num = sizeof(chans_bind);
		memcpy(hopping_frequency, chans_bind, rf_ch_num);
	}
	hopping_frequency_no = 0;
	packet_count = 0;
}

// channels determined by last byte of tx address
static void __attribute__((unused)) symax_set_channels(uint8_t address)
{
	static const uint8_t start_chans_1[] = {0x0a, 0x1a, 0x2a, 0x3a};
	static const uint8_t start_chans_2[] = {0x2a, 0x0a, 0x42, 0x22};
	static const uint8_t start_chans_3[] = {0x1a, 0x3a, 0x12, 0x32};

	uint8_t laddress = address & 0x1f;
	uint8_t i;
	uint32_t *pchans = (uint32_t *)hopping_frequency;   // avoid compiler warning

	rf_ch_num = 4;

	if (laddress < 0x10)
	{
		if (laddress == 6)
			laddress = 7;
		for(i=0; i < rf_ch_num; i++)
			hopping_frequency[i] = start_chans_1[i] + laddress;
	}
	else
		if (laddress < 0x18)
		{
			for(i=0; i < rf_ch_num; i++)
				hopping_frequency[i] = start_chans_2[i] + (laddress & 0x07);
			if (laddress == 0x16)
			{
				hopping_frequency[0]++;
				hopping_frequency[1]++;
			}
		}
		else
			if (laddress < 0x1e)
			{
				for(i=0; i < rf_ch_num; i++)
					hopping_frequency[i] = start_chans_3[i] + (laddress & 0x07);
			}
			else
				if (laddress == 0x1e)
					*pchans = 0x38184121;
				else
					*pchans = 0x39194121;
}

static void __attribute__((unused)) symax_init2()
{
static	uint8_t chans_data_x5c[] = {0x1d, 0x2f, 0x26, 0x3d, 0x15, 0x2b, 0x25, 0x24,
								0x27, 0x2c, 0x1c, 0x3e, 0x39, 0x2d, 0x22};

	if (sub_protocol==SYMAX5C)
	{
		rf_ch_num = sizeof(chans_data_x5c);
		memcpy(hopping_frequency, chans_data_x5c, rf_ch_num);
	}
	else
	{
		symax_set_channels(rx_tx_addr[0]);
		NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, rx_tx_addr, 5);
	}
	hopping_frequency_no = 0;
	packet_count = 0;
}

uint16_t symax_callback()
{
	switch (phase)
	{
		case SYMAX_INIT1:
			symax_init1();
			phase = SYMAX_BIND2;
			return SYMAX_FIRST_PACKET_DELAY;
			break;
		case SYMAX_BIND2:
			bind_counter = SYMAX_BIND_COUNT;
			phase = SYMAX_BIND3;
			SYMAX_send_packet(1);
			break;
		case SYMAX_BIND3:
			if (bind_counter == 0)
			{
				symax_init2();
				phase = SYMAX_DATA;
				BIND_DONE;
			}
			else
			{
				SYMAX_send_packet(1);
				bind_counter--;
			}
			break;
		case SYMAX_DATA:
			SYMAX_send_packet(0);
			break;
	}
	return SYMAX_PACKET_PERIOD;
}

uint16_t initSymax()
{	
	packet_count = 0;
	flags = 0;
	BIND_IN_PROGRESS;	// autobind protocol
	symax_init();
	phase = SYMAX_INIT1;
	return	SYMAX_INITIAL_WAIT;
}

#endif

