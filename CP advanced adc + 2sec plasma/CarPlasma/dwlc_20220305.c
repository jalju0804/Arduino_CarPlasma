//===================================================================
//                      CAR PLASMA STERILIZER
//
// FILE		: dwlc_20220305.c
// DATE		: March 5th, 2022
// CPU		: ATtiny24A @ IRC 8MHz
// COMPILER	: WinAVR - 20100110
// IDE      : Microchip Studio 7
// ISP		: unknown
//
// REVISION	: 	2022-03-05	initial version
//===================================================================

#define F_CPU 1000000UL // IRC 8MHz, CKSEL = “0010”, SUT = “10”, and CKDIV8

/*** INCLUDE(S) ***/
//#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

/*** CONSTANT(S) ***/
#define UV_EN		PB0 // Output, Active high
#define CHARGER_IN	PB1 // Input, HIGH=충전완료, LOW=충전중
#define TOUCH_IN	PB2 // Input, Active low

#define FAN_EN		PA0 // Output, Active high
#define BLUELED_EN	PA1 // Output, Active high
#define REDLED_EN	PA2 // Output, Active high
#define ADC3_BAT	PA3 // Input(ad)
#define DC18_EN		PA6 // Output, Active high
#define PLASMA_EN	PA7 // Output, Active high

//bit mask definition for system flag
#define FLAG_SYSTICK_bm		((unsigned char)(1<<0)) // 2.048msec tick // timer
#define FLAG_CHECK_TOUCHKEY_bm		((unsigned char)(1<<1)) // 
#define FLAG_KEYLOCK_bm		((unsigned char)(1<<2)) //key high check
#define FLAG_PLASMA_EVENT_bm			((unsigned char)(1<<3)) // 
#define FLAG_PLASMA_bm			((unsigned char)(1<<4)) // plasma 
#define FLAG_BIT5_bm			((unsigned char)(1<<5)) // x
#define FLAG_PWR_EVENT_bm		((unsigned char)(1<<6)) // 초기화 
#define FLAG_PWR_bm			((unsigned char)(1<<7)) // pwr on

#define N_CHAT_THRESH	32 // 2msec * 16 = 32msec
#define N_SLEEP_TIME	180

enum sysmode { NOMAL, PLASMA_LO, SLEEP };
enum chargingmode { NO_CHARGING, NEED_CHARGING, DO_CHARGING };

#define N_PLASMA_LO_PERIOD		8000	// 2sec
#define N_PLASMA_LO_DUTY		40	// 2msec * 10 = 20msec

#define N_LED_PERIOD			8000 // 2msec * 500 = 1000msec
#define N_LED_DUTY				4000	// 2msec * 250 = 500msec

#define N_POWER_ONOFF			100

#define N_BATLEVEL_3V3			675
	
/*** VARIABLE(S) ***/
volatile unsigned char flag 	= 	0;

volatile unsigned char tim_base	=	0;

volatile unsigned char touch_chat	=	0;
volatile unsigned char touch_prev = 0;
volatile unsigned char touch_curr = 0;
volatile unsigned char touch_data = 0;

volatile unsigned char sys_mode = 0;
volatile unsigned char charger_mode = 0;

volatile uint16_t plasma_period = 0;
volatile uint16_t plasma_duty = 0;
volatile uint16_t plasma_cnt = 0;

volatile uint16_t led_period = 0;
volatile uint16_t led_duty = 0;
volatile uint16_t led_cnt = 0;

volatile int power_touch_chat = 0;


/*** PROTOTYPE(S) ***/
void InitSystem(void);
void ReadAdcAndUpdateLed(void);
void HandleSystemTick(void);
void HandleTouchKeyInput(void);
void HandleSystemPower(void);

/*** IMPLEMENTATION(S) ***/
void InitSystem(void)
{
	cli();

	DDRB  = (1<<PB0);				//UV_EN=output
	PORTB = (1<<PB2) | (0<<PB0);	//UV_EN=off, TOUCH_IN=pull-up
	
	DDRA  = (1<<PA7) | (1<<PA6) | (1<<PA2) | (1<<PA1) | (1<<PA0); //PLASMA_EN, DC18_EN, REDLED_EN, BLUELED_EN, FAN_EN=output
	PORTA = (0<<PA7) | (0<<PA6) | (0<<PA2) | (0<<PA1) | (0<<PA0); //PLASMA_EN, DC18_EN, REDLED_EN, BLUELED_EN, FAN_EN=off
	
	DIDR0 = (1<<ADC3D);								// ADC3 Digital Input Disable
	ADMUX =  (1<<MUX1) | (1<<MUX0);		// Reference Vol : 1.1V, ADC3
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);	// ADC Frequency, /64
	ADCSRB = (0<<ADLAR);							// left adjusted & Free Running mode

	TCCR0A = (1<<WGM01) | (1<<WGM00);	// OC0x disconnected, fast PWM mode
	TCCR0B = (1<<CS01);					// CLKtn=CLKio/8 => 1MHz/8=125KHz (8usec)
	TIMSK0 |= (1<<TOIE0);
	
	flag |= FLAG_PWR_bm;
	sei();
}

void ReadAdcAndUpdateLed(void)
{
	uint16_t value;
	
	if(flag & FLAG_PWR_bm) // 
	{
		ADCSRA |= (1<<ADSC);				// Start ADC Conversion
		while((ADCSRA & (1<<ADIF)) == 0);	// Wait for completion of ADC

		ADCSRA |= (1<<ADIF);				// Clear ADC Interrupt flag

		value = ADCL;
		value = ((ADCH)<<8)|value; 
		
		if( PINB & (1<<CHARGER_IN) )
		{
			charger_mode = DO_CHARGING;
			//PORTA  &= ~(1<<BLUELED_EN); // BLUELED Off
			PORTA  |= (1<<REDLED_EN); // REDLED On
			flag |= FLAG_PWR_bm;

		}
		else
		{
			if ( value < N_BATLEVEL_3V3 ) // low than 3.3V
			{
				if(!(flag & FLAG_BIT5_bm)) PORTA &= ~(1<<REDLED_EN); 
				charger_mode = NEED_CHARGING;
				led_duty = N_LED_DUTY;
				led_period = N_LED_PERIOD;
				flag |= FLAG_PWR_bm;
				
			}
			else
			{
				charger_mode = NO_CHARGING;
				PORTA  &= ~(1<<REDLED_EN); // REDLED Off
				flag |= FLAG_PWR_bm;
				led_cnt=0;
			}
			
		}
		
	}
}

void HandleSystemTick(void)
{
	if( (flag & FLAG_SYSTICK_bm) != 0 )
	{
		flag &= ~FLAG_SYSTICK_bm;
		
		//1. wake-up check routine

		//2. other timer services
		if( flag & FLAG_PLASMA_bm )
		{
			switch(sys_mode)
			{
				//case PLASMA_HI: 
				case PLASMA_LO: // = PLASMA_LO
					++plasma_cnt;
					
					if( plasma_duty == plasma_cnt )
					{
						PORTA  &= ~(1<<PLASMA_EN); // PLASMA Off	
					} 
					
					if( plasma_period == plasma_cnt )
					{
						PORTA  |= (1<<PLASMA_EN); // PLASMA On
						plasma_cnt = 0;
					}
				break;
			}
		}
				
		//3. other timer services`
		if(flag & FLAG_BIT5_bm)
		{
			switch(charger_mode)
			{
				case NO_CHARGING:
				PORTA &= ~(1<<REDLED_EN);
				break;
				case NEED_CHARGING:
				++led_cnt;
				if(led_duty == led_cnt)
				{
					PORTA &= ~(1<<REDLED_EN);
				}
				if( led_period == led_cnt )
				{
					PORTA  |= (1<<REDLED_EN); // REDLED On
					led_cnt = 0;
				}
				break;
				case DO_CHARGING:
				PORTA |= (1<<REDLED_EN);
				break;					
			}
		}
		
		//4. touch key status check
		flag |= FLAG_CHECK_TOUCHKEY_bm; //
	}
}

void HandleTouchKeyInput(void)
{
	if( flag & FLAG_CHECK_TOUCHKEY_bm )
	{
		flag &= ~FLAG_CHECK_TOUCHKEY_bm;
	
		touch_curr = PINB & (1<<TOUCH_IN);
			if(touch_curr == touch_prev)
			{
					if(++touch_chat > N_CHAT_THRESH )
					{
						touch_chat = 0;
				
						if( !(touch_prev & (1<<TOUCH_IN)) )
						{
							if( !(flag & FLAG_KEYLOCK_bm) )
							{
								if(++power_touch_chat > N_POWER_ONOFF)
								{
									flag |= FLAG_KEYLOCK_bm;
									tim_base = 0; // clear sleep-timer while key is pressed.
						
									if( ++sys_mode > SLEEP ) sys_mode = PLASMA_LO;
									switch(sys_mode)
									{
										case NOMAL:
										flag |= FLAG_PWR_bm; // 1
										flag |= FLAG_PWR_EVENT_bm; 
										break;
							
										case PLASMA_LO:
										flag |= FLAG_BIT5_bm;
										flag |= FLAG_PWR_bm; // 1
										flag |= FLAG_PWR_EVENT_bm; // 1
										flag |= FLAG_PLASMA_bm;
										flag |= FLAG_PLASMA_EVENT_bm;
										break;
	
										case SLEEP:
										PORTA  &= ~(1<<FAN_EN); // FAN Off
										PORTA  &= ~(1<<DC18_EN); // DC18 Off
										PORTA  &= ~(1<<PLASMA_EN); // PLASMA Off
										PORTA  &= ~(1<<BLUELED_EN); // BLUELED Off
										PORTB  &= ~(1<<UV_EN); // UV Off
										PORTA  &= ~(1<<REDLED_EN);
								
										flag |= FLAG_PLASMA_EVENT_bm;
										flag |= FLAG_PWR_bm;
										flag &= ~FLAG_BIT5_bm;
										flag &= ~FLAG_PLASMA_bm;
										plasma_cnt	=	0;
										//flag |= FLAG_PWR_EVENT_bm;
									
										break;
									}
								}
							}
						}
						else
						{
							flag &= ~(FLAG_KEYLOCK_bm);
							power_touch_chat = 0;
						}
					}
			}
			else
			{
				touch_prev = touch_curr;
				touch_chat = 0;
				power_touch_chat = 0;
			}
	}
}

void HandleSystemPower(void)
{
	if(!(flag & FLAG_PWR_EVENT_bm)) 
		return;
	else // flag pwr event bm == 1
		flag &= ~FLAG_PWR_EVENT_bm;
	
	/*if( flag & FLAG_PWR_bm ) //flag pwr bm == 1
	{//Power on sequence
		PORTA  |= (1<<FAN_EN); // FAN On
		PORTB  |= (1<<UV_EN); // UV On
	}
	*/
	/*else //flag pwr bm == 0 && flag pwr event bm == 0
	{//Power off sequence

		PORTA  &= ~(1<<FAN_EN); // FAN Off
		PORTA  &= ~(1<<DC18_EN); // DC18 Off
		PORTA  &= ~(1<<PLASMA_EN); // PLASMA Off
		PORTA  &= ~(1<<BLUELED_EN); // BLUELED Off
		PORTA  &= ~(1<<REDLED_EN); // REDLED Off
		PORTB  &= ~(1<<UV_EN); // UV Off

		//0. 인터럽트 disable
		cli();

		//1. PCINT1 인터럽트 설정
		GIMSK &= ~(1<<PCIE0);	// PCIE disable
		PCMSK0 = (1<<PCINT10);	// PCINT10 Enable
		GIMSK = (1<<PCIE0);		// PCIE enable
		//상관 없음
		
	
		//2. Watchdog 인터럽트 설정
		
		wdt_reset();
		WDTCSR |= (1<<WDCE) | (1<<WDE);
		WDTCSR |= (1<<WDIE) | (1<<WDE) | (1<<WDP2) | (1<<WDP1); //~1.0s
	
		
		
		//3. 인터럽트 enable & 슬립실행
		sei();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();
		
		//4. 인터럽트 disable & Watchdog off
		cli();
		 //2012-12-13 remove, disable watchdog wakeup
		wdt_reset();
		MCUCR &= ~(1<<WDRF);
		WDTCSR |= (1<<WDCE) | (1<<WDE);
		WDTCSR = 0x00;
	
		GIMSK &= ~(1<<PCIE0);	// PCIE disable
		MCUCR &= ~(1<<WDRF);
		WDTCSR |= (1<<WDCE) | (1<<WDE);
		PCMSK0 = 0x00;	// PCINTx disable
		tim_base = 0;
		sei();
		
	}
	*/
}

void HandlePlasmaMode(void)
{
	if( !(flag & FLAG_PLASMA_EVENT_bm) )
		return;
	else
		flag &= ~FLAG_PLASMA_EVENT_bm;
		
	switch(sys_mode)
	{
		case PLASMA_LO:
			PORTA  |= (1<<DC18_EN); // DC18 On
			PORTA  |= (1<<PLASMA_EN); // PLASMA On
			flag |= FLAG_PLASMA_bm;
			PORTA |= (1<<BLUELED_EN) | (1<<FAN_EN); //BLUE LED + FAN ON
			PORTB |= (1<<UV_EN); //UV ON
			plasma_cnt	=	0;
			plasma_duty	=	N_PLASMA_LO_DUTY;
			plasma_period	=	N_PLASMA_LO_PERIOD;
			break;
	}
	
}

ISR(PCINT0_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
}



ISR(TIM0_OVF_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
	flag |= FLAG_SYSTICK_bm; //2ms tick
}



ISR(WDT_vect)
{
	//PORTB ^= (1<<PB3); //debug code, remove me
	//PORTB &= ~(1<<PB3); //debug code, remove me
}



int main(void)
{
	InitSystem();
	
	while(1)
	{
		HandleTouchKeyInput();
		ReadAdcAndUpdateLed();
		HandleSystemTick();
		HandleSystemPower();
		HandlePlasmaMode();
	}
	return 0;
}//main ends



/*** END-OF-FILE ***/



