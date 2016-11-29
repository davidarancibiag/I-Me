#include <Arduino.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_VS1053.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/dtostrf.h>		// For dtostrf function

#define Serial SerialUSB

const uint8_t pin_SPDT_SD =		 6;
const uint8_t pin_SPDT_USB =	12;
const uint8_t pin_Vbus =		 5;
const uint8_t pin_BattStat =	10;
const uint8_t pin_Button =		11;
const uint8_t pin_Led_Green =	16;
const uint8_t pin_Led_Blue =	15;
const uint8_t pin_Led_Red =		 8;		// 8 in final app
const uint8_t pin_RTC_Int =		13;
const uint8_t pin_SD_CS =		 4;
const uint8_t pin_SD_CD =		 7;
const uint8_t pin_Codec_DREQ =	 0;		// VS1053 Data request, ideally an Interrupt pin
const uint8_t pin_Codec_XRST =	 1;		// VS1053 reset pin (output)
const uint8_t pin_Codec_XCS =	18;		// VS1053 chip select pin (output)
const uint8_t pin_Codec_XDCS =	19;		// VS1053 Data/command select pin (output)

volatile boolean state_button = 0;
volatile boolean is_button_changed = 0;
volatile boolean is_button_pressed = 0;
volatile boolean is_debounce_running = 0;
volatile boolean state_button_previous = 0;
volatile boolean button_time_Overflow = 0;
volatile boolean waiting_off_state = 0;
volatile boolean new_audiofile = 0;
volatile uint16_t counter_button = 0;
volatile uint32_t counter_button_time = 0;
volatile uint32_t counter_button_time1 = 0;

volatile uint8_t private_first_loop = 0;
volatile uint8_t private_minute = 0;
volatile uint8_t private_second = 0;
volatile uint8_t audio_file_index = 0;

volatile uint16_t counter_PWMLED = 10;
volatile boolean is_counting_up = 1;

#define DEVICE_ID		1			// Change manually
#define THRESHOLD_TIME_1 400		// ms
#define THRESHOLD_TIME_2 1500		// ms
#define THRESHOLD_TIME_3 10000		// ms
#define DEBOUNCE_TIME 30			// ms
#define PRIVATE_TIME 1				// minutes
#define TEMPERATURE_PERIOD 10		// seconds
#define BATTERY_PERIOD 60			// seconds
#define BATTERY_THRESHOLD_VOLTAGE 3.5f	// Volts
#define SLEEP_TIME_CHECK_BATT	1	// minutes


#define LED_ON LOW
#define LED_OFF HIGH
#define PRESSED true
#define UNPRESSED false
#define SPDT_SDCARD_uC		LOW
#define SPDT_SDCARD_CRDR	HIGH
#define SPDT_USB_uC			LOW
#define SPDT_USB_CRDR		HIGH


#define DS3231_ADDRESS  0x68
#define DS3231_ALARM1   0X07
#define DS3231_ALARM2   0X0B
#define DS3231_CONTROL  0x0E
#define DS3231_STATUSREG 0x0F

#define DS3231_CONTROL_A1IE 0x01
#define DS3231_CONTROL_A2IE 0x02
#define DS3231_CONTROL_INTCN 0x04
#define DS3231_CONTROL_RS1 0x08
#define DS3231_CONTROL_RS2 0x10
#define DS3231_CONTROL_CONV 0x20
#define DS3231_CONTROL_BBSQW 0x40
#define DS3231_CONTROL_EOSC 0x80

#define DS3231_STATUS_A1F 0x01
#define DS3231_STATUS_A2F 0x02
#define DS3231_STATUS_BSY 0x04
#define DS3231_STATUS_EN32kHz 0x08
#define DS3231_STATUS_OSF 0x80

#define DS3231_EN32kHz_ON true
#define DS3231_EN32kHz_OFF false


uint8_t state_machine = 0;
uint8_t state_machine_previous = 0;
uint8_t state_machine_transition = 0;

struct dataRTC{
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
};

struct dataTEMP{
	uint8_t address;
	uint16_t temp_bits;
	float temp;
};

struct dataAXIS{
	uint16_t x;
	uint16_t y;
	uint16_t z;
};

struct dataIMU{
	dataAXIS ACC;
	dataAXIS GYRO;
	dataAXIS MAG;
};

struct dataflag{
	bool time;
	bool date;
	bool imu;
	bool temp;
	bool battery;
	bool eda;
	bool privacy;
	bool audiofile_index;
};

struct datastruct{
	dataRTC TIME;
	dataRTC TIME_ANT;
//	dataRTC TIME_PRIVATE_END;
	dataIMU IMU;
	float TEMP_INT;
	float TEMP_EXT;
	float battery;
	float eda;
	bool privacy;
	uint8_t audiofile_index;
	dataflag flag;
} volatile data;

enum led_modes_type
{
	led_mode_blink_5s,
	led_mode_blink_10s,
	led_mode_blink_short,
	led_mode_blink_extended,
	led_mode_dimmer,
	led_mode_on,
	led_mode_off,
	led_mode_off_all
} ;

enum led_color_type
{
	led_color_green,
	led_color_blue,
	led_color_red,
	led_color_bluegreen
} ;

enum state_machine_type
{
	state_init,
	state_sleep,
	state_rec,
	state_privado,
	state_bateria_baja,
	state_bateria_cargando,
	state_bateria_completa,
	state_programador,
	state_reset
};

enum state_machine_transition_type
{
	S0,		// No transition
	S1,		// VBus HIGH while button is pressed
	S2,		// Switch pressed for less than 1 second
	S3,		// Switch pressed for 1 second
	S4,		// Switch pressed for 30 seconds
	S5,		// Battery voltage below threshold
	S6,		// VBus HIGH and BattStat HIGH
	S7,		// VBus HIGH and BattStat LOW
	S8,		// Vbus falling edge
	T1		// Private mode timeout
};


//RTC_DS3231 rtc;

LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define TMP102_I2C_ADDRESS0 0b1001000  // TMP102 I2C address if A0PIN tied to ground.
#define TMP102_I2C_ADDRESS1 0b1001001  // TMP102 I2C address if A0PIN tied to VCC.
//#define TMP102_I2C_ADDRESS 0b1001010  // TMP102 I2C address if A0PIN tied to SDA.
//#define TMP102_I2C_ADDRESS 0b1001011  // TMP102 I2C address if A0PIN tied to SCL.

void Timer_Led_Init_for_Dimmer(uint8_t color);
void Timer_Led_Init_for_Blink(uint8_t color, uint8_t mode_blink);
void Timer_Led_Stop(void);
void Led_Function(uint8_t color, uint8_t mode);
void Timer_Button_Init (void);
void General_Clock_7_Init (void);
void State_Machine_Next_State (void);
void State_Machine_Action (void);
float getTemp102(bool selector);
uint16_t saveRecordedData(boolean isrecord);
uint16_t saveRemainingRecordedData(void);
uint16_t getADCvalue(uint8_t mux_value);
void Recording_End(void);
void Timer_Sampling_Enable(void);
void IMU_init(void);
void RTC_init(void);
void SD_Card_init(void);
void ADC_init(void);
void Timer_Sampling_init(void);
void Audio_Codec_init(void);
void Button_Calculate_Pressed_Time(void);
void Save_Data_to_SD(void);
void Set_Private_End_Time(uint8_t minute);
void Files_Data_Audio_init(void);
void Get_Date_and_Time(void);

uint8_t bcd2bin (uint8_t val);
uint8_t bin2bcd (uint8_t val);
void RTC_adjust(void);
void RTC_now(void);
uint8_t Read_I2C_Register(uint8_t addr, uint8_t reg);
void Write_I2C_Register(uint8_t addr, uint8_t reg, uint8_t val);
void Get_Compiler_Date_Time(void);
void print_date_time(void);
void RTC_Set_Alarm_Every_X_Minutes(uint8_t minutes);
void RTC_Clear_Interrupt_Flags (void);
void RTC_Disable_Alarm (void);

#define BATTERY_ADC_MUX		0x7
#define EDA1_ADC_MUX		0x0
#define EDA2_ADC_MUX		0x4

volatile boolean syncWithRTC = 0;

File dataFile;  // the file we will save our sampled data to
File audioFile;  // the file we will save our recording to

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(pin_Codec_XRST, pin_Codec_XCS, pin_Codec_XDCS, pin_Codec_DREQ, pin_SD_CS);

#define AUDIOBUFFSIZE 128  // 64 or 128 bytes.
uint8_t audiobuffer[2][AUDIOBUFFSIZE];

#define DATABUFFSIZE 100	// number of characters per sampling
char databuffer[2][DATABUFFSIZE];
uint16_t count_char[2];

#define _BV(x) (1<<(x))

boolean who_save = 0;
volatile boolean datatosd = 0;
volatile uint8_t dataposarray = 0;		// sensors buffer arrays full of data
volatile uint8_t audioposarray = 0;		// codec buffer arrays full of data


void setup()
{
	// Don't forget to deactive Watchdog, because it is still on after software reset!
	
	General_Clock_7_Init ();			// Clock input for button and led calculations
	state_machine = state_init;
	State_Machine_Action ();
	
	pinMode(pin_Button, INPUT_PULLDOWN);   // Button pin with pull-down resistor	
	pinMode(pin_SPDT_SD, OUTPUT);	 // SPDT SD Card selector
	pinMode(pin_SPDT_USB, OUTPUT);   // SPDT USB selector
	digitalWrite(pin_SPDT_SD, SPDT_SDCARD_uC);		// Activate connexion between card reader and microcontroller
	digitalWrite(pin_SPDT_USB, SPDT_USB_CRDR);		// Activate connexion between card reader and USB
	
	Serial.begin(115200);
	delay(1000);
	Serial.println("I-ME datalogger v1.0");
   
	Wire.begin();				// start the I2C library
	Wire.setClock(800000L);		// Run at 400kHz (check the number difference!)
 
	IMU_init();
	RTC_init();
	SD_Card_init();
	ADC_init();
	Timer_Sampling_init();
	Timer_Button_Init ();
	
	data.flag.time = 1;	 // For first date and time data

	attachInterrupt(pin_Button	, ISR_Button	, CHANGE);
	attachInterrupt(pin_Vbus	, ISR_VBus		, CHANGE);
	attachInterrupt(pin_BattStat, ISR_BattStat	, CHANGE);
	
	Serial.println("End Setup\n");
	
	state_machine = state_sleep;
	state_machine_previous = state_init;
	State_Machine_Action ();
	state_machine_transition = S0;
}

void loop()
{
	if (state_machine_transition)		// If any change occurs, check the state machine
	{
		State_Machine_Next_State ();
		
		if (state_machine != state_machine_previous)					// Avoid to set parameters multiple times if the same state is reached
		{
			State_Machine_Action ();
		}
	}
	
	if ((is_button_changed == true) && (is_debounce_running == false))
	{
		Button_Calculate_Pressed_Time();
	}
	
	if (datatosd)
	{
		Save_Data_to_SD();
	}
}

void ISR_Button()
{
	//	counter_button++;
	//	Serial.print("Change ");
	//	Serial.println(counter_button);
	if (!is_debounce_running)
	{
		Serial.print("\tButton -> ");
		is_button_changed = true;
		//EIC->INTENCLR.bit.EXTINT0 = 1;	//Disable button interrupt until TC_MC0_interrupt se active
		is_debounce_running = true;
		
		TcCount16* TIMER_BUTTON = (TcCount16*) TC5; // get timer struct
		
		if (waiting_off_state)
		{
			counter_button_time = TIMER_BUTTON->COUNT.reg;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
			
			TIMER_BUTTON->CC[0].reg = counter_button_time + DEBOUNCE_TIME;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
		}
		else
		{
			TIMER_BUTTON->CC[0].reg = DEBOUNCE_TIME;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
			
			Timer_Button_Stop();							// To avoid freeze program (button bug)
			
			TIMER_BUTTON->CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER | TC_CTRLBCLR_ONESHOT;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
		}
	}

	EIC->INTFLAG.bit.EXTINT0 = 1;	// Clear the button interrupt flag
}


void ISR_VBus()
{
	delayMicroseconds(10000);						// Improve this solution
	boolean Vbus_value = digitalRead(pin_Vbus);
	if (Vbus_value)
	{
		Serial.println("VBus HIGH");
		if (waiting_off_state == 1)
		{
			Serial.println("S1 transition");
			state_machine_transition = S1;			// To programmer
			waiting_off_state = 0;
			Timer_Button_Stop();
			return;
		}
		
		boolean BattStat_value = digitalRead(pin_BattStat);
		if (BattStat_value)
		{
			state_machine_transition = S6;
		}
		else
		{
			state_machine_transition = S7;
		}
	}
	else								// USB disconnected
	{
		Serial.println("VBus LOW");
		
		if(state_machine == state_bateria_cargando)
		{
			data.battery = getADCvalue(BATTERY_ADC_MUX) * 2 * 0.00322581;	// get battery voltage
			if (data.battery < BATTERY_THRESHOLD_VOLTAGE)
			{
				state_machine_transition = S5;	// To bateria baja
			}
			else
			{
				state_machine_transition = S8;	// To sleep
			}
		}
		else
		{
			state_machine_transition = S8;	// To sleep
		}
		
	}
}


void ISR_BattStat()
{
	delayMicroseconds(10000);						// Improve this solution
	boolean BattStat_value = digitalRead(pin_BattStat);
	boolean Vbus_value = digitalRead(pin_Vbus);
	if (BattStat_value)
	{
		Serial.println("BattStat_value HIGH");
		if (Vbus_value)
		{
			state_machine_transition = S6;
		}
	}
	else
	{
		Serial.println("BattStat_value LOW");
		if (Vbus_value)
		{
			state_machine_transition = S7;
		}
	}
}

void Timer_Button_Stop(void)
{
	TcCount16* TIMER_BUTTON = (TcCount16*) TC5; // get timer struct
	
	TIMER_BUTTON->CTRLBSET.reg = TC_CTRLBCLR_CMD_STOP;
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
}


void ISR_RTC(void)
{
	Serial.print("Alarm! \t");
	RTC_now();
	print_date_time();
	data.battery = getADCvalue(BATTERY_ADC_MUX) * 2 * 0.00322581;	// get battery voltage
	if (data.battery < BATTERY_THRESHOLD_VOLTAGE)
	{
		detachInterrupt(pin_RTC_Int);
		Serial.println("Batt Low");
		state_machine_transition = S5;	// To bateria baja
	}
	else
	{
		Serial.println("Set new Alarm");
		RTC_Set_Alarm_Every_X_Minutes(SLEEP_TIME_CHECK_BATT);
	}
}


void TC4_Handler()
{
	TcCount8* TIMER_LEDS = (TcCount8*) TC4; // get timer struct

	if (TIMER_LEDS->INTFLAG.bit.OVF == 1)  // Overflow caused the interrupt
	{
		if (is_counting_up)
		{
			counter_PWMLED++;
		}
		else
		{
			counter_PWMLED--;
		}

		if (counter_PWMLED >= 82)
		{
			is_counting_up = 0;
		}
		else if (counter_PWMLED <= 10)	// Lower values present rare blink
		{
			is_counting_up = 1;
		}
		
		TIMER_LEDS->CC[0].reg = counter_PWMLED;
		while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1); // wait for sync
		
		TIMER_LEDS->CC[1].reg = counter_PWMLED;
		while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1); // wait for sync

		TIMER_LEDS->INTFLAG.bit.OVF = 1;    // writing a one clears the Overflow flag
	}
}


void TC5_Handler()
{
	TcCount16* TIMER_BUTTON = (TcCount16*) TC5; // get timer struct

	if (TIMER_BUTTON->INTFLAG.bit.MC0 == 1)  // CC0 Match caused the interrupt
	{
		Serial.print("Debounce Ready -> ");
		is_debounce_running = false;
		//		EIC->INTENSET.bit.EXTINT0 = 1;			// Reactivate Button interrupt
		TIMER_BUTTON->INTFLAG.bit.MC0 = 1;    // writing a one clears the Match CC0 flag
	}
	/*	else if (TIMER_BUTTON->INTFLAG.bit.MC1 == 1)  // CC1 Match caused the interrupt
	{
	//Serial.println("\tMatch1");
	TIMER_BUTTON->INTFLAG.bit.MC1 = 1;    // writing a one clears the Match CC1 flag
	}*/
	else if (TIMER_BUTTON->INTFLAG.bit.OVF == 1)  // Overflow caused the interrupt
	{
		Serial.println("\tOverflow");
		is_debounce_running = false;
		button_time_Overflow = 1;
		TIMER_BUTTON->INTFLAG.bit.OVF = 1;    // writing a one clears the Overflow flag
		System_Reset();
	}
}


void System_Reset()
{
	//To write to this register, you must write 0x5FA to the VECTKEY field, otherwise the processor ignores the write.
	//SYSRESETREQ will cause a system reset asynchronously, so need to wait afterwards.
	SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
	while(1);
}


void Sleep_Standby()
{
	//To write to this register, you must write 0x5FA to the VECTKEY field, otherwise the processor ignores the write.
	//SLEEPDEEP will cause a system go to STANDBY Sleep mode. Lowest power consumption.
	//SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos) | SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk ;

	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
}


void Sleep_Idle(uint8_t mode)
{
	if (mode < 3)
	{
		PM->SLEEP.reg = PM_SLEEP_IDLE(mode);
		SCB->AIRCR = (0x5FA<<SCB_AIRCR_VECTKEY_Pos) | SCB_SCR_SLEEPONEXIT_Msk;
	}
	else
	{
		// Error. Do nothing.
	}
}


void Led_Function(uint8_t color, uint8_t mode)
{
	switch (mode)
	{
		case led_mode_blink_5s :
		case led_mode_blink_10s :
		case led_mode_blink_extended :
		if (color == led_color_green)
		{
			pinMode(pin_Led_Red, OUTPUT);
			pinMode(pin_Led_Blue, OUTPUT);
			digitalWrite(pin_Led_Red, LED_OFF);
			digitalWrite(pin_Led_Blue, LED_OFF);
			
			Timer_Led_Init_for_Blink(color, mode);
		}
		else if (color == led_color_blue)
		{
			pinMode(pin_Led_Green, OUTPUT);
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Green, LED_OFF);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			Timer_Led_Init_for_Blink(color, mode);
		}
		else if (color == led_color_bluegreen)
		{
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			Timer_Led_Init_for_Blink(color, mode);
		}
		else if (color == led_color_red)
		{
			// Nothing to do
			Timer_Led_Stop();
		}
		break;
		
		case led_mode_blink_short :				// Could be implemented by PWM and interrupt
		if (color == led_color_green)
		{
			pinMode(pin_Led_Green, OUTPUT);
			pinMode(pin_Led_Blue, OUTPUT);
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Blue, LED_OFF);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			digitalWrite(pin_Led_Green, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Green, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Green, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Green, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Green, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Green, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Green, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Green, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Green, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Green, LED_OFF);
		}
		else if (color == led_color_blue)
		{
			pinMode(pin_Led_Green, OUTPUT);
			pinMode(pin_Led_Blue, OUTPUT);
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Green, LED_OFF);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			digitalWrite(pin_Led_Blue, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Blue, LED_OFF);
		}
		else if (color == led_color_red)
		{
			pinMode(pin_Led_Green, OUTPUT);
			pinMode(pin_Led_Blue, OUTPUT);
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Green, LED_OFF);
			digitalWrite(pin_Led_Blue, LED_OFF);
			
			digitalWrite(pin_Led_Red, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Red, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Red, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Red, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Red, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Red, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Red, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Red, LED_OFF);
			delay(100);
			digitalWrite(pin_Led_Red, LED_ON);
			delay(100);
			digitalWrite(pin_Led_Red, LED_OFF);
		}
		break;
		
		case led_mode_dimmer :
		if (color == led_color_green)
		{
			pinMode(pin_Led_Red, OUTPUT);
			pinMode(pin_Led_Blue, OUTPUT);
			digitalWrite(pin_Led_Red, LED_OFF);
			digitalWrite(pin_Led_Blue, LED_OFF);
			
			Timer_Led_Init_for_Dimmer(color);
			
		}
		else if (color == led_color_blue)
		{
			pinMode(pin_Led_Green, OUTPUT);
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Green, LED_OFF);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			Timer_Led_Init_for_Dimmer(color);
		}
		else if (color == led_color_bluegreen)
		{
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Red, LED_OFF);
			
			Timer_Led_Init_for_Dimmer(color);
		}
		else if (color == led_color_red)
		{
			//Nothing to do
			Timer_Led_Stop();
		}
		break;
		
		case led_mode_on :
		Timer_Led_Stop();
		pinMode(pin_Led_Blue, OUTPUT);
		pinMode(pin_Led_Green, OUTPUT);
		pinMode(pin_Led_Red, OUTPUT);
		digitalWrite(pin_Led_Blue, LED_OFF);
		digitalWrite(pin_Led_Green, LED_OFF);
		digitalWrite(pin_Led_Red, LED_OFF);
		
		if (color == led_color_green)
		{
			pinMode(pin_Led_Green, OUTPUT);
			digitalWrite(pin_Led_Green, LED_ON);
		}
		else if (color == led_color_blue)
		{
			pinMode(pin_Led_Blue, OUTPUT);
			digitalWrite(pin_Led_Blue, LED_ON);
		}
		else if (color == led_color_bluegreen)
		{
			pinMode(pin_Led_Blue, OUTPUT);
			pinMode(pin_Led_Green, OUTPUT);
			digitalWrite(pin_Led_Blue, LED_ON);
			digitalWrite(pin_Led_Green, LED_ON);
		}
		else if (color == led_color_red)
		{
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Red, LED_ON);
		}
		break;
		
		case led_mode_off :
		Timer_Led_Stop();
		if (color == led_color_green)
		{
			pinMode(pin_Led_Green, OUTPUT);
			digitalWrite(pin_Led_Green, LED_OFF);
		}
		else if (color == led_color_blue)
		{
			pinMode(pin_Led_Blue, OUTPUT);
			digitalWrite(pin_Led_Blue, LED_OFF);
		}
		else if (color == led_color_red)
		{
			pinMode(pin_Led_Red, OUTPUT);
			digitalWrite(pin_Led_Red, LED_OFF);
		}
		break;
		
		case led_mode_off_all :
		Timer_Led_Stop();
		pinMode(pin_Led_Green, OUTPUT);
		pinMode(pin_Led_Blue, OUTPUT);
		pinMode(pin_Led_Red, OUTPUT);
		digitalWrite(pin_Led_Green, LED_OFF);
		digitalWrite(pin_Led_Blue, LED_OFF);
		digitalWrite(pin_Led_Red, LED_OFF);
		break;
	}
}


void Timer_Led_Stop(void)
{
	// The type cast must fit with the selected timer mode
	TcCount8* TIMER_LEDS = (TcCount8*) TC4; // get timer struct
	
	TIMER_LEDS->CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
}


void Timer_Led_Init_for_Dimmer(uint8_t color)
{
	if (color == led_color_blue)
	{
		PORT->Group[1].PINCFG[8].bit.PMUXEN = 0x1;	// Blue Led Pin PB08
		PORT->Group[1].PMUX[4].bit.PMUXE = PORT_PMUX_PMUXE_E_Val;
	}
	else if (color == led_color_green)
	{
		PORT->Group[1].PINCFG[9].bit.PMUXEN = 0x1;	// Green Led Pin PB09
		PORT->Group[1].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_E_Val;
	}
	else if (color == led_color_bluegreen)
	{
		PORT->Group[1].PINCFG[8].bit.PMUXEN = 0x1;	// Blue Led Pin PB08
		PORT->Group[1].PMUX[4].bit.PMUXE = PORT_PMUX_PMUXE_E_Val;
		PORT->Group[1].PINCFG[9].bit.PMUXEN = 0x1;	// Green Led Pin PB09
		PORT->Group[1].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_E_Val;
	}
	
	counter_PWMLED = 10;
	is_counting_up = 1;
	
	// The type cast must fit with the selected timer mode
	TcCount8* TIMER_LEDS = (TcCount8*) TC4; // get timer struct

	TIMER_LEDS->CTRLA.reg = TC_CTRLA_SWRST;			// Reset and Disable TCCx
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	// Set Timer counter Mode to 8 bits, Will run in standby (sleep), Normal PWM mode, Prescaler = 2.
	TIMER_LEDS->CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_RUNSTDBY | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER_DIV2;
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	TIMER_LEDS->CTRLC.reg = TC_CTRLC_INVEN0 | TC_CTRLC_INVEN1;	// Invert PWM outputs
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);				// wait for sync
	
	TIMER_LEDS->PER.reg = 82;						// Set PWM Frequency
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
	
	// Interrupts
	TIMER_LEDS->INTENCLR.reg = 0xFF;           // disable all interrupts
	TIMER_LEDS->INTENSET.bit.OVF = 1;          // activate overflow

	// Enable InterruptVector
	NVIC_EnableIRQ(TC4_IRQn);
	
	TIMER_LEDS->CTRLA.reg |= TC_CTRLA_ENABLE;		// Enable TC
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
}


void Timer_Led_Init_for_Blink(uint8_t color, uint8_t blink_mode)
{
	if (color == led_color_blue)
	{
		PORT->Group[1].PINCFG[8].bit.PMUXEN = 0x1;	// Blue Led Pin PB08
		PORT->Group[1].PMUX[4].bit.PMUXE = PORT_PMUX_PMUXE_E_Val;
	}
	else if (color == led_color_green)
	{
		PORT->Group[1].PINCFG[9].bit.PMUXEN = 0x1;	// Green Led Pin PB09
		PORT->Group[1].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_E_Val;
	}
	else if (color == led_color_bluegreen)
	{
		PORT->Group[1].PINCFG[8].bit.PMUXEN = 0x1;	// Blue Led Pin PB08
		PORT->Group[1].PMUX[4].bit.PMUXE = PORT_PMUX_PMUXE_E_Val;
		PORT->Group[1].PINCFG[9].bit.PMUXEN = 0x1;	// Green Led Pin PB09
		PORT->Group[1].PMUX[4].bit.PMUXO = PORT_PMUX_PMUXO_E_Val;
	}
	
	// The type cast must fit with the selected timer mode
	TcCount8* TIMER_LEDS = (TcCount8*) TC4; // get timer struct

	TIMER_LEDS->CTRLA.reg = TC_CTRLA_SWRST;			// Reset and Disable TCCx
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
	
	// Set Timer counter Mode to 8 bits, Will run in standby (sleep), Normal PWM mode, Prescaler = 1024
	TIMER_LEDS->CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_RUNSTDBY  | TC_CTRLA_WAVEGEN_NPWM | TC_CTRLA_PRESCALER_DIV1024;
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	TIMER_LEDS->CTRLC.reg = TC_CTRLC_INVEN0 | TC_CTRLC_INVEN1;	// Invert PWM outputs
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);				// wait for sync
	
	if (blink_mode == led_mode_blink_5s)
	{
		TIMER_LEDS->PER.reg = 80;						// Set PWM Frequency
		while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		
		if (color == led_color_blue)
		{
			TIMER_LEDS->CC[0].reg = 1;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_green)
		{
			TIMER_LEDS->CC[1].reg = 1;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_bluegreen)
		{
			TIMER_LEDS->CC[0].reg = 1;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
			TIMER_LEDS->CC[1].reg = 1;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
	}
	else if (blink_mode == led_mode_blink_10s)
	{
		TIMER_LEDS->PER.reg = 160;						// Set PWM Frequency
		while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		
		if (color == led_color_blue)
		{
			TIMER_LEDS->CC[0].reg = 2;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_green)
		{
			TIMER_LEDS->CC[1].reg = 2;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_bluegreen)
		{
			TIMER_LEDS->CC[0].reg = 2;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
			TIMER_LEDS->CC[1].reg = 2;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
	}
	else if (blink_mode == led_mode_blink_extended)
	{
		TIMER_LEDS->PER.reg = 80;						// Set PWM Frequency
		while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		
		if (color == led_color_blue)
		{
			TIMER_LEDS->CC[0].reg = 10;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_green)
		{
			TIMER_LEDS->CC[1].reg = 10;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
		else if (color == led_color_bluegreen)
		{
			TIMER_LEDS->CC[0].reg = 10;						// Set blue duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
			TIMER_LEDS->CC[1].reg = 10;						// Set green duty cycle
			while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
		}
	}
	
	// Interrupts
	TIMER_LEDS->INTENCLR.reg = 0xFF;           // disable all interrupts

	TIMER_LEDS->CTRLA.reg |= TC_CTRLA_ENABLE;		// Enable TC
	while (TIMER_LEDS->STATUS.bit.SYNCBUSY == 1);	// wait for sync
}


void Timer_Button_Init (void)
{
	// The type cast must fit with the selected timer mode
	TcCount16* TIMER_BUTTON = (TcCount16*) TC5; // get timer struct

	TIMER_BUTTON->CTRLA.reg &= ~TC_CTRLA_ENABLE;		// Disable TCCx
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	TIMER_BUTTON->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;	// Set Timer counter Mode to 16 bits
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	TIMER_BUTTON->CTRLA.reg |= TC_CTRLA_WAVEGEN_NFRQ;	// Set TC as normal Normal Frq
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	TIMER_BUTTON->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;	// Set perscaler
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);			// wait for sync

	TIMER_BUTTON->CC[0].reg = DEBOUNCE_TIME;			// Set debounce value
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	//	TIMER_BUTTON->CC[1].reg = 1000;						// Set double click max time
	//	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	// Interrupts
	TIMER_BUTTON->INTENCLR.reg = 0xFF;					// disable all interrupts
	TIMER_BUTTON->INTENSET.bit.MC0 = 1;					// activate Match CC0
	//	TIMER_BUTTON->INTENSET.bit.MC1 = 1;					// activate Match CC1
	TIMER_BUTTON->INTENSET.bit.OVF = 1;					// activate Overflow

	NVIC_EnableIRQ(TC5_IRQn);							// Enable InterruptVector

	TIMER_BUTTON->CTRLA.reg |= TC_CTRLA_ENABLE;			// Enable TC
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	TIMER_BUTTON->CTRLBSET.reg = TC_CTRLBCLR_CMD_STOP;	// Stop TC
	while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1);		// wait for sync

	Serial.println("Timer5 (button) initialized.");
}


void General_Clock_7_Init (void)
{
	// Setup clock GCLK7 divided by 3 (2^(DIV+1))
	GCLK->GENDIV.reg = (uint32_t)(GCLK_GENDIV_ID(7) | GCLK_GENDIV_DIV(0));
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	
	// Setup clock GCLK7 Enable, with source OSCULP32K
	GCLK->GENCTRL.reg = (uint32_t)((GCLK_GENCTRL_ID(7) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL ));
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
	
	// Enable clock for TC
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK7 | GCLK_CLKCTRL_ID_TC4_TC5) ;
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
}


void State_Machine_Action (void)
{			
	switch (state_machine)
	{
		case state_init:
		Serial.println("State init");
		//Led_Function(led_color_green, led_mode_dimmer);		// Need to be changed after
		Led_Function(led_color_red, led_mode_blink_short);
		break;
		
		case state_sleep:
		Serial.println("State sleep");
		Led_Function(led_color_blue, led_mode_blink_5s);
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		digitalWrite(pin_SPDT_SD, SPDT_SDCARD_uC);		// Activate connexion between SD-Card and microcontroller
		digitalWrite(pin_SPDT_USB, SPDT_USB_CRDR);		// Activate connexion between USB and Card Reader
		attachInterrupt(pin_RTC_Int	, ISR_RTC		, FALLING);
		RTC_Set_Alarm_Every_X_Minutes(SLEEP_TIME_CHECK_BATT);	// Activate RTC alarm
		//Sleep_Idle(1);										// Does not work for the moment
		//Sleep_Standby();
		Serial.println("Sleeping?");
		break;
		
		case state_rec:
		Serial.println("State recording");
		Led_Function(led_color_green, led_mode_blink_5s);
		if (state_machine_previous == state_privado)
		{
			break;
		}
		else
		{
			RTC_Disable_Alarm ();
			Files_Data_Audio_init();
			Audio_Codec_init();
			musicPlayer.startRecordOgg(true);
			Get_Date_and_Time();
			Timer_Sampling_Enable();
			break;	
		}
		break;
		
		case state_privado:
		Serial.println("State privado");
		Led_Function(led_color_green, led_mode_blink_extended);
		Set_Private_End_Time(PRIVATE_TIME);
		private_first_loop = 1;
		break;
		
		case state_bateria_baja:
		Serial.println("State bateria baja");
		Led_Function(led_color_bluegreen, led_mode_blink_10s);
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		RTC_Disable_Alarm ();
		break;
		
		case state_bateria_cargando:
		Serial.println("State bateria cargando");
		Led_Function(led_color_blue, led_mode_dimmer);
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		RTC_Disable_Alarm ();
		digitalWrite(pin_SPDT_SD, SPDT_SDCARD_CRDR);		// Activate connexion between card reader and SD card
		digitalWrite(pin_SPDT_USB, SPDT_USB_CRDR);			// Activate connexion between card reader and USB
		break;
		
		case state_bateria_completa:
		Serial.println("State bateria completa");
		Led_Function(led_color_green, led_mode_on);
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		RTC_Disable_Alarm ();
		digitalWrite(pin_SPDT_SD, SPDT_SDCARD_CRDR);		// Activate connexion between card reader and SD card
		digitalWrite(pin_SPDT_USB, SPDT_USB_CRDR);			// Activate connexion between card reader and USB
		break;
		
		case state_programador:
		Serial.println("State programador");
		Led_Function(led_color_bluegreen, led_mode_dimmer);
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		RTC_Disable_Alarm ();
		digitalWrite(pin_SPDT_SD, SPDT_SDCARD_uC);		// Activate connexion between SD-Card and microcontroller
		digitalWrite(pin_SPDT_USB, SPDT_USB_uC);		// Activate connexion between USB and microcontroller
		break;
		
		case state_reset:
		Serial.println("State reset");
		if ((state_machine_previous == state_rec) || (state_machine_previous == state_privado))
		{
			Recording_End();
		}
		System_Reset();
		break;
		
		default:
		Serial.println("State DEFAULT - To sleep");
		Serial.println("State sleep");
		Led_Function(led_color_blue, led_mode_blink_5s);
		break;
	}
}


void State_Machine_Next_State (void)
{
	switch (state_machine)
	{
		case state_sleep:
		state_machine_previous = state_sleep;
		switch (state_machine_transition)
		{	
			case S1:
			Serial.println("\tFrom sleep - Transition S1 - To programador");
			state_machine = state_programador;
			break;
			
			case S3:
			Serial.println("\tFrom sleep - Transition S3 - To rec");
			state_machine = state_rec;
			break;
			
			case S4:
			Serial.println("\tFrom sleep - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S5:
			Serial.println("\tFrom sleep - Transition S5 - To bateria baja");
			state_machine = state_bateria_baja;
			break;
			
			case S6:
			Serial.println("\tFrom sleep - Transition S6 - To bateria cargando");
			state_machine = state_bateria_cargando;
			break;
			
			default:
			Serial.println("\tFrom sleep - Transition not valid - stays in sleep");
			state_machine = state_sleep;
			break;
		}
		break;
		
		case state_rec:
		state_machine_previous = state_rec;
		switch (state_machine_transition)
		{	
			case S1:
			Serial.println("\tFrom rec - Transition S1 - To programador");
			state_machine = state_programador;
			break;
			
			case S2:
			Serial.println("\tFrom rec - Transition S2 - To Privado");
			state_machine = state_privado;
			break;
			
			case S3:
			Serial.println("\tFrom rec - Transition S3 - To Sleep");
			state_machine = state_sleep;
			break;
			
			case S4:
			Serial.println("\tFrom rec - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S5:
			Serial.println("\tFrom rec - Transition S5 - To bateria baja");
			state_machine = state_bateria_baja;
			break;
			
			case S6:
			Serial.println("\tFrom rec - Transition S6 - To bateria cargando");
			state_machine = state_bateria_cargando;
			break;
			
			default:
			Serial.println("\tFrom rec - Transition not valid - stays in rec");
			state_machine = state_rec;
			break;
		}
		break;
		
		case state_privado:
		state_machine_previous = state_privado;
		switch (state_machine_transition)
		{	
			case T1:
			Serial.println("\tFrom privado - Transition T1 - To rec");
			state_machine = state_rec;
			break;
			
			case S1:
			Serial.println("\tFrom privado - Transition S1 - To programador");
			state_machine = state_programador;
			break;
			
			case S3:
			Serial.println("\tFrom privado - Transition S3 - To sleep");
			state_machine = state_sleep;
			break;
			
			case S4:
			Serial.println("\tFrom privado - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S5:
			Serial.println("\tFrom privado - Transition S5 - To bateria baja");
			state_machine = state_bateria_baja;
			break;
			
			case S6:
			Serial.println("\tFrom privado - Transition S6 - To bateria cargando");
			state_machine = state_bateria_cargando;
			break;
			
			default:
			Serial.println("\tFrom privado - Transition not valid - stays in privado");
			state_machine = state_privado;
			break;
		}
		break;
		
		case state_bateria_baja:
		state_machine_previous = state_bateria_baja;
		switch (state_machine_transition)
		{
			case S1:
			Serial.println("\tFrom bat. baja - Transition S1 - To programador");
			state_machine = state_programador;
			break;
			
			case S4:
			Serial.println("\tFrom bat. baja - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S6:
			Serial.println("\tFrom bat. baja - Transition S6 - To bateria cargando");
			state_machine = state_bateria_cargando;
			break;
			
			default:
			Serial.println("\tFrom bat. baja - Transition not valid - stays in bateria baja");
			state_machine = state_bateria_baja;
			break;
		}
		break;
		
		case state_bateria_cargando:
		state_machine_previous = state_bateria_cargando;
		switch (state_machine_transition)
		{
			case S4:
			Serial.println("\tFrom bat. cargando - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S7:
			Serial.println("\tFrom bat. cargando - Transition S7 - To bateria completa");
			state_machine = state_bateria_completa;
			break;
			
			case S5:
			Serial.println("\tFrom bat. cargando - Transition S6 - To bateria baja");
			state_machine = state_bateria_baja;
			break;
			
			case S8:
			Serial.println("\tFrom bat. cargando - Transition S8 - To sleep");
			state_machine = state_sleep;
			break;
			
			default:
			Serial.println("\tFrom bat. cargando - Transition not valid - stays in bateria cargando");
			state_machine = state_bateria_cargando;
			break;
		}
		break;
		
		case state_bateria_completa:
		state_machine_previous = state_bateria_completa;
		switch (state_machine_transition)
		{
			case S4:
			Serial.println("\tFrom bat. completa - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			case S6:
			Serial.println("\tFrom bat. completa - Transition S6 - To bat. cargando");
			state_machine = state_bateria_cargando;
			break;
			
			case S8:
			Serial.println("\tFrom bat. completa - Transition S8 - To sleep");
			state_machine = state_sleep;
			break;
			
			default:
			Serial.println("\tFrom bat. completa - Transition not valid - stays in bateria completa");
			state_machine = state_bateria_completa;
			break;
		}
		break;
		
		case state_programador:
		state_machine_previous = state_programador;
		switch (state_machine_transition)
		{
			case S4:
			Serial.println("\tFrom programador - Transition S4 - To reset");
			state_machine = state_reset;
			break;
			
			default:
			Serial.println("\tFrom programador - Transition not valid - stays in programador");
			state_machine = state_programador;
			break;
		}
		break;
		
		default: //go to sleep
		state_machine = state_sleep;
		break;
	}
	
	state_machine_transition = S0;	// to avoid enter multiples times into the state machine
}

void Recording_End(void)
{
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	NVIC_DisableIRQ(TC3_IRQn);
	
	dataFile.close();
	
	musicPlayer.stopRecordOgg();
	saveRemainingRecordedData();
	Serial.println("End recording");
	// close it up
	audioFile.close();
}


void TC3_Handler()
{
	TcCount16* TC = (TcCount16*) TC3; // get timer struct

	if (TC->INTFLAG.bit.MC0 == 1)
	{  // A compare to cc0 caused the interrupt
		
		if (!syncWithRTC)
		{
			RTC_now();
			if (data.TIME.second != data.TIME_ANT.second)
			{
				syncWithRTC = 1;
			}
			else
			{
				return;
			}
		}
		
		digitalWrite(pin_Led_Red, HIGH);
		
		static uint16_t tick_seconds_temp = TEMPERATURE_PERIOD - 1;	// for synchronize slow samples (temperature)
		static uint16_t tick_seconds_batt = BATTERY_PERIOD - 1;		// for synchronize slow samples (battery)
		static uint16_t counter_samples = 0;						// for synchronize fast samples (eda)
		
		uint8_t count_temp = 0;
		
		//........................................................................ IMU
		imu.readAccel();		// Get Accelerometer values
		imu.readGyro();			// Get Gyroscope values
		imu.readMag();			// Get Magnetometer values
		
		// Convert data from float to unsigned int (0-8192)
		data.IMU.ACC.x = ((int)(imu.ax) >> 3) + 4096;
		data.IMU.ACC.y = ((int)(imu.ay) >> 3) + 4096;
		data.IMU.ACC.z = ((int)(imu.az) >> 3) + 4096;
		data.IMU.GYRO.x = ((int)(imu.gx) >> 3) + 4096;
		data.IMU.GYRO.y = ((int)(imu.gy) >> 3) + 4096;
		data.IMU.GYRO.z = ((int)(imu.gz) >> 3) + 4096;
		data.IMU.MAG.x = ((int)(imu.mx) >> 3) + 4096;
		data.IMU.MAG.y = ((int)(imu.my) >> 3) + 4096;
		data.IMU.MAG.z = ((int)(imu.mz) >> 3) + 4096;
		
		data.flag.imu = 1;		// Flag to save to sd card
		
		//........................................................................ RTC
		RTC_now();		// Get RTC data
		
		if (data.TIME.second != data.TIME_ANT.second)		// Update previous time values
		{
//			Serial.println(counter_samples);
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "%u", counter_samples);
			count_char[dataposarray] += count_temp;
			
			tick_seconds_temp++;
			tick_seconds_batt++;
			counter_samples = 0;
			
			data.TIME_ANT.second = data.TIME.second;
			data.TIME_ANT.minute = data.TIME.minute;
			data.TIME_ANT.hour = data.TIME.hour;
			data.flag.time = 1;
			
			if (data.TIME.day != data.TIME_ANT.day)		// Update previous date values
			{
				data.TIME_ANT.day = data.TIME.day;
				data.TIME_ANT.month = data.TIME.month;
				data.TIME_ANT.year = data.TIME.year;
				data.flag.date = 1;
			}
		}
		
		//........................................................................ EDA
		if ((counter_samples%5) == 0)	// EDA sample
		{
			float Vb;
			float Vo;
			Vo = getADCvalue(EDA1_ADC_MUX) * 0.00322581;
			Vb = getADCvalue(EDA2_ADC_MUX) * 0.00322581;
			data.eda = (3.3 - Vb) / (Vb - Vo);	//Rpiel = (Vcc - Vb)/(Vb - Vo)
			data.flag.eda = 1;
		}
		
		//........................................................................ TEMP
		if ((tick_seconds_temp == TEMPERATURE_PERIOD) && (counter_samples == 0))	// temp sample
		{
			data.TEMP_INT = getTemp102(0);
			data.TEMP_EXT = getTemp102(1);
			data.flag.temp = 1;
			tick_seconds_temp = 0;
		}
		
		//........................................................................ BATT
		if ((tick_seconds_batt == BATTERY_PERIOD) && (counter_samples == 0))		// battery sample
		{
			data.battery = getADCvalue(BATTERY_ADC_MUX) * 2 * 0.00322581;
			data.flag.battery = 1;
			tick_seconds_batt = 0;
			
			if (data.battery < BATTERY_THRESHOLD_VOLTAGE)				// exit rec mode if battery is below minimum charge
			{
				state_machine_transition = S5;
			}
		}

		//........................................................................ Privado
		if (state_machine == state_privado)	// private mode sample
		{
			if (private_first_loop == 1)
			{
				data.privacy = 1;	//P1
				data.flag.privacy = 1;
				private_first_loop = 0;
			}
			else if ((data.TIME.minute == private_minute) && (data.TIME.second == private_second))
			{
				data.privacy = 0;	//P0
				data.flag.privacy = 1;
				state_machine_transition = T1;
			}
			else
			{
				//data.privacy = 0;	//NULL
				data.flag.privacy = 0;
			}
		}
		
		//........................................................................ Audio
		if (new_audiofile == 1)
		{
			data.audiofile_index = audio_file_index;
			data.flag.audiofile_index = 1;
			new_audiofile = 0;	
		} 
		
		
		/*
		Serial.print(data.flag.time);
		Serial.print(data.flag.imu);
		Serial.print(data.flag.eda);
		Serial.print(data.flag.temp);
		Serial.print(data.flag.battery);
		Serial.println(data.flag.privacy);
		*/
		
		// Create character array with sampled data
		if (data.flag.date)
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",%u-%u-%u,", data.TIME.year, data.TIME.month, data.TIME.day);
			count_char[dataposarray] += count_temp;

			data.flag.date = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",,");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.time)
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "%u:%u:%u,", data.TIME.hour, data.TIME.minute, data.TIME.second);
			count_char[dataposarray] += count_temp;

			data.flag.time = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.imu)
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "%u,%u,%u,%u,%u,%u,%u,%u,%u,", data.IMU.ACC.x, data.IMU.ACC.y, data.IMU.ACC.z, data.IMU.GYRO.x, data.IMU.GYRO.y, data.IMU.GYRO.z, data.IMU.MAG.x, data.IMU.MAG.y, data.IMU.MAG.z);
			count_char[dataposarray] += count_temp;

			data.flag.imu = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",,,,,,,,,");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.eda)
		{
			dtostrf(data.eda, 4, 2, databuffer[dataposarray] + count_char[dataposarray]);
			count_char[dataposarray] += 4;
			
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;

			data.flag.eda = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.temp)
		{
			dtostrf(data.TEMP_INT, 4, 1, databuffer[dataposarray] + count_char[dataposarray]);
			count_char[dataposarray] += 4;
			
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
			
			dtostrf(data.TEMP_EXT, 4, 1, databuffer[dataposarray] + count_char[dataposarray]);
			count_char[dataposarray] += 4;
			
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;

			data.flag.temp = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",,");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.battery)
		{
			dtostrf(data.battery, 4, 2, databuffer[dataposarray] + count_char[dataposarray]);
			count_char[dataposarray] += 4;
			
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
			
			data.flag.battery = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.privacy)
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "P%u,", data.privacy);
			count_char[dataposarray] += count_temp;

			data.flag.privacy = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
		}
		
		if (data.flag.audiofile_index)
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "%u,", data.audiofile_index);
			count_char[dataposarray] += count_temp;

			data.flag.audiofile_index = 0;
		}
		else
		{
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], ",");
			count_char[dataposarray] += count_temp;
		}
		count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "\n");
		count_char[dataposarray] += count_temp;
		
		dataposarray++;		// move to the next data sensors buffer


		// Get audio codec data
		uint16_t wordswaiting = musicPlayer.recordedWordsWaiting();
		
		if (wordswaiting*2 > AUDIOBUFFSIZE)
		{
			for (uint16_t addr=0; addr < AUDIOBUFFSIZE; addr+=2)
			{
				uint16_t t = musicPlayer.recordedReadWord();
				audiobuffer[audioposarray][addr] = t >> 8;
				audiobuffer[audioposarray][addr+1] = t;
			}
			audioposarray++;			// move to the next data codec buffer
		}
		/*
		Serial.print(wordswaiting);
		Serial.print('\t');
		Serial.print(who_save);
		Serial.print('\t');
		Serial.print(array_occupied);
		Serial.print('\t');
		Serial.println(array_sensors_occupied);
		*/
		
		counter_samples++;
		who_save = !who_save;
		
		datatosd = 1;
		
		digitalWrite(pin_Led_Red, LOW);
		
		TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the flag ovf flag
	}
}


float getTemp102(bool selector){
	byte firstbyte, secondbyte; //these are the bytes we read from the TMP102 temperature registers
	int val; /* an int is capable of storing two bytes, this is where we "chuck" the two bytes together. */
	float convertedtemp;
	
	if(selector){
		//Wire.beginTransmission(TMP102_I2C_ADDRESS1); //Say hi to the sensor.
		//Wire.write(0x00);
		//Wire.endTransmission();
		Wire.requestFrom(TMP102_I2C_ADDRESS1, 2);
	}
	else{
		//Wire.beginTransmission(TMP102_I2C_ADDRESS0); //Say hi to the sensor.
		//Wire.write(0x00);
		//Wire.endTransmission();
		Wire.requestFrom(TMP102_I2C_ADDRESS0, 2);
	}
	Wire.endTransmission();
	
	firstbyte  = (Wire.read());
	secondbyte = (Wire.read());

	val = ((firstbyte) << 4);
	val |= (secondbyte >> 4);

	convertedtemp = val*0.0625;
	
	return convertedtemp;
}

/*
uint16_t saveRecordedData(boolean isrecord) {
uint16_t written = 0;

if (!isrecord)
{
uint16_t wordswaiting = musicPlayer.recordedWordsWaiting();
//Serial.print(wordswaiting); Serial.println(" remaining");
// wrapping up the recording!
uint16_t addr = 0;
for (int x=0; x < wordswaiting-1; x++)
{
// fill the buffer!
uint16_t t = musicPlayer.recordedReadWord();
audiobuffer[0][addr] = t >> 8;
audiobuffer[0][addr+1] = t;
if (addr > AUDIOBUFFSIZE)
{
if (! audioFile.write(audiobuffer[0], AUDIOBUFFSIZE))
{
Serial.println("Couldn't write!");
while (1);
}
audioFile.flush();
addr = 0;
}
}
if (addr != 0)
{
if (!audioFile.write(audiobuffer[0], addr)) {
Serial.println("Couldn't write!"); while (1);
}
written += addr;
}

audioFile.flush();

wordswaiting = musicPlayer.recordedWordsWaiting();
Serial.print(wordswaiting); Serial.println(" remaining");
}

return written;
}
*/
uint16_t saveRemainingRecordedData(void)
{
	uint16_t written = 0;	//Bytes written to file
	uint16_t wordswaiting = musicPlayer.recordedWordsWaiting();

	Serial.print(wordswaiting); Serial.println(" words remaining");
	// wrapping up the recording!
	uint16_t addr = 0;
	for (uint16_t x=0; x < wordswaiting; x++)
	{
		// fill the buffer!
		uint16_t t = musicPlayer.recordedReadWord();
		audiobuffer[0][addr] = t >> 8;
		audiobuffer[0][addr+1] = t;
		
		addr += 2;
		
		if (addr >= AUDIOBUFFSIZE)
		{
			if (! audioFile.write(audiobuffer[0], AUDIOBUFFSIZE))
			{
				//Serial.println("Couldn't write!");
				while (1);
			}
			//Serial.print(x+1); Serial.println(" words saved");
			addr = 0;
			written += AUDIOBUFFSIZE;
		}
	}
	
	do
	{
		wordswaiting = musicPlayer.recordedWordsWaiting();
		delayMicroseconds(10000);
	} while (wordswaiting < AUDIOBUFFSIZE);
	
	Serial.print(wordswaiting); Serial.println(" words remaining 2nd lap");
	addr = 0;
	for (uint16_t x=0; x < wordswaiting; x++)
	{
		// fill the buffer!
		uint16_t t = musicPlayer.recordedReadWord();
		audiobuffer[0][addr] = t >> 8;
		audiobuffer[0][addr+1] = t;
		
		addr += 2;
		
		if (addr >= AUDIOBUFFSIZE)
		{
			if (! audioFile.write(audiobuffer[0], AUDIOBUFFSIZE))
			{
				Serial.println("Couldn't write!");
				while (1);
			}
			//Serial.print(x+1); Serial.println(" words saved");
			addr = 0;
			written += AUDIOBUFFSIZE;
		}
	}
	
	if (addr != 0)
	{
		if (!audioFile.write(audiobuffer[0], addr)) {
			Serial.println("Couldn't write!"); while (1);
		}
		//Serial.print(addr); Serial.println(" bytes saved");
		written += addr;
	}
	
	wordswaiting = musicPlayer.recordedWordsWaiting();
	Serial.print(wordswaiting); Serial.println(" words still remaining");
	//Serial.print(written); Serial.println(" Bytes written");
	return written;
}

uint16_t getADCvalue(uint8_t mux_value)
{
	// Select analog input pin
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->INPUTCTRL.bit.MUXPOS = mux_value; // Selection for the positive ADC input
	
	// Clear the Data Ready flag
	ADC->INTFLAG.bit.RESRDY = 1;
	
	// Start conversion
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->SWTRIG.bit.START = 1;

	// Store the value
	while ( ADC->INTFLAG.bit.RESRDY == 0 );   // Waiting for conversion to complete
	return ADC->RESULT.reg;
}


void Timer_Sampling_Enable(void)
{
	NVIC_EnableIRQ(TC3_IRQn);
	
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void IMU_init(void)
{
	// Before initializing the IMU, there are a few settings
	// we may need to adjust. Use the settings struct to set
	// the device's communication mode and addresses:
	imu.settings.device.commInterface = IMU_MODE_I2C;
	imu.settings.device.mAddress = LSM9DS1_M;
	imu.settings.device.agAddress = LSM9DS1_AG;
	// The above lines will only take effect AFTER calling
	// imu.begin(), which verifies communication with the IMU
	// and turns it on.
	if (!imu.begin())
	{
		Serial.println("Failed to communicate with LSM9DS1.");
		while (1);
	}
	Serial.println("IMU initialized.");
}

void RTC_init(void)
{
		Get_Compiler_Date_Time();
		print_date_time();
		RTC_adjust();
}

void SD_Card_init(void)
{
	if (!SD.begin(pin_SD_CS))		// see if the card is present and can be initialized:
	{
		Serial.println("SD Card failed, or not present");
		while (1);
	}
	Serial.println("SD-Card initialized.");
}

			
void Files_Data_Audio_init(void)
{
	RTC_now();		// Get the current date into DateTime object
	
	char year_d = ((data.TIME.year - 2000) / 10) + '0';		// Get the tens of year
	char year_u = (data.TIME.year - 2000) % 10 + '0';		// Get the ones of year
	char month_d = data.TIME.month / 10 + '0';				// Get the tens of month
	char month_u = data.TIME.month % 10 + '0';				// Get the ones of month
	char day_d = data.TIME.day / 10 + '0';					// Get the tens of day
	char day_u = data.TIME.day % 10 + '0';					// Get the ones of day
	
	char files_directory [9] = {year_d, year_u, '-', month_d, month_u, '-', day_d, day_u, '\0'};
	char filename_data [11]  = {year_d, year_u, month_d, month_u, day_d, day_u, '.', 'C', 'S', 'V', '\0'};
	char filename_audio [13] = {year_d, year_u, month_d, month_u, day_d, day_u, '0', '1', '.', 'O', 'G', 'G', '\0'};
	char filename_data_path [22];
	char filename_audio_path [22];

	if(!SD.exists(files_directory))
	{
		Serial.println("Directory does not exist");
		SD.mkdir(files_directory);
	}
	else
	{
		Serial.println("Directory already exist");
	}
	
	sprintf(filename_data_path,  "%s/%s", files_directory, filename_data);		// Concatenate and add '/' character after folder name
	sprintf(filename_audio_path, "%s/%s", files_directory, filename_audio);		// Concatenate and add '/' character after folder name
	
	// Datafile routine
	if(!SD.exists(filename_data_path))
	{
		Serial.print("File for Data: ");
		Serial.println(filename_data);
		
		dataFile = SD.open(filename_data_path, FILE_WRITE);
		//dataFile = SD.open(filename_data, O_CREAT | O_WRITE);

		// if the file is available, write to it:
		if (dataFile) {
			//dataFile.println("Sample Data,,,2g,2g,2g,245dps,245dps,245dps,2gauss,2gauss,2gauss,,,,,");
			dataFile.print("Sample Data,,,");
			dataFile.print(imu.settings.accel.scale);
			dataFile.print("g,");
			dataFile.print(imu.settings.accel.scale);
			dataFile.print("g,");
			dataFile.print(imu.settings.accel.scale);
			dataFile.print("g,");
			dataFile.print(imu.settings.gyro.scale);
			dataFile.print("dps,");
			dataFile.print(imu.settings.gyro.scale);
			dataFile.print("dps,");
			dataFile.print(imu.settings.gyro.scale);
			dataFile.print("dps,");
			dataFile.print(imu.settings.mag.scale);
			dataFile.print("gauss,");
			dataFile.print(imu.settings.mag.scale);
			dataFile.print("gauss,");
			dataFile.print(imu.settings.mag.scale);
			dataFile.println("gauss,,,,,");
			dataFile.print("Device ID,"); dataFile.println(DEVICE_ID);
			
			dataFile.close();
			
			data.flag.date = 1;								// First line in SCV with date printed
		}
		
		else	// if the file isn't open, pop up an error:
		{
			Serial.print("Error opening ");
			Serial.println(filename_data);
			while (1);
		}
	}
	else
	{
		Serial.print("File ");
		Serial.print(filename_data);
		Serial.println(" already exists");
	}
	
	dataFile = SD.open(filename_data_path, FILE_WRITE);
	
	// Audiofile routine
	uint8_t index;
	for (index = 1; index < 100; index++)
	{
		filename_audio_path[15] = '0' + (uint8_t)(index / 10);
		filename_audio_path[16] = '0' + (index % 10);
		
		// create if does not exist, do not open existing
		if (! SD.exists(filename_audio_path))
		{
			Serial.print("File for Audio: ");
			Serial.println(filename_audio_path);
			
			new_audiofile = 1;											// used in sampling loop for print audiofile index
			audio_file_index = index;									// used in sampling loop for print audiofile index
			audioFile = SD.open(filename_audio_path, FILE_WRITE);
			//dataFile = SD.open(filename_data, O_CREAT | O_WRITE);
			if (! audioFile)
			{
				Serial.print("Error opening ");
				Serial.println(filename_audio_path);
				while (1);
			}
			break;
		}
	}
	if (index == 10)
	{
		Serial.println("Audio File not created. Index exceed number 99");
		state_machine_transition = S4;
	}
}


void ADC_init(void)
{
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_ADC ) ;	// Enable clock for ADC
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

	PORT->Group[0].PINCFG[7].bit.PMUXEN = 0x1;	// PA07 -> Batt Voltage
	PORT->Group[0].PMUX[3].bit.PMUXO = 0x1;//PORT_PMUX_PMUX0_B
	
	PORT->Group[0].PINCFG[2].bit.PMUXEN = 0x1;	// PA02 -> EDA 1
	PORT->Group[0].PMUX[1].bit.PMUXE = 0x1;//PORT_PMUX_PMUXE_B
	
	PORT->Group[0].PINCFG[4].bit.PMUXEN = 0x1;	// PA04 -> EDA 2
	PORT->Group[0].PMUX[2].bit.PMUXE = 0x1;//PORT_PMUX_PMUXE_B
	
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->INPUTCTRL.bit.MUXPOS = 0x7; // Selection for the positive ADC input

	/*	ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT;             // Enable ADC
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	*/

	ADC->CTRLA.bit.ENABLE = 0x01;           // Enable ADC
	while (ADC->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	// Start conversion
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->SWTRIG.bit.START = 1;

	// Clear the Data Ready flag
	ADC->INTFLAG.bit.RESRDY = 1;

	//The first conversion after the reference is changed must not be used.
	Serial.println("ADC initialized.");
}

void Timer_Sampling_init(void)
{
	// Enable clock for TC
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

	// The type cast must fit with the selected timer mode
	TcCount16* TC = (TcCount16*) TC3; // get timer struct

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;		// Disable TCCx
	while (TC->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16; // Set Timer counter Mode to 16 bits
	while (TC->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
	while (TC->STATUS.bit.SYNCBUSY == 1);	// wait for sync

	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;	// Set perscaler
	while (TC->STATUS.bit.SYNCBUSY == 1);			// wait for sync

	TC->CC[0].reg = 1560;//1870;//935;//1560;//3124;//4687;//46874;		// 1560 for 30Hz
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	// Interrupts
	TC->INTENSET.reg = 0;              // disable all interrupts
	TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0

	// Enable InterruptVector
	NVIC_EnableIRQ(TC3_IRQn);
	
	Serial.println("Timer3 initialized.");
}

void Audio_Codec_init(void)
{
	if (!musicPlayer.begin())	// initialize the music player
	{
		Serial.println("VS1053 not found");
		while (1);
	}
	
	// load plugin from SD card! We'll use mono 16.1KHz, medium quality
	if (! musicPlayer.prepareRecordOgg("v16k1q05.img"))
	{
		Serial.println("Couldn't load plugin!");
		while (1);
	}
	
	Serial.println("Audio Codec initialized.");
}


void Button_Calculate_Pressed_Time(void)
{
	TcCount16* TIMER_BUTTON = (TcCount16*) TC5; // get timer struct
	state_button = digitalRead(pin_Button);
	
	if (state_button != state_button_previous)
	{
		if (state_button == PRESSED)
		{
			Serial.println("ON");			
			waiting_off_state = 1;
		}
		else
		{
			Serial.println("OFF");
			Serial.print("\t");
			
			waiting_off_state = 0;
			
			counter_button_time = TIMER_BUTTON->COUNT.reg;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
			
			Timer_Button_Stop();
			
			TIMER_BUTTON->COUNT.reg = 0;
			while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
			
			if (counter_button_time <= THRESHOLD_TIME_1)
			{
				Serial.print("S2 - Click simple\t");
				state_machine_transition = S2;
			}
			else if (counter_button_time > THRESHOLD_TIME_1 && counter_button_time <= THRESHOLD_TIME_2)
			{
				Serial.print("S1 - 1s\t");
				state_machine_transition = S3;
			}
			else if (counter_button_time > THRESHOLD_TIME_2 && counter_button_time <= THRESHOLD_TIME_3)
			{
				Serial.print("Out of threshold limits");
				state_machine_transition = S0;	// Do nothing
			}
			else  //(counter_button_time > THRESHOLD_TIME_3)
			{
				Serial.print("Reset\t");
				state_machine_transition = S4;
			}

			Serial.print(counter_button_time);
			Serial.println(" ms\n");
		}
	}
	else
	{
		Serial.print("\nERROR!\t\t\t");
		Serial.print(state_button);
		Serial.print("\t");
		Serial.print(state_button_previous);
		Serial.println("\n");
		
		waiting_off_state = 0;
		Timer_Button_Stop();
		
		TIMER_BUTTON->COUNT.reg = 0;
		while (TIMER_BUTTON->STATUS.bit.SYNCBUSY == 1); // wait for sync
	}

	state_button_previous = state_button;
	is_button_changed = false;
	//		EIC->INTENSET.bit.EXTINT0 = 1;			// Reactivate Button interrupt
}

void Save_Data_to_SD(void)
{
	//digitalWrite(pin_led2, HIGH);  // for debug led2
	NVIC_DisableIRQ(TC3_IRQn);
	
	// Save data to SD Card
	if (who_save)	// Save Codec Data
	{
		if (audioposarray == 1)
		{
			audioFile.write(audiobuffer[0], AUDIOBUFFSIZE);
			audioposarray = 0;
		}
		else if (audioposarray == 2)
		{
			audioFile.write(audiobuffer[0], AUDIOBUFFSIZE);
			audioFile.write(audiobuffer[1], AUDIOBUFFSIZE);
			audioposarray = 0;
		}
		else if (audioposarray > 2)
		{
			Serial.println("ERROR_Codec_>2");
			audioposarray = 0;
		}

	}
	else		// Save Sampled Data
	{
		if (dataposarray == 1)
		{
			dataFile.write(databuffer[0],count_char[0]);
			dataposarray = 0;
			count_char[0] = 0;
		}
		else if (dataposarray == 2)
		{
			dataFile.write(databuffer[0],count_char[0]);
			dataFile.write(databuffer[1],count_char[1]);
			dataposarray = 0;
			count_char[0] = 0;
			count_char[1] = 0;
		}
		else if (dataposarray > 2)
		{
			Serial.println("ERROR_Sensors_>2");
		}

	}
	
	//digitalWrite(pin_led2, LOW);  // for debug led2
	datatosd = 0;
	NVIC_EnableIRQ(TC3_IRQn);
}

void Set_Private_End_Time(uint8_t minute)
{
	private_minute = (data.TIME.minute + minute) % 60;
	private_second = data.TIME.second;
//	Serial.print("private_time ");
//	Serial.print(private_minute);
//	Serial.print(":");
//	Serial.println(private_second);
}

void Get_Date_and_Time(void)
{	
	RTC_now();							// Get the current date into DateTime object
	
	syncWithRTC = 0;					// For synchronization in the sampling loop
	
	data.TIME_ANT.day	 = data.TIME.day;			
	data.TIME_ANT.month  = data.TIME.month;
	data.TIME_ANT.year	 = data.TIME.year;
	data.TIME_ANT.hour	 = data.TIME.hour;			
	data.TIME_ANT.minute = data.TIME.minute;
	data.TIME_ANT.second = data.TIME.second;
}


void print_date_time(void)
{
	Serial.print(data.TIME.day);
	Serial.print("/");
	Serial.print(data.TIME.month);
	Serial.print("/");
	Serial.print(data.TIME.year);
	Serial.print("\t");
	Serial.print(data.TIME.hour);
	Serial.print(":");
	Serial.print(data.TIME.minute);
	Serial.print(":");
	Serial.println(data.TIME.second);
}


void Get_Compiler_Date_Time(void)
{
	char *time_string = (char*)F(__TIME__);
	char *date_string = (char*)F(__DATE__);
	
	data.TIME.hour = (time_string[0] - '0') * 10 + (time_string[1] - '0');
	data.TIME.minute = (time_string[3] - '0') * 10 + (time_string[4] - '0');
	data.TIME.second = (time_string[6] - '0') * 10 + (time_string[7] - '0');

	data.TIME.year = 2000 + ((date_string[9] - '0') * 10 + (date_string[10] - '0'));
	if(date_string[4] == ' ')
	{
		data.TIME.day = date_string[5] - '0';
	}
	else
	{
		data.TIME.day = (date_string[4] - '0') * 10 + (date_string[5] - '0');
	}
	
	if(strncmp(date_string,"Jan",3) == 0)
	{
		data.TIME.month = 1;
	}
	else if(strncmp(date_string,"Feb",3) == 0)
	{
		data.TIME.month = 2;
	}
	else if(strncmp(date_string,"Mar",3) == 0)
	{
		data.TIME.month = 3;
	}
	else if(strncmp(date_string,"Apr",3) == 0)
	{
		data.TIME.month = 4;
	}
	else if(strncmp(date_string,"May",3) == 0)
	{
		data.TIME.month = 5;
	}
	else if(strncmp(date_string,"Jun",3) == 0)
	{
		data.TIME.month = 6;
	}
	else if(strncmp(date_string,"Jul",3) == 0)
	{
		data.TIME.month = 7;
	}
	else if(strncmp(date_string,"Aug",3) == 0)
	{
		data.TIME.month = 8;
	}
	else if(strncmp(date_string,"Sep",3) == 0)
	{
		data.TIME.month = 9;
	}
	else if(strncmp(date_string,"Oct",3) == 0)
	{
		data.TIME.month = 10;
	}
	else if(strncmp(date_string,"Nov",3) == 0)
	{
		data.TIME.month = 11;
	}
	else if(strncmp(date_string,"Dec",3) == 0)
	{
		data.TIME.month = 12;
	}
}

uint8_t bcd2bin (uint8_t val)
{
	return val - 6 * (val >> 4);
}

uint8_t bin2bcd (uint8_t val)
{
	return val + 6 * (val / 10);
}

uint8_t Read_I2C_Register(uint8_t addr, uint8_t reg)
{
	Wire.beginTransmission(addr);
	Wire.write((byte)reg);
	Wire.endTransmission();

	Wire.requestFrom(addr, (byte)1);
	return Wire.read();
}

void Write_I2C_Register(uint8_t addr, uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(addr);
	Wire.write((byte)reg);
	Wire.write((byte)val);
	Wire.endTransmission();
}

bool RTC_lostPower(void)
{
	return (Read_I2C_Register(DS3231_ADDRESS, DS3231_STATUSREG) >> 7);
}

void RTC_adjust(void)
{
	Wire.beginTransmission(DS3231_ADDRESS);
	Wire.write((byte)0); // start at location 0
	Wire.write(bin2bcd(data.TIME.second));
	Wire.write(bin2bcd(data.TIME.minute));
	Wire.write(bin2bcd(data.TIME.hour));
	Wire.write(bin2bcd(0));
	Wire.write(bin2bcd(data.TIME.day));
	Wire.write(bin2bcd(data.TIME.month));
	Wire.write(bin2bcd(data.TIME.year - 2000));
	Wire.endTransmission();

	uint8_t statreg = Read_I2C_Register(DS3231_ADDRESS, DS3231_STATUSREG);
	statreg &= ~0x80; // flip OSF bit
	Write_I2C_Register(DS3231_ADDRESS, DS3231_STATUSREG, statreg);
}

void RTC_now(void)
{
	Wire.beginTransmission(DS3231_ADDRESS);
	Wire.write((byte)0);
	Wire.endTransmission();

	Wire.requestFrom(DS3231_ADDRESS, 7);
	data.TIME.second = bcd2bin(Wire.read() & 0x7F);
	data.TIME.minute = bcd2bin(Wire.read());
	data.TIME.hour = bcd2bin(Wire.read());
	Wire.read();
	data.TIME.day = bcd2bin(Wire.read());
	data.TIME.month = bcd2bin(Wire.read());
	data.TIME.year = bcd2bin(Wire.read()) + 2000;
}


void RTC_Set_Alarm_Every_X_Minutes(uint8_t minutes)
{
	RTC_now();
	
	Wire.beginTransmission(DS3231_ADDRESS);
	Wire.write(DS3231_ALARM1);									// start at location Alarm1
	Wire.write((bin2bcd(data.TIME.second))&0x7F);				// A1M1 set to 0
	Wire.write((bin2bcd((data.TIME.minute+minutes)%60))&0x7F);	// A1M2 set to 0
	Wire.write(0x80);											// A1M3 set to 1
	Wire.write(0x80);											// A1M4 set to 1
	Wire.endTransmission();
	
	RTC_Clear_Interrupt_Flags();
	
	uint8_t ctrl;
	ctrl = Read_I2C_Register(DS3231_ADDRESS, DS3231_CONTROL);
	ctrl |= DS3231_CONTROL_INTCN;       // set INTCN bit to 1
	ctrl |= DS3231_CONTROL_A1IE;        // set A1IE bit to 1
	ctrl &= ~DS3231_CONTROL_A2IE;       // set A2IE bit to 0
	Write_I2C_Register(DS3231_ADDRESS, DS3231_CONTROL, ctrl);
}


void RTC_Clear_Interrupt_Flags (void)
{
	uint8_t ctrl;
	ctrl = Read_I2C_Register((uint8_t)DS3231_ADDRESS, (uint8_t)DS3231_STATUSREG);
	ctrl &= ~0x03; // set interrupt flags bits to 0
	Write_I2C_Register(DS3231_ADDRESS, DS3231_STATUSREG, ctrl);
}

void RTC_Disable_Alarm(void)
{
	uint8_t ctrl;
	ctrl = Read_I2C_Register(DS3231_ADDRESS, DS3231_CONTROL);
	ctrl |= DS3231_CONTROL_INTCN;       // set INTCN bit to 1
	ctrl &= ~DS3231_CONTROL_A1IE;        // set A1IE bit to 0
	ctrl &= ~DS3231_CONTROL_A2IE;       // set A2IE bit to 0
	Write_I2C_Register(DS3231_ADDRESS, DS3231_CONTROL, ctrl);
}