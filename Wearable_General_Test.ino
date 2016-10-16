#include <Arduino.h>
#include <SparkFunLSM9DS1.h>
#include "RTClib.h"
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "FeatherM0_RTC.h"
#include "Wearable_General_Test.h"
#include <Adafruit_VS1053.h>
#include <avr/dtostrf.h>		// For dtostrf function


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
	bool audio;
};

struct datastruct{
	dataRTC TIME;
	dataRTC TIME_ANT;
	dataIMU IMU;
	float TEMP_INT;
	float TEMP_EXT;
	float battery;
	float eda;
	boolean privacy;
	dataflag flag;
} volatile data;


RTC_DS3231 rtc;

//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define TMP102_I2C_ADDRESS0 0b1001000  // TMP102 I2C address if A0PIN tied to ground.
#define TMP102_I2C_ADDRESS1 0b1001001  // TMP102 I2C address if A0PIN tied to VCC.
//#define TMP102_I2C_ADDRESS 0b1001010  // TMP102 I2C address if A0PIN tied to SDA.
//#define TMP102_I2C_ADDRESS 0b1001011  // TMP102 I2C address if A0PIN tied to SCL.

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

#define Serial  SerialUSB

float getTemp102(bool selector);
uint16_t saveRecordedData(boolean isrecord);
uint16_t getADCvalue(uint8_t mux_value);

const int pin_led = 13;  // debug pin for compare led
const int pin_led2 = 12;  // debug pin for compare led
const int pinButton = 1;
const int chipSelect = 4;

// define the pins used
#define RESET A3      // VS1053 reset pin (output)
#define CS A4        // VS1053 chip select pin (output)
#define DCS A5        // VS1053 Data/command select pin (output)
#define CARDCS 4     // Card chip select pin
#define DREQ A2       // VS1053 Data request, ideally an Interrupt pin

volatile uint16_t  counter_samples = 0;

#define BATTERY_ADC_MUX 0x7
#define EDA_ADC_MUX		0x0
uint16_t ADCvalueRead;
float battery_voltage;
volatile boolean syncWithRTC = 0;

char filename_data[12];
char filename_audio[12];
char filename_directory[6];

File dataFile;  // the file we will save our sampled data to
File audioFile;  // the file we will save our recording to

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(RESET, CS, DCS, DREQ, CARDCS);

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

void setup() {
	pinMode(pin_led, OUTPUT);   // for debug led
	pinMode(pin_led2, OUTPUT);   // for debug led2
	
	digitalWrite(pin_led, HIGH); // for debug led
	
	pinMode(pinButton, INPUT_PULLDOWN);   // Button pin with pull-down resistor
	
	Serial.begin(115200);
	Serial.println("I-ME datalogger v1.0");
	
	Wire.begin();				// start the I2C library
	Wire.setClock(800000L);		// Run at 400kHz (check the number difference!)

	//-----------------------------------------------------------------------------------------------
	// IMU Initialization
	
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

	// End IMU Initialization
	//-----------------------------------------------------------------------------------------------
	
	//-----------------------------------------------------------------------------------------------
	// RTC Initialization
	if (!rtc.begin()) 
	{
		Serial.println("Failed to communicate with RTC");
		while (1);
	}
	Serial.println("RTC initialized.");
	
	if (rtc.lostPower()) 
	{
		Serial.println("RTC lost power, lets set the time!");
		// following line sets the RTC to the date & time this sketch was compiled
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		// rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	}
	
	DateTime now = rtc.now();
	
	data.TIME.second = now.second();
	data.TIME.minute = now.minute();
	data.TIME.hour = now.hour();
	data.TIME.day = now.day();
	data.TIME.month = now.month();
	data.TIME.year = now.year();
	
	// End RTC Initialization
	//-----------------------------------------------------------------------------------------------
	
	//-----------------------------------------------------------------------------------------------
	// SD-Card Initialization
	if (!SD.begin(chipSelect))		// see if the card is present and can be initialized:
	{
		Serial.println("SD Card failed, or not present");
		while (1);
	}
	Serial.println("SD-Card initialized.");
	
	// End SD-Card Initialization
	//-----------------------------------------------------------------------------------------------
	
	//-----------------------------------------------------------------------------------------------
	// Data file Initialization
	filename_data[0] = ((data.TIME.year - 2000) / 10) + '0';
	filename_data[1] = (data.TIME.year - 2000) % 10 + '0';
	filename_data[2] = data.TIME.month / 10 + '0';
	filename_data[3] = data.TIME.month % 10 + '0';
	filename_data[4] = data.TIME.day / 10 + '0';
	filename_data[5] = data.TIME.day % 10 + '0';
	filename_data[6] = '_';
	filename_data[7] = 'D';
	filename_data[8] = '.';
	filename_data[9] = 'C';
	filename_data[10] = 'S';
	filename_data[11] = 'V';
	filename_data[12] = '\0';
	
	if(!SD.exists(filename_data))
	{
		Serial.print("File for Data: ");
		Serial.println(filename_data);
		
		dataFile = SD.open(filename_data, FILE_WRITE);
		//dataFile = SD.open(filename_data, O_CREAT | O_WRITE);

		// if the file is available, write to it:
		if (dataFile) {
			dataFile.println("Sensor Name,Timestamp,,Accelerometer,,,Gyroscope,,,Magnetometer,,,EDA,Temperature,,Battery,Privacy");
			dataFile.println("Variable Name,TIME_DATE,TIME_HOURS,ACCL_X,ACCL_Y,ACCL_Z,GYRO_X,GYRO_Y,GYRO_Z,MAG_X,MAG_Y,MAG_Z,EDA_V,TEMP_IN,TEMP_EXT,BATT_V,P_ON/P_OFF");
			dataFile.println("Sample Rate(Hz),--,1,,,30,,,30,,,30,3,,0.1,0.0166667,--");
			dataFile.println("Variable Type,String,String,int,int,int,int,int,int,int,int,int,int,float,float,float,flag");
			dataFile.println("Variable Range/Format,YYYY-MM-DD,HH:MM:SS,0-8192,0-8192,0-8192,0-8192,0-8192,0-8192,0-8192,0-8192,0-8192,0-1024,0-50,0-50,0-10,P_ON/P_OFF");
			dataFile.println("Sample Data,,,2g,2g,2g,245dps,245dps,245dps,2gauss,2gauss,2gauss,,,,,");
			/*  dataFile.print("Sample Data,,,");
			dataFile.print(ACC_RANGE_X);
			dataFile.print("g,");
			dataFile.print(ACC_RANGE_Y);
			dataFile.print("g,");
			dataFile.print(ACC_RANGE_Y);
			dataFile.print("g,")
			dataFile.print(GYRO_RANGE_X);
			dataFile.print("dps,");
			dataFile.print(GYRO_RANGE_X);
			dataFile.print("dps,");
			dataFile.print(GYRO_RANGE_X);
			dataFile.print("dps,");
			dataFile.print(MAG_RANGE_X);
			dataFile.print("gauss,");
			dataFile.print(MAG_RANGE_X);
			dataFile.print("gauss,");
			dataFile.print(MAG_RANGE_X);
			dataFile.print("gauss,,,,,");
			dataFile.print("Device ID"); dataFile.println(DEVICE_ID);*/
			dataFile.print("Device ID"); dataFile.println(1);
			
			dataFile.close();
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
		
		data.TIME_ANT.day = data.TIME.day;			// To Avoid write date again in the file
		data.TIME_ANT.month = data.TIME.month;
		data.TIME_ANT.year = data.TIME.year;
	}
	//-----------------------------------------------------------------------------------------------
	// End Data file Initialization

	//-----------------------------------------------------------------------------------------------
	// ADC Initialization
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_ADC ) ;	// Enable clock for ADC
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

	PORT->Group[0].PINCFG[7].bit.PMUXEN = 0x1;
	PORT->Group[0].PMUX[3].bit.PMUXO = 0x1;
	
	PORT->Group[0].PINCFG[0].bit.PMUXEN = 0x1;
	PORT->Group[0].PMUX[0].bit.PMUXE = 0x1;
	
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->INPUTCTRL.bit.MUXPOS = 0x7; // Selection for the positive ADC input

	/*
	while (ADC->STATUS.bit.SYNCBUSY == 1); // wait for sync
	ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT;             // Enable ADC
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
	
	// End ADC Initialization
	//-----------------------------------------------------------------------------------------------


	//-----------------------------------------------------------------------------------------------
	// Timer 3 Initialization

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
	// End Timer 3 Initialization
	//-----------------------------------------------------------------------------------------------
	
	
	//-----------------------------------------------------------------------------------------------
	// Timer 4 Initialization

	// Enable clock for TC
	//	REG_GCLK_CLKCTRL |= (uint16_t) (GCLK_CLKCTRL_ID_TC4_TC5) ;
	//	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync
	/*
	// The type cast must fit with the selected timer mode
	TcCount16* TCButton = (TcCount16*) TC5; // get timer struct

	TCButton->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCCx
	while (TCButton->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TCButton->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
	while (TCButton->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TCButton->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
	while (TCButton->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TCButton->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;   // Set perscaler
	while (TCButton->STATUS.bit.SYNCBUSY == 1); // wait for sync

	// TC->PER.reg = 0xFF;   // Set counter Top using the PER register but the 16/32 bit timer counts allway to max
	// while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	TCButton->CC[0].reg = 46874;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

	// Interrupts
	TCButton->INTENSET.reg = 0;              // disable all interrupts
	TCButton->INTENSET.bit.OVF = 0;          // disable overfollow
	TCButton->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
	
	TCButton->COUNT.reg = 0;					// Counter set to zero
	*/
	// End Timer 4 Initialization
	//-----------------------------------------------------------------------------------------------

	//-----------------------------------------------------------------------------------------------
	// Audio Codec Initialization
	if (!musicPlayer.begin())	// initialise the music player
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
	
	musicPlayer.startRecordOgg(true); // use microphone (for linein, pass in 'false')
	
	Serial.println("Audio Codec initialized.");
	// End Audio Codec Initialization
	//-----------------------------------------------------------------------------------------------

	//-----------------------------------------------------------------------------------------------
	// Audio File Initialization

	// Check if the file exists already
	strcpy(filename_audio, "REC00.OGG");
	for (uint8_t i = 0; i < 100; i++)
	{
		filename_audio[3] = '0' + i/10;
		filename_audio[4] = '0' + i%10;
		// create if does not exist, do not open existing
		if (! SD.exists(filename_audio))
		{
			break;
		}
	}
	
	audioFile = SD.open(filename_audio, FILE_WRITE);
	//audioFile = SD.open(filename, O_CREAT | O_WRITE);
	if (! audioFile)
	{
		Serial.print("Error opening ");
		Serial.println(filename_audio);
		while (1);
	}
	Serial.print("File for Audio: ");
	Serial.println(filename_audio);
	
	// End Audio File Initialization
	//-----------------------------------------------------------------------------------------------

	
	data.flag.time = 1;	 // For first date and time data

	dataFile = SD.open(filename_data, FILE_WRITE);
	//dataFile = SD.open(filename_data, O_CREAT | O_WRITE);
	
	attachInterrupt(pinButton, ISR_Button, RISING);
	
	digitalWrite(pin_led,LOW);
	delay(200);
	digitalWrite(pin_led,HIGH);
	delay(200);
	digitalWrite(pin_led,LOW);
	delay(200);
	digitalWrite(pin_led,HIGH);
	delay(200);
	digitalWrite(pin_led,LOW);
	delay(200);
	digitalWrite(pin_led,HIGH);
	delay(200);
	digitalWrite(pin_led,LOW);
	
	now = rtc.now();
	data.TIME.second = now.second();
	data.TIME_ANT.second = data.TIME.second;
	
	// Enable TC
	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync	
	
	Serial.println("End Setup\n");
}


void loop()
{
	if (datatosd)
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
}


void ISR_Button()
{
	TcCount16* TC = (TcCount16*) TC3; // get timer struct
	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
	NVIC_DisableIRQ(TC3_IRQn);
	
	dataFile.close();
	
	musicPlayer.stopRecordOgg();
	saveRecordedData(false);
	Serial.println("End recording");
	// close it up
	audioFile.close();
	Serial.println("Program Finished");
	
	//, Timer4 Activated");
	/*	// Enable TC4
	TcCount16* TCButton = (TcCount16*) TC4; // get timer struct
	TCButton->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TCButton->STATUS.bit.SYNCBUSY == 1); // wait for sync
	Serial.print("Counter : ");
	Serial.println(TCButton->COUNT.reg);
	Serial.println("");
	*/
	
	// Low power mode
}


void TC3_Handler()
{
	TcCount16* TC = (TcCount16*) TC3; // get timer struct

	if (TC->INTFLAG.bit.MC0 == 1)
	{  // A compare to cc0 caused the interrupt
		
		if (!syncWithRTC)
		{
			DateTime now = rtc.now();
			data.TIME.second = now.second();
			if (data.TIME.second != data.TIME_ANT.second)
			{
				syncWithRTC = 1;
			}
			else
			{
				return;
			}
		}
		
		digitalWrite(pin_led, HIGH);  // for debug led
		
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
		DateTime now = rtc.now();		// Get RTC data structure
		
		// Copy RTC data to main structure
		data.TIME.second = now.second();
		data.TIME.minute = now.minute();
		data.TIME.hour = now.hour();
		data.TIME.day = now.day();
		data.TIME.month = now.month();
		data.TIME.year = now.year();
		
		if (data.TIME.second != data.TIME_ANT.second)		// Update previous time values
		{
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
			data.eda = getADCvalue(EDA_ADC_MUX) * 0.00322581;
			data.flag.eda = 1;
		}
		
		//........................................................................ TEMP
		if ((counter_samples%300) == 0)	// temp sample
		{
			data.TEMP_INT = getTemp102(0);
			data.TEMP_EXT = getTemp102(1);
			data.flag.temp = 1;
		}
		
		//........................................................................ BATT
		if ((counter_samples%1800) == 0)	// battery sample
		{	
			data.battery = getADCvalue(BATTERY_ADC_MUX) * 2 * 0.00322581;
			data.flag.battery = 1;			
			counter_samples = 0;
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
		uint8_t count_temp = 0;
		
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
			count_temp = sprintf(databuffer[dataposarray] + count_char[dataposarray], "%u,", data.flag.time);
			count_char[dataposarray] += count_temp;

			data.flag.time = 0;
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
		digitalWrite(pin_led, LOW);  // for debug led
		
		datatosd = 1;
		
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


uint16_t saveRecordedData(boolean isrecord) {
	uint16_t written = 0;
	
	if (!isrecord) {
		uint16_t wordswaiting = musicPlayer.recordedWordsWaiting();
		//Serial.print(wordswaiting); Serial.println(" remaining");
		// wrapping up the recording!
		uint16_t addr = 0;
		for (int x=0; x < wordswaiting-1; x++) {
			// fill the buffer!
			uint16_t t = musicPlayer.recordedReadWord();
			audiobuffer[0][addr] = t >> 8;
			audiobuffer[0][addr+1] = t;
			if (addr > AUDIOBUFFSIZE) {
				if (! audioFile.write(audiobuffer[0], AUDIOBUFFSIZE)) {
					Serial.println("Couldn't write!");
					while (1);
				}
				audioFile.flush();
				addr = 0;
			}
		}
		if (addr != 0) {
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