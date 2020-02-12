// The pin definitions are as per obfuscated Arduino pin defines -- see aka for ATMEL pin names as found on the MEGA328P spec sheet
#define Enc_A_PIN 2     // This generates the interrupt, providing the click  aka PD2 (Int0)
#define Enc_B_PIN 14    // This is providing the direction aka PC0,A0
#define Enc_PRESS_PIN 3 // aka PD3 ((Int1)
#define LED1_PIN 10     // aka PB2,SS Wired to RED LED, Also used as slave select pin for the SD card 
//#define LED2_PIN    todo -- move to txo

// A0 used for rotary knob -- unavailable for analog input 	
// A1 used for NTC temp sensors
// A2 used for NTC temp sensors
// A3 used for analog RPM output from Lightspeed ignition -- Need to disconnect R3 or reroute LED2 to PD1 ( TXO ) serial debug
// A4 used for analog MP output from Lightspeed ignition -- Need to remove R7, conflict with I2C therefore I2C is not available	
// A5, aka SCL, aka SS used as SS on SD card -- conflict with I2C, I2C not available
// ADC6 used for NTC temp sensors
// ADC7 used for VBUS voltage reading


// ADC0 used for rotary knob 
#define NTC1_ADC    1 // A1_PC1
#define NTC2_ADC    2
#define RPM_ADC 	3
#define FP_ADC 		4
// ADC 5 used for Slave Select on SD card SPI interface
#define SD_SS_PIN 19	// aka A5
#define NTC3_ADC    6
#define VBUS_ADC 	7      // ADC7
// LiquidCrystal lcd(9, 8, 6, 7, 4, 5); // in 4 bit interface mode

