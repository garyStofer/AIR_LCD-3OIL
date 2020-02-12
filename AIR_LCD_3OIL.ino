
/*
	Multi function display device used in experimental aircraft to measure, display and alarm for situations when readings exceed built in limits.

Primary funtion: 
	A.)Measure oil temperature with 3 NTC resistors and display engine oil outlet and inlet temps and display the delta temp acrosss
	the oil cooler. Indicate alarm situation if oil inlet temp comes close to reaching redline.
	B.) Measure Battery voltage and indicate alarm when  low or high limits are exceeded.
	C.) Measure Fuel pressure and indicate alarm when low or high limits are exceeded.
Secondary function:
	Record the above readings every n seconds on a SD card.
	Measure, display and record engine RPM as provided by Lightning ignition module (analog signal).
	Detect end of flight by checking RPM and record the duration of the flight. 

*/
// See file build_opt.h for #defines to enable individual measurments and their display


// Note : This sketch needs to be compiled and run on a device with Optiboot since the watchdog doesn't work under the standard Bootloader.
// NOTE : Before uploading sketch make sure that the Board type is set to OPTIBOOT on a 32 Pin CPU -- otherwise the programmer runs at the wronng baud rate

// Last compiled and tested with Arduino IDE 1.6.6

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  BUILD OPTIONS  see file build_opt.h   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include "build_opts.h"		// Controls build time features

#include <LiquidCrystal.h>
#include <EEPROM.h>
#ifdef WITH_SD_CARD
#include <SD.h>
#define SD_RECORD_PER 30		// every 30 seconds record a sample in the SD 	
#endif
#include <avr/wdt.h>			// Device must have OPTIBOOT bootloader, otherwise watchdog is not working
#include "NTC.h"
#include "Encoder.h"
// LCD defines 
#define LCD_COLS 8
#define LCD_ROWS 2
#define LINE1 0
#define LINE2 1
// lcd special characters
static byte DeltaChar[8] = 
  {
    0B00000,
    0B00000,
    0B00100,
    0B01010,
    0B10001,
    0B11111,
    0B00000,
    0B00000
  };
/*
byte DeltaChar2[8] = {
  
  0B10000,
  0B11000,
  0B10100,
  0B10010,
  0B10100,
  0B11000,
  0B10000,
  0B00000
};
*/
static byte UpChar[8] = {
  0B00100,
  0B01110,
  0B11111,
  0B00100,
  0B00100,
  0B00100,
  0B00100,
  0B00000
};

static byte DownChar[8] = {
  0B00100,
  0B00100,
  0B00100,
  0B00100,
  0B11111,
  0B01110,
  0B00100,
  0B00000
};
//#define ADC_REF_1 (5.0)
#define ADC_REF_1 (4.95)  // measured on individual baord -- This is the 5V regulator
#define ADC_BW_1 (ADC_REF_1 / 1024 )
#define VBUS_ADC_BW  (ADC_REF_1 *(14+6.8)/(1024*6.8))    //adc bit weight for voltage divider 14.0K and 6.8k to gnd using 5V reference 

#define ADC_REF_2 (1.1)
#define ADC_BW_2  (ADC_REF_2/ 1024)

// There seems to be an error in the Lightspeed ignition analog outputs of about 5% short
#define LGHTSPD_err_rpm  (1.05)
//#define LGHTSPD_err_mp  (1.05)

// Fuel pressure signal comes from resistive sensor connected to the analog guage (Van's )
// voltages are as measured on actual sending unit. 
// Calibrated with external instrument and curve fitted  via EXCEL  see  file .xls
#define FuelPressureV_0 	(0.702)	// offset
#define FuelPressureSlope   (-44.795)


#define FuelP_HIGH_Alarm (8.0)
#define FuelP_LOW_Alarm	(3.0)

#define UPDATE_PER 500   // in ms second, this is the measure interval and also the LCD update interval

// units for display
#define FAR "F" 
#define CEL "C"
#define DegSym (char(223))  // degree symbol display character for LCD
#define DeltaSym ( char(7))
#define UpSym ( char(6))
#define DownSym ( char(5))

#define TIMEOUT_TIME 	10  // how long in seconds the non primary dislpay stays before reverting back to Oil temp (primary) 
#define VOLT_HIGH_ALARM (14.8)
#define VOLT_LOW_ALARM 	(11.0)
#define OIL_TEMP_ALARM 230    // when the red light comes on, in deg F
#define TrendTime 40          // trend window in seconds
#define TrendHysteresis 4     // in 1/10 deg C makes a hysteresis for the temp trend check, applies + and - , i.e 10 is +- 1 degree 

enum Displays {
  DISPLAY_BEGIN =0,
  MENU_OIL_NTC,
  MENU_TIME,
  MENU_V_Bus,
  MENU_RPM,
  MENU_FP,
  DISP_END        // this must be the last entry
};

bool MetricDisplay = false;

#define AVG_WIDTH 2	// power of 2s, i.e. 3 bit wide is 8 locations , at 05s per update == 4 Seconds
#define AVG_SAMPLE (1 << AVG_WIDTH ) 
struct avg_ndx{
unsigned int ndx :AVG_WIDTH ;
} avg;

// Global LCD control class
LiquidCrystal lcd(9, 8, 6, 7, 4, 5); // in 4 bit interface mode

#ifdef WITH_SD_CARD
// SD file for datalogging 
File dataFile;
#endif


/* Note regarding contrast control */
// Contrast control simply using a PWM doesn't work because the PWM needs to be filtered with a R-C, but the LCD itself is pulling the LCD control
// input up, so the R-C from the MCU would have to be relatively low in impendance <300ohms, making for a big capacitor and lots of current
// Would have to feed the filtered voltage into an OP-Amp to drive the LCD input properly.
// For now the hardware has a resistor pair that sets the contrast.

 

// The Arduino IDE Setup function -- called once upon reset
void setup()
{
  unsigned err;

  // The two LEDs
  // <<< Not connected -- pin used for analog input 
  //  pinMode(LED2_PIN, OUTPUT);    // BLUE
  //  digitalWrite( LED2_PIN, LOW); // 

  pinMode(LED1_PIN, OUTPUT);    // RED
  digitalWrite( LED1_PIN, LOW);

  EncoderInit( Enc_A_PIN, Enc_B_PIN, Enc_PRESS_PIN );

  wdt_enable(WDTO_8S);
  
	 
#ifdef DEBUG
   Serial.begin(115200);
   Serial.print("AIR-LCDuino OIL temp Display\n");
#endif   
   
  lcd.begin(LCD_COLS, LCD_ROWS);              // initialize the LCD columns and rows
  lcd.createChar(DeltaSym, DeltaChar);  
  lcd.createChar(UpSym, UpChar);  
  lcd.createChar(DownSym, DownChar); 

  lcd.home ();                   // cursor 0,0
  lcd.print("OIL TEMP");
  lcd.setCursor ( 0, 1 );
  lcd.print(" V 2.1  ");


#ifdef WITH_SD_CARD
	 
  pinMode(SD_SS_PIN, OUTPUT);  // SCL used as SS pin on SD card -- I2c and SD card can not operate at the same time
  digitalWrite( SD_SS_PIN, LOW);

  if (  SD.begin(SD_SS_PIN) ) // the Slave Select pin is shared with SCL from I2C, therefore SD and I2C are mutually exclusive
  {
	dataFile = SD.open("N427GS.txt", FILE_WRITE);
    
    if (dataFile) 
    {
      dataFile.println("Time, Vbus_volt, ExtIn, ExtOut, Oil_IN ,OilTUnits, RPM, FuelPr_psi");
      dataFile.flush();
    }
#ifdef DEBUG    
    else
      Serial.println("No SD file open");
  }
  else
      Serial.println("No SD card");
#else
  }
#endif
  
  if (!dataFile)
  {
    digitalWrite( SD_SS_PIN, LOW);   // Turn SS low,  LED  off
    lcd.setCursor ( 0, 1 );
    lcd.print("NoSDcard");
    delay(1000);        
  }

#endif 

  EEPROM.get( 0, MetricDisplay);
 
}

// checks if a Long button press was issued and switches the EEprom and Metric display global.
void CheckToggleMetric( void )  
{
      if (LongPressCnt)   // toggle between imperial and metric display
      {
        if ( MetricDisplay != 0 )
          MetricDisplay = 0;
        else
          MetricDisplay = 1;

        EEPROM.put( 0, MetricDisplay);
        LongPressCnt = 0;
      }
}

void lcdPrintUnits(char ln, char Sym, char * units)
{
        lcd.print("      ");     
        lcd.setCursor ( 6, ln );
        lcd.print(Sym); 
        lcd.print(units);  
}

// The Arduino IDE loop function -- Called contineously
void loop()
{
	float Vbus_Volt;					 // The 12v Bus voltage
	float rpm;	// The RPM result
	float Fuelp;	// The fuelpressure result
	short rounded;
	float temp_1, temp_2, temp_3;
	char const *  units = "";
	unsigned short hours, seconds, minutes;		// uptime
		
	static int rpm_avg[AVG_SAMPLE] = {0};// RPM average array
	static int rpm_total = 0;			 // RPM average running total
	static int fp_avg[AVG_SAMPLE] = {0}; // Fuelpressure average array
	static int fp_total = 0;			 // FP average running total	
	static char PrevEncCnt = EncoderCnt;  // used to indicate that user turned knob
	static unsigned char PrevShortPressCnt = 0;
	static char p;
	static unsigned short REDledAlarm = 0;
	static unsigned long t = millis();
	static unsigned timeout =0;

 
  
  wdt_reset();


  // anything to display ?
  if ( t + UPDATE_PER > millis() && EncoderCnt == PrevEncCnt && ShortPressCnt == PrevShortPressCnt)
    return;

  
  // update every n msec
  t = millis();
  seconds = t/1000;			// uptime in total seconds 
  minutes = seconds /60;	// uptime in total minutes
  hours = seconds / 3600;   // uptime in total hours
  
  if (timeout != 0 && seconds > timeout ) 
  {
	EncoderCnt= MENU_OIL_NTC; // default back to primary oil temp display after timeout
	timeout = 0;
  }
 
 // read all the analog inputs 
  Vbus_Volt = analogRead(VBUS_ADC) *VBUS_ADC_BW;

 if (Vbus_Volt > VOLT_HIGH_ALARM  || Vbus_Volt < VOLT_LOW_ALARM )
  {
		if ( !(REDledAlarm & 1<<MENU_V_Bus))
			EncoderCnt = MENU_V_Bus;	
		
        REDledAlarm |= 1<<MENU_V_Bus;

  }
  else
	  REDledAlarm &= ~(1<<MENU_V_Bus);
	  
 
  temp_1 = ntcRead(NTC1_R25C, NTC1_BETA, NTC1_ADC);
  temp_2 = ntcRead(NTC2_R25C, NTC2_BETA, NTC2_ADC);
  temp_3 = ntcRead(NTC3_R25C, NTC3_BETA, NTC3_ADC);
  
   if (OIL_TEMP_ALARM < CtoF( temp_3))		// using the temp sensor thats inside the oil stream upfront of the engine
   {	   
	    if (! (REDledAlarm & (1<<MENU_OIL_NTC)))	// first time switch disply to this
			EncoderCnt = MENU_OIL_NTC;
			
         REDledAlarm |= 1<<MENU_OIL_NTC;
		 
   }
   else
	    REDledAlarm &= ~(1<<MENU_OIL_NTC);
	   
 
	/* Note on analogReference function
		
	   The analogReference function does not actually command the refrence, but rather just copies the requested 
	   reference input selection to a static variable which then is used at the time of the next analogRead(). 
	   This implementation of course does not work as expected since there has to be a delay between switching 
	   the refrence and the following reading of the ADC to let the ADC_REF capacitor charge.  
	   The code sequence below with a dummy analogRead and delay is a workaround for this poor implementation found
	   in the Arduiono wiring_alanog.c file
	   
	*/
  analogReference( INTERNAL); 
  analogRead(0);	// empty read just to set the Reference so that cap can charge 
  delay(25);		// time to charge vref cap -- at least 4 ms

  fp_total -= fp_avg[avg.ndx];
  fp_avg[avg.ndx] = analogRead(FP_ADC);
  fp_total += fp_avg[avg.ndx];
  
  rpm_total -= rpm_avg[avg.ndx];
  rpm_avg[avg.ndx] = analogRead(RPM_ADC);
  rpm_total += rpm_avg[avg.ndx];
  
  avg.ndx++;
  

  
  rpm = rpm_total/AVG_SAMPLE * ADC_BW_2 * 10000.0; 	// scale according to Lightspeed is 1mv per 10 rev, i.e 0.3 V == 3000RPM
  rpm *= LGHTSPD_err_rpm;    // Error compensation for Lightspeed outputs.	  
  
  
  Fuelp = fp_total/AVG_SAMPLE * ADC_BW_2; 	// in volts now
    
  if ( Fuelp <= 0.400 ||  Fuelp >= 0.660)	// sensor is not connected properly or it failed.
	Fuelp= -99.0;		
  else
  { 
	Fuelp = Fuelp - FuelPressureV_0;
	Fuelp *=  FuelPressureSlope;
  }
 
  // highest priority if multiple alarms come at the same time 
  if (Fuelp > FuelP_HIGH_Alarm  || Fuelp < FuelP_LOW_Alarm )
  {
	  if (!(REDledAlarm & (1<<MENU_FP)))
		EncoderCnt = MENU_FP;
	
	  REDledAlarm |= 1<<MENU_FP;
      
  }
  else 
	  REDledAlarm &= ~(1<<MENU_FP);

  
  // switch back to 5v ref for next time around
  // 5V rail, is used since the NTCs are powered through 1K resistors from the same 5V.
  // this therefore makes the readings ratiometric and the precision of the 5V rail falls out of the equation.
  analogReference( DEFAULT); 
  analogRead(0);	// empty read just to set the Reference so that cap can charge 
  //no delay needed here, it takes 500ms until we get back into the top of loop again
	  
  // Start of the individual readings display
  lcd.home(  );  // Don't use LCD clear because of screen flicker
  lcd.display();	// re-enable display for alarm blinking mode 

re_eval:
  // Limit this display to valid choices, wrap around if invalid choice is reached
  
#ifdef MENUWRAP 
  if (EncoderCnt <= DISPLAY_BEGIN )
    EncoderCnt = DISP_END - 1;
    
  if (EncoderCnt >= DISP_END)
    EncoderCnt = DISPLAY_BEGIN+1;
#else  // don't wrap -- lock at end points
  if (EncoderCnt <= DISPLAY_BEGIN )
    EncoderCnt = DISPLAY_BEGIN +1;
    
  if (EncoderCnt >= DISP_END)
    EncoderCnt = DISP_END-1;
#endif


  switch (EncoderCnt)		// for display items
  {

    case MENU_V_Bus:
      if (timeout == 0 )			// So that the voltage display times out and switches back to primary oil temp
        timeout = seconds +TIMEOUT_TIME;  	
        
    if (REDledAlarm & 1<<MENU_V_Bus && digitalRead(LED1_PIN) )
		lcd.noDisplay();
	
      lcd.print("Voltage ");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( Vbus_Volt ) ;
      lcd.print(" V  ");
      
   
      break;

    case MENU_RPM:
	  if (timeout == 0 )			
        timeout = seconds +TIMEOUT_TIME;  
	
      lcd.print("  RPM   ");
      lcd.setCursor ( 0, LINE2 );
	  lcd.print("  ");
      lcd.print( (short) rpm );
      lcd.print("     ");
      break;
	  
	case  MENU_FP:
	  if (timeout == 0 )			
        timeout = seconds +TIMEOUT_TIME; 
	
	if (REDledAlarm & 1<<MENU_FP && digitalRead(LED1_PIN) )
		lcd.noDisplay();


	  lcd.print("FuelPr  ");
      lcd.setCursor ( 0, LINE2 );
      lcd.print( Fuelp );
      lcd.print(" PSI  ");
      break;



    case MENU_OIL_NTC:
    {
      static bool disp_Delta = true;
      static int prev_reading ;  // two vars for trend display 
      static int curr_reading ;
      char trend = ' ';
      
      CheckToggleMetric();

      if (ShortPressCnt != PrevShortPressCnt)   // entering setup
      {
        disp_Delta = !disp_Delta;
      }
      
      if (seconds % TrendTime == 0)
      {
        prev_reading = curr_reading;
        curr_reading = (int) temp_3 *10 ;      // Trend must be taken from the 3rd probe that's immersed in oil for fast response
      }
      
      if (curr_reading > prev_reading+TrendHysteresis) 
          trend =UpSym;
      else if (curr_reading < prev_reading-TrendHysteresis)
        trend = DownSym;
        
    
        
      if (MetricDisplay)
      {
         units = CEL;
      }
      else
      {
        units = FAR;
        temp_1 = CtoF( temp_1);
        temp_2 = CtoF( temp_2);
        temp_3 = CtoF( temp_3);
      }
      
	  if (REDledAlarm & 1<<MENU_OIL_NTC && digitalRead(LED1_PIN) )
		lcd.noDisplay();
      
	  if (disp_Delta == false )
      {
        lcd.print(temp_1,1);
        lcdPrintUnits(LINE1,DegSym,(char *)units);
        
        lcd.setCursor ( 0, LINE2 );
        lcd.print(temp_2,1);
        lcdPrintUnits(LINE2,DegSym,(char *)units);
      } 
      else
      {  
        lcd.print(temp_3,1);
        lcd.print(trend);
        lcdPrintUnits(0,DegSym,(char *) units); 
          
        lcd.setCursor ( 0, LINE2 );
        lcd.print(temp_1 - temp_2,1);
        lcdPrintUnits(LINE2,DeltaSym,(char *)units);
	  }
	  
      break;

    }
	case MENU_TIME:
	{
		if (timeout == 0 )
			timeout = seconds + 10;
		
		lcd.print("Up Time ");
		lcd.setCursor ( 0, LINE2 );
		lcd.print( hours);
		lcd.print(":");
		if (minutes %60 <10)
			lcd.print( "0");
		lcd.print( minutes%60);
		lcd.print(":");
		if (seconds%60 <10)
			lcd.print( "0");
		lcd.print(seconds%60);
		lcd.print("     ");
	
	}
	break;
			  
   default: // this is here in case the Menu items are contineous, i.e commented out items 
      // go in the same direction as last knob input from user and re-evaluate again.
      // will wrap around in re_eval;
      EncoderCnt += EncoderDirection;
      goto re_eval;
      break;
  }


  if (PrevEncCnt != EncoderCnt )	// if user switched display during timeout start timeout time afresh next round
    timeout = 0;
    
  
#ifdef WITH_SD_CARD  
      // store readings every SD_RECORD_PER seconds in SD card
      if (t % (SD_RECORD_PER * 1000) < UPDATE_PER  && dataFile  )
      {
		dataFile.print( hours);
		dataFile.print(':');
        dataFile.print( minutes%60);
		dataFile.print(':');
		dataFile.print( seconds%60);
		dataFile.print(',');
		dataFile.print(Vbus_Volt);
		dataFile.print("V,");
        dataFile.print(temp_1);
        dataFile.print(',');
        dataFile.print (temp_2);
        dataFile.print(',');
        dataFile.print (temp_3);
        dataFile.print(',');
        dataFile.print(units);
		dataFile.print(',');
		dataFile.print((short) rpm);
		dataFile.print(",");
		dataFile.print(Fuelp);
        dataFile.flush();
      }
#endif 
  
#ifdef ALARMS_A
  // set the RED alarm led
	  if (REDledAlarm)
	  {
	    if (digitalRead(LED1_PIN) )
	      digitalWrite( LED1_PIN, LOW);
	    else
		    digitalWrite( LED1_PIN, HIGH);

	  }
	  else
		digitalWrite( LED1_PIN, LOW);
#endif
	// at the end of the flight when motor is shut down, record total flight time
	// this also stops recording after 10 munutes when testing on the ground
	if ( minutes > 10  && rpm < 500)
	{
#ifdef WITH_SD_CARD  		
		if ( dataFile  )
		{
			dataFile.print( hours);
			dataFile.print(':');
			dataFile.print( minutes%60);
			dataFile.print(':');
			dataFile.print( seconds%60);
			dataFile.println(",,,,,, -- END of Flight --");
			dataFile.close();	// stop further recording  -- this causes datafile to evaluate to false
		}
#endif
	  EncoderCnt = MENU_TIME;	//keep displaying the time for logbook entry
	}
	
	PrevEncCnt = EncoderCnt;
	PrevShortPressCnt = ShortPressCnt;
}

