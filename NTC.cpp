/* 
 * File:   NTC.c
 * Author: Gary Stofer
 *
 * Created on Nov 4, 2016, 9:35 AM
 * 
 * Module to read temperature from and attached NTC device and compute temperatur using the R@25C and BETA methode. 
 */


#include "NTC.h"



// The NTC and R_Ref forms a Voltage divier from VCC 5V to gnd
#define T_25 (25.0+273.15)		// temperatur 25C in Kelvin

// Read the ADC and compute the temperature from the resistance of the NTC according to R@25C and Beta of the NTC device as defined in the .h file. 
float 
ntcRead(float R25C, int Beta, int adc_input)
{
	short adc_val;
	float v_ntc;
	float r_ntc;
	float T;

	v_ntc =  analogRead(adc_input) * NTC_BW;
	
	r_ntc = v_ntc / ((V_REF - v_ntc)/ R_Ref);
	
	T = log(r_ntc / R25C) / Beta ;//
	T += 1/T_25;
	T = 1/T;

	T = T-273.15;
 
  return T;
	
}

