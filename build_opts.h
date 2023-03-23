/* This file controls the build-time features */
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~  BUILD OPTIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~//
#include "BoardPins.h"
// for serial debug
#define DEBUG
#define N_NUMBER "N427GS"
	
// see BoardPins.h for pins	



#define WITH_SD_CARD  // Note that SD_CARD and WITH_BARO_HYG_TEMP are mutually exclusive because they share a pin
#define  ALARMS_A 
// #define MENUWRAP   // allows the menu to wrap around from last to first enntry etc..
