#define REVISION_NUM "1.0.0"
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////BPS_DEF.h////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/*BPS.def */
/* BPS LIMIT SETPOINTS */
#define LIMIT_CURRENT_HIGH           80000
#define LIMIT_CURRENT_LOW         (-40000)
#define LIMIT_VOLTAGE_HIGH         3600000
#define LIMIT_VOLTAGE_LOW          2500000
#define LIMIT_TEMP_HIGH                 65
#define LIMIT_VOLTAGE_ARRAY_CUTON  3400000
#define LIMIT_VOLTAGE_ARRAY_CUTOFF 3500000

/* OTHER BPS SETTINGS*/
#define MODULES_USED         20
#define MODULES_MAX          48
#define RELAYON            HIGH
#define RELAYOFF            LOW
#define DURATION_PRECHARGE 2000
#define DURATION_CHARGE    1000
#define VREF                  5

//////////////////////////////////////////
/*PIN ASSIGNMENTS FOR LEDS*/
#define LED_GOOD    13
#define LED_OV      12
#define LED_UV      12
#define LED_OT      12
#define LED_OC      12
#define LED_OTHER   13

/*PIN ASSIGNMENTS FOR MUXES */
#define MUXE0     6
#define MUXE1     7
#define MUXE2     8
#define MUXS0     2
#define MUXS1     3
#define MUXS2     4
#define MUXS3     5

/*PIN ASSIGNMENTS FOR ADC */
#define pin_Isense         A0
#define pin_Tsense         A1

/*PIN ASSIGNMENTS FOR OUTPUT RELAYS */
#define relay_main          9
#define relay_array        10
#define relay_precharge    11
////////////////////////////////////////////

/* INTERNALLY USED CONSTANTS */
#define ERR_NONE    0
#define ERR_OV      1
#define ERR_UV      2
#define ERR_OT      3
#define ERR_OC      4
#define ERR_OTHER   5
#define SSEG_NOTHING 0xFF
#define LED_NOTHING  0xFF
/////////////////////////////////////////////////////////////////////////////////////////////////////
////////end BPS_DEF.h////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////BPS.h/////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
typedef struct BPS_STRUCT {
  uint32_t V[MODULES_MAX]; //Voltage in microVolts
  int8_t  T[MODULES_MAX]; //Temp in Celsius
  int32_t  I; //Current in milliAmps (discharge positive)
  uint8_t  num_modules;
  uint8_t  is_good;
  uint8_t  is_started;
} BPS_STRUCT;

typedef struct ERR_STRUCT {
  uint32_t V; //Voltage in microVolts
  int8_t  T; //Temp in Celsius
  int32_t  I; //Current in milliAmps (discharge positive)
  uint8_t  offender_module;
  uint8_t  error;
} ERR_STRUCT;

////////////////////////////////////////////////////////////////////////////
/* TASK LAYER ///////////////////////////////////////////////////         */

void     BPSinit(void);
void     BPScheck(void);
void     DISPLAYout(void);

/* MIDDLE LAYER ////////////////////////////////////////////////////////////
   Items in this layer control the collecting of specific pieces of 
   information from the different input sources. It also includes
   functions for handling the output relays, and onboard displays         */

uint32_t Vget(uint8_t index);
int8_t   Tget(uint8_t index);
int32_t  Iget(void);
void     MUXset(uint8_t index);
void     CARstart(void);
void     CARshutdown(uint8_t errorcd, uint8_t offender,uint32_t V, int8_t T, int32_t I);
void     SSEGwrite(uint8_t val);
void     LEDwrite(uint8_t led);
void     ARRAYcheck(uint8_t i);

/* LOW LEVEL //////////////////////////////////////////////////////////////////////////////////
   Handles helper functions which directly access microcontroller hardware */

uint16_t ADCget(uint8_t pin);
void     I2Ctx(unsigned char address, unsigned char reg, unsigned char data, unsigned char data2);
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////end BPS.h///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////BPS.c///////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <avr/pgmspace.h>
#include <Wire.h>
//#include "BPS_DEF.h"
//#include "BPS.h"

BPS_STRUCT BPS = { {0}, {0}, 0, MODULES_USED, 0, 0};
ERR_STRUCT ERR = {0};

//PROGMEM float Thash[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-5,-4.9,-4.8,-4.7,-4.5,-4.4,-4.3,-4.2,-4.1,-3.9,-3.8,-3.7,-3.6,-3.4,-3.3,-3.2,-3.1,-3,-2.9,-2.7,-2.6,-2.5,-2.4,-2.3,-2.1,-2,-1.9,-1.8,-1.7,-1.6,-1.4,-1.3,-1.2,-1.1,-1,-0.9,-0.7,-0.6,-0.5,-0.4,-0.3,-0.2,-0.1,0,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2,2.2,2.3,2.4,2.5,2.6,2.7,2.8,2.9,3,3.1,3.2,3.3,3.5,3.6,3.7,3.8,3.9,4,4.1,4.2,4.3,4.4,4.5,4.6,4.7,4.8,4.9,5,5.1,5.3,5.4,5.5,5.6,5.7,5.8,5.9,6,6.1,6.2,6.3,6.4,6.5,6.6,6.7,6.8,6.9,7,7.1,7.2,7.3,7.4,7.5,7.6,7.7,7.8,7.9,8,8.1,8.2,8.3,8.4,8.5,8.6,8.7,8.8,8.9,9,9.1,9.2,9.3,9.4,9.5,9.6,9.7,9.8,9.9,10,10.1,10.2,10.3,10.4,10.5,10.6,10.7,10.8,10.9,11,11.1,11.2,11.3,11.4,11.5,11.6,11.7,11.8,11.9,12,12.1,12.2,12.3,12.4,12.5,12.6,12.7,12.8,12.9,13,13.1,13.2,13.3,13.4,13.5,13.6,13.7,13.8,13.9,14,14.1,14.2,14.3,14.4,14.5,14.6,14.7,14.7,14.8,14.9,15,15.1,15.2,15.3,15.4,15.5,15.6,15.7,15.8,15.9,16,16.1,16.2,16.3,16.4,16.5,16.6,16.7,16.8,16.9,17,17.1,17.2,17.3,17.4,17.5,17.5,17.6,17.7,17.8,17.9,18,18.1,18.2,18.3,18.4,18.5,18.6,18.7,18.8,18.9,19,19.1,19.2,19.3,19.4,19.5,19.6,19.7,19.8,19.9,20,20.1,20.1,20.2,20.3,20.4,20.5,20.6,20.7,20.8,20.9,21,21.1,21.2,21.3,21.4,21.5,21.6,21.7,21.8,21.9,22,22.1,22.2,22.3,22.4,22.5,22.6,22.7,22.8,22.9,22.9,23,23.1,23.2,23.3,23.4,23.5,23.6,23.7,23.8,23.9,24,24.1,24.2,24.3,24.4,24.5,24.6,24.7,24.8,24.9,25,25.1,25.2,25.3,25.4,25.5,25.6,25.7,25.8,25.9,26,26.1,26.2,26.3,26.4,26.5,26.6,26.7,26.8,26.9,27,27.1,27.2,27.3,27.4,27.5,27.6,27.7,27.8,27.9,28,28.1,28.2,28.3,28.4,28.5,28.6,28.7,28.8,28.9,29,29.1,29.2,29.3,29.4,29.5,29.6,29.7,29.8,29.9,30,30.1,30.2,30.3,30.4,30.5,30.6,30.7,30.8,30.9,31,31.1,31.2,31.3,31.4,31.5,31.6,31.7,31.8,31.9,32,32.1,32.2,32.3,32.4,32.5,32.6,32.7,32.8,32.9,33,33.1,33.3,33.4,33.5,33.6,33.7,33.8,33.9,34,34.1,34.2,34.3,34.4,34.5,34.6,34.7,34.8,34.9,35,35.2,35.3,35.4,35.5,35.6,35.7,35.8,35.9,36,36.1,36.2,36.3,36.5,36.6,36.7,36.8,36.9,37,37.1,37.2,37.3,37.4,37.5,37.7,37.8,37.9,38,38.1,38.2,38.3,38.4,38.5,38.7,38.8,38.9,39,39.1,39.2,39.3,39.4,39.6,39.7,39.8,39.9,40,40.1,40.2,40.4,40.5,40.6,40.7,40.8,40.9,41.1,41.2,41.3,41.4,41.5,41.6,41.8,41.9,42,42.1,42.2,42.4,42.5,42.6,42.7,42.8,42.9,43.1,43.2,43.3,43.4,43.6,43.7,43.8,43.9,44,44.2,44.3,44.4,44.5,44.7,44.8,44.9,45,45.1,45.3,45.4,45.5,45.6,45.8,45.9,46,46.2,46.3,46.4,46.5,46.7,46.8,46.9,47.1,47.2,47.3,47.4,47.6,47.7,47.8,48,48.1,48.2,48.4,48.5,48.6,48.8,48.9,49,49.2,49.3,49.4,49.6,49.7,49.8,50,50.1,50.2,50.4,50.5,50.7,50.8,50.9,51.1,51.2,51.4,51.5,51.6,51.8,51.9,52.1,52.2,52.4,52.5,52.6,52.8,52.9,53.1,53.2,53.4,53.5,53.7,53.8,54,54.1,54.3,54.4,54.6,54.7,54.9,55,55.2,55.3,55.5,55.6,55.8,55.9,56.1,56.2,56.4,56.6,56.7,56.9,57,57.2,57.4,57.5,57.7,57.8,58,58.2,58.3,58.5,58.7,58.8,59,59.2,59.3,59.5,59.7,59.8,60,60.2,60.3,60.5,60.7,60.9,61,61.2,61.4,61.6,61.7,61.9,62.1,62.3,62.4,62.6,62.8,63,63.2,63.4,63.5,63.7,63.9,64.1,64.3,64.5,64.7,64.9,65.1,65.2,65.4,65.6,65.8,66,66.2,66.4,66.6,66.8,67,67.2,67.4,67.6,67.8,68.1,68.3,68.5,68.7,68.9,69.1,69.3,69.5,69.7,70,70.2,70.4,70.6,70.8,71.1,71.3,71.5,71.7,72,72.2,72.4,72.7,72.9,73.1,73.4,73.6,73.9,74.1,74.3,74.6,74.8,75.1,75.3,75.6,75.8,76.1,76.3,76.6,76.9,77.1,77.4,77.7,77.9,78.2,78.5,78.7,79,79.3,79.6,79.8,80.1,80.4,80.7,81,81.3,81.6,81.9,82.2,82.5,82.8,83.1,83.4,83.7,84,84.3,84.7,85,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PROGMEM int8_t Thash[] = {-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-5,-5,-5,-5,-5,-4,-4,-4,-4,-4,-4,-4,-4,-3,-3,-3,-3,-3,-3,-3,-3,-3,-2,-2,-2,-2,-2,-2,-2,-2,-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,10,10,11,11,11,11,11,11,11,11,11,11,12,12,12,12,12,12,12,12,12,12,13,13,13,13,13,13,13,13,13,13,14,14,14,14,14,14,14,14,14,14,15,15,15,15,15,15,15,15,15,15,15,16,16,16,16,16,16,16,16,16,16,17,17,17,17,17,17,17,17,17,17,18,18,18,18,18,18,18,18,18,18,18,19,19,19,19,19,19,19,19,19,19,20,20,20,20,20,20,20,20,20,20,20,21,21,21,21,21,21,21,21,21,21,22,22,22,22,22,22,22,22,22,22,23,23,23,23,23,23,23,23,23,23,23,24,24,24,24,24,24,24,24,24,24,25,25,25,25,25,25,25,25,25,25,26,26,26,26,26,26,26,26,26,26,27,27,27,27,27,27,27,27,27,27,28,28,28,28,28,28,28,28,28,28,29,29,29,29,29,29,29,29,29,29,30,30,30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,31,31,31,32,32,32,32,32,32,32,32,32,32,33,33,33,33,33,33,33,33,33,34,34,34,34,34,34,34,34,34,34,35,35,35,35,35,35,35,35,35,36,36,36,36,36,36,36,36,36,37,37,37,37,37,37,37,37,37,37,38,38,38,38,38,38,38,38,38,39,39,39,39,39,39,39,39,39,40,40,40,40,40,40,40,40,41,41,41,41,41,41,41,41,41,42,42,42,42,42,42,42,42,43,43,43,43,43,43,43,43,43,44,44,44,44,44,44,44,44,45,45,45,45,45,45,45,45,46,46,46,46,46,46,46,46,47,47,47,47,47,47,47,47,48,48,48,48,48,48,48,49,49,49,49,49,49,49,49,50,50,50,50,50,50,50,51,51,51,51,51,51,51,52,52,52,52,52,52,52,53,53,53,53,53,53,53,54,54,54,54,54,54,54,55,55,55,55,55,55,56,56,56,56,56,56,56,57,57,57,57,57,57,58,58,58,58,58,58,59,59,59,59,59,59,60,60,60,60,60,60,61,61,61,61,61,61,62,62,62,62,62,62,63,63,63,63,63,64,64,64,64,64,65,65,65,65,65,65,66,66,66,66,66,67,67,67,67,67,68,68,68,68,69,69,69,69,69,70,70,70,70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,74,74,74,74,75,75,75,75,76,76,76,76,77,77,77,77,78,78,78,79,79,79,79,80,80,80,80,81,81,81,82,82,82,83,83,83,83,84,84,84,85,85,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128,-128};
////////////////////////////////////////////////////////////////////////////
/* TASK LAYER //////////////////////////////////////////////////////////////
   Task list:
   init()       --> called in the beginning of the startup thread
   BPScheck()   --> called in the WHILE(1) loop in ThdBPScheck
   DISPLAYout() --> called in the WHILE(1) loop in ThdDISPLAYout
   All of the other tasks are implemented directly in the high-level layer
*/
void BPSinit(void) {
   int i = 0;
   const uint8_t pins[] = {MUXE0,MUXE1,MUXE2,MUXS0,MUXS1,MUXS2,MUXS3,LED_OV,LED_UV,LED_OT,LED_OC,LED_OTHER,LED_GOOD};

   //Serial.begin(9600);
   //analogReference(INTERNAL);

   //low = enabled
   digitalWrite(MUXE0, HIGH); 
   digitalWrite(MUXE1, HIGH); 
   digitalWrite(MUXE2, HIGH);

   for(i=0;i<sizeof(pins);i++) {
      pinMode(pins[i], OUTPUT);
   }

   Wire.begin();
   I2Ctx(0x40, 0x00, 0x00, 0x00);

   return;
}
/*                              BPScheck                                 */
void BPScheck(void) {
   uint8_t i;
   for(i=0;i<BPS.num_modules;i++){
      //check the array
      ARRAYcheck(i);
      //check volts, temps, and currents
      if(BPS.T[i] == -128 || BPS.V[i] == 0) {
         CARshutdown(ERR_OTHER,i,BPS.V[i],BPS.T[i],BPS.I);
      }

      else if(BPS.I > LIMIT_CURRENT_HIGH || BPS.I < LIMIT_CURRENT_LOW) {
         CARshutdown(ERR_OC,i,BPS.V[i],BPS.T[i],BPS.I);
      }

      else if(BPS.V[i] > LIMIT_VOLTAGE_HIGH) {
         CARshutdown(ERR_OV,i,BPS.V[i],BPS.T[i],BPS.I);
      }

      else if(BPS.V[i] < LIMIT_VOLTAGE_LOW) {
         CARshutdown(ERR_UV,i,BPS.V[i],BPS.T[i],BPS.I);
      }

      else if(BPS.T[i] > LIMIT_TEMP_HIGH) {
         CARshutdown(ERR_OT,i,BPS.V[i],BPS.T[i],BPS.I);
      }

      else {
        ERR.error = ERR_NONE;
        BPS.is_good = 1;
      }

   } // END FOR

   return;
}
/*                                   ThdDISPLAYout                      */
void DISPLAYout(void) {
   LEDwrite(LED_NOTHING);
   switch(ERR.error) {
      case ERR_NONE:
         SSEGwrite(SSEG_NOTHING);
         LEDwrite(LED_GOOD);
         break;
      case ERR_OTHER:
         SSEGwrite(ERR.offender_module);
         LEDwrite(LED_OTHER);
         break;
      case ERR_OV:
         SSEGwrite(ERR.offender_module);
         LEDwrite(LED_OV);
         break;
      case ERR_UV:
         SSEGwrite(ERR.offender_module);
         LEDwrite(LED_UV);
         break;
      case ERR_OT:
         SSEGwrite(ERR.offender_module);
         LEDwrite(LED_OT);
         break;
      case ERR_OC:
         SSEGwrite(ERR.offender_module);
         LEDwrite(LED_OC);
         break;
   } //END SWITCH

   return;
}
////////////////////////////////////////////////////////////////////////////


/* MIDDLE LAYER ////////////////////////////////////////////////////////////
   Items in this layer control the collecting of specific pieces of 
   information from the different input sources. It also includes
   functions for handling the output relays, and onboard displays
   Vget
   Tget
   Iget
   MUXsex
   CARstart
   CARshutdown
   SSEGwrite
   LEDwrite
   ARRAYcheck
*/

uint32_t Vget(uint8_t index) {
   uint32_t result = 3200000;
   //query MCP register using SPI
   //convert to microVolts

   return result;
}

int8_t Tget(uint8_t index) {
   uint16_t adc = 0;
   MUXset(index);
   adc = ADCget(pin_Tsense);
   int8_t result = (int8_t) pgm_read_byte_near( Thash + adc );

   return result;
}

int32_t Iget(void) {
   //int32_t conversion_rate = 0;
   //int32_t adc = ADCget(pin_Isense);
   int32_t result = 0;//(conversion_rate * adc * VREF) / 1023;
   
   return  result;
}

void MUXset(uint8_t index){
   uint8_t port = index % 16;
   uint8_t mux  = 1U << (index / 16);
   digitalWrite(MUXE0, !bitRead(mux,  0) );
   digitalWrite(MUXE1, !bitRead(mux,  1) );
   digitalWrite(MUXE2, !bitRead(mux,  2) );
   digitalWrite(MUXS0, bitRead(port, 0) );
   digitalWrite(MUXS1, bitRead(port, 1) );
   digitalWrite(MUXS2, bitRead(port, 2) );
   digitalWrite(MUXS3, bitRead(port, 3) );

   return;
}

void CARstart(void) {
   digitalWrite(relay_precharge,RELAYON);
   delay(DURATION_PRECHARGE);
   digitalWrite(relay_main,RELAYON);
   delay(DURATION_CHARGE);
   digitalWrite(relay_precharge,RELAYOFF);
   digitalWrite(relay_array,RELAYON);
   BPS.is_started = 1;
   return;
}

void CARshutdown(uint8_t errorcd, uint8_t offender,uint32_t V, int8_t T, int32_t I) {
   digitalWrite(relay_main,RELAYOFF);
   digitalWrite(relay_array,RELAYOFF);
   BPS.is_good = 0;
   ERR.error = errorcd;
   ERR.offender_module = offender;
   ERR.V = V;
   ERR.T = T;
   ERR.I = I;
   
   return;
}

/* writes the 2 digit number to the display*/
void SSEGwrite(uint8_t val) {
   const char seg_table[10] = 
   {
      //0gfedcba
      0b00111111, //0
      0b00000110, //1
      0b01011011, //2
      0b01001111, //3
      0b01100110, //4
      0b01101101, //5
      0b01111101, //6
      0b00000111, //7
      0b01111111, //8
      0b01101111  //9
   };
   uint8_t dig1 = (val % 100) / 10;
   uint8_t dig2 = (val % 100) % 10;
   uint8_t code1 = seg_table[dig1];
   uint8_t code2 = seg_table[dig2];
   if(val == SSEG_NOTHING){ code1 = 0x00; code2 = 0x00; }
   I2Ctx(0x40, 0x12, code2, code1);

   return;
}

void LEDwrite(uint8_t led) {
   if(led == LED_NOTHING) {
      digitalWrite(LED_GOOD,  LOW);
      digitalWrite(LED_OTHER, LOW);
      digitalWrite(LED_OV,    LOW);
      digitalWrite(LED_UV,    LOW);
      digitalWrite(LED_OT,    LOW);
      digitalWrite(LED_OC,    LOW);
   }
   else {
      digitalWrite(led, HIGH);
   } //END IF

   return;
}

void ARRAYcheck(uint8_t i) {
   uint8_t status = digitalRead(relay_array);
   if(status && BPS.V[i] > LIMIT_VOLTAGE_ARRAY_CUTOFF) { /*array is enabled and overvoltage*/
      digitalWrite(relay_array,RELAYOFF);
   }
   else if (!status && BPS.V[i] < LIMIT_VOLTAGE_ARRAY_CUTON) { /*array is disabled and undervoltage*/
      digitalWrite(relay_array,RELAYON);
   }

   return;
}
///////////////////////////////////////////////////////////////////////////////////////////////

/* LOW LEVEL //////////////////////////////////////////////////////////////////////////////////
   Handles helper functions which directly access microcontroller hardware
   ADCget
   I2Ctx
*/
uint16_t ADCget(uint8_t pin) {
   uint8_t i = 0;
   uint32_t sum = 0;
   for(i=0;i<10;i++) {
      sum += analogRead(pin);
   }

   return (uint16_t) sum / 10;
}

void I2Ctx(unsigned char address, unsigned char reg, unsigned char data, unsigned char data2) {
   Wire.beginTransmission(address);
   Wire.write(reg);
   Wire.write(data);
   Wire.write(data2);
   Wire.endTransmission();

   return;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////end BPS.c///////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////BEGIN BPS_RTOS INO//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
#include <NilRTOS.h>
#include "nilconf.h"

SEMAPHORE_DECL(semBPSGOOD, 0);
SEMAPHORE_DECL(semV, 0);
SEMAPHORE_DECL(semT, 0);
SEMAPHORE_DECL(semI, 0);
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdSTARTUP, 128);
NIL_THREAD(ThdSTARTUP, arg) {
 BPSinit();
 while (TRUE) {
   nilSemWait(&semBPSGOOD);
   CARstart();
   while (TRUE) {
     nilThdSleepSeconds(255);
   } //END WHILE
 } //END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdBPScheck, 128);
NIL_THREAD(ThdBPScheck, arg) {
 uint8_t flag_start = FALSE; 
 while (TRUE) {
  if(flag_start == FALSE) {
    nilSemWait(&semV);
    nilSemWait(&semT);
    nilSemWait(&semI);
  } //END IF
  BPScheck(); //MAIN ROUTINE THAT CHECKS IF ALL VALUES ARE IN RANGE
  if(BPS.is_good && flag_start == FALSE){
     flag_start = TRUE;
     nilSemSignal(&semBPSGOOD); 
  } //END IF
  nilThdSleepMilliseconds(20);
 } //END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdVget, 128);
NIL_THREAD(ThdVget, arg) {
 while (TRUE) {
   uint8_t i;
   for(i = 0; i<BPS.num_modules; i++) {
     BPS.V[i] = Vget(i);
     nilThdSleepMilliseconds(20);
   }
   nilSemSignal(&semV);
 } // END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdTget, 128);
NIL_THREAD(ThdTget, arg) {
 uint8_t i;
 while (TRUE) {
   for(i = 0; i<BPS.num_modules; i++) {
     BPS.T[i] = Tget(i);
     nilThdSleepMilliseconds(20);
   }
   nilSemSignal(&semT);
 } // END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdIget, 128);
NIL_THREAD(ThdIget, arg) {
 while (TRUE) {
   BPS.I = Iget();
   nilThdSleepMilliseconds(500);
   nilSemSignal(&semI);
 } // END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdUSBout, 128);
NIL_THREAD(ThdUSBout, arg) {
 uint8_t flag_started = FALSE;
 Serial.begin(19200);
 Serial.println("===============USB ONLINE ===============");
 while (TRUE) {
   uint8_t i;
   for(i = 0; i<BPS.num_modules; i++) {
     Serial.print("V[");Serial.print(i);Serial.print("]=");Serial.println(BPS.V[i]);
     Serial.print("T[");Serial.print(i);Serial.print("]=");Serial.println(BPS.T[i]);
     Serial.print("I=");Serial.println(BPS.I);
     if(ERR.error != ERR_NONE){
       Serial.println("===============BPS FAULT! ===============");
       Serial.print("Error="); 
       switch(ERR.error){
         case    ERR_OV: Serial.println("Over Voltage"); break;
         case    ERR_UV: Serial.println("Under Voltage"); break;
         case    ERR_OT: Serial.println("Over Temperature"); break;
         case    ERR_OC: Serial.println("Over Current"); break;
         case ERR_OTHER: Serial.println("Sensors not working"); break;
       }
       Serial.print("Module="); Serial.println(ERR.offender_module);
       Serial.print("V="); Serial.println(ERR.V);
       Serial.print("T="); Serial.println(ERR.T);
       Serial.print("I="); Serial.println(ERR.I);
     }
     if(BPS.is_started && flag_started == FALSE){
       Serial.println("===============CAR STARTED===============");
       flag_started = TRUE;
     }
     nilThdSleepMilliseconds(40);
   }

 } // END WHILE
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_WORKING_AREA(waThdDISPLAYout, 128);
NIL_THREAD(ThdDISPLAYout, arg) {
 while (TRUE) {
   DISPLAYout();
   nilThdSleepMilliseconds(500);
 }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
NIL_THREADS_TABLE_BEGIN()
NIL_THREADS_TABLE_ENTRY("STARTUP", ThdSTARTUP, NULL, waThdSTARTUP, sizeof(waThdSTARTUP))
NIL_THREADS_TABLE_ENTRY("BPScheck", ThdBPScheck, NULL, waThdBPScheck, sizeof(waThdBPScheck))
NIL_THREADS_TABLE_ENTRY("Vget", ThdVget, NULL, waThdVget, sizeof(waThdVget))
NIL_THREADS_TABLE_ENTRY("Tget", ThdTget, NULL, waThdTget, sizeof(waThdTget))
NIL_THREADS_TABLE_ENTRY("Iget", ThdIget, NULL, waThdIget, sizeof(waThdIget))
NIL_THREADS_TABLE_ENTRY("USBout", ThdUSBout, NULL, waThdUSBout, sizeof(waThdUSBout))
NIL_THREADS_TABLE_ENTRY("DISPLAYout", ThdDISPLAYout, NULL, waThdDISPLAYout, sizeof(waThdDISPLAYout))
NIL_THREADS_TABLE_END()
/*-------------------------------------------------------------------------------------------------*/
void setup() {nilSysBegin();}
void loop() {/* IDLE thread */}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////END BPS_RTOS.ino//////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
