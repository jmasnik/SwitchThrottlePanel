#include "Arduino.h"
#include "HID-Project.h"

#define SWITCH_COUNT 4
#define AXIS_COUNT 5
#define BUTTON_TMR_MAX 200

// cudlik
typedef struct {
  uint8_t pin;
  
  uint8_t button_a;
  uint8_t button_a_tmr;
  
  uint8_t button_b;
  uint8_t button_b_tmr;
  
  uint8_t actual_value;
} hwswitch;

// analogova osa
typedef struct {
  uint8_t pin;              // na jakem pinu to je
  int adc_real;             // aktualni hodnota nactena z adc
  uint16_t adc_limit_low;   // hodnoty pod ignorujeme
  uint16_t adc_limit_high;  // hodnoty nad ignorujeme
  double adc_real_dbl;      // realna hodnota olimitovana, prepocitana na 0.0 - 1.0
  int16_t out_low;          // vystupni rozsah od
  int16_t out_high;         // vystupni rozsah do
  int32_t out_actual;       // aktualni vystup
  int32_t out_prev;         // predchozi vystup
} hwaxis;

hwswitch switch_list[SWITCH_COUNT];
hwaxis axis_list[AXIS_COUNT]; 

const int pinLed = LED_BUILTIN;
const int pinButton = 2;

uint16_t i;
uint8_t gp_write;

unsigned long ms_serial;
unsigned long ms_gamepad;

void setup() {

  ms_serial = 0;
  ms_gamepad = 0;

  Serial.begin(115200);
  
  switch_list[0].pin = 2;
  switch_list[0].button_a = 1;
  switch_list[0].button_b = 2;

  switch_list[1].pin = 3;
  switch_list[1].button_a = 3;
  switch_list[1].button_b = 4;

  switch_list[2].pin = 4;
  switch_list[2].button_a = 5;
  switch_list[2].button_b = 6;

  switch_list[3].pin = 5;
  switch_list[3].button_a = 7;
  switch_list[3].button_b = 8;

  // plyn
  axis_list[0].pin = A5;
  axis_list[0].adc_limit_low = 0;
  axis_list[0].adc_limit_high = 1022;
  axis_list[0].adc_real = 0;
  axis_list[0].adc_real_dbl = 0;
  axis_list[0].out_actual = 0;
  axis_list[0].out_prev = 0;
  axis_list[0].out_low = -32768;
  axis_list[0].out_high = 32767;

  // vrtule
  axis_list[1].pin = A4;
  axis_list[1].adc_limit_low = 0;
  axis_list[1].adc_limit_high = 1022;
  axis_list[1].adc_real = 0;
  axis_list[1].adc_real_dbl = 0;
  axis_list[1].out_actual = 0;
  axis_list[1].out_prev = 0;
  axis_list[1].out_low = -32768;
  axis_list[1].out_high = 32767;

  // smes
  axis_list[2].pin = A3;
  axis_list[2].adc_limit_low = 0;
  axis_list[2].adc_limit_high = 1022;
  axis_list[2].adc_real = 0;
  axis_list[2].adc_real_dbl = 0;
  axis_list[2].out_actual = 0;
  axis_list[2].out_prev = 0;
  axis_list[2].out_low = -128;
  axis_list[2].out_high = 127;

  // kolektiv
  axis_list[3].pin = A0;
  axis_list[3].adc_limit_low = 200;
  axis_list[3].adc_limit_high = 443;
  axis_list[3].adc_real = 0;
  axis_list[3].adc_real_dbl = 0;
  axis_list[3].out_actual = 0;
  axis_list[3].out_prev = 0;
  axis_list[3].out_low = -32768;
  axis_list[3].out_high = 32767;

  // kolektiv plyn
  axis_list[4].pin = A1;
  axis_list[4].adc_limit_low = 6;
  axis_list[4].adc_limit_high = 1015;
  axis_list[4].adc_real = 0;
  axis_list[4].adc_real_dbl = 0;
  axis_list[4].out_actual = 0;
  axis_list[4].out_prev = 0;
  axis_list[4].out_low = -32768;
  axis_list[4].out_high = 32767;

  pinMode(pinLed, OUTPUT);

  for(i = 0; i < SWITCH_COUNT; i++){
    pinMode(switch_list[i].pin, INPUT_PULLUP);
  }

  for(i = 0; i < SWITCH_COUNT; i++){
    if(digitalRead(switch_list[i].pin) == HIGH){
      switch_list[i].actual_value = 1;
    } else {
      switch_list[i].actual_value = 0;
    }
  }

  gp_write = 1;

  Gamepad.begin();
}

void loop() {
  uint16_t adc_lim;
  int32_t out_range;

  for(i = 0; i < SWITCH_COUNT; i++){
    if(switch_list[i].button_a_tmr == 0 && switch_list[i].button_b_tmr == 0){
      if(digitalRead(switch_list[i].pin) == HIGH && switch_list[i].actual_value == 0){
        // zmena na 1
        switch_list[i].actual_value = 1;
        switch_list[i].button_a_tmr = BUTTON_TMR_MAX;
      }
      if(digitalRead(switch_list[i].pin) == LOW && switch_list[i].actual_value == 1){
        // zmena na 0
        switch_list[i].actual_value = 0;
        switch_list[i].button_b_tmr = BUTTON_TMR_MAX;
      }
    }
  }

  for(i = 0; i < SWITCH_COUNT; i++){
    
    if(switch_list[i].button_a_tmr > 0){
      if(switch_list[i].button_a_tmr == BUTTON_TMR_MAX){
        Gamepad.press(switch_list[i].button_a);
        gp_write = 1;
      }
      if(switch_list[i].button_a_tmr == 1){
        Gamepad.release(switch_list[i].button_a);
        gp_write = 1;
      }
      switch_list[i].button_a_tmr--;
    }

    if(switch_list[i].button_b_tmr > 0){
      if(switch_list[i].button_b_tmr == BUTTON_TMR_MAX){
        Gamepad.press(switch_list[i].button_b);
        gp_write = 1;
      }
      if(switch_list[i].button_b_tmr == 1){
        Gamepad.release(switch_list[i].button_b);
        gp_write = 1;
      }
      switch_list[i].button_b_tmr--;
    }    
  
  }

  // Nacteni analogovych vstupu
  for(i = 0; i < AXIS_COUNT; i++){
    // nacteme ADC
    axis_list[i].adc_real = analogRead(axis_list[i].pin);
    
    // orizneme na limit
    adc_lim = (uint16_t)axis_list[i].adc_real;
    if(adc_lim < axis_list[i].adc_limit_low) adc_lim = axis_list[i].adc_limit_low;
    if(adc_lim > axis_list[i].adc_limit_high) adc_lim = axis_list[i].adc_limit_high;

    // posuneme do nuly
    adc_lim = adc_lim - axis_list[i].adc_limit_low;

    // prepocitame na 0 - 1
    axis_list[i].adc_real_dbl = (double)adc_lim / (double)(axis_list[i].adc_limit_high - axis_list[i].adc_limit_low);

    // ulozime si predchozi vystup
    axis_list[i].out_prev = axis_list[i].out_actual;

    // vypocitame vystup
    out_range = (int32_t)axis_list[i].out_high - (int32_t)axis_list[i].out_low;
    axis_list[i].out_actual = axis_list[i].out_low + (axis_list[i].adc_real_dbl * out_range);

    // zmena?
    if(axis_list[i].out_actual != axis_list[i].out_prev){
      gp_write = 1;
    }
  }

  // Poslani do compu
  if(gp_write == 1 && (millis() - ms_gamepad > 5 || millis() < ms_gamepad)){
    Gamepad.xAxis((int16_t)axis_list[3].out_actual);
    Gamepad.yAxis((int16_t)axis_list[4].out_actual);
    Gamepad.rxAxis((int16_t)axis_list[0].out_actual);
    Gamepad.ryAxis((int16_t)axis_list[1].out_actual);
    Gamepad.rzAxis((int8_t)axis_list[2].out_actual);
    Gamepad.write();
    gp_write = 0;
    ms_gamepad = millis();
  }

  // Vypis na seriak
  if(millis() - ms_serial > 1000 || millis() < ms_serial){
    for(i = 0; i < AXIS_COUNT; i++){
      Serial.print(axis_list[i].adc_real);
      Serial.print("  ");
    }
    /*
    for(i = 0; i < AXIS_COUNT; i++){
      Serial.print(axis_list[i].adc_real_dbl);
      Serial.print("  ");
    } 
    */ 
    for(i = 0; i < AXIS_COUNT; i++){
      Serial.print(axis_list[i].out_actual);
      Serial.print("  ");
    } 
    Serial.println("");
    ms_serial = millis();
  }

}
