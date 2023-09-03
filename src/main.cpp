#include "Arduino.h"
#include "HID-Project.h"
#include <Adafruit_MCP23X17.h>

#define SWITCH_COUNT 10
#define AXIS_COUNT 6
#define BUTTON_TMR_MAX 150
#define SWITCH_TMR_MAX 300
#define PIN_MCP1_RST 7

#define BUS_INTERNAL 1
#define BUS_MCP1 2

#define TYPE_SWITCH 1
#define TYPE_BUTTON 2

#define MCP_GPB0 8

// cudlik
typedef struct {
  uint8_t pin;              // ktery pin to je na HW
  uint8_t bus;              // kde ten pin je
  uint8_t type;             // jaky typ cvakatka to je
  uint8_t button_a;         // co mackam na gamepadu
  uint16_t button_a_tmr;
  uint8_t button_b;         // co mackam na gamepadu
  uint16_t button_b_tmr;
  uint8_t actual_value;     // jaky je aktualni stav
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

const int pinButton = 2;

uint16_t i;
uint8_t gp_write;

unsigned long ms_serial;
unsigned long ms_gamepad;
unsigned long ms_buttons;

Adafruit_MCP23X17 mcp1;

void initButton(uint8_t id, uint8_t bus, uint8_t pin, uint8_t gamepad_button);

void setup() {
  uint8_t cntr;

  ms_serial = 0;
  ms_gamepad = 0;
  ms_buttons = 0;

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(50000);
  
  cntr = 0;

  switch_list[cntr].bus = BUS_INTERNAL;
  switch_list[cntr].pin = 2;
  switch_list[cntr].type = TYPE_SWITCH;
  switch_list[cntr].button_a = 1;
  switch_list[cntr].button_b = 2;
  cntr++;

  switch_list[cntr].bus = BUS_INTERNAL;
  switch_list[cntr].pin = 3;
  switch_list[cntr].type = TYPE_SWITCH;
  switch_list[cntr].button_a = 3;
  switch_list[cntr].button_b = 4;
  cntr++;

  switch_list[cntr].bus = BUS_INTERNAL;
  switch_list[cntr].pin = 4;
  switch_list[cntr].type = TYPE_SWITCH;
  switch_list[cntr].button_a = 5;
  switch_list[cntr].button_b = 6;
  cntr++;

  switch_list[cntr].bus = BUS_INTERNAL;
  switch_list[cntr].pin = 5;
  switch_list[cntr].type = TYPE_SWITCH;
  switch_list[cntr].button_a = 7;
  switch_list[cntr].button_b = 8;
  cntr++;

  initButton(cntr, BUS_INTERNAL, 6, 9);
  cntr++;
  initButton(cntr, BUS_INTERNAL, 7, 10);
  cntr++;
  initButton(cntr, BUS_INTERNAL, 8, 11);
  cntr++;
  initButton(cntr, BUS_INTERNAL, 9, 12);
  cntr++;
  initButton(cntr, BUS_INTERNAL, 10, 13);
  cntr++;
  initButton(cntr, BUS_INTERNAL, 11, 14);
  cntr++;

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

  // bocni
  axis_list[5].pin = A2;
  axis_list[5].adc_limit_low = 0;
  axis_list[5].adc_limit_high = 1023;
  axis_list[5].adc_real = 0;
  axis_list[5].adc_real_dbl = 0;
  axis_list[5].out_actual = 0;
  axis_list[5].out_prev = 0;
  axis_list[5].out_low = -128;
  axis_list[5].out_high = 127;

  // pin s interni LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // reset MCP1 neresetuje
  //pinMode(PIN_MCP1_RST, OUTPUT);

  //digitalWrite(PIN_MCP1_RST, LOW);
  //delay(200);
  //digitalWrite(PIN_MCP1_RST, HIGH);
  //delay(500);

  // mcp init
  /*
  if (!mcp1.begin_I2C()) {
    while(1){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
    }
  }
  */

  // nastaveni typu pinu
  for(i = 0; i < SWITCH_COUNT; i++){

    switch_list[i].button_a_tmr = 0;
    switch_list[i].button_b_tmr = 0;

    if(switch_list[i].bus == BUS_INTERNAL){
      pinMode(switch_list[i].pin, INPUT_PULLUP);
    }
    
    if(switch_list[i].bus == BUS_MCP1){
      //mcp1.pinMode(switch_list[i].pin, INPUT_PULLUP);
    }
  }

  // aktualni stav spinacu
  for(i = 0; i < SWITCH_COUNT; i++){
    if(switch_list[i].bus == BUS_INTERNAL){
      if(digitalRead(switch_list[i].pin) == HIGH){
        switch_list[i].actual_value = 1;
      } else {
        switch_list[i].actual_value = 0;
      }
    }

/*
    if(switch_list[i].bus == BUS_MCP1){
      if(mcp1.digitalRead(switch_list[i].pin) == HIGH){
        switch_list[i].actual_value = 1;
      } else {
        switch_list[i].actual_value = 0;  
      }
    }
    */

  }

  gp_write = 1;

/*
  uint8_t val;
  while(1){
    val = mcp1.digitalRead(MCP_GPB0);
    Serial.println(val);
    delay(250);
  }
*/

  // gamepad init
  Gamepad.begin();

  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint16_t adc_lim;
  int32_t out_range;
  int pin_val;
  unsigned long millis_act;

  for(i = 0; i < SWITCH_COUNT; i++){
    if(switch_list[i].button_a_tmr == 0 && switch_list[i].button_b_tmr == 0){
      
      if(switch_list[i].bus == BUS_INTERNAL){
        pin_val = digitalRead(switch_list[i].pin);
      }
      if(switch_list[i].bus == BUS_MCP1){
        //pin_val = mcp1.digitalRead(switch_list[i].pin);
        //delay(100);
        pin_val = 1;
      }

      if(switch_list[i].type == TYPE_SWITCH){
        if(pin_val == HIGH && switch_list[i].actual_value == 0){
          // zmena na 1
          switch_list[i].actual_value = 1;
          switch_list[i].button_a_tmr = SWITCH_TMR_MAX;
        }
        if(pin_val == LOW && switch_list[i].actual_value == 1){
          // zmena na 0
          switch_list[i].actual_value = 0;
          switch_list[i].button_b_tmr = SWITCH_TMR_MAX;
        }
      }

      if(switch_list[i].type == TYPE_BUTTON){
        if(pin_val == LOW){
          // je stisknut
          switch_list[i].actual_value = 0;
          switch_list[i].button_a_tmr = BUTTON_TMR_MAX;
        }
        if(pin_val == HIGH){
          // neni stisknut
          switch_list[i].actual_value = 1;
        }
      }

    }
  }

  // mackani tlacitek na gmaepadu
  millis_act = millis();
  if(millis_act != ms_buttons){
    for(i = 0; i < SWITCH_COUNT; i++){
      
      if(switch_list[i].button_a_tmr > 0){

        if(switch_list[i].type == TYPE_SWITCH){
          if(switch_list[i].button_a_tmr == SWITCH_TMR_MAX){
            Gamepad.press(switch_list[i].button_a);
            gp_write = 1;
          }
          if(switch_list[i].button_a_tmr == 1){
            Gamepad.release(switch_list[i].button_a);
            gp_write = 1;
          }
          switch_list[i].button_a_tmr--;
        }

        if(switch_list[i].type == TYPE_BUTTON){
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
      }

      if(switch_list[i].button_b_tmr > 0){
        if(switch_list[i].type == TYPE_SWITCH){
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
    }
    ms_buttons = millis_act;
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
    Gamepad.zAxis((int8_t)axis_list[5].out_actual);
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

/**
 * Definice cudliku
 */
void initButton(uint8_t id, uint8_t bus, uint8_t pin, uint8_t gamepad_button){
  if(id >= SWITCH_COUNT) return;

  switch_list[id].bus = bus;
  switch_list[id].pin = pin;
  switch_list[id].type = TYPE_BUTTON;
  switch_list[id].button_a = gamepad_button;
  switch_list[id].button_a_tmr = 0;
  switch_list[id].button_b = 0;
  switch_list[id].button_b_tmr = 0;
  switch_list[id].actual_value = 1;  
}