#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>//LiquidCrystal_I2C lcd(0x27,20,4);  //LCD i2c adress sometimes 0x3f or 0x27
#include <Adafruit_INA219.h>
LiquidCrystal_I2C lcd(0x27,16,2);
//Icons
uint8_t Battery[8]  = {0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};
uint8_t Panel[8]  = {0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x00};
//Constants
#define bulk_voltage_max 12.5
#define bulk_voltage_min 11
#define absorption_voltage 14.7
#define float_voltage_max 13
#define battery_min_voltage 10
#define solar_min_voltage 19
#define charging_current 2.0
#define absorption_max_current 2.0
#define absorption_min_current 0.1
#define float_voltage_min 13.2
#define float_voltage 13.4
#define float_max_current 0.12
Adafruit_INA219 ina219;
byte BULK = 0;        //Give values to each mode
byte ABSORPTION = 1;
byte FLOAT = 2;
byte mode = 0;        //We start with mode 0 BULK
//Inputs
#define solar_voltage_in 34
#define battery_voltage_in 39
//Outputs
#define PWM_out 13
#define load_enable 17
//Variables
float bat_voltage = 0;
int pwm_value = 0;
float solar_current = 0;
float current_factor = 0.185;       //Value defined by manufacturer ACS712 5A
float solar_voltage = 0;
float solar_power = 0;
String load_status = "OFF";
int pwm_percentage = 0;
int16_t Vin; 
int16_t Vout; 
int16_t Curr;
float dutyCycle = 0.5;    // Initial duty cycle
float prev_power ;  // Power output of the panel
float delta_duty = 0.01;  // Change in duty cycle for each iteration
void affichage();
static const u1_t PROGMEM APPEUI[8]={ 0x09, 0xF2, 0x20, 0xFF, 0xFF, 0x41, 0x40, 0xA8 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]={ 0x05, 0x73, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = { 0xA3, 0x6F, 0x18, 0xCF, 0x22, 0x0C, 0xA8, 0x6B, 0x35, 0x47, 0x94, 0xEA, 0x61, 0x19, 0x61, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
static osjob_t sendjob;
const unsigned TX_INTERVAL =  15;
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32} 
};
//////////////void declaration/////////////////
void sensor_reading();
/////////////////////////////////
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}
void do_send(osjob_t* j){
   sensor_reading(); 
    byte mydata[3]; 
    mydata[0] = Vin ;
    mydata[1] = Vout ; 
    mydata[2] = Curr; 
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else 
    {
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
}
void setup() {
    Serial.begin(9600);
    Serial.println(F("Starting"));
  pinMode(solar_voltage_in,INPUT);    //Set pins as inputs
  pinMode(battery_voltage_in,INPUT);
  pinMode(PWM_out,OUTPUT);            //Set pins as OUTPUTS
  digitalWrite(PWM_out,LOW);          //Set PWM to LOW so MSOFET is off
  pinMode(load_enable,OUTPUT);
  digitalWrite(load_enable,LOW);      //Start with the relay turned off
  Serial.begin(9600);
  lcd.init();                 
  lcd.backlight();              
  lcd.createChar(0, Battery);
  lcd.createChar(1, Panel);
  if (! ina219.begin()) 
   {
    Serial.println("Failed to find INA219 chip");
    while (1) 
    { 
      delay(10);
    }
   }
    #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}
void loop() 
{
 solar_voltage = get_solar_voltage(15);
  bat_voltage =   get_battery_voltage(15);  
Serial.println(bat_voltage);
delay(1000);
 affichage();
} 
void sensor_reading() 
{ 
    while(true) {
  solar_current = get_solar_current(100);
  solar_power = bat_voltage * solar_current; 
  pwm_percentage = map(pwm_value,0,255,0,100);
 if ( solar_power > prev_power) {
    dutyCycle += delta_duty;
  }
    else {
    dutyCycle -= delta_duty;
  }
    set_duty_cycle(dutyCycle);
   prev_power = solar_power;
  if(bat_voltage < battery_min_voltage)
  {
    digitalWrite(load_enable,LOW);        //We DISABLE the load if battery is undervoltage
  }
  else{
    digitalWrite(load_enable,HIGH);       //We ENABLE the load if battery charged
  }
  ///////////////////////////FLOAT///////////////////////////
  ///////////////////////////////////////////////////////////
  if(mode == FLOAT){
    if(bat_voltage < float_voltage_min)
    {
      mode = BULK;  
    }
    else{
      if(solar_current > float_max_current)
      {    //If we exceed max current value, we change mode
        mode = BULK;  
      }//End if > 
      else{
        if(bat_voltage > float_voltage)
        {
          pwm_value--;
          pwm_value = constrain(pwm_value,0,254);
        }
        else {
          pwm_value++;
          pwm_value = constrain(pwm_value,0,254);
        }        
      }//End else > float_max_current
      
      ledcWrite(PWM_out,pwm_value);
    }
  }//END of mode == FLOAT
  //Bulk/Absorption
  else{
    if(bat_voltage < bulk_voltage_min)
    {
      mode = BULK;  
    }
    else if(bat_voltage > bulk_voltage_max)
    {
      mode = ABSORPTION;
    }
    ////////////////////////////BULK///////////////////////////
    ///////////////////////////////////////////////////////////
    if(mode == BULK){
      if(solar_current > charging_current)
      {
        pwm_value--;
        pwm_value = constrain(pwm_value,0,254);
      }
      else {
        pwm_value++;
        pwm_value = constrain(pwm_value,0,254);
      }
      ledcWrite(PWM_out,pwm_value);
    }//End of mode == BULK
    /////////////////////////ABSORPTION/////////////////////////
    ///////////////////////////////////////////////////////////
    if(mode == ABSORPTION){
      if(solar_current > absorption_max_current)
      {    //If we exceed max current value, we reduce duty cycle
        pwm_value--;
        pwm_value = constrain(pwm_value,0,254);
      }//End if > absorption_max_current
      else{
        if(bat_voltage > absorption_voltage)
        {
          pwm_value++;
          pwm_value = constrain(pwm_value,0,254);
        }
        else {
          pwm_value--;
          pwm_value = constrain(pwm_value,0,254);
        }
        if(solar_current < absorption_min_current)
        {
          mode = FLOAT;
        }
      }//End else > absorption_max_current
      ledcWrite( PWM_out,pwm_value);
    }// End of mode == absorption_max_current
    
  }//END of else mode == FLOAT
         delay(100);
}
  Vin= solar_voltage;
  Vout= bat_voltage;
  Curr= solar_current;        
}
float get_solar_voltage(int n_samples)
{ 
   float voltage = 0;
  for(int i=0; i < n_samples; i++)
  {
    voltage +=0.0094907223* analogRead(solar_voltage_in);
    delay(1);
  }
  voltage = voltage/n_samples;
  if(voltage < 0){voltage = 0;}
  return(voltage);
}
float get_battery_voltage(int n_samples)
{
  float voltage = 0;
  for(int i=0; i < n_samples; i++)
  {
    voltage += analogRead(battery_voltage_in) * 0.0046003417968 ;
    delay(1);      
  }
  voltage = voltage/n_samples-0.2;
  if(voltage < 0){voltage = 0;}
  return(voltage);
}
float get_solar_current(int n_samples)
{
  float current = 0;
  current = ina219.getCurrent_mA();
  return(current);
}
void affichage(){
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0);               //Column 0 row 0
    lcd.write(1);                     //Panel icon
    lcd.print("");                   //Empty space
    lcd.print(solar_voltage);                 //Soalr voltage 
    lcd.print("V");                   //Volts
    lcd.print(" ");                //Empty spaces
    lcd.write(0);                     //Battery icon
    lcd.print("");                   //Empty space
    lcd.print(bat_voltage);                //Battery voltsge
    lcd.print("V");                   //Volts
    lcd.setCursor(0,1); //Column 0 row 1  
    lcd.print("I ");                  //Empty spaces
    lcd.print(solar_current);                  //Solar current
    lcd.print("mA"); 
}
