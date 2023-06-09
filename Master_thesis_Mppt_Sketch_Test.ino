#include <lmic.h> // using lmic library for lorawan
#include <hal/hal.h> //using hal library
#include <SPI.h> //using spi library for spi protocol
#include <Wire.h> //using wire library for i2c  protocol
#include <LiquidCrystal_I2C.h>//LiquidCrystal_I2C lcd(0x27,20,4);  //LCD i2c adress sometimes 0x3f or 0x27
#include <Adafruit_INA219.h> //using adafruit library for the ina219
//Constants
#define bulk_voltage_max 12.5 // maximum voltage in bulk stage
#define bulk_voltage_min 11  // minimum voltage in bulk stage
#define absorption_voltage 14.7 // voltage in absorbation stage
#define float_voltage_min 13 //minimum current in float stage
#define battery_min_voltage 10 // minimum voltage in the battery
#define solar_min_voltage 19 //minimum voltage from the solar panel 
#define charging_current 2.0 //current in charging
#define absorption_max_current 2.0 // maximum current in absorbation stage
#define absorption_min_current 0.1 //minimum current in absorbation stage
#define float_voltage_max 13.4 // maximum voltage in float stage
#define float_voltage 13.2 
#define float_max_current 0.12 // maximum current in float stage
//Inputs
#define solar_voltage_in 34 //input of solar panel
#define battery_voltage_in 39 //input of battery
//Outputs
#define PWM_out 13 //output of pwm
#define load_enable 17 //output of load
LiquidCrystal_I2C lcd(0x27,16,2); // declaration of lcd from library
//Icons
uint8_t Battery[8]  = {0x0E, 0x1B, 0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x1F};
uint8_t Panel[8]  = {0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x15, 0x1F, 0x00};
Adafruit_INA219 ina219; // declaration of ina219 from library
byte BULK = 0;        //Give values to each mode
byte ABSORPTION = 1;
byte FLOAT = 2;
byte mode = 0;        //We start with mode 0 BULK
//Variables
float bat_voltage = 0; //initial battery voltage
int pwm_value = 0; //initial pwm value
float solar_current = 0; //initial solar current
float current_factor = 0.185;       //Value defined by manufacturer ACS712 5A
float solar_voltage = 0; //initial solar voltage
float solar_power = 0;  //initial solar power
String load_status = "OFF"; 
int pwm_percentage = 0; //initial pwm percentage
int16_t Vin;  
int16_t Vout; 
int16_t Curr;
float dutyCycle = 0.5;    // Initial duty cycle
float prev_power ;  // Power output of the panel
float delta_duty = 0.01;  // Change in duty cycle for each iteration
// Declare a function for displaying values on the LCD
void affichage();
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x09, 0xF2, 0x20, 0xFF, 0xFF, 0x41, 0x40, 0xA8 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x05, 0x73, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xA3, 0x6F, 0x18, 0xCF, 0x22, 0x0C, 0xA8, 0x6B, 0x35, 0x47, 0x94, 0xEA, 0x61, 0x19, 0x61, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
static osjob_t sendjob;
// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
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
//fonction to sending data (voltage and current)
void do_send(osjob_t* j){
   sensor_reading();// Read sensor values 
byte mydata[3]; // Array to store data
  mydata[0] = Vin; // Store solar voltage in data array
  mydata[1] = Vout; // Store battery voltage in data array
  mydata[2] = Curr; // Store solar current in data array
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else 
    {
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0); // Send data
        Serial.println(F("Packet queued"));
    }
}
void setup() {
    Serial.begin(9600); //serial monitor initialization
    Serial.println(F("Starting")); 
  pinMode(solar_voltage_in,INPUT);    //Set pins as inputs
  pinMode(battery_voltage_in,INPUT);
  pinMode(PWM_out,OUTPUT);            //Set pins as OUTPUTS
  digitalWrite(PWM_out,LOW);          //Set PWM to LOW so MSOFET is off
  pinMode(load_enable,OUTPUT);       //Set pins as OUTPUTS
  digitalWrite(load_enable,LOW);      //Start with the relay turned off
  lcd.init();    // lcd initialization             
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
    // For Pinoccio Scout boards
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
 solar_voltage = get_solar_voltage(15); //reading the solar voltage from the fonction
  bat_voltage =   get_battery_voltage(15);  //reading the battery voltage from the fonction
Serial.println(bat_voltage); // prnting the battery voltage
delay(1000);
 affichage(); 
} 
void sensor_reading()  // fonction for reading thesensor values
{ 
  // this part presente the relation between strategy p&o and bulk,absorption and float algorithm
    while(true) //meaning that the loop will continue to run indefinitely until the program is stopped or interrupted. {
  solar_current = get_solar_current(100); //reading the solar current from the fonction
  solar_power = bat_voltage * solar_current; // getting the solar power by multipying the battery voltage and the solar current
  pwm_percentage = map(pwm_value,0,255,0,100);
 // If power has increased, increase duty cycle
        if ( solar_power > prev_power) {
    dutyCycle += delta_duty; 
  }
   // If power has decreased, decrease duty cycle
        else {
    dutyCycle -= delta_duty; 
  }
     // Update duty cycle of DC-DC converter
    set_duty_cycle(dutyCycle);
   prev_power = solar_power; // Save previous power value
  if(bat_voltage < battery_min_voltage) //comparing the battery voltage and the minimum battery voltage
  {
    digitalWrite(load_enable,LOW);        //We DISABLE the load if battery is undervoltage
  }
  else{
    digitalWrite(load_enable,HIGH);       //We ENABLE the load if battery charged
  }
  ///////////////////////////FLOAT///////////////////////////
  ///////////////////////////////////////////////////////////
 // this part snippet checks if the battery is in FLOAT mode and switches to BULK mode if either the battery voltage is below the minimum voltage threshold or the solar current exceeds the maximum current limit allowed in the FLOAT mode.
    if(mode == FLOAT){
    if(bat_voltage < float_voltage_min) //comparing the battery voltage and the minimum voltage in float stage
    {
      mode = BULK;  
    }
    else{
      if(solar_current > float_max_current) //comparing the solar current and the maximum current in float stage
      {    //If we exceed max current value, we change mode
        mode = BULK;  
      }//End if > 
      else{
        if(bat_voltage > float_voltage) //comparing the battery voltage and the voltage in float stage
        {
          pwm_value--; //decrement the pwm value if the condition is true
          pwm_value = constrain(pwm_value,0,254);
        }
        else {
          pwm_value++; // increment the pwm if the condition is false
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
    // the charging mode is set to "BULK", the program adjusts the PWM duty cycle based on the difference between the solar current and the charging current.By adjusting the PWM duty cycle in this way, the program can regulate the charging current to the battery and prevent overcharging or undercharging.  
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
         delay(100);// Wait for some time before next iteration
}
  Vin= solar_voltage;
  Vout= bat_voltage;
  Curr= solar_current;        
}
//the function returns the average voltage value as a floating-point value. The unit of the voltage value returned depends on the voltage reference used by the ADC and the calibration factor applied.
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
//The function get_battery_voltage is used to measure the battery voltage by taking multiple samples and calculating the average value. 
float get_battery_voltage(int n_samples)
{
  float voltage = 0;
  for(int i=0; i < n_samples; i++)
  {
    voltage += analogRead(battery_voltage_in) * 0.0046003417968 ;
    delay(1);      
  }
  voltage = voltage/n_samples-0.2;//A constant offset of -0.2 is subtracted from the average voltage value.
  if(voltage < 0){voltage = 0;}
  return(voltage);
}
//The get_solar_current function assumes that the INA219 sensor object (ina219) is properly initialized and connected to the solar current source. It relies on the functionality provided by the INA219 library to accurately measure the current.
float get_solar_current(int n_samples)
{
  float current = 0;
  current = ina219.getCurrent_mA(); //getting current from ina219
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
