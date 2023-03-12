/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


// Thermocouple header
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include "Adafruit_MCP9600.h"

#define I2C_ADDRESS (0x67)

/* DEVICE CONFIGS */
#define MCP_I2C_ADDRESS (0x67)

Adafruit_MCP9600 mcp1;
Adafruit_MCP9600 mcp2;

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

// -- Setting up Fan things ---------

/* PIN DEFINITIONS */
#define fan_in1_pin 9  //used for fan PWM control
#define fan_in2_pin 10
#define fan_enable_pin 2
#define heat_enable_pin 3 // Pin to turn on off heat
int onboard_thermistor_pin = A0; //analog input pin used to read the onboard thermistor 



extern const float fan_min_duty;
extern const float fan_max_duty;

#define FAN_MIN_DUTY_CYCLE 0.85   //0.7 * 24V
#define FAN_MAX_DUTY_CYCLE 0.99999  //0.99 * 24V

extern int fanspeed;
extern int choice;

int fanspeed = 0;  //setting for the fan speed, updated when the user adjusts the fan speed slider
int counter = 0;
int onboardtemp = 0;
// ----------------------------------



/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  setup_fan();  //setup fan pins, leave fan off
  set_fan_speed(0);  //set fan speed to 100, but does NOT enable fan

  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  setup_thermocouples();


  ble.verbose(false);  // debug info is a little annoying after this point!

  Serial.println("Setup Fan");
  

  Serial.println("Setup Heat");
  disable_heat();
  setup_heat();


  Serial.println("Enable and Set initial Speed to 0");
  


  /* Wait for connection */
  // while (! ble.isConnected()) {
  //     delay(500);
  // }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
                        // wait for a second

  // Check for user input
  char inputs[BUFSIZE+1];

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  // if (len == 0) return;

  /* Got a packet! */
  printHex(packetbuffer, len);

  
  
  // Buttons
  if (len != 0) {
    if (packetbuffer[1] == 'F') {

      
      int add = 0;
      add = packetbuffer[2] - '0';

      Serial.print("add100: ");
      Serial.println(add);
      fanspeed = 100 * add;
      add = packetbuffer[3] - '0';
      Serial.print("add 10:  ");
      Serial.println(add);
      fanspeed += 10 * add;
      add = packetbuffer[4] - '0';
      Serial.print("add  0:   ");
      Serial.println(add);

      fanspeed += add;
      // Serial.print('100: '); Serial.println(packetbuffer[2]);
      // Serial.print(' 10: '); Serial.println(packetbuffer[3]);
      // Serial.print('  1: '); Serial.println(packetbuffer[4]);
      // Serial.print('Setting fanspeed'); Serial.println(fanspeed);
      set_fan_speed(fanspeed);
      
    } else if (packetbuffer[1] == 'B') {

      if (packetbuffer[2] == '0') {
        Serial.println("Disabling fan");
        disable_fan();
      } else if (packetbuffer[2] == '1') {
        Serial.println("Enabling fan");
        enable_fan();
      } else {
        Serial.print("packet buffer for B weird");
      };
    
    } else if (packetbuffer[1] == 'H') {

      if (packetbuffer[2] == '0') {
        Serial.println("Disabling Heat");
        disable_heat();
      } else if (packetbuffer[2] == '1') {
        Serial.println("Enabling Heat");
        enable_heat();
      } else {
        Serial.print("packet buffer for H weird");
      };
    
    };
  };

  
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);


    if (inputs[0] == 'F') {

      
      int add = 0;
      add = inputs[1] - '0';

      Serial.print("add100: ");
      Serial.println(add);
      fanspeed = 100 * add;
      add = inputs[2] - '0';
      Serial.print("add 10:  ");
      Serial.println(add);
      fanspeed += 10 * add;
      add = inputs[3] - '0';
      Serial.print("add  0:   ");
      Serial.println(add);

      fanspeed += add;
      // Serial.print('100: '); Serial.println(packetbuffer[2]);
      // Serial.print(' 10: '); Serial.println(packetbuffer[3]);
      // Serial.print('  1: '); Serial.println(packetbuffer[4]);
      // Serial.print('Setting fanspeed'); Serial.println(fanspeed);
      set_fan_speed(fanspeed);
      
    } else if (inputs[0] == 'B') {

      if (inputs[1] == '0') {
        Serial.println("Disabling fan");
        disable_fan();
      } else if (inputs[1] == '1') {
        Serial.println("Enabling fan");
        enable_fan();
      } else {
        Serial.print("INPUT for B weird");
      };
    
    } else if (inputs[0] == 'H') {

      if (inputs[1] == '0') {
        Serial.println("Disabling Heat");
        disable_heat();
      } else if (inputs[1] == '1') {
        Serial.println("Enabling Heat");
        enable_heat();
      } else {
        Serial.print("INPUT for H weird");
      };
    
    };

    // ble.print("");
    // ble.println(inputs);
    
    // check response stastus
    // if (! ble.waitForOK() ) {
    //   Serial.println(F("Failed to send?"));
    // }
  }

  delay(50);
  // Run blub
  // Serial.println("Probe 1");
  // Serial.println("---------------");
  // Serial.print("Hot Junction: "); Serial.println(mcp1.readThermocouple());
  // Serial.print("Cold Junction: "); Serial.println(mcp1.readAmbient());
  // Serial.print("ADC: "); Serial.print(mcp1.readADC() * 2); Serial.println(" uV");

  // Serial.println("Probe 2");
  // Serial.println("---------------");
  // Serial.print("Hot Junction: "); Serial.println(mcp2.readThermocouple());
  // Serial.print("Cold Junction: "); Serial.println(mcp2.readAmbient());
  // Serial.print("ADC: "); Serial.print(mcp2.readADC() * 2); Serial.println(" uV");
  onboardtemp = read_onboard_temp();
  Serial.println("---------------------------------------");
  Serial.print("Probe 1: "); Serial.print(mcp1.readThermocouple()); Serial.print(" Probe 2: "); Serial.println(mcp2.readThermocouple());
  Serial.print("Onboard: "); Serial.print(onboardtemp);
  
  uint8_t temp = 'T'; //mcp.readThermocouple();
  int T1 = mcp1.readThermocouple();
  int T2 = mcp2.readThermocouple();
  // ble.write(temp);

  int hundreds1 = (T1 % 1000) / 100; // 4
  int tens1 = (T1 % 100) / 10; // 6
  int ones1 = T1 % 10; // 2
  int hundreds2 = (T2 % 1000) / 100; // 4
  int tens2 = (T2 % 100) / 10; // 6
  int ones2 = T2 % 10; // 2
  
  uint8_t sendbuffer[8];
   
  sendbuffer[0] = '!';
  sendbuffer[1] = 'T';
  // sendbuffer[2] = mcp.readThermocouple();
  sendbuffer[2] = hundreds1;
  sendbuffer[3] = tens1;
  sendbuffer[4] = ones1;

  // sendbuffer[2] = mcp.readThermocouple();
  sendbuffer[5] = hundreds2;
  sendbuffer[6] = tens2;
  sendbuffer[7] = ones2;

  

  ble.write(sendbuffer, 8);
  
  // if (mcp.readThermocouple() > 25.0){
  //     digitalWrite(6, HIGH);  // turn the LED on (HIGH is the voltage level)
  // }
  // else {
  //     digitalWrite(6, LOW);  // turn the LED on (HIGH is the voltage level)
  // }


 
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}



void setup_fan() {

  pinMode(fan_in1_pin, OUTPUT);
  pinMode(fan_in2_pin, OUTPUT);
  digitalWrite(fan_enable_pin, 0);
  pinMode(fan_enable_pin, OUTPUT);
  TCCR1A = 0b10110000 | (TCCR1A & 0b00001111);  //configure the PWM lines to be complimentary for H-bridge driver


  return;
}

void set_fan_speed(int speed) {
  //speed is set from 0-100 pct of operating speed

  const float fan_min_duty = FAN_MIN_DUTY_CYCLE * 255;
  const float fan_max_duty = FAN_MAX_DUTY_CYCLE * 255;

  if (speed > 100)
    speed = 100;

  if (speed < 1)
    speed = 1;

  fanspeed = speed;  //update global variable


  float calc = float(speed) / 100 * (fan_max_duty - fan_min_duty) + fan_min_duty;
  int val = int(calc);
  Serial.println("speed  ");
  Serial.println("---- ");
  Serial.println(speed);
  Serial.println(calc);
  Serial.println(val);
  Serial.println("---- ");

  //write PWM registers directly to avoid issue with AnalogWrite(255) being treated as a DigitalWrite()
  OCR1A = val;
  OCR1B = val;

  return;
}

void enable_fan() {
  digitalWrite(fan_enable_pin, 1);
  return;
}

void disable_fan() {
  digitalWrite(fan_enable_pin, 0);
  return;
}

int is_fan_on() {
  return digitalRead(fan_enable_pin);
}


void setup_heat() {
  digitalWrite(heat_enable_pin, 0);
  pinMode(heat_enable_pin, OUTPUT);
  digitalWrite(heat_enable_pin, 0);
  return;
}


void enable_heat() {
  digitalWrite(heat_enable_pin, 1);
  return;
}

void disable_heat() {
   digitalWrite(heat_enable_pin, 0);
  return;
}


/* **********************************  TEMP READBACK  *********************************** */

//read the onboard temp from the thermistor mounted under the heater element 
int read_onboard_temp() {
  
  int Vo;
  const float R1 = 10000;
  const float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
  float logR2, R2, T;

  //Estimate  onboard temp using thermistor
  Vo = analogRead(onboard_thermistor_pin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;

  return T; 
}


void setup_thermocouples() {

  Serial.println("");
  Serial.println("");
  Serial.println("=================================================");
  Serial.println("=================================================");
  Serial.println("Setting up thermocouple 1");
  Serial.println("-------------------------------------------------");
  
  Serial.println("MCP9600 HW test");

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */
  if (! mcp1.begin(0x67)) {
      Serial.println("Sensor not found. Check wiring!");
      while (1);
  }

  Serial.println("Found MCP9600!");

  mcp1.setADCresolution(MCP9600_ADCRESOLUTION_18);
  Serial.print("ADC resolution set to ");
  switch (mcp1.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");
  

  mcp1.setThermocoupleType(MCP9600_TYPE_K);
  Serial.print("Thermocouple type set to ");
  switch (mcp1.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp1.setFilterCoefficient(3);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp1.getFilterCoefficient());

  mcp1.setAlertTemperature(1, 30);
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp1.getAlertTemperature(1));
  mcp1.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp1.enable(true);

  Serial.println("=================================================");
  Serial.println("=================================================");
  Serial.println("");
  

  Serial.println("");
  Serial.println("");
  Serial.println("=================================================");
  Serial.println("=================================================");
  Serial.println("Setting up thermocouple 2");
  Serial.println("-------------------------------------------------");
  
  if (! mcp2.begin(0x60)) {
      Serial.println("Sensor not found. Check wiring!");
      while (1);
  }

  mcp2.setADCresolution(MCP9600_ADCRESOLUTION_18);
  Serial.print("ADC resolution set to ");
  switch (mcp2.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;
  }
  Serial.println(" bits");


  mcp2.setThermocoupleType(MCP9600_TYPE_K);
  Serial.print("Thermocouple type set to ");
  switch (mcp2.getThermocoupleType()) {
    case MCP9600_TYPE_K:  Serial.print("K"); break;
    case MCP9600_TYPE_J:  Serial.print("J"); break;
    case MCP9600_TYPE_T:  Serial.print("T"); break;
    case MCP9600_TYPE_N:  Serial.print("N"); break;
    case MCP9600_TYPE_S:  Serial.print("S"); break;
    case MCP9600_TYPE_E:  Serial.print("E"); break;
    case MCP9600_TYPE_B:  Serial.print("B"); break;
    case MCP9600_TYPE_R:  Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp2.setFilterCoefficient(3);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp2.getFilterCoefficient());

  mcp2.setAlertTemperature(1, 30);
  Serial.print("Alert #1 temperature set to ");
  Serial.println(mcp2.getAlertTemperature(1));
  mcp2.configureAlert(1, true, true);  // alert 1 enabled, rising temp

  mcp2.enable(true);

  Serial.println("=================================================");
  Serial.println("=================================================");
  Serial.println("");
  

}


