#include <Arduino.h>
#include "SeaRobLogger.h"
#include "SeaRobPotentiometer.h"
#include "SeaRob_INA219.h"
#include "SeaRob_SSD1306.h"
#include "SeaRob_DRV8871.h"

// NOTE: Requires the Arduino Nano Board

// Constants: PIN map
#define PIN_I2C_SDA                 A4 // Dedicated SDA Output Pin (nano only)
#define PIN_I2C_SCL                 A5 // Dedicated SCL Output Pin (nano only)

#define PIN_PF_1_INPUT              A1  // Analog input from potentiometer
#define PIN_PF_1_IN1                3   // Digital PWM Pin, output to DRV8871 motor driver
#define PIN_PF_1_IN2                5   // Digital PWM Pin, output to DRV8871 motor driver

#define PIN_PF_2_INPUT              A2  // Analog input from potentiometer
#define PIN_PF_2_IN1                6   // Digital PWM Pin, output to DRV8871 motor driver
#define PIN_PF_2_IN2                9   // Digital PWM Pin, output to DRV8871 motor driver

// Constants: OLED Display subsystem
#define OLED_I2C_ADDRESS            0x3C
#define OLED_DRAW_INTERVAL          40 // 25 frames per second maximum; slower if system clock runs slower


// Constants: Sensor subsystem
#define SENSOR1_I2C_ADDRESS         0x40
#define SENSOR2_I2C_ADDRESS         0x41
#define METER_READ_INTERVAL         3000


// Globals: Global Run state of the entire aplication.
unsigned long   startTime = 0; 
unsigned long   lastUpdateTime = 0;
String          buildName = String("cool-pf-control");
const char      buildTimestamp[] =  __DATE__ " " __TIME__;
const char      buildDatestamp[] = __DATE__;


// Globals: OLED Display subsystem, connected via I2C interface
boolean           useDisplay = true; 
SeaRob_SSD1306 *  seaDisplay = NULL;
unsigned long     lastDrawTime = 0;


// Globals: PowerFunctions Circuit 1
boolean                   usePowerSlot1 = true;
float                     currentPowerLevel1 = 0.0; // 0.0-1.0
SeaRobPotentiometer  *    inputKnob1 = NULL;
SeaRob_DRV8871 *          outputMotor1 = NULL;
SeaRob_INA219 *           sensor1 = NULL;
SeaRob_INA219::Sample     sensorSample1;

boolean                   usePowerSlot2 = true;
float                     currentPowerLevel2 = 1.0; // 0.0-1.0
SeaRobPotentiometer  *    inputKnob2 = NULL;
SeaRob_DRV8871 *          outputMotor2 = NULL;
SeaRob_INA219 *           sensor2 = NULL;
SeaRob_INA219::Sample     sensorSample2;

/*
 * Entrypoint: called once when the program first starts, just to initialize all the sub-components.
 */
void setup() {  
  
  // Record what time we started.
  startTime = millis();
  
  // Init the serial line; important for debug messages back to the Arduino Serial Monitor.
  Serial.begin(19200);
  bcloggerP(PSTR("")); // Skip line to seperate from last instance of the program.
  bcloggerP(PSTR("setup: \"%s\""), buildName.c_str());
  bcloggerP(PSTR("build: \"%s\""), buildTimestamp);

  // Init the OLED display
  if (useDisplay) {
    bcloggerP(PSTR("setup: OLED start..."));
    // TODO: convert to SeaRobObject with init
    seaDisplay = new SeaRob_SSD1306(buildTimestamp, OLED_I2C_ADDRESS);
    bcloggerP(PSTR("setup: OLED complete"));
  }

  if (true) {
    Wire.begin();
    Wire.setTimeout(3000);
  }

  if (usePowerSlot1) {
    bcloggerP(PSTR("setup: powerFunctions1 start..."));

    inputKnob1 = new SeaRobPotentiometer(PIN_PF_1_INPUT);
    inputKnob1->Init(startTime);
    outputMotor1 = new SeaRob_DRV8871(PIN_PF_1_IN1, PIN_PF_1_IN2);
    outputMotor1->Init(startTime);
    sensor1 = new SeaRob_INA219(SENSOR1_I2C_ADDRESS);
    sensor1->Init(startTime);

    // TODO: error handling if one of these subsystems fails.
    // Error
    // while (1) { delay(10); }
     
    bcloggerP(PSTR("setup: powerFunctions1 complete"));
  } 

  if (usePowerSlot2) {
    bcloggerP(PSTR("setup: powerFunctions2 start..."));

    //inputKnob1 = new SeaRobPotentiometer(PIN_PF_1_INPUT);
    //inputKnob1->Init(startTime);
    outputMotor2 = new SeaRob_DRV8871(PIN_PF_2_IN1, PIN_PF_2_IN2);
    outputMotor2->Init(startTime);
    sensor2 = new SeaRob_INA219(SENSOR2_I2C_ADDRESS);
    sensor2->Init(startTime);
     
    bcloggerP(PSTR("setup: powerFunctions2 complete"));
  } 

  // Init the rest of our internal state.
  bcloggerP(PSTR("setup: complete for \"%s\""), buildName.c_str()); 
}


/**
 * Main Loop: called over and over again as the robot runs.
 * Disopatches this event to all the subsystems.
 */
void loop() {

  // Get the current time.
  lastUpdateTime = millis();

  if (usePowerSlot1) {

      // Read desired powerlevel from the input knob.
      if (inputKnob1) {
        inputKnob1->ProcessLoop(lastUpdateTime);
        currentPowerLevel1 = inputKnob1->getValue();
      }

      // Update motor to latest power level and then let it run its state machine.
      if (outputMotor1) {
        outputMotor1->setDirection(MotorDirection_Forward);
        outputMotor1->setPower(currentPowerLevel1);
        outputMotor1->ProcessLoop(lastUpdateTime);
      }

      if (sensor1) {
        sensor1->ProcessLoop(lastUpdateTime);
        if ((sensorSample1.sampleTime + METER_READ_INTERVAL) < lastUpdateTime) {
          sensor1->getSample(&sensorSample1, lastUpdateTime);
        
          Serial.print(F("Bus Voltage:   ")); Serial.print(sensorSample1.busvoltage); Serial.println(F(" V"));
          Serial.print(F("Shunt Voltage: ")); Serial.print(sensorSample1.shuntvoltage); Serial.println(F(" mV"));
          Serial.print(F("Load Voltage:  ")); Serial.print(sensorSample1.loadvoltage); Serial.println(F(" V"));
          Serial.print(F("Current:       ")); Serial.print(sensorSample1.current_mA); Serial.println(F(" mA"));
          Serial.print(F("Power:         ")); Serial.print(sensorSample1.power_mW); Serial.println(F(" mW"));
          Serial.println(F("------------------------------------"));
        } 
      }
  }

  if (usePowerSlot2) {

      // Update motor to latest power level and then let it run its state machine.
      if (outputMotor2) {
        outputMotor2->setDirection(MotorDirection_Forward);
        outputMotor2->setPower(currentPowerLevel2);
        outputMotor2->ProcessLoop(lastUpdateTime);
      }
      
     if (sensor2) {
        sensor2->ProcessLoop(lastUpdateTime);
        if ((sensorSample2.sampleTime + METER_READ_INTERVAL) < lastUpdateTime) {
          sensor2->getSample(&sensorSample2, lastUpdateTime);
        
          Serial.print(F("Bus Voltage:   ")); Serial.print(sensorSample2.busvoltage); Serial.println(F(" V"));
          Serial.print(F("Shunt Voltage: ")); Serial.print(sensorSample2.shuntvoltage); Serial.println(F(" mV"));
          Serial.print(F("Load Voltage:  ")); Serial.print(sensorSample2.loadvoltage); Serial.println(F(" V"));
          Serial.print(F("Current:       ")); Serial.print(sensorSample2.current_mA); Serial.println(F(" mA"));
          Serial.print(F("Power:         ")); Serial.print(sensorSample2.power_mW); Serial.println(F(" mW"));
          Serial.println(F("------------------------------------"));
        } 
      }
  } 

  if (useDisplay) {
    if ((lastDrawTime + OLED_DRAW_INTERVAL) < lastUpdateTime) {  // Periodically draw to screen. 
      lastDrawTime = lastUpdateTime;
      Adafruit_SSD1306 &display = seaDisplay->getDisplayRef();
       
      // Reset the internal pixel-buffer, lets draw the entire screen.
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
   
      // Title with Fun little animation to prove that we are not locked up.
      seaDisplay->drawTitle("cool-pf-ctl", 32, 1);
      seaDisplay->drawCircleAnimation(lastUpdateTime, 123, 103, 0, true);
      seaDisplay->drawCircleAnimation(lastUpdateTime, 24, 4, 0, false);
      int linePos = 11;
      display.drawLine(0, linePos, 128, linePos, SSD1306_WHITE); 

      #define LINE_BUFFER_SIZE 50
      int colOffset = 0;
      
      // Power Functions Port 1
      if (usePowerSlot1) {
        display.setTextSize(2);
        display.setCursor(colOffset, 18);
        char l1Buffer[10];
        snprintf(l1Buffer, LINE_BUFFER_SIZE, "%4d", (int) sensorSample1.current_mA);
        display.println(l1Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 25);
        display.println(F("mA"));
      
        display.setTextSize(2);
        display.setCursor(colOffset, 37);
        char l2Buffer[10];
        snprintf(l2Buffer, LINE_BUFFER_SIZE, "%4d", (int) (sensorSample1.busvoltage * 1000.0f));
        display.println(l2Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 45);
        display.println(F("mV"));
      }

      if (usePowerSlot1) {
        display.setTextSize(1);
        display.setCursor(colOffset + 6, 56);
        char lYoBuffer[20];
        snprintf(lYoBuffer, LINE_BUFFER_SIZE, "pf-1: %d", (int) (currentPowerLevel1 * 100.0));
        display.println(lYoBuffer);
      }
    
      display.drawLine(63, 12, 63, 64, SSD1306_WHITE); // middle vertical line
    
      colOffset = 68;
      
      // Power Functions Port 2
      if (usePowerSlot2) {
        display.setTextSize(2);
        display.setCursor(colOffset, 18);
        char l3Buffer[10];
        snprintf(l3Buffer, LINE_BUFFER_SIZE, "%4d", (int) sensorSample2.current_mA);
        display.println(l3Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 25);
        display.println(F("mA"));

        display.setTextSize(2);
        display.setCursor(colOffset, 37);
        char l4Buffer[10];
        snprintf(l4Buffer, LINE_BUFFER_SIZE, "%4d", (int) (sensorSample2.busvoltage * 1000.0f));
        display.println(l4Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 45);
        display.println(F("mV")); 
      }

      if (usePowerSlot2) {
        display.setTextSize(1);
        display.setCursor(colOffset + 6, 56);
        char lYoBuffer[20];
        snprintf(lYoBuffer, LINE_BUFFER_SIZE, "pf-2: %d", (int) (currentPowerLevel2 * 100.0));
        display.println(lYoBuffer);
      } 
      
      // Push the buffer to the physical OLED
      display.display();
    }
  }
}
