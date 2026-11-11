#include <Arduino.h>
#include "SeaRobLogger.h"
#include "SeaRobSpringButton.h"
#include "SeaRob_INA219.h"
#include "SeaRob_SSD1306.h"

// NOTE: Requires the Arduino Nano Board

// Constants: PIN map
#define PIN_I2C_SDA                 A4 // Dedicated SDA Output Pin (nano only)
#define PIN_I2C_SCL                 A5 // Dedicated SCL Output Pin (nano only)

// Constants: LCD Display subsystem
#define OLED_I2C_ADDRESS            0x3C
#define OLED_DRAW_INTERVAL          40 // 25 frames per second maximum; slower if system clock runs slower

// Constants: Sensor subsystem
#define SENSOR_I2C_ADDRESS          0x41
#define METER_READ_INTERVAL         1000


// Globals: Global Run state of the entire aplication.
unsigned long   startTime = 0; 
unsigned long   lastUpdateTime = 0;
String          buildName = String("cool-pf-meter");
const char      buildTimestamp[] =  __DATE__ " " __TIME__;
const char      buildDatestamp[] = __DATE__;


// Globals: OLED Display subsystem, connected via I2C interface
boolean           useDisplay = true; 
SeaRob_SSD1306 *  seaDisplay = NULL;
unsigned long     lastDrawTime = 0;


// Globals: Current/Voltage Senesor subsystem.
boolean                 usePowerMeter = true;
SeaRob_INA219           *powerSensor = NULL;

//SeaRob_INA219          powerSensor(SENSOR_I2C_ADDRESS);
SeaRob_INA219::Sample   currentSample;


// Global: State Buttons
boolean               useStateButtons = false;
SeaRobSpringButton *  stateButton = NULL;


/*
 * Entrypoint: called once when the program first starts, just to initialize all the sub-components.
 */
void setup() {  
  
  // Record what time we started.
  startTime = millis();
  
  // Init the serial line; important for debug messages back to the Arduino Serial Monitor.
  Serial.begin(9600);
  bcloggerP(PSTR("")); // Skip line to seperate from last instance of the program.
  bcloggerP(PSTR("setup: \"%s\""), buildName.c_str());
  bcloggerP(PSTR("build: \"%s\""), buildTimestamp);

  if (useDisplay) {
    bcloggerP(PSTR("setup: OLED start..."));
    seaDisplay = new SeaRob_SSD1306(buildTimestamp, OLED_I2C_ADDRESS);
    bcloggerP(PSTR("setup: OLED complete"));
  }

  if (usePowerMeter) {
    bcloggerP(PSTR("setup: powerMeter start..."));
    powerSensor = new SeaRob_INA219(SENSOR_I2C_ADDRESS);
    powerSensor->Init(startTime);
    bcloggerP(PSTR("setup: powerMeter complete"));
  }

  if (useStateButtons) {
    //bclogger("setup: state buttons start...");
    //bclogger("setup: state buttons complete");
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

  if (useStateButtons) {
  }
  
  if (usePowerMeter) {
    // Take a periodic sample from the sensor. 
    if (((currentSample.sampleTime + METER_READ_INTERVAL) < lastUpdateTime)) {
     powerSensor->getSample(&currentSample, lastUpdateTime);
      
      Serial.print(F("Bus Voltage:   ")); Serial.print(currentSample.busvoltage); Serial.println(F(" V"));
      Serial.print(F("Shunt Voltage: ")); Serial.print(currentSample.shuntvoltage); Serial.println(F(" mV"));
      Serial.print(F("Load Voltage:  ")); Serial.print(currentSample.loadvoltage); Serial.println(F(" V"));
      Serial.print(F("Current:       ")); Serial.print(currentSample.current_mA); Serial.println(F(" mA"));
      Serial.print(F("Power:         ")); Serial.print(currentSample.power_mW); Serial.println(F(" mW"));
      Serial.println(F("------------------------------------")); 
    }
  }

  if (useDisplay) {
     if ((lastDrawTime + OLED_DRAW_INTERVAL) < lastUpdateTime) {
        lastDrawTime = lastUpdateTime;
        Adafruit_SSD1306 &display = seaDisplay->getDisplayRef();
      
        // Clear the internal buffer
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
   
        // Title with Fun little animation to prove that we are not locked up.
        seaDisplay->drawTitle("cool-pf-mtr", 32, 1);
        seaDisplay->drawCircleAnimation(lastUpdateTime, 123, 103, 0, true);
        seaDisplay->drawCircleAnimation(lastUpdateTime, 24, 4, 0, false);
        int linePos = 11;
        display.drawLine(0, linePos, 128, linePos, SSD1306_WHITE); 
      

     #define LINE_BUFFER_SIZE 50
        int colOffset = 0;
        
        // Power Functions Port 1
        display.setTextSize(2);
        display.setCursor(colOffset, 18);
        char l1Buffer[10];
        snprintf(l1Buffer, LINE_BUFFER_SIZE, "%4d", (int) currentSample.current_mA);
        display.println(l1Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 25);
        display.println(F("mA"));
    
        display.setTextSize(2);
        display.setCursor(colOffset, 37);
        char l2Buffer[10];
        snprintf(l2Buffer, LINE_BUFFER_SIZE, "%4d", (int) (currentSample.busvoltage * 1000.0f));
        display.println(l2Buffer);
        display.setTextSize(1);
        display.setCursor(colOffset + 48, 45);
        display.println(F("mV"));
    
        display.drawLine(63, 12, 63, 64, SSD1306_WHITE); // middke line
        
        // Push the buffer to the physical OLED
        display.display();
     }
  }
}
