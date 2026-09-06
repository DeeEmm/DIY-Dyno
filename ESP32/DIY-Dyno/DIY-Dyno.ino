/***********************************************************
 * @name The DIY Flow Bench project
 * @details Measure and display volumetric air flow using an ESP32 & Automotive MAF sensor
 * @link https://diyflowbench.com
 * @author DeeEmm aka Mick Percy deeemm@deeemm.com
 * 
 * @file DIY-Flow-Bench.ino
 * 
 * @brief define variables, configure hardware + main program loop
 * 
 * @remarks For more information please visit the WIKI on our GitHub project page: https://github.com/DIY-Flow-Bench/DIY-Flow-Bench/wiki
 * Or join our support forums: https://github.com/DIY-Flow-Bench/DIY-Flow-Bench/discussions
 * You can also visit our Facebook community: https://www.facebook.com/groups/diyflowbench/
 * 
 * @license Except where noted this project and all associated files are provided for use under the GNU GPL3 license:
 * https://github.com/DIY-Flow-Bench/DIY-Flow-Bench/blob/master/LICENSE
 * 
 * The standard project board is the ESP32DUINO used in conjunction with the DIY-Flow-Bnch shield
 * Other ESP32 based boards and custom shields can be made to work. 
 * You can define custom pin definitions to suit your project in the GUI
 * 
 * Default Temperature / Barometric Pressure / Relative Humidity uses a BME280 device connected via I2C
 * The generic I2C address for the BME280 is 0x77 / 0x76 (NOTE: BME NOT BMP!!)
 *
 * Default MAF / Reference Pressure / Pitot / Differential Pressure sensors use ADS1115 Analog to digital converter via I2C
 * The generic I2C address for the ADS is 0x48 but can also be set to 0x49 / 0x4A / 0x4B by pin linking at the device
 *
 * Default MAF unit recommended is the AUDI RS4 (BOSCH 0280218067) This will measure up to approx 860cfm
 * Other MAF sensors are supported by creation and inclusion of MAF transfer function coefficients within the code
 *
 * DEPENDENCIES
 * This program has a number of core libraries that must be available for it to work. Please refer to platformio.ini > lib_deps
 * 
 *
 **/

#include <Arduino.h>
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
// #include <MD_REncoder.h>
#include <math.h>
#include <Preferences.h>

#include "datahandler.h"
#include "constants.h"
#include "system.h"
#include "structs.h"
#include "mafdata.h"

#include "hardware.h" 
#include "sensors.h"
#include "calculations.h"
#include "webserver.h"
#include "publichtml.h" 
#include "messages.h"
#include "API.h"
#include "Wire.h"

// #include "ADS1X15.h" // DM 'Lite' Library (Disabled for loadcell/tacho config)

// #ifndef SET_LOOP_TASK_STACK_SIZE
// #define SET_LOOP_TASK_STACK_SIZE( LOOP_TASK_STACK_SIZE );
// #endif

SET_LOOP_TASK_STACK_SIZE( LOOP_TASK_STACK_SIZE );

// Initiate Structs
BenchSettings settings;
DeviceStatus status;
SensorData sensorVal;
ValveLiftData valveData;
Language language;
CalibrationData calVal;
Configuration config;
Pins pins;

// Initiate Classes
Preferences _prefs;
DataHandler _data;
API _api;
Calculations _calculations;
Hardware _hardware;
Messages _message;
Sensors _sensors;
Webserver _webserver;
PublicHTML _public_html;

// Initiate Variables
TaskHandle_t sensorDataTask = NULL;
TaskHandle_t enviroDataTask = NULL;
// portMUX_TYPE mmux = portMUX_INITIALIZER_UNLOCKED;

// Tachometer speed and pulse variables
volatile uint32_t speedPulseCount = 0;
volatile uint32_t lastPulseTime = 0;
volatile uint32_t pulsePeriodUs = 0;

void IRAM_ATTR speedPulseISR() {
  uint32_t now = micros();
  uint32_t period = now - lastPulseTime;
  if (period > 1000) { // simple 1ms debounce (max 60k RPM)
    pulsePeriodUs = period;
    lastPulseTime = now;
    speedPulseCount++;
  }
}

double getTachoRPM() {
  extern struct Pins pins;
  if (pins.SPEED_SENS < 0) return 0.0;
  
  uint32_t now = micros();
  // If no pulses in the last 1.5 seconds, we have stopped
  if (now - lastPulseTime > 1500000) {
    return 0.0;
  }
  
  if (pulsePeriodUs == 0) return 0.0;
  
  double rpm = 60000000.0 / pulsePeriodUs;
  return rpm;
}

// HX711 Bit banged Load Cell reader
long lastLoadcellRaw = 0;

long getRawLoadcell(int dout_pin, int sck_pin) {
  if (dout_pin < 0 || sck_pin < 0) return 0;
  
  // Wait up to 5ms for DOUT to go LOW (data ready)
  int waitCount = 0;
  while (digitalRead(dout_pin) == HIGH) {
    delayMicroseconds(50);
    waitCount++;
    if (waitCount > 100) { // 5ms timeout
      return lastLoadcellRaw; 
    }
  }
  
  unsigned long val = 0;
  portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
  taskENTER_CRITICAL(&myMutex);
  
  for (int i = 0; i < 24; i++) {
    digitalWrite(sck_pin, HIGH);
    delayMicroseconds(1);
    val = val << 1;
    if (digitalRead(dout_pin) == HIGH) {
      val++;
    }
    digitalWrite(sck_pin, LOW);
    delayMicroseconds(1);
  }
  
  // 25th pulse for Channel A Gain 128
  digitalWrite(sck_pin, HIGH);
  delayMicroseconds(1);
  digitalWrite(sck_pin, LOW);
  delayMicroseconds(1);
  
  taskEXIT_CRITICAL(&myMutex);
  
  if (val & 0x800000) {
    val |= 0xFF000000;
  }
  
  lastLoadcellRaw = (long)val;
  return lastLoadcellRaw;
}

// Custom sign function
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

// char charDataJSON[256];
String jsonString;

// set up task timers to measure task frequency
int adcStartTime =  micros();
int bmeStartTime =  micros();
int loopStartTime = micros();

int runTask = SSE_TASK;
int adcTaskCount = 0;

/***********************************************************
 * @brief TASK: Get bench sensor data (Load cell, Tacho, corrections)
 * @struct sensorVal global struct containing sensor values
 * */	
  void TASKgetSensorData( void * parameter ){

  extern struct DeviceStatus status;
  extern struct SensorData sensorVal;
  extern struct CalibrationData calVal;
  extern struct BenchSettings settings;
  extern struct Configuration config;
  extern struct Pins pins;
  
  Sensors _sensors;
  Hardware _hardware;

  for( ;; ) { // Infinite loop

      // Can we run??
      if (runTask == ADC_TASK) {

        // Set / reset scan timers
        status.adcScanTime = (micros() - adcStartTime); // how long since we started the timer? 
        adcStartTime = micros(); // start the timer
        status.bmeScanCountAverage = (status.bmeScanAlpha * status.bmeScanCount) + (1.0 - status.bmeScanAlpha) * status.bmeScanCountAverage;  // calculate Exponential moving average
        status.bmeScanCount = 1; // reset to 1 so first scan value in GUI is valid
        status.adcScanCount += 1;
        
        // Get reference voltages
        sensorVal.VCC_5V_BUS = _hardware.get5vSupplyVolts();
        sensorVal.VCC_3V3_BUS = _hardware.get3v3SupplyVolts();

        // 1. Get RPM from digital input (Swirl / Engine Speed)
        double rpm = getTachoRPM();
        if (rpm < 0.0) rpm = 0.0;
        sensorVal.Swirl = rpm;

        // 2. Get Load Force from HX711 (Force)
        double force_kg = 0.0;
        if (pins.LC_DOUT > -1 && pins.LC_SCK > -1) {
          long raw_load = getRawLoadcell(pins.LC_DOUT, pins.LC_SCK);
          double lc_scale = config.dMAF_MV_TRIM;
          if (lc_scale == 0.0) lc_scale = 10000.0; // avoid division by zero
          
          force_kg = (double)(raw_load - calVal.flow_offset) / lc_scale;
        }
        sensorVal.LoadForceKG = force_kg;

        // 3. Calculate Torque (Nm) = Force (kg) * 9.80665 * arm_length_meters
        double arm_length = config.dPREF_MV_TRIM;
        if (arm_length == 0.0) arm_length = 0.150; // default 150mm arm
        double torque_nm = force_kg * 9.80665 * arm_length;
        if (torque_nm < 0.0) torque_nm = 0.0;
        sensorVal.TorqueNM = torque_nm;

        // 4. Calculate HP = Torque (lb-ft) * RPM / 5252
        double torque_lb_ft = torque_nm * 0.737562149;
        double hp = (torque_lb_ft * rpm) / 5252.0;
        if (hp < 0.0) hp = 0.0;
        sensorVal.PowerHP = hp;

        // 5. SAE Atmospheric Corrections J1349
        double temp_c = sensorVal.TempDegC;
        double baro_hpa = sensorVal.BaroHPA;
        double rel_h = sensorVal.RelH;
        
        double p_vapor = 0.0;
        if (rel_h > 0.0 && temp_c > -40.0) {
          p_vapor = (rel_h / 100.0) * 6.11 * pow(10.0, (7.5 * temp_c) / (237.3 + temp_c));
        }
        double p_dry = baro_hpa - p_vapor;
        if (p_dry < 500.0) p_dry = 1000.0;
        
        double cf = 1.18 * (990.0 / p_dry) * sqrt((temp_c + 273.15) / 298.15) - 0.18;
        if (cf < 0.8) cf = 0.8;
        if (cf > 1.25) cf = 1.25;
        
        sensorVal.TorqueCorrectedNM = torque_nm * cf; // Corrected Torque
        sensorVal.PowerHPCorrected = hp * cf;       // Corrected Horsepower

        // Backwards compatibility legacy assignments
        sensorVal.FDiff = torque_nm;

      adcTaskCount += 1;
      runTask = SSE_TASK;
    }
    vTaskDelay( VTASK_DELAY_ADC );  // mSec delay to prevent Watch Dog Timer (WDT) triggering and yield if required
  }
}



/***********************************************************
 * @brief TASK: Get environental sensor data (BME280 - Temp/Baro/RelH)
 * @struct sensorVal global struct containing sensor values
 * @struct status global struct containing system status values
 * @remarks Interrogates BME280 and saves sensor data to struct
 * */	
void TASKgetEnviroData( void * parameter ){

  extern struct DeviceStatus status;
  extern struct SensorData sensorVal;
  extern struct Configuration config;

  Calculations _calculations;

  for( ;; ) { // Infinite loop

    // Can we run ??
    if (runTask == BME_TASK) {  

        // Set / reset scan timers
        status.bmeScanTime = (micros() - bmeStartTime); // how long since we started the timer? 
        bmeStartTime = micros(); // start the timer
        status.adcScanCountAverage = (status.adcScanAlpha * status.adcScanCount) + (1.0 - status.adcScanAlpha) * status.adcScanCountAverage;   // calculate Exponential moving average     
        status.adcScanCount = 1;
        status.bmeScanCount += 1;
        
        // Get temp sensor data
        sensorVal.TempDegC = _sensors.getTempValue();        
        sensorVal.TempDegF = _calculations.convertTemperature(_sensors.getTempValue(), DEGF);

        // Get baro sensor data
        sensorVal.BaroHPA = _sensors.getBaroValue();
        sensorVal.BaroPA = sensorVal.BaroHPA * 100.00F;
        sensorVal.BaroKPA = sensorVal.BaroPA * 0.001F;

        // Get humidity sensor data
        sensorVal.RelH = _sensors.getRelHValue();

      runTask = SSE_TASK;
    }
    vTaskDelay( VTASK_DELAY_BME ); // mSec delay to prevent Watch Dog Timer (WDT) triggering and yield if required
	}
}





/***********************************************************
 * @brief Default setup function
 * @details Initialises system and sets up core tasks
 * @note We can assign tasks to specific cores if required (currently disabled)
 ***/
void setup(void) {

  extern struct Pins pins;
  extern struct Configuration config;

  #ifdef VERBOSE
    settings.verbose_print_mode = true; 
  #endif

  // REVIEW
  // set message queue length
  // xQueueCreate( 8, 1024);
  // xQueueCreate( 256, 2048);
  // xQueueCreate( 1024, 4096);
    
  // Initialise Data environment
  _data.begin();
  
  // Initialise Hardware
  _hardware.begin();

  // Initialise sensors
  _sensors.begin();

  // Configure tachometer/load cell pins and interrupt
  if (pins.SPEED_SENS > -1) {
    pinMode(pins.SPEED_SENS, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pins.SPEED_SENS), speedPulseISR, RISING);
    _message.serialPrintf("Attach speed interrupt on pin %d\n", pins.SPEED_SENS);
  }
  if (pins.LC_DOUT > -1 && pins.LC_SCK > -1) {
    pinMode(pins.LC_DOUT, INPUT_PULLUP);
    pinMode(pins.LC_SCK, OUTPUT);
    digitalWrite(pins.LC_SCK, LOW); // Reset HX711 normal state
    _message.serialPrintf("Initialise HX711 pins: DOUT=%d, SCK=%d\n", pins.LC_DOUT, pins.LC_SCK);
  }

  // Confirm default core - NOTE: setup() and loop() are automatically created on default core 
  uint8_t defaultCore = xPortGetCoreID();                   // This core (1)
  uint8_t secondaryCore = (defaultCore > 0 ? 0 : 1);        // Secondary core (0)

  #ifdef WEBSERVER_ENABLED  
    _webserver.begin();
  #endif

  xTaskCreatePinnedToCore(TASKgetSensorData, "GET_SENS_DATA", SENSOR_TASK_MEM_STACK, NULL, 2, &sensorDataTask, secondaryCore); 
  // xTaskCreate(TASKgetSensorData, "GET_SENS_DATA", SENSOR_TASK_MEM_STACK, NULL, 2, &sensorDataTask); 

  xTaskCreatePinnedToCore(TASKgetEnviroData, "GET_ENVIRO_DATA", ENVIRO_TASK_MEM_STACK, NULL, 2, &enviroDataTask, secondaryCore); 
  // xTaskCreate(TASKgetEnviroData, "GET_ENVIRO_DATA", ENVIRO_TASK_MEM_STACK, NULL, 2, &enviroDataTask); 

  if (config.bSWIRL_ENBLD){
    // TODO #227
    // MD_REncoder Encoder = MD_REncoder(SWIRL_ENCODER_A, SWIRL_ENCODER_B);
  }

  // Report free stack and heap to serial monitor
  _message.serialPrintf("Free Stack: EnviroTask=%s  \n", _calculations.byteDecode(uxTaskGetStackHighWaterMark(enviroDataTask))); 
  _message.serialPrintf("Free Stack: SensorTask=%s  \n", _calculations.byteDecode(uxTaskGetStackHighWaterMark(sensorDataTask))); 
  _message.serialPrintf("Free Stack: LoopTask=%s    \n", _calculations.byteDecode(uxTaskGetStackHighWaterMark(NULL))); 
  _message.serialPrintf("Free Heap=%s / Max Allocated Heap=%s \n", _calculations.byteDecode(ESP.getFreeHeap()), _calculations.byteDecode(ESP.getMaxAllocHeap())); 

}




/***********************************************************
 * @brief MAIN LOOP
 * @details processes API requests 
 * @details pushes data to clients via SSE
 * 
 * @note Use non-breaking delays for throttling events
 ***/
void loop () {

  
  // Process API comms
  if (settings.api_enabled) {        
    if (millis() > status.apiPollTimer && runTask == SSE_TASK) {

        status.apiPollTimer = millis() + API_SCAN_DELAY_MS; 

        if (Serial.available() > 0) {
          status.serialData = Serial.read();
          _api.ParseMessage(status.serialData);
        }
    }                            
  }

  
  // TODO: PID Vac source Analog VFD control [if PREF not within limits]
  // _hardware.setVFDRef();
  // _hardware.setBleedValveRef();
  

  #ifdef WEBSERVER_ENABLED
    if (millis() > status.ssePollTimer && runTask == SSE_TASK) {      

      status.ssePollTimer = millis() + SSE_UPDATE_RATE; // Only reset timer when task executes
      
      // Build Server Side Events (SSE) data
      switch (status.GUIpage) {
        case INDEX_PAGE:{
          jsonString = _data.buildIndexSSEJsonData();
          break;
        }
        case MIMIC_PAGE:{
          jsonString = _data.buildMimicSSEJsonData();
          break;
        }
      }
      // Push SSE data to client
      _webserver.events->send(String(jsonString).c_str(),"JSON_DATA",millis()); 

      if (adcTaskCount > 2) {
        runTask = BME_TASK;
        adcTaskCount = 0;
      } else {
        runTask = ADC_TASK;
      }
    }
  #endif


  if (status.shouldReboot) {
    _message.serialPrintf("Rebooting...");
    delay(100);
    ESP.restart();
  }

  vTaskDelay( 1 );  //mSec delay to prevent Watch Dog Timer (WDT) triggering for empty task

  // Measure scan time
  status.loopScanTime = (micros() - loopStartTime); 
  loopStartTime = micros(); // start the timer
  
}
