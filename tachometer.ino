/*
 * Simple tachometer code for the ESP32 with arduino core 
 * Uses an Infra-Red proximity sensor (on pin 15) and
 * sends RPM out over a Bluetooth Serial connection 20.
 * 
 * It assumes that TWO transitions of the sensor equals ONE revolution. 
 * i.e. LOW-HI then HI-LOW (or vice-versa) equals one Rev.
 * It is sampling at 20 times per second. This should be fast enough 
 * to record upto 6,000 RPM. It could go a lot faster if necessary by increasing
 * the sample rate.
 * 
 * V0.3 - 6 Jan 2021 - RKW
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define IR_SENSOR_PIN 15
#define SAMPLE_RATE 20 //samples per second


const uint32_t samplePeriod = 1000000/SAMPLE_RATE; //in microseconds
const uint32_t sampleMillis = samplePeriod/1000;

uint32_t IRprevious = 0;//Records the previous InfraRed sensor state
uint32_t prevPulseTime = 0;//records millis() when new pulse
byte IRpulse = 0;//Records the number of sensor transitions
uint16_t sampleCtr = 0;
uint32_t averageRPM = 0;
uint32_t previousRPM = 0;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = millis();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Rev Counter");

  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every samplePeriod microseconds.
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, samplePeriod, true);

  // Start an alarm
  timerAlarmEnable(timer);
}

void loop() {
  byte IRstate = 0;
  uint32_t period = 0xFFFF;
  uint16_t rpm = 0;
  
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    uint32_t isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
#ifdef DEBUG
    Serial.print("onTimer no. ");
    Serial.print(isrCount);
    Serial.print(" at ");
    Serial.print(isrTime);
    Serial.println(" ms");
#endif
    IRstate = digitalRead(IR_SENSOR_PIN);
    if ( IRstate != IRprevious ) {//Sensor transition has occurred
      IRpulse++;
      IRprevious = IRstate;
      if (IRpulse > 1) { //two transitions = 1 rev
        IRpulse =0;
        //Calc period of revolution
        period = isrTime - prevPulseTime;//milliseconds
        prevPulseTime = isrTime;

      } 
    } else period += sampleMillis;//No new pulse this time around
    
    if (period) rpm = 60 * 1000 / period;
    //provide a bit of smoothing
    averageRPM = (previousRPM + rpm) / 2;
    previousRPM = rpm; 
       
    if (++sampleCtr > 3) { //send the result every 4th sample
      SerialBT.println(averageRPM);
      sampleCtr = 0;
    }
  }
}
