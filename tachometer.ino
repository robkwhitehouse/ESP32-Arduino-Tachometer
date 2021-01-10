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
 * V0.5 - 7 Jan 2021 - RKW
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define IR_SENSOR_PIN 15
#define SAMPLE_RATE 400 //samples per second - should be fast enough for upto 6,000 RPM


const uint32_t samplePeriod = 1000000/SAMPLE_RATE; //in microseconds
const uint32_t sampleMillis = samplePeriod/1000;
uint32_t pulsePeriod = 0;
uint32_t ticks = 0;

uint32_t IRprevious = 0;//Records the previous InfraRed sensor state
uint32_t prevPulseTime = 0;//records millis() when new pulse
byte IRpulse = 0;//Records the number of sensor transitions
uint16_t sampleCtr = 0;
uint32_t averageRPM = 0;
uint32_t previousRPM = 0;
unsigned long prevBTsend = 0;

hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t isrCounter = 0;
volatile uint32_t lastIsrAt = 0;

class ringBuffer {
  private:
  byte index = 0;
  public:
  uint32_t _array[10];
  void add(uint32_t val) {
    _array[index] = val;
    if (index == 9) index = 0;
    else index++;
    }
  uint32_t average() {
    uint32_t sum = 0;
    for ( int i=0; i <10; i++ ) sum += _array[i];
    return( sum/10 );
  }
} smoother; 


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
  delay(300);
  Serial.println("Bluetooth Tacho V0.5");
}

void loop() {
  byte IRstate = 0;
  uint16_t rpm = 0;
  unsigned long now;
  
 
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
    ticks++; 
    IRstate = digitalRead(IR_SENSOR_PIN);
    if ( IRstate != IRprevious ) {//Sensor transition has occurred
      IRpulse++;
      IRprevious = IRstate;
      if (IRpulse > 1) { //two transitions = 1 rev
        IRpulse =0;
        //Calc period of revolution
        pulsePeriod = isrTime - prevPulseTime;//milliseconds
        prevPulseTime = isrTime;
   

      } 
    } else pulsePeriod += sampleMillis;//No new pulse this time around
    
    //Now calculate the RPM - need to check that period is not zero
    if (pulsePeriod) rpm = 60 * 1000 / pulsePeriod;
    smoother.add(rpm);
//    Serial.println(rpm);
    
    //Send the result over Bluetooth 5 times per second
    now=ticks*sampleMillis;//Time since start in millis
    if ( now  > (prevBTsend + 200) ) {
    //provide a bit of smoothing
      averageRPM = smoother.average();     
      prevBTsend = now;
      SerialBT.println(averageRPM);
      Serial.println(averageRPM);
    }
  }
}
