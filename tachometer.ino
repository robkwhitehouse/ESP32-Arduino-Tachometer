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
 * V1.0 - 17 Jan 2021 - R K Whitehouse
 */
#define VERSION 1.0
//#define DEBUG 1
#define DEBUG(X)Serial.println(X);
#include "BluetoothSerial.h"
#include <EEPROM.h>


BluetoothSerial SerialBT;

#define IR_SENSOR_PIN 15
#define SAMPLE_RATE 200 //samples per second - should be fast enough for up to 6,000 RPM


const uint32_t samplePeriod = 1000000/SAMPLE_RATE; //in microseconds
const uint32_t sampleMillis = samplePeriod/1000;
unsigned long pulsePeriod = 0;
uint32_t ticks = 0;

uint32_t IRprevious = 0;//Records the previous InfraRed sensor state
unsigned long prevPulseTime = 0;//records micros() when new pulse
byte IRpulse = 0;//Records the number of sensor transitions
uint16_t sampleCtr = 0;
uint32_t averageRPM = 0;
uint32_t previousRPM = 0;
unsigned long prevBTsend = 0;
String calString("");
float calibFactor = 1.0;
unsigned EEPROMindex = 0;
String ident = "Tacho";


hw_timer_t * timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long isrCounter = 0;
volatile unsigned long lastIsrAt = 0;

/* 
 *  A ring buffer datatype - needed because we are generating the RPM data
 * at a much higher frequencey than we are sending out. So this buffer keeps  
 * a rolling sequence of the most recently sampled data.
 * 
 */

class RingBuffer {

  public:
    //Method to add a value to the buffer
    void add(uint32_t val) {
      _array[index] = val;
      if (index == maxIndex) index = 0;
      else index++;
    }
    //Method to compute the average of all the stored values
    unsigned average() {
      unsigned sum;
      sum = 0;
      for ( int i=0; i <= maxIndex; i++ ) sum += _array[i];
      return( sum /(maxIndex + 1) );
    }
    //Constructor - has a size parameter for the buffer size
    RingBuffer(unsigned _size) {
      maxIndex = _size - 1;
      for ( int i=0; i <= maxIndex; i++ ) _array[i] = 0;
    }
    
  private:
    uint32_t _array[200];//Kludge! to allocate memory. max size of buffer is 200  
    byte index = 0;
    unsigned maxIndex;
};

RingBuffer smoother(60); 

/*
 * Here is the interrupt Servide Routine - called by  TIMER0 every samplePeriod
 * microseconds
 */

void IRAM_ATTR onTimer(){
  // Increment the counter and set the time of ISR
  portENTER_CRITICAL_ISR(&timerMux);
  isrCounter++;
  lastIsrAt = micros();
  portEXIT_CRITICAL_ISR(&timerMux);
  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
  // It is safe to use digitalRead/Write here if you want to toggle an output
}

void setup() {

  Serial.begin(250000);
  delay(2000);
  Serial.print("Bluetooth Tacho Version ");
  Serial.println(VERSION);
  if (!EEPROM.begin(100)) {
    Serial.println("Error: could not initialise EEPROM segment");
    delay(5000);
  }

  SerialBT.begin("Rev Counter");

  delay(1000);
  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
  // info).
  timer = timerBegin(0, 80, true);
  Serial.println("Past timerBegin()");
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every samplePeriod microseconds.
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, samplePeriod, true);

  // Start an alarm
  timerAlarmEnable(timer);

  //Check to see if we have previously saved calibration data in EEPROM
  String existing_id;
  int n = EPreadString(EEPROMindex, existing_id);
  Serial.print("Reading existing ID -- ");
  if (n) Serial.println(existing_id);
  else Serial.println("None found");
  if ( existing_id.compareTo(ident) == 0 ) { 
    //retrieve calibration data from EEPROM
    EEPROMindex += sizeof(ident);
    String cf;
    n = EPreadString(EEPROMindex,cf);
    if (n) calibFactor = cf.toFloat(); 
  }

  Serial.print(" Calibration =  ");
  Serial.println(calibFactor);
}

void loop() {
  byte IRstate = 0;
  uint16_t rpm = 0;
  unsigned long now;


  //Calibration - read a factor from Bluetooth terminal - if available. 

  if (SerialBT.available() > 0) {
    char c=SerialBT.read();
    if(c=='c') {
        calString = "";
        while ( SerialBT.available() > 0 ) {
          c = SerialBT.read();
          if (c == '\n' || c == '\r' ) break;
          calString += c;
        }
        if ( calString.toFloat() != 0 ) calibFactor = calString.toFloat();
        Serial.println(calibFactor);
        SerialBT.println(calibFactor);

        //Save the calibration factor to EEPROM (actually an area of flash)
        int n = EPwriteString(0,ident);
        n += EPwriteString(sizeof(ident),calString);
        Serial.print(n);
        Serial.println(" bytes written to EEPROM");
        
    }
  }
 
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE){
    unsigned long isrCount = 0, isrTime = 0;
    // Read the interrupt count and time
    portENTER_CRITICAL(&timerMux);
    isrCount = isrCounter;
    isrTime = lastIsrAt;
    portEXIT_CRITICAL(&timerMux);
    // Print it
#ifdef DEBUG1
    Serial.print(" at ");
    Serial.print(isrTime);
    Serial.println(" us");
#endif
    ticks++; 
    IRstate = digitalRead(IR_SENSOR_PIN);
    if ( IRstate != IRprevious ) {//Sensor transition has occurred
      IRpulse++;
      IRprevious = IRstate;
      if (IRpulse > 1) { //two transitions = 1 rev
        IRpulse =0;
        //Calc period of revolution
        pulsePeriod = isrTime - prevPulseTime;//microseconds
        prevPulseTime = isrTime;
      } else pulsePeriod += samplePeriod;//Only one transition 
    } else pulsePeriod += samplePeriod;//No new pulse this time around
    
    //Now calculate the RPM - need to check that period is not zero
    if (pulsePeriod) rpm = (60 * 1000000) / pulsePeriod;
    smoother.add(rpm);
   
    //Send the result over Bluetooth 5 times per second
    now=ticks*sampleMillis;//Time since start in millis
    if ( now  > (prevBTsend + 200) ) {
    //provide a bit of smoothing 
      averageRPM = smoother.average();       
      prevBTsend = now;
      SerialBT.println(averageRPM * calibFactor);
#ifdef DEBUG
      Serial.println(averageRPM * calibFactor);
#endif
    }
  }
}

int EPwriteString(char add,String data)
{
  int _size = data.length();
  int i;
  for(i=0;i<_size;i++)
  {
    EEPROM.write(add+i,data[i]);
  }
  EEPROM.write(add+_size,'\0');   //Add termination null character for String Data
  EEPROM.commit();
  delay(200);
  return(_size);
}

int EPreadString(unsigned add, String data)
{
  int i;
  int len=0;
  unsigned char k;

  len=0;
  k=EEPROM.read(add);
  while(k != '\0' && len<500)   //Read until null character
  {    
    data += k;
    k=EEPROM.read(add+len);
    len++;
  }
  data += '\0';
  return(len);
}
