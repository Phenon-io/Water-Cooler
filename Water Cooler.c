#include <LiquidCrystal.h>
// for the clock
#include <RTClib.h>
#include <Stepper.h>
#include <DHT.h>
#include <time.h>

// PORTS
//===INPUT===
//b
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
volatile unsigned char* pin_b  = (unsigned char*) 0x23;
//e
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;
//k
volatile unsigned char* port_k = (unsigned char*) 0x108;
volatile unsigned char* ddr_k  = (unsigned char*) 0x107;
volatile unsigned char* pin_k  = (unsigned char*) 0x106;

//===OUTPUT===
//a
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a  = (unsigned char*) 0x21;
volatile unsigned char* pin_a  = (unsigned char*) 0x20;
//f
volatile unsigned char* port_f = (unsigned char*) 0x31;
volatile unsigned char* ddr_f  = (unsigned char*) 0x30;
volatile unsigned char* pin_f  = (unsigned char*) 0x2F;


#define RDA 0x80
#define TBE 0x20 

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0  = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//ISR
//volatile unsigned char *my_PCICR = (unsigned char *)0x67;
//volatile unsigned char *my_PCMSK1 = (unsigned char *)0x69;


// LCD pins <--> Arduino pins
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//===CLOCK===
RTC_DS1307 rtc;
DateTime now;

//===STEPPER===
const int stepsPerRevolution = 200;
Stepper myStepper(stepsPerRevolution, 6,7,8,9);
enum Vent{
  OPEN,
  CLOSED
};
Vent vent = OPEN;

//===DHT===
#define DHTPIN 13
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
const double threshold = 18.5;
// NOTES:
// 64f is about 17.7C, which is about the temp I keep the house at right now.

//Water Level
const unsigned int WATER_THRESHOLD = 1; //1/n level
const int W_DENOMINATOR = 256;  //n = W_DENOMINATOR/1024

enum State{
  DISABLED,
  IDLE,
  ERROR,
  RUNNING
};
State current_state = DISABLED;

/*
ISR(PCINT0_vect){
  Serial.println("Interrupt\n");
  current_state = IDLE;
}
*/

void setup() {  
  //stepper setup
  int rpms = 60;
  myStepper.setSpeed(rpms);

  dht.begin();

  lcd.begin(16,2);

  U0init(9600);
  adc_init();

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  now = rtc.now();

  //Port Initialization
  //INPUTS
  // Water Level Monitor
  *ddr_b &= ~(1 << 4); // Set PB4 as input
  *port_b |= (1 << 4); // Enable pull-up resistor on PB4
  // Humidity and Temp Sensor - USES LIBRARY
  // Buttons
  // START handled as ISR
  // STOP, VENT
  *ddr_k &= 0x79; //set 01111001 (7, 1, 2 pin)
  *port_k |= 0x86; // Enable pull-up resistor on PK1,2,7
  *ddr_e &= ~(1 << 1); // Set PE1 as input
  *port_e |= (1 << 1); // Enable pull-up resistor on PE1
  
  //*ddr_b &= ~(1 << 0); // Set PB0 as input
  //*port_b |= (1 << 0); // Enable pull-up resistor on PB0
  // Set PCINT1 as trigger - rising edge
  //*my_PCICR |= (1 << PCIE0);
  //*my_PCMSK1 |= (1 << PCINT0);
  //sei();
  attachInterrupt(digitalPinToInterrupt(20), interrupt, CHANGE);
  sei();
  //OUTPUTS
  // LED pins (fan motor is on 6)
  *ddr_a |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6);
  // fan motor pin (if I move it later)
  // *ddr_a |= (1 << 6);
  // STEPPER USES LIBRARY
  // LCD IS NOT HANDLED WITH PORTS/PINS, IT USES A LIBRARY

}
void interrupt(){
  Serial.println("interrupt\n");
  current_state = IDLE;
}
void loop() {
  //COMPONENT TESTS
  test_rtc();
  unsigned char buffer[50] = {'a'};
  unsigned char tmbuffer[9];
  const char* timestamp = rtc.now().timestamp(DateTime::TIMESTAMP_TIME).c_str();
  strcpy((char*)tmbuffer, timestamp);
  unsigned char* justN[2] = {'\n', '\0'};
  int stringLength = 0;
  
  
  int n = 0;
  //Note: this gets a lot cleaner if a function is made for transition messages
  switch (current_state) {
    case DISABLED:
      unsigned char disabled_message[] = "Transitioned to Disabled at ";
      sprintf(buffer, "%s%s%s", disabled_message, tmbuffer, justN);
      stringLength = strlen((char*)buffer);
      n = 0;
      while(n < stringLength) U0putchar(buffer[n++]);
      disabled();
      break;
    case IDLE:
      
      unsigned char idle_message[] = "Transitioned to Idle at ";
      sprintf(buffer, "%s%s%s", idle_message, tmbuffer, justN);
      stringLength = strlen((char*)buffer);
      n = 0;
      while(n < stringLength) U0putchar(buffer[n++]);
      idle();
      break;
    case RUNNING:
      unsigned char running_message[] = "Transitioned to Running at ";
      sprintf(buffer, "%s%s%s", running_message, tmbuffer, justN);
      stringLength = strlen((char*)buffer);
      n = 0;
      while(n < stringLength) U0putchar(buffer[n++]);
      running();
      break;
  }
  wait(10); //wait 10 seconds
}
void disabled(){
  Serial.println("made it here\n");
  if(digitalRead(20) == LOW){
    Serial.println("pin is low\n");
  }
  // this function does nothing but wait for an interupt
  led_update();
  while(1){
    if(current_state == IDLE) return;
  }
}
void idle(){
  led_update();
  int water_reading = 0;
  float water_level = 0;
  while(1){
    if(rtc.now() >= now+60) display();
    water_reading = adc_read() - '0';
    water_level = (water_reading * 5.0) / W_DENOMINATOR;
    Serial.println(water_level);
    if(*pin_k & (1 << 1)){
      current_state = DISABLED;
      return;
    }
    if(*pin_k & (1 << 2)){
      vent_update();
    }
    if(dht.readTemperature() > threshold){
      current_state = RUNNING;
      return;
    }
    if(water_level <= WATER_THRESHOLD){
      error("Water level is too low\n");
      return;
    } //water level lower than threshold 
  }
}
void error(char* error){
  unsigned char buffer[50] = {0};
  unsigned char tmbuffer[9];
  const char* timestamp = rtc.now().timestamp(DateTime::TIMESTAMP_TIME).c_str();
  strcpy((char*)tmbuffer, timestamp);
  unsigned char* justN[2] = {'\n', '\0'};
  unsigned char error_message[] = "Transitioned to Error at ";

  led_update();
  if(rtc.now() >= now+60) display();

  sprintf(buffer, "%s%s%s", error_message, tmbuffer, justN);
  int stringLength = strlen((char*)buffer);
  int n = 0;
  while(n < stringLength) U0putchar(buffer[n++]);
      
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.write(error);
  int water_reading = 0;
  float water_level = 0;
  while(1){
    water_reading = adc_read() - '0';
    water_level = (water_reading * 5.0) / W_DENOMINATOR;
    if((*pin_k & (1 << 1)) && water_level > WATER_THRESHOLD){
      current_state = DISABLED;
      return;
    }
    if(*pin_k & (1 << 2)){
      vent_update();
    }
    if(current_state == IDLE) return;
  }
}
void running(){
  led_update();
  int water_reading = 0;
  float water_level = 0;
  while(1){
    water_reading = adc_read() - '0';
    water_level = (water_reading * 5.0) / W_DENOMINATOR;
    if(rtc.now() >= now+60) display();
    if(*pin_k & (1 << 1)){
      current_state = DISABLED;
      return;
    }
    if(*pin_k & (1 << 2)){
      vent_update();
    }
    if (dht.readTemperature() <= threshold){
      current_state = IDLE;
      return;
    } 
    if(current_state == IDLE) return;
    if(water_level < WATER_THRESHOLD){
      error("Water level is too low\n");
      return;
    } //water level lower than threshold
  }
}
void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit  7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
   // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
   // setup the MUX Register
  *my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000; // set bit  6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
    // Set the input channel to the water level sensor pin (PH5)
  *my_ADMUX &= 0b11100000; // Clear channel selection bits
  *my_ADMUX |= 0b00000101; // Set channel 5 (PH5)
}

unsigned int adc_read() {
  // Start conversion
  *my_ADCSRA |= 0b10000000; // Set ADSC bit to start conversion
  // Wait for conversion complete
  while (*my_ADCSRA & 0b10000000);
  // Read the ADC result
  return *my_ADC_DATA;
}
void U0init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
void wait(int seconds) {
  DateTime start_time = rtc.now();
  while((start_time + seconds) <= rtc.now()); //Wait until seconds have passed
  return;
}
void test_rtc(){
  DateTime now = rtc.now();
  Serial.print("Date & Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(now.dayOfTheWeek());
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.println(now.second(), DEC);
}
void test_DHT(){
  int temperature = 0;
  int humidity = 0;

  // Check the results of the readings.
  // If the reading is successful, print the temperature and humidity values.
  // If there are errors, print the appropriate error messages.
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C\tHumidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
}
void display(){
  int temperature = dht.readTemperature();
  int humidity = dht.readHumidity();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(temperature, DEC);
  lcd.setCursor(0,1);
  lcd.print(humidity, DEC);
}
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}
void vent_update(){
  unsigned char buffer[50];
  unsigned char vent_state[7];
  if(vent == OPEN)vent = CLOSED;
  else vent = OPEN;
  unsigned char vent_message = "Vent state has changed to ";
  if(vent == OPEN) strcpy((char*)vent_state, "open\n");
  else strcpy((char*)vent_state, "closed\n");
  strcpy((char*)buffer, (const char*)vent_message);
  strncat((char*)buffer, (const char*)vent_state, sizeof(buffer) - strlen((const char*)buffer) - 1);
  int n = 0;
  while(n < strlen((const char*)buffer)) U0putchar(buffer[n++]);
}
void led_update(){
  *pin_a &= 0x00; //clear all LEDs
  //LEDs are under PA0,2,4,6
  //0 IDLE
  //2 DISABLED
  //4 ERROR
  //6 RUNNING
  switch(current_state){
    case DISABLED:
      *pin_a |= (1 << 2);
      break;
    case IDLE:
      *pin_a |= (1 << 0);
      break;
    case RUNNING:
      //Note: THIS WILL ALSO TURN ON THE FAN with current config
      *pin_a |= (1 << 6);
      break;
    case ERROR:
      *pin_a |= (1 << 4);
      break;
  }

}