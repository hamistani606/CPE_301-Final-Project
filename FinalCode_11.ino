/*
* Name: Heather Amistani, Daisy Madera, & Calvin Lu  
* Assignment: Final Project
* Class: CPE301 Spring 2023
* Due Date: 05/11/2024
* Purpose: A water cooler that displays temperature and humidity 
*/

#include <LiquidCrystal.h> // Include library for LCD
#include <DHT.h>           // Include library for DHT11 sensor
#include <Stepper.h>       // Include library for Stepper motor
#include <RTClib.h>        // Include library for Real Time Clock

unsigned char* PORT_C = (unsigned char*) 0x28;
unsigned char* DDR_C = (unsigned char*) 0x27;
volatile unsigned char* PIN_C = (unsigned char*) 0x26;

unsigned char* PORT_A = (unsigned char*) 0x22;
unsigned char* DDR_A = (unsigned char*) 0x21;
volatile unsigned char* PIN_A = (unsigned char*) 0x20;

unsigned char* PORT_E = (unsigned char*) 0x2E;
unsigned char* DDR_E = (unsigned char*) 0x2D;
volatile unsigned char* PIN_E = (unsigned char*) 0x2C;

unsigned char* PORT_G = (unsigned char*) 0x34;
unsigned char* DDR_G = (unsigned char*) 0x33;
volatile unsigned char* PIN_G = (unsigned char*) 0x32;

unsigned char* PORT_D = (unsigned char*) 0x2B;
unsigned char* DDR_D = (unsigned char*) 0x2A;
volatile unsigned char* PIN_D = (unsigned char*) 0x29;

unsigned char* PORT_H = (unsigned char*) 0x102;
unsigned char* DDR_H = (unsigned char*) 0x101;
volatile unsigned char* PIN_H = (unsigned char*) 0x100;


//serial
#define RDA 0x80
#define TBE 0x20 
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;

// State definitions
enum SystemState { DISABLED, IDLE, ERROR, RUNNING };
volatile SystemState currentState = DISABLED;

// RTC object
RTC_DS3231 rtc;

// Define the number of steps per revolution of the motor
const int stepsPerRev = 200;
Stepper myStepper = Stepper(stepsPerRev, 8, 10, 9, 11);
unsigned long motorStepStartTime = 0;
unsigned long motorStepDuration = 5000; // Duration for each motor step in milliseconds
bool motorStepInProgress = false;

//Fan Motor
int speedPin = 5; //PE3
int dir1 = 4;//PG5
int dir2 = 3;//PE5
int mSpeed = 90;

// Pin Definitions
#define START_BUTTON_PIN 2 // Interrupt pin
#define STOP_BUTTON_PIN 32
#define RESET_BUTTON_PIN 34
#define YELLOW_LED_PIN 22
#define GREEN_LED_PIN 26
#define BLUE_LED_PIN 24
#define RED_LED_PIN 28

#define WATER_SENSOR_PIN A5 //Pin for water level sensor
#define FAN_PIN 5 //Pin for fan motor control
#define TEMP_THRESHOLD 21.5 //Threshold in 
#define POTENTIOMETER_PIN A0 //Pin for potentiometer

DHT dht(7, DHT11); // DHT sensor

const int RS = 27, EN = 25, D4 = 37, D5 = 35, D6 = 33, D7 = 31;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7); //Pins for the LCD 

volatile unsigned long lastDebounceTime = 0;  // Last time the ISR was triggered
const unsigned long debounceDelay = 200;      // Debounce delay in milliseconds

int waterLevel = 0;

//Functions
void adc_init();
uint16_t adc_read(uint8_t channel);
void toggleState();
void testInterrupt();
void handleStartPress();
void handleStopPress();
void handleResetPress();
void updateTemperatureAndDisplay();
void updateFanMotor();
void handleMotorControl();


void setup() {
  U0init(9600);

  //Fan Motor
  *DDR_E |= 0x08; //set speedpin(PE3) to output
  *DDR_G |= 0x20;//set dir1(PG5) to output 
  *DDR_E |= 0x20;// set dir2(PE5) to output
  lcd.begin(16, 2); // Number of columns and rows for LCD
  *DDR_E |= 0x08; //set FAN_PIN (PE3) to output

  *DDR_E &=0xEF; //set START_BUTTON_PIN (PE4) to input 
  *PORT_E |= 0x10; //enable the pullup resistor 

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), handleStartPress, CHANGE);

  *DDR_C &= 0xDF; //set STOP_BUTTON_PIN (PC5) to input 
  *PORT_C |= 0x20; //enable the pullup resistor
  *DDR_A |= 0x01; //set YELLOW_LED_PIN (PA0) to output  
  *DDR_A |= 0x10; //set GREEN_LED_PIN (PA4) to output
  *DDR_A |= 0x04; //set BLUE_LED_PIN (PA2) to output
  *DDR_A |= 0x40; //set RED_LED_PIN (PA6) to output
  *DDR_H |=  0x08; //sensor input PH 3
  *PORT_A |=0x01; //set YELLOW_LED_PIN (PA0) High
  *PORT_A &= 0xEF; //clear GREEN_LED_PIN (PA4) to low 
  *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low
  *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
  *DDR_A |= 0x08;//set water sensor (PH3) to output

  dht.begin();
  lcd.begin(16, 2);
  lcd.print("System Ready");
  rtc.begin();
  adc_init();
}

void loop() {
  handleStopPress();
  handleResetPress();

  if (currentState == IDLE || currentState == RUNNING) {

    *PORT_H |=0x08; //set water sensor (PH3) High
    waterLevel = adc_read(5);

    updateTemperatureAndDisplay();
    handleMotorControl();
       
    if (waterLevel < 20) {
      currentState = ERROR;

      *PORT_A |= 0x40; //set RED_LED_PIN (PA6) High
      *PORT_A &= 0xFE; //clear YELLOW_LED_PIN (PA0) Low
      *PORT_A &= 0xEF; //clear GREEN_LED_PIN (PA4) Low
      *PORT_A &= 0xFB;//clear BLUE_LED_PIN (PA2) Low

      lcd.clear();
      lcd.print("ERROR: Low Water!");
    }

    *PORT_H &= 0xF7; //clear water sensor (PH3) low
  }

  // Delay to minimize bouncing issues without using delay() function
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
  }
}

//serial
void U0init(unsigned long U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUBRR0  = tbaud;
}



void adc_init() {
  // Set reference to AVcc
  ADMUX = (1 << REFS0);
  // Enable ADC and set prescaler to 128
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel) {
  // Select ADC channel with safety mask
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  // Start single conversion
  ADCSRA |= (1 << ADSC);
  // Wait for conversion to complete
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

void toggleState() {
  unsigned long currentTime = millis();
  // Check if the interrupt is triggered too soon after the last
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    // Toggle state only if the delay has passed
    if (currentState == DISABLED) {
      currentState = IDLE;
      *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) Low
      *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to High

    } else {
        currentState = DISABLED;
        *PORT_A |=0x01; //set YELLOW_LED_PIN (PA0) High
        *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to low 
      }

    // Update the last debounce time
    lastDebounceTime = currentTime;
  }
}

void testInterrupt() {
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {
    lastInterruptTime = interruptTime;
  }
}

void handleStartPress() {
  static unsigned long lastInterruptTime = 0; // Track the last time the button was pressed
  unsigned long currentTime = millis(); // Get current time

  if ((currentTime - lastInterruptTime > debounceDelay) && ((*PIN_E & 0x10) == LOW)) { // Check for debounce and button state
    currentState = IDLE; // Change state to IDLE

    *PORT_A |= 0x10; //set GREEN_LED_PIN (PA4) to High
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) Low
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low

    lastInterruptTime = currentTime; // Update last interrupt time
  }
    
}

void handleStopPress() {
  static unsigned long lastInterruptTime = 0; // Track the last time the button was pressed
  unsigned long currentTime = millis(); // Get current time

  bool temp = (*PIN_C & 0x20); //read STOP_BUTTON_PIN (PC5)

  if ((currentTime - lastInterruptTime > debounceDelay) && ((*PIN_C & 0x20) == LOW)) { // Check for debounce and button state
    currentState = DISABLED; // Change state to IDLE

    *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to low
    *PORT_A |= 0x01;//set YELLOW_LED_PIN (PA0) to High
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low

    lastInterruptTime = currentTime; // Update last interrupt time
    lcd.clear();
    lcd.print("System Ready!");
  }
}

void handleResetPress() {
  static unsigned long lastInterruptTime = 0; // Track the last time the button was pressed
  unsigned long currentTime = millis(); // Get current time
  bool temp = (*PIN_C & 0x20);

  if ((currentTime - lastInterruptTime > debounceDelay) && ((*PIN_C & 0x08) == LOW && currentState == ERROR)) { // Check for debounce and button state
    currentState = IDLE; // Change state to IDLE

    *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to High
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) Low
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
    *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low
  

    // Log the time
    DateTime now = rtc.now();
    lastInterruptTime = currentTime; // Update last interrupt time
  }
}

void updateTemperatureAndDisplay(){
  static unsigned long previousFanToggleTime = 0; //Variable to stor the previous fan toggle time
  static unsigned long fanToggleInterval = 0; //Variable to stor the fan toggle interval
  static bool fanOn = false; //Flag to track the fan state
  static unsigned long fanStartTime = 0; // Variable to store the start time of fan operation
  const unsigned long fanDuration = 5000; // Fan operation duration in milliseconds

  float temperature = dht.readTemperature(); //Celsius
  float humidity = dht.readHumidity();

  //Display values on the LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");

  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print(" %");

      // Check if state should transition to IDLE or RUNNING
  if(currentState == IDLE && temperature > TEMP_THRESHOLD) {
    currentState = RUNNING;

    *PORT_A &= 0xEF;//clear GREEN_LED_PIN (PA4) to Low
    *PORT_A |=0x04; //set BLUE_LED_PIN (PA2) High
    *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
    *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) Low

    //Start Fan
    fanOn = true;
    *PORT_G &= 0xDF;//clear dir1 (PG5) to low
    //digitalWrite(dir1, LOW);

    *PORT_E |= 0x20;//set dir2 (PE5) to high
    //digitalWrite(dir2, HIGH);

  } else if (currentState == RUNNING && temperature <= TEMP_THRESHOLD) {
      currentState = IDLE;

      *PORT_A &=0xFB; //clear BLUE_LED_PIN (PA2) Low
      *PORT_A &=0xBF; //clear RED_LED_PIN (PA6) Low
      *PORT_A &=0xFE; //clear YELLOW_LED_PIN (PA0) Low
      *PORT_A |= 0x10;//set GREEN_LED_PIN (PA4) to High

      //Stop Fan 
      fanOn = false;
  }

  if(fanOn){
    *PORT_E |= 0x08;//set speedPin (PE3) to high
    //digitalWrite(speedPin, HIGH);
  } else{
    *PORT_E &= 0xF7;//clear speedPin (PE3) to low
    //digitalWrite(speedPin, LOW);
  }

}

void handleMotorControl() {
  if (currentState != DISABLED) { // Check if the system is not disabled
    if (!motorStepInProgress) {
      int potentiometerValue = adc_read(0); // Read the potentiometer value
      int motorSpeed = map(potentiometerValue, 0, 1023, 1, 10); // Map potentiometer value to motor speed
      myStepper.setSpeed(motorSpeed); // Set stepper motor speed based on potentiometer value
      myStepper.step(stepsPerRev); // Rotate stepper motor
      myStepper.step(-stepsPerRev); // Rotate stepper motor

      // Log the time
      DateTime now = rtc.now();

      motorStepStartTime = millis();
      motorStepInProgress = true;
    } else {
      unsigned long currentTime = millis();
      if (currentTime - motorStepStartTime >= motorStepDuration) {
        int potentiometerValue = adc_read(0); // Read the potentiometer value
        int motorSpeed = map(potentiometerValue, 0, 1023, 1, 10); // Map potentiometer value to motor speed

        myStepper.setSpeed(motorSpeed); // Set stepper motor speed based on potentiometer value
        myStepper.step(stepsPerRev); // Keep rotating stepper motor in the same direction
        myStepper.step(-stepsPerRev); // Rotate stepper motor
        
        // Log the time
        DateTime now = rtc.now();

        motorStepStartTime = currentTime;
      }
    }
  } else {
    myStepper.setSpeed(0); // Stop the motor if the system is disabled
  }
}