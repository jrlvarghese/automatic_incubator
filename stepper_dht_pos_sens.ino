#include <Wire.h>
#include <LiquidCrystal_SR.h>
#include "DHT.h"
#include "RTClib.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Stepper.h>

// Defining LCD and Pins for interfacing.
LiquidCrystal_SR lcd(5,7,6); 
//LiquidCrystal_SR lcd(6, 5, 9); // Pin 6 - Data Enable/ SER, Pin 5 - Clock/SCL, Pin 9 -SCK

// Define DHT pins and initialize DHT sensor
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Define RTC Module
RTC_DS1307 RTC;

// DS18B20 temperature module connected to pin 8
#define DS18B20_PIN 8

// Define number of steps per rotation:
const int stepsPerRevolution = 2048;
// Wiring:
// Pin 10 to IN1 on the ULN2003 driver
// Pin 11 to IN2 on the ULN2003 driver
// Pin 12 to IN3 on the ULN2003 driver
// Pin 13 to IN4 on the ULN2003 driver
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 10, 12, 11, 13);

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

// Pin 9 temp control output 
#define heatPin 9

// Pin 8 DS18B20 sensor

// Analog pin A4, A5 for RTC

// Pin 2 for rotary encoder push switch
const int menuPin = 2;
volatile bool buttonState = false;
bool prevButtonState = true;
volatile unsigned long lastInterruptTime = 0;

// Rotary encoder pins on physical pins 26, 25 ie A3, A2 respectively
#define rotPinA A3
#define rotPinB A2

// Analog pin A0, A1 for IR position sensor
#define pol_a_pin A0
#define pol_b_pin A1

uint8_t rot_interval = 6;
uint8_t temp = 0;
bool rot_trigger = false;
volatile bool pinALastState = false;
bool pinACurrState = false;
int count = 0;
unsigned int prevCount = 50;

bool pol_a = false;             // Analog pin A0
bool pol_b = false;             // Analog pin A1, for sensing position of lever
bool lever_status = true;

unsigned long start_time = 0;
unsigned long curr_time = 0;
unsigned long sense_interval = 0;
unsigned long disp_interval = 0;
unsigned long max_wait = 60;

float humidity = 0;
float dht_temperature = 0;
float temperature = 0;
float tempArray[5] = {0,0,0,0,0};
uint8_t avgCounter = 0;
float upperLimit = 0;
float lowerLimit = 0;
float prevReading = 0;
float avgTemp = 0;

bool menuState = false;
bool selected = false;
bool disp = false;
String menuOptions[6] = {"TEMPERATURE MAX", "TEMPERATURE MIN", "MOVE TRAY", "MOVING INTERVAL", "DATE & TIME", "EXIT"};

void setup(){
  pinMode(pol_a_pin, INPUT);
  pinMode(pol_b_pin, INPUT);
  pinMode(heatPin, OUTPUT);
  digitalWrite(heatPin, LOW);
  pinMode(rotPinA, INPUT);
  pinMode(rotPinB, INPUT);

  // Attach interrupt for the menu pin
  attachInterrupt(digitalPinToInterrupt(menuPin),isr, FALLING);

  // read state of rotary encoder pin
  pinALastState = digitalRead(rotPinA);

  // define default upper temperature limit and lower temp limit
  upperLimit = 37.5;
  lowerLimit = 37.0;
     
  lcd.begin(16,2);               // Initializing LCD
  lcd.home ();                   // Setting Cursor at Home i.e. 0,0
  dht.begin();
  Wire.begin();
  RTC.begin();
  Serial.begin(9600);  

  // check if RTC is running
  if(!RTC.isrunning()){
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println("Time updated.");
  }else{
    Serial.println("RTC working.");
  }

  // begin onewire communication for temperature sensor
  sensors.begin();

  lcd.home();
  lcd.print("Starting...");
  delay(1000);

  rot_trigger = false;
  // set stepper speed and initialize
  myStepper.setSpeed(10);

  // Read the status of lever sensors
  lcd.setCursor(0, 1);
  lcd.print("Getting pos sensor");
  
  pol_a = digitalRead(pol_a_pin);
  pol_b = digitalRead(pol_b_pin);
  Serial.println(pol_a);
  Serial.println(pol_b);

  start_time = millis();
  // if both the poles are away from sensors move to A
  if((pol_a == false) && (pol_b == false)){
    while(!pol_a){
    myStepper.step(10);
    // while moving read the pole A status
    pol_a = digitalRead(pol_a_pin);
    curr_time = millis();
    if((curr_time - start_time) >= 60000){
      lever_status = false;
      break;
      }
    }    
  }
  // switch of all stepper pins
  stepperOff();
  // compare button state
  prevButtonState = buttonState;
}

void loop(){
  bool menuCounter = false;
  // track if there is buttonstate change then enter into menu
  if(buttonState != prevButtonState){
    menuState = true;
    prevButtonState = buttonState;
  }
  // if menu button pressed enter into menu loop
  while(menuState){    
    count = getEncoder(count);
    delay(1);
    (count>5)?count=5:count;
    (count<0)?count=0:count;
    // navigate through different menu options
    switch(count){
      case 0: //SET MAXIMUM TEMPERATURE
        if(prevCount!=count){
          updateMenu(menuOptions,count,6);          
          prevCount = count;
        }
        if(buttonState!=prevButtonState){
          selected = true;
          prevButtonState = buttonState;
        }
        while(selected){
          // display temperature only once
          if(float_abs_diff(upperLimit, prevReading)){
            lcd_print("Temp Max: ", upperLimit);
          }
          prevReading = upperLimit;
          // get encoder readings
          count = getEncoder(count);
          // increment the value
          if(count>prevCount)upperLimit += 0.1;
          // decrement the value
          if(count<prevCount)upperLimit -= 0.1;
          // check if it reached lower limit
          if(upperLimit <= lowerLimit){
            upperLimit = lowerLimit;
          }
          prevCount = count;    // remember the count to track changes
          // exit from the loop if button pressed
          if(buttonState!=prevButtonState){
            selected = false;
            count = 0;
            prevCount = -1;
            prevReading = 0;
            prevButtonState = buttonState;
            break;
          }
        }
        break;
      case 1: // SET MINIMUM TEMPERATURE
        if(prevCount!=count){
          updateMenu(menuOptions, count, 6);          
          prevCount = count;
        }
        
        if(buttonState!=prevButtonState){
          selected = true;
          prevButtonState = buttonState;
        }
        while(selected){
          // disply only once
          if(float_abs_diff(lowerLimit, prevReading)){
            lcd_print("Temp Min: ", lowerLimit);
          }
          prevReading = lowerLimit;             // remember previous reading
          
          count = getEncoder(count);            // Get encoder readings
          if(count>prevCount)lowerLimit += 0.1; // increment
          if(count<prevCount)lowerLimit -= 0.1; // decrement
          // check if lower limit reached upper limit value
          if(lowerLimit >= upperLimit){
            lowerLimit = upperLimit;
          }
          prevCount = count;                    // remember previous encoder count
          // if button pressed exit from loop
          if(buttonState!=prevButtonState){
            prevButtonState = buttonState;
            selected = false;
            count = 1;
            prevCount = 0;
            prevReading = 0;
            break;
          }
        }
        break;
      case 2: // SET MINIMUM TEMPERATURE
        if(prevCount!=count){
          updateMenu(menuOptions, count, 6);          
          prevCount = count;
        }
        
        if(buttonState!=prevButtonState){
          selected = true;
          prevButtonState = buttonState;
        }

        //display only once
        disp = true;
        while(selected){
          if(disp){
            lcd.clear();
            lcd.print("Moving");
            disp = false;
          }
          
          count = getEncoder(count);            // Get encoder readings
          if(count>prevCount){
            if(!digitalRead(pol_a_pin))myStepper.step(30);
            stepperOff();
            lcd.clear();
            lcd.print("Move forward");
          }
          if(count<prevCount){
            if(!digitalRead(pol_b_pin))myStepper.step(-30);
            stepperOff();
            lcd.clear();
            lcd.print("Move backward");            
          }
          prevCount = count;                    // remember previous encoder count
          // if button pressed exit from loop
          if(buttonState!=prevButtonState){
            selected = false;
            count = 2;
            prevCount = 0;
            prevReading = 0;
            prevButtonState = buttonState;
            break;
          }
        }
        break;
      case 3:
        if(prevCount!=count){
          updateMenu(menuOptions, count, 6);          
          prevCount = count;
        }
        if(buttonState!=prevButtonState){
          selected = true;
          prevButtonState = buttonState;
        }

        //display only once
        disp = true;
        while(selected){
          if(disp){
            lcd.clear();
            lcd.print("Move every ");
            lcd.print(rot_interval);
            lcd.print(" Hr");
            disp = false;
          }
          
          count = getEncoder(count);            // Get encoder readings
          if(count>prevCount)rot_interval++;    // increment
          if(count<prevCount)rot_interval--;    // decrement
          (rot_interval<1)?rot_interval=1:rot_interval=rot_interval;  // prevent going below 1
          (rot_interval>10)?rot_interval=10:rot_interval=rot_interval;// prevent going above 10
          // if change in interval display
          if(temp != rot_interval){
            lcd.clear();
            lcd.print("Move every ");
            lcd.print(rot_interval);
            lcd.print(" Hr");
          }
          temp = rot_interval;                  // remember previous rot_interval
          prevCount = count;                    // remember previous encoder count
          // if button pressed exit from loop
          if(buttonState!=prevButtonState){
            selected = false;
            count = 3;
            prevCount = 0;
            prevReading = 0;
            prevButtonState = buttonState;
            break;
          }
        }
        break;
      case 4://SET TIME AND DATE
        if(prevCount!=count){
          updateMenu(menuOptions, count, 6);          
          prevCount = count;
        }
        break;
      case 5:
        if(prevCount!=count){
          updateMenu(menuOptions, count, 6);          
          prevCount = count;
        }
        if(prevButtonState!=buttonState){
          menuState = false;
          count = 0;
          prevButtonState = buttonState;
        }
        break; 
      default:
        break;  
    }
  }
  // Get current time
  curr_time = millis();
  // Get date and time from RTC module
  DateTime dt_now = RTC.now();
  String date_string = get_date_string(dt_now);
  
  // get temperature and humidity sensor readings every one second
  if(curr_time - sense_interval >= 1000UL){
    // Request temperature readings from DS18b20 sensor
    sensors.requestTemperatures();
    temperature = sensors.getTempCByIndex(0);
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidity = dht.readHumidity();
    dht_temperature = dht.readTemperature();
    // check if reading was successful
    if(isnan(humidity)||isnan(dht_temperature)){
      Serial.println(F("Failed to read from DHT Sensor!"));
    }
    // calculate average of last readings
    tempArray[avgCounter] = temperature;
    // increment avgCounter to store temperature in array
    (avgCounter>=5)?avgCounter=0:avgCounter++;
    // keep track of the sense_interval
    sense_interval = curr_time;
  }
 
 
  // get average temperature from the array
  avgTemp = get_avg_temp(tempArray);
  // if average_temperature more than upper limit, switch off the heater
  if((avgTemp - upperLimit) > 0.1){
    digitalWrite(heatPin, LOW);
  }
  if((lowerLimit - avgTemp) > 0.1){
    digitalWrite(heatPin, HIGH);
  }

  // Update display every two seconds
  if(curr_time - disp_interval>2000UL){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("T:");
    lcd.print(temperature);
    lcd.print("C");
    lcd.print("  H:");
    lcd.print(int(humidity));
    lcd.print("%");
    // Display heating if on heating mode
    lcd.setCursor(0,1);
    if((lowerLimit - avgTemp) > 0.1){
      lcd.print("    HEATING    ");
    }else{
      lcd.print(date_string);
    }

    // Serial print
    Serial.println(date_string); 
    Serial.print("Temp reading from DS18b20: ");
    Serial.println(temperature);  
    Serial.print("Humidity: ");
    Serial.print(humidity); 
    Serial.print(", DHT temp: ");
    Serial.println(dht_temperature);
    Serial.println();
    // track disply interval 
    disp_interval = curr_time;
    
  }
  

  // Check if triggered time is reached
  if((dt_now.hour()%rot_interval == 0)&&(dt_now.minute()==0)){
    rot_trigger = true;
  }

 
//  if(rot_trigger && lever_status){
//    // Switch of the heating element while rotating
//    digitalWrite(heatPin, LOW);
//    lcd.clear();
//    int count = 5;
//    for(int i=0; i<count; i++){
//      lcd.clear();
//      lcd.print("Rotating in: ");
//      count--;
//      lcd.print(count);
//      lcd.print("s");
//      delay(1000);
//    }
//    // if starting at pole A move to pole B
//    if(pol_a){
//      lcd.clear();
//      lcd.print("Rotating.. to B");
//      move_to(pol_b_pin, 'B');
//      rot_trigger = false;
//    }else if(pol_b){
//      // if at pole B move to pole A
//      lcd.clear();
//      lcd.print("Rotating.. to A");
//      move_to(pol_a_pin, 'A');
//      rot_trigger = false;      
//    }
//    
//  }

//  // Serial monitor, to see positions sensor data
//  Serial.print("Pol A: ");
//  pol_a = digitalRead(pol_a_pin);
//  Serial.println(pol_a);
//
//  Serial.print("Pol B: ");
//  pol_b = digitalRead(pol_b_pin);
//  Serial.println(pol_b);
//  delay(500);
//  Serial.println();
//  stepperOff();

  
}


// ** FUNCTIONS ** //

String get_date_string(DateTime dt){
  String months[12] = {"JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"};
  int date_ = dt.day();
  int month_ = dt.month();
  int year_ = (dt.year())%100;

  int hour_ = dt.hour();
  int minute_ = dt.minute();

  String date_string = "";
  if(date_ < 10){
    date_string += ("0"+String(date_));
  }else{
    date_string += String(date_);
  }

  date_string += ".";

  date_string += months[month_ - 1];

  date_string += ".";

  date_string += String(year_);

  date_string += "  ";

  if(hour_ < 10){
    date_string += ("0"+String(hour_)+":");
  }else{
    date_string += (String(hour_)+":");
  }

  if(minute_ < 10){
    date_string += ("0"+String(minute_));
  }else{
    date_string += String(minute_);
  }
  return date_string;
  
}

bool move_to(int pin, char pol){
  bool pin_status = false;
  while(!pin_status){
    pin_status = digitalRead(pin);
    if(pol == 'A'){
      myStepper.step(10);
    }
    
    if(pol == 'B'){
      myStepper.step(-10);
    }
  }
}

// FUNCTION TO GET AVERAGE OF AN ARRAY
//function to calculate average_temp
 float get_avg_temp(float arr[]){
  float sum = 0;
  for (int i=0; i<5; i++){
    sum+=arr[i];
  }
  return sum/5.0;
 }

// INTERRUPT SERVICE ROUTINE
void isr(){
  unsigned long currentInterruptTime = millis();
  if(currentInterruptTime - lastInterruptTime > 50){
    buttonState = !buttonState; 
    lastInterruptTime = currentInterruptTime;
  }
  
}

// FUNCTION TO SWITCH OFF THE STEPPER MOTOR
void stepperOff(){
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
}


// GET ENCODER READINGS
unsigned int getEncoder(unsigned int cnt){
  // Read present state of the rotary encoder pin A
    pinACurrState = digitalRead(rotPinA);
    // Check rotation status and increment or decrement based on direction
    if(pinACurrState != pinALastState && pinACurrState == true){
      if(digitalRead(rotPinB) != pinACurrState){
        cnt--;
      }else{
        cnt++;
      }
      //Serial.println(count);
    }
    // remember the last state
    pinALastState = pinACurrState;
//    delay(1);
    return cnt;
}

// BASIC LCD PRINTING FUNCTION INPUTS A STRING AND FLOATING NUMBER PRINTS
void lcd_print(String string, float num){
  lcd.clear();
  lcd.print(string);
  lcd.print(num);
}

// COMPARE TWO FLOATING POINT VALUES
// RETURNS TRUE IF NOT EQUAL AND RETURNS FALSE IF EQUAL
bool float_abs_diff(float a, float b){
    if((a-b)>0.09){
      return true;
    }else if((b-a)>0.09){
      return true;
    }else{
      return false;
    }
}

void updateMenu(String arr[], uint8_t m, uint8_t arrLen){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(">");
  lcd.print(arr[m]);
  if(m < (arrLen-1)){
    lcd.setCursor(0,1);
    lcd.print(" ");
    lcd.print(arr[m+1]);
  }
  
}
