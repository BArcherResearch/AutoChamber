 

//House Keeping  ]

//strain sensor 
#include "HX711.h"
#define calibration_factor -1048.25  //calculated with the calibration sketch by sparkfun and 1kg weight on the chamber. Can be adjusted and checked in my custom example. 
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = A2;
//int strain_start;
float strain_end;
HX711 scale;
    

//

#define peri  4
//sleep sketch variables 

int8_t buttonState=0;
#define USE_PROGMEM true
int8_t lastButtonState=0; 

//CH4 sensor
#define sensor A1

//pressure sensor 
#include <Wire.h>
#include "SparkFunBME280.h"
BME280 Pressure;

//CO2 sensor 

#include "SparkFun_SCD30_Arduino_Library.h" 
SCD30 airSensor;

//servo
//#include <Servo.h>
#include <ServoTimer2.h>  // the servo library
#define pawlPin  6
ServoTimer2 servoPawl;  

//int del = 2000;
//int time_time= 0; //variable to set stepper speed
//int steps = 100;
//int steps1 = 1;

//Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int8_t pos = 0;    // variable to store the servo position

 

//intro for sd and rtc
#include <Time.h>
#include <DS1307RTC.h>

#include <SPI.h>
#include <SD.h>
File myFile;
File myFile1;
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

int8_t last_Day = 0;

//long alarmTime =0;
//long alarmTime2 =0;
 
tmElements_t tm;


//intro for HC-12
#include <AltSoftSerial.h>
AltSoftSerial mySerial; //TX, RX

#define setPin 1
boolean onOff = 0;
boolean led_bool = false;

int ExpectedCommand = 0 ; 
int LastCommand = 0;
int input = 0; 


int command1=0;
int command2=0;
int8_t counter2= 0; //counts the number of un read cycles in the radio manual mode, after 5 it takes a measurement thus ~ once every minute and a half. 

//mode switches

//#define mode_switch A3
#define strain_pin A3
//#define setup_switch A2

//intro code from sequence
#define endstop 0
#define SER_Pin 3   //pin 14 on the 75HC595
#define RCLK_Pin 2  //pin 12 on the 75HC595
#define SRCLK_Pin 7 //pin 11 on the 75HC595
//#define VMOT A0 //relay that controls the power supply for the motor 
#define STEP_PIN 5 //motor driver step pin (must be PWM compatible)
#define strain 4


//How many of the shift registers - change this
#define number_of_74hc595s 3 

//do not touch
#define numOfRegisterPins number_of_74hc595s * 8

boolean registers[numOfRegisterPins];



           


//set all register pins to LOW
void clearRegisters(){
  for(int8_t i = numOfRegisterPins - 1; i >=  0; i--){
     registers[i] = LOW;
  }
} 


//Set and display registers
//Only call AFTER all values are set how you would like (slow otherwise)
void writeRegisters(){

  digitalWrite(RCLK_Pin, LOW);

  for(int8_t i = numOfRegisterPins - 1; i >=  0; i--){
    digitalWrite(SRCLK_Pin, LOW);

    int8_t val = registers[i];

    digitalWrite(SER_Pin, val);
    digitalWrite(SRCLK_Pin, HIGH);

  }
  digitalWrite(RCLK_Pin, HIGH);

}

//set an individual pin HIGH or LOW
void setRegisterPin(int8_t index, int8_t value){
  registers[index] = value;
}

////////////////////////////////////////////////////////////////////
//declare variables for the motor pins
#define  DIR_PIN 9  // Blue   - 28BYJ48 pin 1 
#define  EN_PIN 10  // Pink   - 28BYJ48 pin 2 //Motor driver enable pin 
#define  MS1 11  // Yellow - 28BYJ48 pin 3
#define  MS2 12  // Orange - 28BYJ48 pin 4


#define  VIO 15 //motor driver +5v supply 
#define  fan 2
#define  servo_power 3
#define  tran 1 //optical end stop
#define  HC_12 0
#define  blue_LED 5
#define  white_LED 4
#define  ledPin 7//green led on board
#define  LedOff 6//red led
#define  sd_power 13
#define  sensorPower 14 //power for the other sensors
#define  ch4Power 8 //powers on ch4 heating element, which needs 1 minute to warm up before readings

#define VMOT 18
#define Vperi 16
#define motor_bank 22
                        











void setup() {
pinMode(STEP_PIN, OUTPUT);
pinMode(VMOT, OUTPUT);
digitalWrite(VMOT,LOW);

  
//Serial.begin(9600);
pinMode(10, OUTPUT); 
 pinMode(peri, OUTPUT);

//ch4 sensor
pinMode(sensor, INPUT);

//co2 sensor setup
Wire.begin(); 

//pressure sensor setup 
Pressure.setI2CAddress(0x76);

//servo setup
servoPawl.attach(pawlPin); 

  
//mode switch setup
//pinMode (mode_switch, INPUT);
pinMode (strain_pin, OUTPUT);
//pinMode (setup_switch, INPUT);


//Serial.begin(9600);
 pinMode(setPin, OUTPUT);
  mySerial.begin(9600);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(ledPin, led_bool);//turn LED on
  
  digitalWrite(setPin, HIGH);
  digitalWrite(ledPin, led_bool);//turn LED on
  //delay(1000);

  pinMode(endstop, INPUT);
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);


  //reset all register pins
  clearRegisters();
  writeRegisters();
  
        
      setRegisterPin(EN_PIN, LOW);
      setRegisterPin(VIO, LOW);
                 writeRegisters();
                 delay(1000);
  
  //setRegisterPin(relay, LOW);
  
  
//Begin startup with all LEDS flashing on for for 1 second

//allLedflash(1000);

setRegisterPin(sd_power, HIGH);
setRegisterPin(sensorPower, HIGH);
setRegisterPin(tran, HIGH);
digitalWrite(strain_pin, HIGH);

writeRegisters(); 


 //function checking using led signals 

/*
/////////////////
//strain guage setup
scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);

   for(;;){
 if (scale.is_ready()) {
allOkayLED (500); 
break;
  } else {
     notOkayLED(200);
     scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);
  }}

  
allLedflash(1000);

*/

for(int8_t y = 1; y <= 5; y++){
allLedflash(200); }




//close_sequence ();
//open_sequence ();
//sensors_read (2,1);
/*
 setRegisterPin(ch4Power, HIGH);

   writeRegisters();
   delay(5000);
  setRegisterPin(ch4Power, LOW);

   writeRegisters();*/

   digitalWrite(strain_pin, LOW);

      setRegisterPin(sd_power, LOW);
setRegisterPin(sensorPower, LOW);
  setRegisterPin(VIO, LOW);
setRegisterPin(fan, LOW);
  setRegisterPin(servo_power, LOW);
setRegisterPin(tran, LOW);
  setRegisterPin(HC_12, LOW);
setRegisterPin(fan, LOW);

writeRegisters(); 
//digitalWrite(setPin, LOW);
//Serial.begin(9600);
}


void loop() {
  // put your main code here, to run repeatedly:

setRegisterPin(motor_bank, HIGH);

writeRegisters(); 

delay(1000);

setRegisterPin(motor_bank, LOW);

setRegisterPin(VMOT, HIGH);//set the motor voltage high first 

writeRegisters(); 

writeRegisters(); 





delay(1000);//pause to make sure the relay switch has activated, long time because it is mechanical 

      setRegisterPin(VIO, HIGH); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();

                 delay(500); //a little time for the system to react

      setRegisterPin(DIR_PIN, HIGH); //DIR LOW = lid down  
      setRegisterPin(MS1, HIGH);  //MS1 High and Peristaltic_pump LOW = fast steps mode 
      setRegisterPin(MS2, LOW);
      writeRegisters();

    setRegisterPin(EN_PIN, HIGH); //Enable the motor just before movement  
                 writeRegisters();


      
for(int i=0; i<450; i++){ //stepping loop was 450 but changed to 200 for chamber 28.07.2021
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(15);
  digitalWrite(STEP_PIN, LOW);
  delay(15);
}

    setRegisterPin(EN_PIN, LOW); //Enable the motor just before movement  
                 writeRegisters();

                 delay(1000);

        setRegisterPin(DIR_PIN, LOW); //DIR LOW = lid down  
      setRegisterPin(MS1, HIGH);  //MS1 High and Peristaltic_pump LOW = fast steps mode 
      setRegisterPin(MS2, LOW);
      writeRegisters();     

        delay(500);
        
        setRegisterPin(EN_PIN, HIGH); //Enable the motor just before movement  
                 writeRegisters();   


      
for(int i=0; i<450; i++){ //stepping loop was 450 but changed to 200 for chamber 28.07.2021
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(15);
  digitalWrite(STEP_PIN, LOW);
  delay(15);
}

        

   setRegisterPin(EN_PIN, LOW); //Disable the motor first  
            writeRegisters();

delay(500);

                 setRegisterPin(VIO, LOW); //motor driver +5V turned off first, before motor voltage
                 setRegisterPin(VMOT, LOW);// 
                  writeRegisters();  

delay(9000);

                    

}





void allLedflash (int x){
 setRegisterPin(blue_LED, HIGH); 
 setRegisterPin(white_LED, HIGH);
 setRegisterPin(ledPin, HIGH); 
 setRegisterPin(LedOff, HIGH); 
 writeRegisters();
 delay(x);
 setRegisterPin(blue_LED, LOW); 
 setRegisterPin(white_LED, LOW);
 setRegisterPin(ledPin, LOW); 
 setRegisterPin(LedOff, LOW); 
 writeRegisters();
 delay(x);


}

void allOkayLED (int x){

 for(int8_t i=1;i<=3;i++){ 
 setRegisterPin(ledPin, HIGH);  
 writeRegisters();
 delay(x);
 setRegisterPin(ledPin, LOW); 
 writeRegisters();
 delay(x);}


}

void notOkayLED (int x){

 
 setRegisterPin(LedOff, HIGH); 
 writeRegisters();
 delay(x);
 setRegisterPin(LedOff, LOW);
 writeRegisters();
 delay(x);


}



void radio_report (int x){

  //sd power on 
   setRegisterPin(sd_power, HIGH);
   setRegisterPin(sensorPower, HIGH);
   writeRegisters();

   delay(200);
//
//   while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately
//           delay(2000);    
//notOkayLED(500);


//}


////read endstop state////
   setRegisterPin(tran, HIGH);
   writeRegisters();
  delay(50);
  int8_t lid_position = digitalRead(endstop);
  delay(100);
  setRegisterPin(tran, LOW);
   writeRegisters();
                 

   tmElements_t tm;
   RTC.read(tm);
   
char data[12];
int8_t* DAY = (tm.Day);
int8_t* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open((F("Status.txt")), FILE_WRITE); //not sure if this f will work
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(F(", "));
    myFile1.print(data2);
    myFile1.print(F(", "));
    myFile1.print(x);
    myFile1.print(F(", "));
    myFile1.print(F("Optical_state: "));
    myFile1.println(lid_position);
    // close the file:
    myFile1.close();}
    delay(100);

   setRegisterPin(sd_power, LOW);
   setRegisterPin(sensorPower, LOW);
   writeRegisters();
}
