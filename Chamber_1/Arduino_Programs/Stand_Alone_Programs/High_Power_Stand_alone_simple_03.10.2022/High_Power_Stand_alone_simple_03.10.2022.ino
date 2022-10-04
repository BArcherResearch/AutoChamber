//High_power_stand_alone_simple
//By Ben Archer
//21.09.2022
//Created using Arduino 1.8.12

//Upload to ATMEGA328p using the ISP programming method for optimal results

//Remember to insert you load cell calibration factor on line 17
//To adjust the chamber lid closing to your chamber, adjust the number of cycles on line 869
 
 

//House Keeping  ]

//strain sensor 
#include "HX711.h"
#define calibration_factor  1536  //1536 = load cell in chamber b1. chamber a1 is -720, chamber c1 = 2238 , chamber D1 = 1918, chamber E1 = 6918
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = A2;
//int strain_start;
float strain_end; //changed to int to reduce bytes and just give closing force to the closes kg instead of decimal kg ...i hope
HX711 scale;
    

//

#define peri  4
//sleep sketch variables 

//int8_t buttonState=0;

#define USE_PROGMEM true
//int8_t lastButtonState=0; 

//CH4 sensor
#define sensor A1
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//pressure sensor 
#include <Wire.h>
#include "SparkFunBME280.h"
BME280 Pressure;
//BME280 Pressure2;

//CO2 sensor 

 
//#define TCAADDR 0x70

#include "SparkFun_SCD30_Arduino_Library.h" 
//SCD30 airSensor1;

//SCD30 airSensor2;

SCD30 airSensor;

/*void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
*/


//servo
//#include <Servo.h>
#include <ServoTimer2.h>  // the servo library
#define pawlPin  6
ServoTimer2 servoPawl;  


int8_t pos = 0;    // variable to store the servo position
//int8_t day_change=0;
 

//intro for sd and rtc
#include <DS1307RTC.h>

#include <SPI.h>
#include <SD.h>
File myFile;
File myFile1;


int8_t Last_Day = 0;

int8_t alarm_1=0;
int8_t current_time = 0;
//int8_t current_hour=0;
int8_t fan_count = 1; 
//int8_t hour_change=0;

 
tmElements_t tm;


//intro for HC-12
#include <AltSoftSerial.h>
//AltSoftSerial mySerial; //TX, RX


#define setPin 1
//boolean onOff = 0;
//boolean led_bool = false;

//int ExpectedCommand = 0 ; 
//int LastCommand = 0;
//int input = 0; 


//int command1=0;
//int command2=0;
//int8_t counter2= 0; //counts the number of un read cycles in the radio manual mode, after 5 it takes a measurement thus ~ once every minute and a half. 

//mode switches

//#define mode_switch A3
//#define strain_pin A3
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
#define  blue_LED  4 //actually blue
#define  white_LED 7  //actually orange
#define  ledPin 6//green led on board7
#define  LedOff 5//red led6
#define  sd_power 13
#define  sensorPower 14 //power for the other sensors
//#define  ch4Power 8 //powers on ch4 heating element, which needs 1 minute to warm up before readings
//#define  Main_mck_load 23 //mock load for keeping the main circuit on with pulsing
//#define  CH41 8
//#define  CH42 19
#define  CH43 21
#define  CH44 20
#define Vperi 16
#define motor_bank 22
#define VMOT 18
#define strain 17
                        











void setup() {
pinMode(STEP_PIN, OUTPUT);
//pinMode(VMOT, OUTPUT);
//digitalWrite(VMOT,LOW);

  
//Serial.begin(9600);
pinMode(10, OUTPUT); 
 pinMode(peri, OUTPUT);



//co2 sensor setup
Wire.begin(); 


Pressure.setI2CAddress(0x77);

//servo setup
servoPawl.attach(pawlPin); 



//Serial.begin(9600);
 pinMode(setPin, OUTPUT);
 // mySerial.begin(9600);

  
  digitalWrite(setPin, HIGH);


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
              //   delay(1000);
  


setRegisterPin(sd_power, HIGH);
setRegisterPin(sensorPower, HIGH);
setRegisterPin(tran, HIGH);
setRegisterPin(HC_12, HIGH);
setRegisterPin(strain, HIGH);
setRegisterPin(CH43,HIGH);
setRegisterPin(CH44,HIGH);

writeRegisters(); 







//test stastup with no led calls

//rtc
 tmElements_t tm;
     
    if (!RTC.read(tm)) {  //if okay flash green led once for 1 second            
              notOkayLED(100);}




//sd

          pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
              // delay(2000);
notOkayLED(300);

}



//CO2 sensor setup



      while (airSensor.begin() == false){

      notOkayLED(500);}

      //////////////////////////////////////////



//pressure

while (Pressure.beginI2C()==false){

      notOkayLED(700);}

//while (Pressure2.beginI2C()==false){
  
    //  notOkayLED(700);}

////////////////////////////////////////////////////////////    

//strain


scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);


 while(!scale.is_ready()) {

     notOkayLED(900);
     scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);
  }//not sure about this 

//ch4
pinMode(sensor, INPUT);

ads.setGain(GAIN_TWOTHIRDS); 

while (!ads.begin(0x49)) {
     notOkayLED(1100);}

     

/*
//HC-12     
 mySerial.begin(9600);
 delay(500);

digitalWrite(setPin, LOW);           // Set HC-12 into AT Command mode
 delay(100);                          // Wait for the HC-12 to enter AT Command mode
 mySerial.print("AT");               // Send AT Command to HC-12
 delay(100);
 digitalWrite(setPin, HIGH);
 delay(200);
  
 while(!mySerial.available()) { 
     notOkayLED(200);}*/


radio_report (); //writes the letter 9 to the status file upon startup, with the time and date.

//digitalWrite(strain_pin, LOW);

   
//setRegisterPin(strain, LOW);
setRegisterPin(VIO, LOW);
setRegisterPin(servo_power, LOW);
//setRegisterPin(tran, LOW);
setRegisterPin(HC_12, LOW);
setRegisterPin(fan, LOW);



writeRegisters(); 




}



///////////////////////////////////////////////////////////////////////





void loop() {


  
   setRegisterPin(white_LED, HIGH);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(ledPin, HIGH);//
   writeRegisters();

   delay(500);

   setRegisterPin(white_LED, LOW);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(ledPin, LOW);//
   writeRegisters();

tmElements_t tm;
mix_air(5000);


//  setRegisterPin(tran, HIGH);
//  writeRegisters();
 // delay(50);
  int8_t lid_position = digitalRead(endstop);
 // delay(100);
//  setRegisterPin(tran, LOW);
 //  writeRegisters();

 close_sequence ();



RTC.read(tm); 
//delay(50);
alarm_1 = ((tm.Minute)+ 26); //normally 26, just at 2 for demonstration at the IGB audit
current_time = (tm.Minute);

if (alarm_1>59){
 alarm_1= (alarm_1-59); //setting the minutes to the time within 60 for the next hour
}






//measuring loop set to measure continuously before the alarm for the lid open is reached
int8_t chamberCommand = 0;

//while (((hour_change == true) && ((tm.Minute)<alarm_1) && (current_hour != (tm.Hour))) || ((hour_change == true)&& (current_hour== (tm.Hour))) || ((hour_change == false) && ((tm.Minute)< alarm_1)&& (current_hour == (tm.Hour)))){
 while(current_time !=alarm_1){

allLedflash(200);


if (fan_count>=50){
//myFile.close(); //moving file.close to here didn't work
mix_air(5000);
fan_count=1;
}

else{fan_count++;}

//allOkayLED (500);
   
//allLedflash (500);



////read endstop state////
if (chamberCommand == 1){
  // setRegisterPin(tran, HIGH);
  // writeRegisters();
  //delay(50);
  lid_position = digitalRead(endstop);
delay(50);
 // setRegisterPin(tran, LOW);
 //  writeRegisters();}
}

///////writing to file//////////

tmElements_t tm;

/*if (!RTC.read(tm)) {  //if okay flash green led once for 1 second            
               setRegisterPin(white_LED, HIGH);
                  writeRegisters();
               delay(100);
               setRegisterPin(white_LED, LOW);//
   writeRegisters();
   delay(100);
               }*/

RTC.read(tm); 

char data_file[12];
int8_t* DAY = (tm.Day);
int8_t* MONTH = (tm.Month);
int* YEAR = (tm.Year)-30;
current_time = (tm.Minute);

//char data_date[12];
//int8_t DAY1 = (tm.Day);
//int8_t MONTH1 = (tm.Month);
//int DAY1 = (tm.Day);
//int MONTH1 = (tm.Month);
//int YEAR1 = (tm.Year)+1970-2000;

//int8_t seconds1 = (tm.Second);
//int8_t minutes1 = (tm.Minute);
//int8_t hours1 = (tm.Hour);


 // sprintf(data_file, "%u_%u_%u.txt", (tm.Day),(tm.Month),((tm.Year)+1970-2000));
  sprintf(data_file, "%u_%u_%u.txt", DAY,MONTH,YEAR);
 airSensor.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
 airSensor.setAltitudeCompensation(60); //Set altitude of the sensor in m

//delay(50);
//float multiplier = 0.1875F; 



if ((tm.Day)!=Last_Day){
   myFile = SD.open(data_file, FILE_WRITE);
   if (myFile) {

myFile.print(F("Date, "));
//myFile.print(F(", "));
myFile.print(F("Time, "));
//myFile.print(F(", "));

myFile.print(F("CO2, "));
//myFile.print(F(", "));


myFile.print(F("Tmp, "));
//myFile.print(F(", "));


myFile.print(F("H2O, "));
//myFile.print(F(", "));


myFile.print(F("Pa, "));
//myFile.print(F(", "));


myFile.print(F("CH4_3, "));
//myFile.print(F(", "));

myFile.print(F("CH4_4, "));
//myFile.print(F(", "));


myFile.print(F("Comm, "));
//myFile.print(F(", "));

myFile.print(F("Stat, "));
//myFile.print(F(", "));

myFile.println(F("F_e, "));
//myFile.println(F(", "));



// close the file:
   myFile.close();}

  }


delay(1000);

  //Serial.println(data_file);

 myFile = SD.open(data_file, FILE_WRITE);
 delay(50);
 if (myFile) {

//myFile.print(data_file);   ///this coul be day month year to save memory
//myFile.print(alarm_1);
//delay(50);
//myFile.print(F(", "));
delay(250);
myFile.print(tm.Day);
delay(250);
myFile.print(F("_"));
delay(250);
myFile.print(tm.Month);
delay(250);
myFile.print(F("_"));
delay(250);
myFile.print((tm.Year)-30);
delay(250);
myFile.print(F(", "));
delay(250);

myFile.print(tm.Hour);
delay(250);
myFile.print(F(":"));
delay(250);
myFile.print(tm.Minute);
delay(250);
myFile.print(F(":"));
delay(250);
myFile.print(tm.Second);
delay(250);
myFile.print(F(", "));
delay(250);

if (airSensor.dataAvailable()){


myFile.print(airSensor.getCO2());
delay(250);
myFile.print(F(", "));
delay(250);
//myFile.print(avetmp);
//myFile.print(", ");
myFile.print(airSensor.getTemperature());
delay(250);
myFile.print(F(", "));
delay(250);
myFile.print(airSensor.getHumidity());
delay(250);}
myFile.print(F(", "));
delay(250);

/*while (Pressure.readFloatPressure()){
             setRegisterPin(ledPin, HIGH);
                  writeRegisters();
               delay(100);
               setRegisterPin(ledPin, LOW);//
   writeRegisters();
   delay(100);
      }*/
      
myFile.print(Pressure.readFloatPressure());
delay(250);
myFile.print(F(", "));
delay(250);

/*while (!ads.readADC_Differential_0_1()) {
               setRegisterPin(blue_LED, HIGH);
                  writeRegisters();
               delay(100);
               setRegisterPin(blue_LED, LOW);//
   writeRegisters();
   delay(100);}
*/

myFile.print(ads.readADC_Differential_0_1());
delay(250);
myFile.print(F(", "));
delay(250);


myFile.print(ads.readADC_Differential_2_3());
delay(250);
myFile.print(F(", "));
delay(250);


myFile.print(chamberCommand);
delay(250);
myFile.print(F(", "));
delay(250);

myFile.print(lid_position);
delay(250);
myFile.print(F(", "));
delay(250);

myFile.print(strain_end, 3);
delay(250);
myFile.println(F(", "));
delay(250);
myFile.close();} //got rid of file close as uses alot of power
delay(250);
Last_Day = (tm.Day);


delay(50);

chamberCommand = 1;


//RTC.read(tm);
//delay(50);

}






open_sequence ();

delay(240000); //normally 240000 i.e. 4 mins sleep venting time, evens out at ~5 mins after sensors reading and waiting for clock for a minute

/*
//radio communication
   mySerial.println(4321);//send unique code to the radio station to confirm chamber cycle has occured
   // onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
       // onOff = 1;//set boolean to 1
    delay(100);//delay little for better serial communication}


 */
   
}





















void close_sequence (){

   servoPawl.write(1000); //preset servo angle to prevent 

      setRegisterPin(EN_PIN, LOW); //making sure that the motor is not enabled 
      setRegisterPin(VIO, LOW);     //and making sure that the order of switching on is correct 
     // setRegisterPin(strain, HIGH);
      setRegisterPin(ledPin, HIGH);  ///////test      
                 writeRegisters();
      //digitalWrite(strain_pin, HIGH);

                 delay(500);
  

//do one movement of the chamber up and down finishing in the up position, to check all mechanisms are functional
  // setRegisterPin(tran, HIGH); //tran = optical endstop
   setRegisterPin(servo_power, HIGH);
   writeRegisters();


//  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
 // delay(100);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  

  




//setting the motor parameters

setRegisterPin(motor_bank, HIGH);
setRegisterPin(ledPin, LOW);//////////test 

writeRegisters(); 

delay(1000);

setRegisterPin(motor_bank, LOW);
writeRegisters(); 



setRegisterPin(VMOT, HIGH); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();

delay(1000);//pause to make sure the relay switch has activated, long time because it is mechanical 

      setRegisterPin(VIO, HIGH); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();

                 delay(500); //a little time for the system to react

      setRegisterPin(DIR_PIN, HIGH); //DIR LOW = lid down  
      setRegisterPin(MS1, HIGH);  //MS1 High and Peristaltic_pump LOW = fast steps mode 
      setRegisterPin(MS2, LOW);
      writeRegisters();


//////zeroing lid//////
 int8_t lid_position = digitalRead(endstop); //reading end stop to see if it is active 

delay(50);

      setRegisterPin(EN_PIN, HIGH); //Enable the motor just before movement  
                 writeRegisters();

     int val = servoPawl.read();

//moved servo unlatching to after the motor is enabled to prevent uncontrolled lid closure
 
while  ( val<1400){ //normally 1400, testing 1550 for chamber a1 with incorrectly positioned servo arm 
  
   val = ((servoPawl.read())+ 1);
   servoPawl.write(val);
      delay(2); }



    while (lid_position == LOW){
  digitalWrite(STEP_PIN, HIGH);
  delay(2);
  digitalWrite(STEP_PIN, LOW);
  delay(2);
    lid_position = digitalRead(endstop);
    delay(10);
    }



 /////locking lid by moving up slightly and coming down slower//////
       setRegisterPin(DIR_PIN, LOW); //DIR LOW = lid down  
      writeRegisters();

    while (lid_position == HIGH){
  digitalWrite(STEP_PIN, HIGH);
  delay(35);
  digitalWrite(STEP_PIN, LOW);
  delay(35);
    lid_position = digitalRead(endstop);
    delay(10);
    }

    scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0
    delay(100);

    for( int8_t i=0; i<20; i++){ //stepping loop was 450 but changed to 200 for chamber 28.07.2021
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(35);
  digitalWrite(STEP_PIN, LOW);
  delay(35);
}

       setRegisterPin(DIR_PIN, HIGH); //DIR LOW = lid down  
      writeRegisters();


    while (lid_position == LOW){
  digitalWrite(STEP_PIN, HIGH);
  delay(45);
  digitalWrite(STEP_PIN, LOW);
  delay(45);
    lid_position = digitalRead(endstop);
    delay(10);}




 ////close fully////////


      
for(int i=0; i<500; i++){ //stepping loop was 450 but changed to 200 for chamber 28.07.2021
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(15);
  digitalWrite(STEP_PIN, LOW);
  delay(15);
}




for( int8_t i=0; i<127; i++){ //stepping loop ...was 50 steps 
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(35);
  digitalWrite(STEP_PIN, LOW);
  delay(35);
}



// int val;
 val = servoPawl.read();
while  ( val>850){
  
   val = (( servoPawl.read())- 1);
   servoPawl.write(val);
      delay(2); }

 uint8_t z = 1;

//strain_end = ((scale.get_units(1))*0.45359237);

 //while ((strain_end < 10) && (z<=250)){ // b1 = 150, a1= 225

while (z<=35){ // b1 = 150, a1= 225, c1 = 250

  z = (z+1);
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(45);
  digitalWrite(STEP_PIN, LOW);
  delay(45);

 }


      

   setRegisterPin(EN_PIN, LOW); //Disable the motor first  
            writeRegisters();

delay(500);

                 setRegisterPin(VIO, LOW); //motor driver +5V turned off first, before motor voltage
                  writeRegisters();  


strain_end = ((scale.get_units(5))*0.45359237); //was units(20 but not sure why 5 readings should be enough)
 // digitalWrite(strain_pin, LOW);

                            
               //  setRegisterPin(tran, LOW);
                 setRegisterPin(servo_power, LOW);
                // setRegisterPin(strain, LOW);
                 
                 writeRegisters();
                 
                 delay(500);
                 
               setRegisterPin(VMOT, LOW); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();
   

}





void open_sequence (){

  servoPawl.write(1000);

      setRegisterPin(EN_PIN, LOW); //making sure that the motor is not enabled 
      setRegisterPin(VIO, LOW);     //and making sure that the order of switching on is correct 
    //  setRegisterPin(strain, HIGH);
                 writeRegisters();

   
                 delay(500);
  

//do one movement of the chamber up and down finishing in the up position, to check all mechanisms are functional
  // setRegisterPin(tran, HIGH); //tran = optical endstop
   setRegisterPin(servo_power, HIGH);
   writeRegisters();

   

 // scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
 // delay(5000);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0


//opening the pawl 
int val = servoPawl.read();

  while  ( val<1400){ //normally 1400
  
   val = (( servoPawl.read())+ 1);
   servoPawl.write(val);
      delay(2); }

//setting the motor parameters

setRegisterPin(motor_bank, HIGH);

writeRegisters(); 

delay(1000);

setRegisterPin(motor_bank, LOW);
writeRegisters(); 

//digitalWrite(VMOT, HIGH);//set the motor voltage high first
setRegisterPin(VMOT, HIGH); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters(); 
delay(1000);//pause to make sure the relay switch has activated, long time because it is mechanical 

      setRegisterPin(VIO, HIGH); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();

                 delay(500); //a little time for the system to react

      setRegisterPin(DIR_PIN, LOW); //DIR HIGH = lid up  
      setRegisterPin(MS1, HIGH);  //MS1 High and Peristaltic_pump LOW = fast steps mode 
      setRegisterPin(MS2, LOW);
      writeRegisters();


//////zeroing lid//////
 

      setRegisterPin(EN_PIN, HIGH); //Enable the motor just before movement  
                 writeRegisters();


    for(int i=0; i<1350; i++){ //stepping loop
  digitalWrite(STEP_PIN, HIGH);
  delay(2);
  digitalWrite(STEP_PIN, LOW);
  delay(2);

    }

// int val;
 val = servoPawl.read();
while  ( val>850){
  
   val = (( servoPawl.read())- 1);
   servoPawl.write(val);
      delay(2); }

      
   setRegisterPin(EN_PIN, LOW); //Disable the motor first  
            writeRegisters();

delay(500);

strain_end = ((scale.get_units(5))*0.45359237);
//  digitalWrite(strain_pin, LOW);
        
               
                 setRegisterPin(VIO, LOW); //motor driver +5V turned off first, before motor voltage             
                // setRegisterPin(tran, LOW);
                 setRegisterPin(servo_power, LOW);
              //   setRegisterPin(strain, LOW);                 
                 writeRegisters();
                 
                 delay(500);
                 
                 //digitalWrite(VMOT, LOW); 
                 setRegisterPin(VMOT, LOW); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();
   

}


void mix_air (int x) {

   setRegisterPin(fan, HIGH);
   writeRegisters();

   delay(x);

   setRegisterPin(fan, LOW);
   writeRegisters();

   

}


///Function to run the servo and to engage the pawl on the ratchet ///
int incPulse(int val, int inc){
   if( val + inc  > 2000 )
      return 1000 ;
   else
       return val + inc;  
}
//////////////////////////////

///Function to run the servo and to engage the pawl on the ratchet ///
int incPulse1(int val, int inc){
   if( val - inc  < 500 )
      return 500 ;
   else
       return val - inc;  
}

//////////////////////////////




  



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

// for(int8_t i=1;i<=1;i++){ 
 setRegisterPin(ledPin, HIGH);  
 writeRegisters();
 delay(x);
 setRegisterPin(ledPin, LOW); 
 writeRegisters();
 delay(x);//}


}

void notOkayLED (int x){

 
 setRegisterPin(LedOff, HIGH); 
 writeRegisters();
 delay(x);
 setRegisterPin(LedOff, LOW);
 writeRegisters();
 delay(x);


}

void radio_report (){
/*
  //sd power on 
   setRegisterPin(sd_power, HIGH);
   setRegisterPin(sensorPower, HIGH);
   writeRegisters();

   delay(1000);
//

      while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}*/

             





//   while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately
//           delay(2000);    
//notOkayLED(500);


//}


////read endstop state////
 //  setRegisterPin(tran, HIGH);
 //  writeRegisters();
 // delay(50);
  int8_t lid_position = digitalRead(endstop);
//  delay(100);
//  setRegisterPin(tran, LOW);
//   writeRegisters();
                 

   tmElements_t tm;
   RTC.read(tm);
   
//char data[12];
//int8_t* DAY = (tm.Day);
//int8_t* MONTH = (tm.Month);
//int* YEAR = (tm.Year)+1970-2000;

//char data2[12];
//int* seconds = (tm.Second);
//int* minutes = (tm.Minute);
//int* hours = (tm.Hour);

 // sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
//  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open((F("Status.txt")), FILE_WRITE); //not sure if this f will work
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
   // myFile1.print(data);
   myFile1.print(tm.Day);
myFile1.print(F("_"));
myFile1.print(tm.Month);
myFile1.print(F("_"));
myFile1.print((tm.Year)-30);
    myFile1.print(F(", "));
    myFile1.print(tm.Hour);
//delay(200);
myFile1.print(F(":"));
//delay(200);
myFile1.print(tm.Minute);
//delay(200);
myFile1.print(F(":"));
//delay(200);
myFile1.print(tm.Second);
//delay(200);

//delay(200);
   // myFile1.print(data2);
    myFile1.print(F(", "));
    myFile1.print(F("9, "));
  //  myFile1.print(F(", "));
    myFile1.print(F("Optical_state: "));
    myFile1.println(lid_position);
    // close the file:
    myFile1.close();}
   // delay(100);

  /* setRegisterPin(sd_power, LOW);
   setRegisterPin(sensorPower, LOW);
   writeRegisters();*/
}



void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void digitalInterrupt(){ //needed for the digital input interrupt 
  }
  
ISR(WDT_vect){ //DON'T FORGET THIS! Needed for the watch dog timer. This is called after a watch dog timer timeout - this is the interrupt function called after waking up 
    }
