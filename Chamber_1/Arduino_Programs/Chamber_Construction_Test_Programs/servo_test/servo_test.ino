//Battery managment test
//aim is to keep the battery out of the programmed sleep mode (>45mA every 25 seconds)
//strategy is to enable the stepper motor for half a second in between the deepsleep periods
//here I will test what works
//12.08.2020


//06.06.2020
//changing shift registers
//changing HC-12
//changing sd-card and rtc (spi and i2c bus therefore stay the same)

//pins still to add = methane voltage read (maximum two) and power switch 
//co2 is on i2c but also needs a power switch 
//maybe a pressure sensor but this wil be spi or i2c thus will need a power switch.

//free pins = 8, A3, A2, A1, A0
//lets make switches A0 and A1 next to each other.

//works with altsoftserial master.
//..needs servo library replacing. I think that ServoTimer2 will work, uses pwm 
//addedservoTimer2 ... compiled fine with it just included 
//need to reverse the servo directions, I got confused. Also it appears that the lide is opening at the end of the close sequence? check this.
//servo sequence solved, now add back in rtc record to sd ...done
//add optical end stop state to sd card for error tracking...done 
//adding co2 sensor read /// first adding libraries incase of competition..no abvious problems, now adding setup...does not seem to have an issue with the RTC library which also uses i2c (few)

//also need to add power switches for HC-12, CO2 sensor, CH4 sensor, and SD card. Not sure if I need to manage the power to the mode switches or if this is negligable? so for now 4 switches. spare shift register pins = 0, 8, 13, 14, perfect
//memory warning at 77% dynamic mem and 77%program space. Adding pressure sensor library to see if its much more...no change to memory, must be a small library 

//adding the sensor read function and polishing the sd writing code. Adding the headers, and sparate files for data and tracking the state of the chamber. 

//check the sensors are being detected by the board....senors are detected
//check the auto mode is working and recording the data to the sd card correctly...sd not writing...sorted 
//check the radio mode is still working and recording the data to the sd card correctly...working
//add sleep to the auto mode...
//fit the mech to the chamber...
//check the chamber and install the radio comms

//added the sleep sketch....potential errors = the addition of the reversing of the adc bus
//added the sd card power off, the sensor breakout power off, and the ch4 heating element power off. Likely problems include beginning them again.
//may be that there is something missing in the order, that I forgot to re power. But lets test and see. First add the parts to the pcb.
//test results...
///only thing to add is the battery pulse to keep it awake.





/*post trial changes to be made:
 * 
 * make auto mode the low-low switches, incase of crash and restarting in this period      done
 * 
 * Adjust the sampling regieme so that the first measurement is taken during the lid open stage. Think about maybe making the measurement time 5o mins, one sample per 10 mins, and 10 mins of venting?
 * or maybe just leave the sample times as they are for now kept same as before
 * 
 * Another change to make is to add a sampling time in seconds to the file, or to make sure that the date and time are layed out in the same way as the picaro, so the same programme will work added
 * 
 * also remove the average measurement and just write to the sd file 5 times. Nice to have the data/ to be able to track the runs better. done
 * 
 * add the change day after the first sensor reading to stop repetition of the titles in the txt file //done
 * 
 * add a startup message in the setup to be able to track if the system is restarting. might be that by not having the average maths to do it doesn't crash, otherwise it needs a large current smoothing cap again. done
 * 
 * also changed the format of the time in the sensor read. Left the timings for venting etc as is. 
 */


 //further changes
 //putting in delay for co2 sensor at the start to make sure that it is warm and the CO2 does not give a zero value reading for the first of the 5 measurements 
 //putting in pressure sensor temperature read 
 //putting in increased CO2 sensors temperature sensitivity? think both sensors have a max of 1oC sensoitivity

 /*low memory additions:
 * 
 * all int to int8_t
 * all print to F print
 * see if can change some areas to a loop function with an array? 
 * 
 * starting with the F#s...already y implementing this, the warning has gone. now 91% and 73%
 * now implement int changes
 * 
 * note: the servo is now limited to 127 range because of the int8_t to save memory 
 * things that might not work = the file save with f
 * things that can still be changed = the file write with two sets of date variables, and maybe fewer saved dynamic variables in the sensor read portion of the sketch.
 * 
 * adding strain guage measurements through the HX711. Trying to use data pin on D4, SCK on the same pin as the shift registers, power control on the same pin as the stepper driver. 
 * 
 * 
 * 
 * 09.08.2021 changed a few int's to  int8_t in the close sequence, as well as adding a feedback loop between the load cell and the stepper 
 * motor in the close sketch. Currently this is set to 5kgs and 100 steps to be the maximum of each feedback clause. Will do a comparison test between the 
 * chamber clamping force for the feedback and no feedback to make sure this isn't significantly reducing the clamping force due to the motor disruption.
 * 10kg calibration factor = -1258.7
 * 5kg calibration factor =  -1048.25
 * 
 * 
 * 
 * //14.10.2021 adding the v2 sensors script into main
 */
 

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
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

//pressure sensor 
#include <Wire.h>
#include "SparkFunBME280.h"
BME280 Pressure1;
BME280 Pressure2;

//CO2 sensor 

 
#define TCAADDR 0x70

#include "SparkFun_SCD30_Arduino_Library.h" 
SCD30 airSensor1;

SCD30 airSensor2;

//SCD30 airSensor;

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}



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
#define  blue_LED  4 //actually blue
#define  white_LED 7  //actually orange
#define  ledPin 6//green led on board7
#define  LedOff 5//red led6
#define  sd_power 13
#define  sensorPower 14 //power for the other sensors
#define  ch4Power 8 //powers on ch4 heating element, which needs 1 minute to warm up before readings
//#define  Main_mck_load 23 //mock load for keeping the main circuit on with pulsing
//#define  CH41 8
//#define  CH42 19
#define  CH43 21
#define  CH44 20
#define Vperi 16
#define motor_bank 22
#define VMOT 18
                        











void setup() {
pinMode(STEP_PIN, OUTPUT);
//pinMode(VMOT, OUTPUT);
//digitalWrite(VMOT,LOW);

  
//Serial.begin(9600);
pinMode(10, OUTPUT); 
 pinMode(peri, OUTPUT);



//co2 sensor setup
Wire.begin(); 

//pressure sensor setup 
Pressure1.setI2CAddress(0x76);
Pressure2.setI2CAddress(0x77); //


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
setRegisterPin(HC_12, HIGH);

writeRegisters(); 

digitalWrite(strain_pin, HIGH);

 //function checking using led signals 
/*
 
    //RTC
     tmElements_t tm;
     
     if (RTC.read(tm)) {  //if okay flash green led once for 1 second             
               allOkayLED (500);
              int lastButtonState = (tm.Day);
                }

      else if (RTC.chipPresent()) { //if the chip needs resetting flash the red led 

        
                //delay(1000);
                for(;;){
                notOkayLED(500);
                }}
      

      else {   //if the chip not detected at all then keep the LED on 
               for(;;){

               setRegisterPin(LedOff, HIGH);
               writeRegisters();

               }
      }

//flash all LEDs to indicate sd card check


allLedflash(250);

  
      //sd card check
      // setRegisterPin(sd_card_reader, HIGH);
      //writeRegisters(); 
       pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}

allOkayLED (500);              


 //flash all lights to indicate that the sd card is okay and moving onto the next step 

for(int8_t y = 1; y <= 2; y++){
allLedflash(200); }




//CO2 sensor setup

tcaselect(0);

      while (airSensor1.begin() == false){

      notOkayLED(200);}

      allOkayLED (500); 


for(int8_t y = 1; y <= 3; y++){ //flash all lights to indicate that the co2 sensor is okay and moving onto the next step 
allLedflash(200); } 

tcaselect(1);

      while (airSensor2.begin() == false){

      notOkayLED(200);}

      allOkayLED (500); 

for(int8_t y = 1; y <= 4; y++){ //flash all lights to indicate that the co2 sensor is okay and moving onto the next step 
allLedflash(200); } 

//checking if pressure sensor is okay
     
while (Pressure1.beginI2C()==false){

      notOkayLED(200);}

allOkayLED (500);       

for(int8_t y = 1; y <= 5; y++){ //flash all lights to indicate that the sensor is okay and moving onto the next step 
allLedflash(200); } 


while (Pressure2.beginI2C()==false){
  
      notOkayLED(200);}

allOkayLED (500); 

for(int8_t y = 1; y <= 6; y++){ //flash all lights to indicate that the sensor is okay and moving onto the next step 
allLedflash(200); } 

      

/////////////////
//strain guage setup
scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);

   for(;;){
 if (scale.is_ready()) {
allOkayLED (500); 
break;
  } 
  else {
     notOkayLED(200);
     scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
   delay(50);
  }}

  
for(int8_t y = 1; y <=7 ; y++){ //flash all lights to indicate that the sensor is okay and moving onto the next step 
allLedflash(200); }


////////
//ch4 setup 
pinMode(sensor, INPUT);

ads.setGain(GAIN_TWOTHIRDS); 

for(;;){
if (ads.begin(0x49)) {
  allOkayLED (500); 
break;
  }

else{
     notOkayLED(200);
     (ads.begin(0x49));
   delay(50);}
}  


for(int8_t y = 1; y <= 8; y++){
allLedflash(200); }

//hc-12 check

delay(500);
 mySerial.begin(9600);
 delay(500);

digitalWrite(setPin, LOW);           // Set HC-12 into AT Command mode
 delay(100);                          // Wait for the HC-12 to enter AT Command mode
 mySerial.print("AT");               // Send AT Command to HC-12
 delay(100);
 digitalWrite(setPin, HIGH);
 delay(200);

for(;;){
  
 if (mySerial.available()) { 
 delay(100);

 String input1 = mySerial.readString();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);

 allOkayLED (500); 
 break;

 }

 else{

 notOkayLED(200);

delay(500);
 mySerial.begin(9600);
 delay(500);

digitalWrite(setPin, LOW);           // Set HC-12 into AT Command mode
 delay(100);                          // Wait for the HC-12 to enter AT Command mode
 mySerial.print("AT");               // Send AT Command to HC-12
 delay(100);
 digitalWrite(setPin, HIGH);
 delay(200);

 }}


for(int8_t y = 1; y <= 9; y++){
allLedflash(200); }


//////

radio_report (9); //writes the letter 9 to the status file upon startup, with the time and date.

setRegisterPin(blue_LED, HIGH);
writeRegisters();  
delay(250); 
setRegisterPin(ledPin, HIGH);
writeRegisters(); 
delay(250);
setRegisterPin(white_LED, HIGH);
writeRegisters(); 
delay(250); 
setRegisterPin(LedOff, HIGH);
writeRegisters(); 
delay(1000); 
setRegisterPin(blue_LED, LOW); 
setRegisterPin(ledPin, LOW);
setRegisterPin(white_LED, LOW);
setRegisterPin(LedOff, LOW);
writeRegisters(); 

*/



//test stastup with no led calls
/*
//rtc
 tmElements_t tm;
     
     if (!RTC.read(tm)) {  //if okay flash green led once for 1 second             
              notOkayLED(100);}

     int8_t lastButtonState = (tm.Day);

//sd

          pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(300);

}

//CO2 sensor setup

tcaselect(0);

      while (airSensor1.begin() == false){

      notOkayLED(500);}

tcaselect(1);

      while (airSensor2.begin() == false){

      notOkayLED(500);}

//pressure

while (Pressure1.beginI2C()==false){

      notOkayLED(700);}

while (Pressure2.beginI2C()==false){
  
      notOkayLED(700);}

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
*/
/*//HC-12     
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
/*
radio_report (9); *///writes the letter 9 to the status file upon startup, with the time and date.

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


///////////////////////////////////////////////////////////////////////





void loop() {

/*//mode select removed to save memory .with mode select (just manual and auto) 99% program space and 73% dynamic memory . With just auto 92% program space and 71% dynamic memory. with manual mode 96% program storage space and 73% dynamic memory.
//will adjust these values after adjusting strain guage sketch
 //mode select 
 //moved the testing sequences to after the mode has been selected, to save time with opening and closing the chambers for travel etc

int8_t mode=digitalRead(mode_switch);
int8_t setup_command = digitalRead(setup_switch);
delay(200);

//if (setup_command == HIGH && mode ==HIGH){Lid_up ();} //chamber lid continually up
//else if (setup_command == HIGH && mode ==LOW){Lid_down (); }//chamber lid continually down
 if (setup_command == LOW && mode ==LOW){auto_mode ();} //automatic mode
else if (setup_command == LOW && mode ==HIGH){manual_mode();} //manual mode i.e. radio control 
}

*/
 setRegisterPin(servo_power, HIGH);
   writeRegisters();


  
  
/*
//opening the pawl 
int val;
 val = incPulse(servoPawl.read(), 1);
  while  ( val<1000){  //was 1700
  
   val = incPulse( servoPawl.read(), 1);
   servoPawl.write(val);
      delay(2); }

      */

int val = servoPawl.read();

 
while  ( val<1400){
  
   val = ((servoPawl.read())+ 1);
   servoPawl.write(val);
      delay(2); }

delay(3000);

 val = servoPawl.read();
while  ( val>850){
  
   val = (( servoPawl.read())- 1);
   servoPawl.write(val);
      delay(2); }

delay(3000);

}


/*
else if (setup_command == LOW && mode ==HIGH){
  
   
//Begin startup with all LEDS flashing on for for 1 second

//allLedflash(1000);

setRegisterPin(sd_power, HIGH);
setRegisterPin(sensorPower, HIGH);
  //setRegisterPin(relay, HIGH);
writeRegisters(); 

//////////// 2nd batt on
 setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();
del=1750;
//for(int i=0; i<8; i++){
time_time = (25);     //variable to set stepper speed
steps1 = 10;
clockwise();
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
 /////////// 
 
 //function checking using led signals 

    //RTC
     tmElements_t tm;
     
     if (RTC.read(tm)) {  //if okay flash green led once for 1 second             
               allOkayLED (500);
                }

      else if (RTC.chipPresent()) { //if the chip needs resetting flash the red led 

        
                //delay(1000);
                for(;;){
                notOkayLED(500);
                }}
      

      else {   //if the chip not detected at all then keep the LED on 
               for(;;){

               setRegisterPin(LedOff, HIGH);
               writeRegisters();

               }
      }

//flash all LEDs to indicate sd card check


allLedflash(1000);

  
      //sd card check
    //   setRegisterPin(sd_card_reader, HIGH);
     // writeRegisters(); 
       pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin(4)) { //whilst the sd is not okay, the green and red led are turned on alternately
               
notOkayLED(200);

}

allOkayLED (500);              


 //flash all lights to indicate that the sd card is okay and moving onto the next step 
 
allLedflash(1000);


 //checking if CO2 sensor is okay 

      while (airSensor.begin() == false){

      notOkayLED(200);}

      allOkayLED (500); 

 //flash all lights to indicate that the co2 sensor is okay and moving onto the next step 

allLedflash(1000);   

//checking is pressure sensor is okay
     
     while (Pressure.beginI2C()==false){

      notOkayLED(200);}

      allOkayLED (500); 
          
 //flash all lights to indicate that the co2 sensor is okay and moving onto the next step
 for(int8_t y = 1; y <= 5; y++){
allLedflash(200); } 

    setRegisterPin(sd_power, LOW);
setRegisterPin(sensorPower, LOW);
  setRegisterPin(relay, LOW);
setRegisterPin(fan, LOW);
  setRegisterPin(servo_power, LOW);
setRegisterPin(tran, LOW);
  setRegisterPin(HC_12, LOW);
setRegisterPin(fan, LOW);

writeRegisters(); 

  manual_mode();} //manual mode i.e. radio control 
//}
else if (setup_command == LOW && mode ==LOW){
  
   */




/*                                                                   moved to loop setup

//Begin startup with all LEDS flashing on for for 1 second

//allLedflash(1000);

setRegisterPin(sd_power, HIGH);
setRegisterPin(sensorPower, HIGH);
  //setRegisterPin(relay, HIGH);
writeRegisters(); 

//////////// 2nd batt on
 setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();
del=1750;
//for(int i=0; i<8; i++){
time_time = (25);     //variable to set stepper speed
steps1 = 10;
clockwise();
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
 /////////// 
 
 //function checking using led signals 

    //RTC
     tmElements_t tm;
     
     if (RTC.read(tm)) {  //if okay flash green led once for 1 second             
               allOkayLED (500);
                }

      else if (RTC.chipPresent()) { //if the chip needs resetting flash the red led 

        
                //delay(1000);
                for(;;){
                notOkayLED(500);
                }}
      

      else {   //if the chip not detected at all then keep the LED on 
               for(;;){

               setRegisterPin(LedOff, HIGH);
               writeRegisters();

               }
      }

//flash all LEDs to indicate sd card check


allLedflash(1000);

  
      //sd card check
     // setRegisterPin(sd_power, HIGH);
     // writeRegisters(); 
       pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin(4)) { //whilst the sd is not okay, the green and red led are turned on alternately
               
notOkayLED(200);

}

allOkayLED (500);              


 //flash all lights to indicate that the sd card is okay and moving onto the next step 
 
allLedflash(1000);


 //checking if CO2 sensor is okay 

      while (airSensor.begin() == false){

      notOkayLED(200);}

      allOkayLED (500); 

 //flash all lights to indicate that the co2 sensor is okay and moving onto the next step 

allLedflash(1000);   

//checking is pressure sensor is okay
     
      while (Pressure.beginI2C()==false){

      notOkayLED(200);}

      allOkayLED (500); 
          
 //flash all lights to indicate that the co2 sensor is okay and moving onto the next step 
 for(int8_t y = 1; y <= 5; y++){
allLedflash(200); } 

      setRegisterPin(sd_power, LOW);
//setRegisterPin(sensorPower, LOW);
  setRegisterPin(relay, LOW);
setRegisterPin(fan, LOW);
  setRegisterPin(servo_power, LOW);
setRegisterPin(tran, LOW);
  setRegisterPin(HC_12, LOW);
setRegisterPin(fan, LOW);

writeRegisters(); */



////////////////////////////////////////////////////////////////////////////////////////////////////////////
//sleep testing 01.12.2020
/*
  //auto_mode ();} // automatic mode on low power switches incase of startup. 
   setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
//delay(500);
 setRegisterPin(relay, LOW);
 writeRegisters(); 
 /////////// 
Low_Power_sleep(60);}
//}*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
//co2 sensor test 
setRegisterPin(sd_power, HIGH);
setRegisterPin(sensorPower, HIGH);
writeRegisters(); 

delay(5000);

long ch4_mapped2 = 0;
float co2_read=0;
float tmp_read=0;
float h20_read=0;
float pressure = 0;
float tmp_Press = 0;

delay(1000);

 airSensor.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)

  airSensor.setAltitudeCompensation(60); //Set altitude of the sensor in m

  airSensor.setAmbientPressure((pressure/100)); //Current ambient pressure in mBar: 700 to 1200



pressure = (Pressure.readFloatPressure());
delay(1000);
ch4_mapped2 = (map((analogRead(sensor)), 0, 1023, 0, 1000000));
delay(1000);
co2_read = airSensor.getCO2();
//co2_read = 0000;
delay(1000);
tmp_read=(airSensor.getTemperature());
//tmp_read=0000;
delay(1000);
h20_read=(airSensor.getHumidity());
//h20_read=0000;
delay(1000);
tmp_Press = (Pressure.readTempC());
delay(1000);
//Serial.println(data_file);



Serial.println("");

Serial.print("CH4");
Serial.println(ch4_mapped2);
Serial.print("CO2");
Serial.println(co2_read);
Serial.print("temp co2");
Serial.println(tmp_read);
Serial.print("h20");
Serial.println(h20_read);
Serial.print("temp press");
Serial.println(tmp_Press);
Serial.print("Pressure");
Serial.println(pressure);
}

/*
Lid_down ();
delay(1000);
Lid_up ();
allLedflash(200);*/
/*
 setRegisterPin(tran, HIGH);
   setRegisterPin(relay, HIGH);
   setRegisterPin(servo_power, HIGH);
   setRegisterPin(blue_LED, HIGH);    //blue light continuous on indicates Lid_up mode
   writeRegisters();
for(int i=0; i<1; i++){
counterclockwise ();}
//delay(5000);
for(int i=0; i<100; i++){
clockwise();}
//delay(5000);
allLedflash(200);*/

/*
 * Testing large stepper code
 * close_sequence ();
open_sequence ();*/
//allLedflash(1000);

//Low_Power_sleep(60);}










void Lid_up () {

 //setRegisterPin(tran, HIGH);
  // setRegisterPin(relay, HIGH);
  // setRegisterPin(servo_power, HIGH);
   setRegisterPin(blue_LED, HIGH);    //blue light continuous on indicates Lid_up mode
   writeRegisters();

  // setRegisterPin(servo_power, HIGH);
   //writeRegisters();


/*  for (pos = 75; pos <= 120; pos += 1) { // disengage ratchet pawl
    myservo.write(pos);              // 
    delay(15);                       // 
  }*/

for(;;){
//counterclockwise ();
//clockwise();
open_sequence ();
}
}



void Lid_down () {
  
  // setRegisterPin(tran, HIGH);
  // setRegisterPin(relay, HIGH);
  // setRegisterPin(servo_power, HIGH);
   setRegisterPin(white_LED, HIGH);  //white LED continuous on indicates Lid_down mode
   writeRegisters();

   //for(;;){

 // for (pos = 100; pos >= 20; pos -= 1) { // disengage ratchet pawl
 //   myservo.write(pos);              // 
  //  delay(15);                       // 
 // }
/*
int8_t lid_position = digitalRead(endstop);
delay(20);

while (lid_position == LOW){    //while the end stop is blocked, open the lid 
   clockwise();

   //counterclockwise ();
   lid_position = digitalRead(endstop);
  }*/

close_sequence ();
for(;;){

  
}
  
  }




void auto_mode () {
 int8_t day_change=0;
for(;;){
  
   setRegisterPin(white_LED, HIGH);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(ledPin, HIGH);//
   writeRegisters();

   delay(1000);

   setRegisterPin(white_LED, LOW);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(ledPin, LOW);//
   setRegisterPin(sensorPower, HIGH);//for i2c line co2 interruption 
   writeRegisters();
   delay(1000);
   
 /*  ////////////////////
  // false motor load
        setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
   writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
   ////////////////////*/
   
tmElements_t tm;
RTC.read(tm); 

int8_t DAY = (tm.Day);
if (DAY == last_Day) {day_change = 0;}
else if (DAY != last_Day) {day_change = 1;}
else {day_change = 2;}
//setRegisterPin(ch4Power, HIGH);
setRegisterPin(CH43, HIGH);
setRegisterPin(CH44, HIGH);
setRegisterPin(sensorPower, HIGH);
writeRegisters(); 

mix_air(5000);

/*
//false motor load
for(int x=1; x<4; x ++){
if ((x==2) || (x ==4)){
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
   delay(5000);} ///change 16.12
else{
delay(15000);}
   
}*/

delay(55000);

       pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}

allOkayLED (500); 
 
 sensors_read (0, day_change);

 close_sequence ();

//allLedflash(200);

//measuring loop set to once every 5 minutes

for(int8_t i = 1 ; i <=  5; i++){

  /*
 //false motor load
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode*/
   
setRegisterPin(sensorPower, HIGH);
writeRegisters();   
allLedflash(200);
RTC.read(tm); 
last_Day = DAY;
if (DAY == last_Day) {day_change = 0;}
else if (DAY != last_Day) {day_change = 1;}
else {day_change = 2;}

setRegisterPin(sensorPower, HIGH);
writeRegisters(); 

//Low_Power_sleep(60);//1 minute deep sleep

Low_Power_sleep(200);//4 minutes deep sleep 

//setRegisterPin(ch4Power,HIGH);
setRegisterPin(CH43, HIGH);
setRegisterPin(CH44, HIGH);
 setRegisterPin(sensorPower,HIGH);
writeRegisters();

delay(61195);

/*
//false motor load
for(int x=1; x<4; x ++){
if ((x==2) || (x ==4)){
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
   delay(5000);} ///change 16.12
else{
delay(15000);}
   
}*/

mix_air(5000);

       pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}

allOkayLED (500); 
   
sensors_read (1, day_change);
}



 //delay(3000);
 //delay(60000); //1 min
// delay(300000); //5 mins

 //mix_air(5000);

//sensors_read (2, day_change);

 open_sequence ();

 //Low_Power_sleep(60);//1 minute deep sleep

 Low_Power_sleep(240);//5 minutes deep sleep
//setRegisterPin(ch4Power,HIGH);
setRegisterPin(CH43, HIGH);
setRegisterPin(CH44, HIGH);
 setRegisterPin(sensorPower,HIGH);
writeRegisters();

delay(51195);


/*
//false motor load
for(int x=1; x<4; x ++){
if ((x==2) || (x ==4)){
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
   delay(5000);} ///change 16.12
else{
delay(10000);}
   
}*/

RTC.read(tm); 
 last_Day = DAY;

////////low power sleep///////*/

  
//allLedflash(200);
//Low_Power_sleep(120);

 
   
  }}




void manual_mode () {


   int8_t day_change=0;
   setRegisterPin(white_LED, HIGH);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(blue_LED, HIGH);
  
   writeRegisters();

   delay(1000);

   
   setRegisterPin(white_LED, LOW);//White with green led flash for 1 second indicates auto_mode
   setRegisterPin(blue_LED, LOW);  
   writeRegisters();

  for(;;){
   
 digitalWrite(setPin, LOW);           // Set HC-12 into AT Command mode
 delay(100);                          // Wait for the HC-12 to enter AT Command mode
 mySerial.print(F("AT+C003"));               // Send AT Command to HC-12
 delay(100);
 digitalWrite(setPin, HIGH);
 delay(100);

 int8_t counter= 0;
 
 command1 = 0;
 command2 = 0;

    setRegisterPin(HC_12, HIGH);
    setRegisterPin(CH43, HIGH);
    setRegisterPin(CH44, HIGH);
  // setRegisterPin(ch4Power, HIGH);
   writeRegisters();

while (counter<=8) {   //this is the time out counter for communication - to make sure that the valve changer has moved on to the next chamber and that the comman recieved is the final command. 
                       //counter was set to 10 but took it down to 8 to see if this would work faster
//if(mySerial.available() > 1){  

/*
////////batt 2 load

if ((counter == 4) || (counter == 8)){
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
   }

/////*/
    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100); 


  
 /*  myFile = SD.open("data.txt", FILE_WRITE);
   if (myFile) {
    //Serial.print("Writing to test.txt...");

    myFile.println(input);
    // close the file:
    myFile.close();}
    delay(100);*/

    
    //delay(100);
    //chamber();
    if (input==2321){
                chamber1();
                 setRegisterPin(blue_LED, HIGH); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 counter=0;
                 command1=1;
                 
                }
                
     else if (input==4536){
                chamber2();
                 setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 counter=0;
                 command2=1;
                }

     
     else if(input== 5412) {end_transmission();
                 setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 counter=0;
     }

     else {delay(100);
                 setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, HIGH); 
                 writeRegisters();
                 counter=(counter+1);
     
     }

     
    
    input = ("");

    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);
    }

//mySerial.end();
delay(500);
//SD.begin(4);
delay(500);

if (command1==1 && command2==0)
{                 setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH); 
                 writeRegisters();
                 
                 delay(1000);
                 
                                 setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 
                 delay(1000);
                       pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
                   sensors_read (1, 0);
                  // setRegisterPin(ch4Power, HIGH);
                  setRegisterPin(CH43, HIGH);
                  setRegisterPin(CH44, HIGH);
                   writeRegisters();
                   close_sequence ();
                   
                      pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}             

                   radio_report(1);
             
              
                 }
                 
else if (command1==0 && command2==1)
{

    setRegisterPin(tran, HIGH);
   writeRegisters();
  delay(50);
  int8_t lid_position = digitalRead(endstop); //added this to make sure no double up comands. If lid is already open, then ignore the open command.
  delay(100);
  setRegisterPin(tran, LOW);
   writeRegisters();
   
   
                   setRegisterPin(blue_LED, HIGH); 
                 setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();

                 delay(1000);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();

                 if (lid_position == LOW){delay(1000); //ignoring open command
                 mix_air(5000);}
                 else {                   //lid confirmed closed command. 
                 delay(1000);
                 mix_air(5000);
                       pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
                 sensors_read (2, 0);
                 //setRegisterPin(ch4Power, HIGH);
                 setRegisterPin(CH43, HIGH);
                 setRegisterPin(CH44, HIGH);
                 writeRegisters();                
                 open_sequence ();}

   pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}

                   radio_report(2);
  }
  
else if (command1==1 && command2==1){                 setRegisterPin(blue_LED, HIGH); 
                 setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH);
                 //setRegisterPin(relay, HIGH); 
                 writeRegisters();
                 

 
                 delay(1000);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW);
                 //setRegisterPin(relay, LOW); 
                 writeRegisters();
                 delay(1000);

   pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
                 
                 radio_report(3);

                 
              /*   //////////// 2nd batt on
 setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();
clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
 /////////// */
                 
                 }
                 
else if (command1==0 && command2==0){
                 
                 counter2=(counter2+1);
                 
                 setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH);
                // setRegisterPin(relay, HIGH); 
                 writeRegisters();
                 
           
                 delay(500);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();

                 delay(500);

                                  setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH); 
                 //setRegisterPin(relay, LOW);
                 writeRegisters();
                 
                 
                 delay(500);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();

   pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
                 
                 radio_report(4);

                 
                /* //////////// 2nd batt on
 setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
 ///////////    */ 
if (counter2>=5){
 
tmElements_t tm;
RTC.read(tm); 

int8_t DAY = (tm.Day);
if (DAY == last_Day) {day_change = 0;}
else if (DAY != last_Day) {day_change = 1;}
else {day_change = 2;}

                 mix_air(5000);
                       pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
                 sensors_read (1, day_change);
                 counter2 = 0;
                 }}

else {                 setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH); 
                 writeRegisters();
                 
                 
                 delay(500);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();

                 delay(500);

                                  setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH); 
                 writeRegisters();
                 
                 
                 delay(500);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 delay(500);

                                                   setRegisterPin(white_LED, HIGH);
                 setRegisterPin(ledPin, HIGH); 
                 setRegisterPin(LedOff, HIGH); 
                 writeRegisters();
                 
                 
                 delay(500);

                                                  setRegisterPin(blue_LED, LOW); 
                 setRegisterPin(white_LED, LOW);
                 setRegisterPin(ledPin, LOW); 
                 setRegisterPin(LedOff, LOW); 
                 writeRegisters();
                 delay(500);

   pinMode(10, OUTPUT); //ss/ cs
       setRegisterPin(sd_power, HIGH);
writeRegisters(); 
       delay(500);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}
  
                 radio_report(5);
                 
    /* ////keep battery 2 on            
  setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
    setRegisterPin(relay, LOW);  //white LED continuous on indicates Lid_down mode
   writeRegisters();
     //////////////////*/

                
//SD.end();
delay(500);
//mySerial.begin(9600);
delay(500);
    
    }}}



  void chamber2 (){
  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(4321);//send unique code to the receiver to turn on. In this case 1111
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
        onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}
 }



void chamber1 (){
  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(5678);//send unique code to the receiver to turn on. In this case 1111
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
        onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}
 }


 void end_transmission(){

  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(7689);//send unique code to the receiver to turn on. In this case 1111
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
        onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}
 }
 





void close_sequence (){

   servoPawl.write(1000); //preset servo angle to prevent 

      setRegisterPin(EN_PIN, LOW); //making sure that the motor is not enabled 
      setRegisterPin(VIO, LOW);     //and making sure that the order of switching on is correct 
                 writeRegisters();
      digitalWrite(strain_pin, HIGH);

                 delay(500);
  

//do one movement of the chamber up and down finishing in the up position, to check all mechanisms are functional
   setRegisterPin(tran, HIGH); //tran = optical endstop
   setRegisterPin(servo_power, HIGH);
   writeRegisters();


  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(5000);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0

  
/*
//opening the pawl 
int val;
 val = incPulse(servoPawl.read(), 1);
  while  ( val<1000){  //was 1700
  
   val = incPulse( servoPawl.read(), 1);
   servoPawl.write(val);
      delay(2); }

      */

int val = servoPawl.read();

 
while  ( val<1400){
  
   val = ((servoPawl.read())+ 1);
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

      setRegisterPin(DIR_PIN, HIGH); //DIR LOW = lid down  
      setRegisterPin(MS1, HIGH);  //MS1 High and Peristaltic_pump LOW = fast steps mode 
      setRegisterPin(MS2, LOW);
      writeRegisters();


//////zeroing lid//////
 int8_t lid_position = digitalRead(endstop); //reading end stop to see if it is active 

delay(50);

      setRegisterPin(EN_PIN, HIGH); //Enable the motor just before movement  
                 writeRegisters();


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


      
for(int i=0; i<450; i++){ //stepping loop was 450 but changed to 200 for chamber 28.07.2021
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(15);
  digitalWrite(STEP_PIN, LOW);
  delay(15);
}




for( int8_t i=0; i<25; i++){ //stepping loop ...was 50 steps 
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(35);
  digitalWrite(STEP_PIN, LOW);
  delay(35);
}



// int val;
 val = servoPawl.read();
while  ( val>1000){
  
   val = (( servoPawl.read())- 1);
   servoPawl.write(val);
      delay(2); }

 int8_t z = 1;

strain_end = ((scale.get_units(1))*0.45359237);

 while ((strain_end < 4.5) && (z<=100)){

  z = (z+1);
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(45);
  digitalWrite(STEP_PIN, LOW);
  delay(45);

  strain_end = ((scale.get_units(1))*0.45359237);

 }


/*
 * replaced this section for the loadcell strain feedback sequence 
for(int i=0; i<45; i++){ //stepping loop
  //make steps
  digitalWrite(STEP_PIN, HIGH);
  delay(45);
  digitalWrite(STEP_PIN, LOW);
  delay(45);
}
*/
      

   setRegisterPin(EN_PIN, LOW); //Disable the motor first  
            writeRegisters();

delay(500);

                 setRegisterPin(VIO, LOW); //motor driver +5V turned off first, before motor voltage
                  writeRegisters();  


strain_end = ((scale.get_units(20))*0.45359237);
  digitalWrite(strain_pin, LOW);

                            
                 setRegisterPin(tran, LOW);
                 setRegisterPin(servo_power, LOW);
                 writeRegisters();
                 
                 delay(500);
                 
               //  digitalWrite(VMOT, LOW); 
               setRegisterPin(VMOT, LOW); //turning on the +5v for the motor driver VIO after the VMOT
                 writeRegisters();
   

}





void open_sequence (){

  servoPawl.write(1000);

      setRegisterPin(EN_PIN, LOW); //making sure that the motor is not enabled 
      setRegisterPin(VIO, LOW);     //and making sure that the order of switching on is correct 
                 writeRegisters();
     digitalWrite(strain_pin, HIGH);
                 delay(500);
  

//do one movement of the chamber up and down finishing in the up position, to check all mechanisms are functional
   setRegisterPin(tran, HIGH); //tran = optical endstop
   setRegisterPin(servo_power, HIGH);
   writeRegisters();

   

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  delay(5000);
  scale.set_scale(calibration_factor); //This value is obtained by using the SparkFun_HX711_Calibration sketch
  scale.tare();  //Assuming there is no weight on the scale at start up, reset the scale to 0


//opening the pawl 
int val = servoPawl.read();

  while  ( val<1400){
  
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
while  ( val>1000){
  
   val = (( servoPawl.read())- 1);
   servoPawl.write(val);
      delay(2); }

      
   setRegisterPin(EN_PIN, LOW); //Disable the motor first  
            writeRegisters();

delay(500);

strain_end = ((scale.get_units(20))*0.45359237);
  digitalWrite(strain_pin, LOW);
        
                 
                 setRegisterPin(VIO, LOW); //motor driver +5V turned off first, before motor voltage             
                 setRegisterPin(tran, LOW);
                 setRegisterPin(servo_power, LOW);
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



//function to give average co2

/*void sensors_read (int chamberCommand, int last_day ) {

   setRegisterPin(sd_power, HIGH);
    setRegisterPin(sensorPower,HIGH);
   writeRegisters();

   delay(200);

   while (!SD.begin(4)) { //whilst the sd is not okay, the green and red led are turned on alternately
               
notOkayLED(500);


}

allLedflash (500);

  // setRegisterPin(CO2_power, HIGH);
  // writeRegisters();

  // delay(200);

      while (!airSensor.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately
               
notOkayLED(500);


}

allLedflash (500);

   while (!Pressure.beginI2C()) { //whilst the sd is not okay, the green and red led are turned on alternately
               
notOkayLED(500);


}

allLedflash (500);

long ch4_mapped2 = 0;
int co2_read=0;
int tmp_read=0;
int h20_read=0;
long pressure = 0;

   setRegisterPin(tran, HIGH);
   writeRegisters();
   int lid_position = digitalRead(endstop);
   delay(500);
   setRegisterPin(tran, LOW);
   writeRegisters();

for(int x=1;x<=5;x++){
co2_read = (co2_read + airSensor.getCO2());
tmp_read = (tmp_read + airSensor.getTemperature());
h20_read = (h20_read + airSensor.getHumidity());
ch4_mapped2 = (ch4_mapped2+(map((analogRead(sensor)), 0, 1023, 0, 1000000)));
ch4_mapped2 = (ch4_mapped2+(map((analogRead(sensor)), 0, 1023, 0, 1000000)));
pressure = (pressure + Pressure.readFloatPressure());
delay(1000);
}

int aveCo2 = (co2_read/5);
int avetmp = (tmp_read/5);
int aveh20 = (h20_read/5);
long avepressure = (pressure/5);
long avech4 = (ch4_mapped2/5);


///////writing to file//////////

tmElements_t tm;
RTC.read(tm); 

char data_file[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

int DAY1 = (tm.Day);
int MONTH1 = (tm.Month);
int YEAR1 = (tm.Year)+1970-2000;

int seconds1 = (tm.Second);
int minutes1 = (tm.Minute);
int hours1 = (tm.Hour);

char data_time[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);


  sprintf(data_time, "%u:%u:%u", hours,minutes,seconds); 
  delay(100);
 // sprintf(data_date, "%u_%u_%u", DAY1,MONTH1,YEAR1);
  sprintf(data_file, "%u_%u_%uD.txt", DAY,MONTH,YEAR);

/////////////////if day change then new file will be made and new table headers are needed//////////////  

if (last_day == 1){
     myFile = SD.open(data_file, FILE_WRITE);
   if (myFile) {
myFile.print("Date");
myFile.print(", ");
myFile.print("Time");
myFile.print(", ");

myFile.print("ACO2");
myFile.print(", ");
myFile.print("SCO2");
myFile.print(", ");

myFile.print("ATmp");
myFile.print(", ");
myFile.print("STmp");
myFile.print(", ");

myFile.print("AH2O");
myFile.print(", ");
myFile.print("SH2O");
myFile.print(", ");

myFile.print("ABar");
myFile.print(", ");
myFile.print("SBar");
myFile.print(", ");

myFile.print("ACH4");
myFile.print(", ");
myFile.print("SCH4");
myFile.print(", ");

myFile.print("Comm");
myFile.print(", ");
myFile.print("Stat");
myFile.println(", ");

// close the file:
   myFile.close();}

  }

else{}
  
 myFile = SD.open(data_file, FILE_WRITE);
 if (myFile) {


//myFile.print(data_file);
//myFile.print(", ");
//myFile.print(data_time);
myFile.print(DAY1);
myFile.print(":");
myFile.print(MONTH1);
myFile.print(":");
myFile.print(YEAR1);
myFile.print(", ");

myFile.print(seconds1);
myFile.print(":");
myFile.print(minutes1);
myFile.print(":");
myFile.print(hours1);
myFile.print(", ");

myFile.print(aveCo2);
myFile.print(", ");
myFile.print(airSensor.getCO2());
myFile.print(", ");

myFile.print(avetmp);
myFile.print(", ");
myFile.print(airSensor.getTemperature());
myFile.print(", ");

myFile.print(aveh20);
myFile.print(", ");
myFile.print(airSensor.getHumidity());
myFile.print(", ");

myFile.print(avepressure);
myFile.print(", ");
myFile.print(Pressure.readFloatPressure());
myFile.print(", ");


myFile.print(avech4);
myFile.print(", ");
myFile.print(map((analogRead(sensor)), 0, 1023, 0, 1000000));
myFile.print(", ");

myFile.print(chamberCommand);
myFile.print(", ");
myFile.print(lid_position);
myFile.println(", ");
myFile.close();}


   setRegisterPin(sd_power, LOW);
   writeRegisters();

         setRegisterPin(sensorPower, LOW); 
         writeRegisters();
}*/



void sensors_read (int8_t chamberCommand, int8_t last_day ) { //no co2 sensor 



    setRegisterPin(sd_power, HIGH);
    setRegisterPin(sensorPower,HIGH);
    setRegisterPin(CH43,HIGH);
    setRegisterPin(CH44,HIGH);

   writeRegisters();


     delay(1000);  
       while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(100);
notOkayLED(200);}



//CO2 sensor setup

tcaselect(0);

      while (airSensor1.begin() == false){

      notOkayLED(200);
      }


tcaselect(1);

      while (airSensor2.begin() == false){

      notOkayLED(200);

      }

//checking if pressure sensor is okay
     
while (Pressure1.beginI2C()==false){

      notOkayLED(200);

      } 


while (Pressure2.beginI2C()==false){
  
      notOkayLED(200);

      }


//methane sensor
ads.setGain(GAIN_TWOTHIRDS); 


while (!ads.begin(0x49)) {
       notOkayLED(200);
  }



tmElements_t tm;
RTC.read(tm); 

char data_file[12];
int8_t* DAY = (tm.Day);
int8_t* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;


int8_t DAY1 = (tm.Day);
int8_t MONTH1 = (tm.Month);
int8_t YEAR1 = (tm.Year)+1970-2000;

int8_t seconds = (tm.Second);
int8_t minutes = (tm.Minute);
int8_t hours = (tm.Hour);


  sprintf(data_file, "%u_%u_%u.txt", DAY,MONTH,YEAR);

if (last_day == 1){
   // myFile = SD.open(data_date, FILE_WRITE);
   myFile = SD.open(data_file, FILE_WRITE);
   if (myFile) {
    //Serial.println("jhh");
myFile.print(F("Date"));
myFile.print(F(", "));
myFile.print(F("Time"));
myFile.print(F(", "));
//myFile.print(F("TotalSec"));
//myFile.print(F(", "));

myFile.print(F("Sens")); //sensor number i.e. 1 or 2
myFile.print(F(", "));

//myFile.print("ACO2");
//myFile.print(", ");
myFile.print(F("CO2"));
myFile.print(F(", "));

//myFile.print("ATmp");
//myFile.print(", ");
myFile.print(F("Tmp"));
myFile.print(F(", "));

//myFile.print("AH2O");
//myFile.print(", ");
myFile.print(F("H2O"));
myFile.print(F(", "));


//myFile.print("ACH4");
//myFile.print(", ");
myFile.print(F("CH4_d"));
myFile.print(F(", "));

myFile.print(F("CH4_mv"));
myFile.print(F(", "));

//myFile.print("ABar");
//myFile.print(", ");
myFile.print(F("Bar"));
myFile.print(F(", "));

myFile.print(F("TMP_P"));
myFile.print(F(", "));

myFile.print(F("Comm"));
myFile.print(F(", "));

myFile.print(F("Stat"));
myFile.println(F(", "));

myFile.print(F("F_e"));
myFile.println(F(", "));



// close the file:
   myFile.close();}

  }



float ch4_differential = 0;
float ch4_mv = 0;
int co2_read=0;
float tmp_read=0;
float h20_read=0;
int pressure = 0;
int tmp_Press = 0;


////read endstop state////
   setRegisterPin(tran, HIGH);
   writeRegisters();
  delay(50);
  int8_t lid_position = digitalRead(endstop);
  delay(100);
  setRegisterPin(tran, LOW);
   writeRegisters();


///////writing to file//////////



  //setRegisterPin(Peristaltic_pump, HIGH);
 //writeRegisters();

// digitalWrite(peri, HIGH);
 
  for(int8_t x=0;x<=10;x++){

if ( (x % 2) == 0) { //takes alternate readings from sensors 1 and sensors 2 but averages the same time period for both

 airSensor1.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
 airSensor1.setAltitudeCompensation(60); //Set altitude of the sensor in m



 float multiplier = 0.1875F; 

ch4_differential = ads.readADC_Differential_0_1();
ch4_mv = (ch4_differential * multiplier);
delay(100);



tcaselect(0);
airSensor1.setAmbientPressure((pressure/100)); //Current ambient pressure in mBar: 700 to 1200
delay(100);
co2_read = airSensor1.getCO2();
//co2_read = 0000;
delay(100);
tmp_read=(airSensor1.getTemperature());
delay(100);
h20_read=(airSensor1.getHumidity());
delay(100);

pressure = (Pressure2.readFloatPressure());
delay(100);
tmp_Press = (Pressure2.readTempC());
delay(100);
}

else{

 airSensor2.setMeasurementInterval(2); //Change number of seconds between measurements: 2 to 1800 (30 minutes)
 airSensor2.setAltitudeCompensation(60); //Set altitude of the sensor in m

 float multiplier = 0.1875F; 

ch4_differential = ads.readADC_Differential_2_3();
ch4_mv = (ch4_differential * multiplier);
delay(100);
pressure = (Pressure2.readFloatPressure());
delay(100);
tcaselect(1);
airSensor2.setAmbientPressure((pressure/100)); //Current ambient pressure in mBar: 700 to 1200
delay(100);
co2_read = airSensor2.getCO2();
//co2_read = 0000;
delay(100);
tmp_read=(airSensor2.getTemperature());
delay(100);
h20_read=(airSensor2.getHumidity());
delay(100);
tmp_Press = (Pressure2.readTempC());
delay(100);
}
 

delay(1000);
  //Serial.println(data_file);
 myFile = SD.open(data_file, FILE_WRITE);
 if (myFile) {

    //Serial.println("jhh");
//myFile.print(data_file);
//myFile.print(", ");
//myFile.print(data_time);
myFile.print(DAY1);   ///this coul be day month year to save memory
myFile.print(F(":"));
myFile.print(MONTH1);
myFile.print(F(":"));
myFile.print(YEAR1);
myFile.print(F(", "));

myFile.print(hours);
myFile.print(F(":"));
myFile.print(minutes);
myFile.print(F(":"));
myFile.print(seconds);
myFile.print(F(", "));

//myFile.print(Total_seconds);
//myFile.print(F(", "));

/*
if ( (x % 2) == 0) { myFile.print("1");
myFile.print(F(", "));}

else {myFile.print("2");
myFile.print(F(", "));}*/
myFile.print((x % 2));
myFile.print(F(", "));


//myFile.print(aveCo2);
//myFile.print(", ");
myFile.print(co2_read);
myFile.print(F(", "));

//myFile.print(avetmp);
//myFile.print(", ");
myFile.print(tmp_read);
myFile.print(F(", "));

//myFile.print(aveh20);
//myFile.print(", ");
myFile.print(h20_read);
myFile.print(F(", "));


myFile.print(ch4_differential);
myFile.print(F(", "));

myFile.print(ch4_mv);
myFile.print(F(", "));

//myFile.print(avepressure);
//myFile.print(", ");
myFile.print(pressure);
myFile.print(F(", "));

myFile.print(tmp_Press);
myFile.print(F(", "));


myFile.print(chamberCommand);
myFile.print(F(", "));

myFile.print(lid_position);
myFile.println(F(", "));

myFile.print(strain_end, 3);
myFile.println(F(", "));
myFile.close();}
}

last_day = 0;
//if (x>=3){ //100ml per 20 seconds. x>= 3 is 20 seconds, x>=2 is 15 seconds 
 // setRegisterPin(Peristaltic_pump, LOW);
 //writeRegisters();
// digitalWrite(peri, LOW);





//setRegisterPin(relay, HIGH); //added 26.11.2020


delay(1000);

   setRegisterPin(sd_power, LOW);
   //setRegisterPin(relay, LOW); //added 26.11.2020 
   setRegisterPin(sensorPower, LOW); //not sure if this was meant to be low or high, was left high before
  // setRegisterPin(ch4Power,LOW);
   setRegisterPin(CH43,LOW);
   setRegisterPin(CH44,LOW);
         writeRegisters();

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

 for(int8_t i=1;i<=1;i++){ 
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

void radio_report (int8_t x){

  //sd power on 
   setRegisterPin(sd_power, HIGH);
   setRegisterPin(sensorPower, HIGH);
   writeRegisters();

   delay(1000);
//

      while (!SD.begin()) { //whilst the sd is not okay, the green and red led are turned on alternately //changed sd.begin (4) to sd begin () to use the automatic
               delay(2000);
notOkayLED(200);

}

             





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


void Low_Power_sleep (long x){
setRegisterPin(sensorPower, HIGH);
writeRegisters(); 
tmElements_t tm;
RTC.read(tm);

//long Total_seconds = ((tm.Hour*3600)+(tm.Minute*60)+(tm.Second));  

//long T_secs = (tm.Second);
//long T_mins = (tm.Minute);
//long T_hours = (tm.Hour);
//int T_days = (tm.Day);



delay(1000);
//long Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 
long Total_seconds2 = (((tm.Hour)*3600)+((tm.Minute)*60)+(tm.Second));


long alarmTime = (Total_seconds2 + (x));
//long alarmTime2 = (Total_seconds2 + x+30);

delay(1000);

RTC.read(tm);

//T_secs = (tm.Second);
//T_mins = (tm.Minute);
//T_hours = (tm.Hour);
//Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 
Total_seconds2 = (((tm.Hour)*3600)+((tm.Minute)*60)+(tm.Second));

//Serial.print("alarmTime before sleep ");
//Serial.println(alarmTime);

//Serial.print("recorded time before sleep ");
//Serial.println(Total_seconds2);

buttonState = (tm.Day);

setRegisterPin(sensorPower, LOW);
writeRegisters(); 


if (buttonState == lastButtonState) {

 //lastButtonState = buttonState;


while (Total_seconds2 < alarmTime && buttonState == lastButtonState)
{

//for (int v = 0; v < 2; v++){ 
  
 for (int8_t i = 0; i < 19; i++) 
{ pinMode(i, OUTPUT); }     //all pins set to output so save maximum power...there are 19 pins


WDTCSR = (24);//change enable and WDE - also resets 
WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit 
WDTCSR |= (1<<6);//enable interrupt mode 

//Disable ADC - don't forget to flip back after waking up if using ADC in your application 
ADCSRA |= (1 << 7); 
ADCSRA &= ~(1 << 7); //ENABLE SLEEP - this enables the sleep mode 
SMCR |= (1 << 2); //power down mode 
SMCR |= 1;//enable sleep


for(int8_t c=0; c<2; c++) {//gives 16 seconds
MCUCR |= (3 << 5); //set both BODS and BODSE at the same time 
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time 
__asm__ __volatile__("sleep");//in line assembler to go to sleep 

} 
 
  
  
//delay (200);

 // setRegisterPin(ch4Power, HIGH);
 setRegisterPin(CH43, HIGH);
setRegisterPin(CH44, HIGH);
  setRegisterPin(sensorPower, HIGH); // added 26.22.2020
  //setRegisterPin(relay, HIGH);
   setRegisterPin(white_LED, HIGH);
   writeRegisters();
/*//delay(1000);

  
//////////// 2nd batt on
 setRegisterPin(relay, HIGH);  //white LED continuous on indicates Lid_down mode
 writeRegisters();

clockwise(1750, 25, 8);
//delay(500);
 setRegisterPin(relay, LOW);
 writeRegisters(); 
 /////////// */
 
 //delay(500);
tmElements_t tm;  
 //delay(500);
 delay(500);
while(RTC.read(tm)==false){//Serial.println("no rtc");}// sensor board needs to be high to read the rtc
//}
}
delay(500);
//long T_secs = (tm.Second);
//long T_mins = (tm.Minute);
//long T_hours = (tm.Hour);
//Total_seconds2= ((T_hours*3600)+(T_mins*60)+(T_secs));
Total_seconds2 = (((tm.Hour)*3600)+((tm.Minute)*60)+(tm.Second));
delay(500);
buttonState = (tm.Day);
//setRegisterPin(sensorPower, LOW);
//setRegisterPin(white_LED, HIGH);
//writeRegisters(); 

//Serial.print("alarmTime ");
//Serial.println(alarmTime);

//Serial.print("recorded time ");
//Serial.println(Total_seconds2);

//setRegisterPin(white_LED, LOW);
//writeRegisters();

 // setRegisterPin(ch4Power, LOW);
 setRegisterPin(CH43, LOW);
setRegisterPin(CH44, LOW);
  setRegisterPin(sensorPower, LOW); //was high 26.11.2020 
  //setRegisterPin(relay, LOW);
   setRegisterPin(white_LED, LOW);
writeRegisters(); 

//}

}}

else { lastButtonState = buttonState;
  
  //Serial.println("NOT OKAY");
}



/*
///////Turn on power for power for sensors, warm up time//////

         setRegisterPin(sensorPower, HIGH); 
         writeRegisters();

buttonState = (tm.Day);


if (buttonState == lastButtonState) {

 lastButtonState = buttonState;


while (Total_seconds2 < alarmTime2 && buttonState == lastButtonState)
{


  
 //for (int i = 0; i < 19; i++) 
//{ pinMode(i, OUTPUT); }     //all pins set to output so save maximum power...there are 19 pins


WDTCSR = (24);//change enable and WDE - also resets 
WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit 
WDTCSR |= (1<<6);//enable interrupt mode 

//Disable ADC - don't forget to flip back after waking up if using ADC in your application 
ADCSRA |= (1 << 7); 
ADCSRA &= ~(1 << 7); //ENABLE SLEEP - this enables the sleep mode 
SMCR |= (1 << 2); //power down mode 
SMCR |= 1;//enable sleep


for(int i=0; i<2; i++) {//gives 16 seconds
MCUCR |= (3 << 5); //set both BODS and BODSE at the same time 
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time 
__asm__ __volatile__("sleep");//in line assembler to go to sleep 
} 

ADCSRA |= (1 >> 7); //enable adc?
  
  
  delay (1000);
RTC.read(tm);
T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs));
buttonState = (tm.Day);


}}*/

ADCSRA |= (1 << 7); 
//ADCSRA &= ~(0 << 7); //ENABLE SLEEP - this enables the sleep mode 

//&= ~(1 << 5)
pinMode(sensor, INPUT);
pinMode(endstop, INPUT);
//pinMode (mode_switch, INPUT);
//pinMode (setup_switch, INPUT);

  
}

/*
void Low_Power_sleep(){

////check the time using the real time clock. If it is after 6pm or before 9am stay in the while loop and do nothing/////////////
tmElements_t tm;
 
RTC.read(tm);

//long Total_seconds = ((tm.Hour*3600)+(tm.Minute*60)+(tm.Second));  

long T_secs = (tm.Second);
long T_mins = (tm.Minute);
long T_hours = (tm.Hour);
long T_days = (tm.Day);



delay(500);
long Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 
delay(50);

char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

char data3[12];



/*
//while (Total_seconds2 < 32400 || Total_seconds2 > 64800){ //sleep from 6pm to 9am.
//while (Total_seconds2 < 32400 || Total_seconds2 > 39600){ //sleep from 11am to 9am.
 // while (Total_seconds2 < 50400 || Total_seconds2 > 64800){ //sleep from 11am to 9am.
//while (Total_seconds2 < 600 || Total_seconds2 > 82800){ //sleep from 11:50pm to 12:10am.
while (Total_seconds2 < 32400 || Total_seconds2 > 61200){ //sleep from 5pm to 9am.
  
data[12];
DAY = (tm.Day);
MONTH = (tm.Month);
YEAR = (tm.Year)+1970-2000;

data2[12];
seconds = (tm.Second);
minutes = (tm.Minute);
hours = (tm.Hour);

data3[12];



//
    
 //for (int i = 0; i < 16; i++) 
//{ if(i != 2)//just because the button is hooked up to digital pin 2 
//pinMode(i, OUTPUT); }

WDTCSR = (24);//change enable and WDE - also resets 
WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit 
WDTCSR |= (1<<6);//enable interrupt mode 

//Disable ADC - don't forget to flip back after waking up if using ADC in your application 
ADCSRA |= (1 << 7); 
ADCSRA &= ~(1 << 7); //ENABLE SLEEP - this enables the sleep mode 
SMCR |= (1 << 2); //power down mode 
SMCR |= 1;//enable sleep


for(int i=0; i<2; i++) {//gives 16 seconds
MCUCR |= (3 << 5); //set both BODS and BODSE at the same time 
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time 
__asm__ __volatile__("sleep");//in line assembler to go to sleep 

}}*/

/*tmElements_t tm;
 
RTC.read(tm);
T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
T_days = (tm.Day);

Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 

 */

/* 
RTC.read(tm);

//long Total_seconds = ((tm.Hour*3600)+(tm.Minute*60)+(tm.Second));  

T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
T_days = (tm.Day);



delay(500);
Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 


 //long alarmTime2 = (Total_seconds2 + 30);
 long alarmTime2 = (Total_seconds2 + 50); //this is the sleep time. SO for a 15 minute run, this will be set to 14:30
 long alarmTime3 = (Total_seconds2 + 10); //this is the precision activation time


delay(500);

RTC.read(tm);

T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs)); 



buttonState = (tm.Day);





while (Total_seconds2 < alarmTime2) //
{


  
// for (int i = 0; i < 16; i++) 
//{ if(i != 2)//just because the button is hooked up to digital pin 2 
//pinMode(i, OUTPUT); }

WDTCSR = (24);//change enable and WDE - also resets 
WDTCSR = (33);//prescalers only - get rid of the WDE and WDCE bit 
WDTCSR |= (1<<6);//enable interrupt mode 

//Disable ADC - don't forget to flip back after waking up if using ADC in your application 
ADCSRA |= (1 << 7); 
ADCSRA &= ~(1 << 7); //ENABLE SLEEP - this enables the sleep mode 
SMCR |= (1 << 2); //power down mode 
SMCR |= 1;//enable sleep


for(int i=0; i<2; i++) {//gives 16 seconds
MCUCR |= (3 << 5); //set both BODS and BODSE at the same time 
MCUCR = (MCUCR & ~(1 << 5)) | (1 << 6); //then set the BODS bit and clear the BODSE bit at the same time 
__asm__ __volatile__("sleep");//in line assembler to go to sleep 
} 
 
  
  
delay (500);
tmElements_t tm;
RTC.read(tm);
T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs));
buttonState = (tm.Day);


}



//else { lastButtonState = buttonState;
  
  //Serial.println("NOT OKAY");
//}

//////the initial wake up time alarm has been reached, the slider position is now zeroed ready for an accurate start time//////


 /* ///////////reset pinmodes/////////
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(signalPin,OUTPUT); 
  pinMode(volt_read, INPUT);
  pinMode(enPin,OUTPUT);
  //digitalWrite(enPin,LOW);
  pinMode(rain_analog, INPUT);
  pinMode(led,OUTPUT);
  pinMode(relay, OUTPUT);
 // digitalWrite(relay, HIGH);

  pinMode(limitSwitch, INPUT);
   pinMode(raindigi, INPUT);

pinMode(transistors, OUTPUT);


//zeroing the slider position

  delay(500);
  digitalWrite(enPin,HIGH);
  digitalWrite(transistors, HIGH);
    digitalWrite(relay, HIGH);
    delay(1000);

    digitalWrite(enPin,LOW);
delay(50);
  while (digitalRead(limitSwitch) == LOW)
{ 
  digitalWrite(dirPin,HIGH); // Enables the motor to move in a particular direction
  // Makes 200 pulses for making one full cycle rotation
  //for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin,HIGH); 
    delayMicroseconds(100); 
    digitalWrite(stepPin,LOW); 
    delayMicroseconds(100); 
  }//}

  
   digitalWrite(enPin,HIGH);

///////Now the slider is zeroed, we wait for the accurate start time alarm////////   */
/*
while (Total_seconds2 < alarmTime3){ //waiting for more precise start time
  RTC.read(tm);
T_secs = (tm.Second);
T_mins = (tm.Minute);
T_hours = (tm.Hour);
Total_seconds2 = ((T_hours*3600)+(T_mins*60)+(T_secs));
}



  
}*/

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
