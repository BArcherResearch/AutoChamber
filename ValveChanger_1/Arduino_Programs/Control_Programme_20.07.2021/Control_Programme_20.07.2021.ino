
//WInd sensor feedback loop
//this sketch is a safety to make sure the LGR does not break, e.g. when in manual mode forgetting to switch the two solenoids on at once 

//To begin with, the LGR should not be plugged into the valve changer module. , so as to make sure the LGR begins.
//once a conformation signal is recieved from the valve changer board, the relay is opened, turning the LGR on. 
//Once the LGR begins to pump air, plug it back into the valve changer. This should turn off the LED as confirmation that air flow is detected 
//now the circuit is in protection mode, the next time no air flow is detected the system will hard shut down the LGR .

//pull sdo pin on pressure sensor high to use 0x77 address on the pressure sensor. Pull it low for the 0x76 address 

//20.07.2021 edit to include the pressure control setting in the manual chamber method 
//will put the pressure check in the loop with the lgr off switch loop check. and set the lgr pressure at the start after the off switch has been selected once





/*Wind Sensor Signals    Arduino
 GND                    GND
 +V                     5V
 RV                     A1    // modify the definitions below to use other pins
 TMP                    A0    // modify the definitions below to use other pins
 

 
 Hardware setup:
 Wind Sensor is powered from a regulated five volt source.
 RV pin and TMP pin are connected to analog innputs.
 
 */
 #include "SparkFunBME280.h"
BME280 Pressure;
BME280 Pressure2;

#include <Time.h>
#include <DS1307RTC.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
File myFile;
File myFile1;
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
 
#define analogPinForRV    A3   // change to pins you the analog pins are using
#define analogPinForTMP   A2


int lgr_relay = 6;

int rst = 5; 

int mode_signal= 8; 

int lgr_control= 9; 

int lgr_on_led= 2; 

int lgr_off_led = 3; 

int rv = A3; 

int tmp = A2; 

int lgr_manual_switch = A1; 



int switch_state; 

// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it. 

const float zeroWindAdjustment =  .5; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;


int TMP_Therm_ADunits2;  //temp termistor value from wind sensor
float RV_Wind_ADunits2;    //RV output from wind sensor 
float RV_Wind_Volts2;

int TempCtimes100_2;
float zeroWind_ADunits_2;
float zeroWind_volts_2;
float WindSpeed_MPH2;



int TMP_Therm_ADunits3;  //temp termistor value from wind sensor
float RV_Wind_ADunits3;    //RV output from wind sensor 
float RV_Wind_Volts3;

int TempCtimes100_3;
float zeroWind_ADunits_3;
float zeroWind_volts_3;
float WindSpeed_MPH3;



int TMP_Therm_ADunits4;  //temp termistor value from wind sensor
float RV_Wind_ADunits4;    //RV output from wind sensor 
float RV_Wind_Volts4;

int TempCtimes100_4;
float zeroWind_ADunits_4;
float zeroWind_volts_4;
float WindSpeed_MPH4;



int TMP_Therm_ADunits5;  //temp termistor value from wind sensor
float RV_Wind_ADunits5;    //RV output from wind sensor 
float RV_Wind_Volts5;

int TempCtimes100_5;
float zeroWind_ADunits_5;
float zeroWind_volts_5;
float WindSpeed_MPH5;




void setup() {
//pressure sensor setup 
  Serial.begin(9600);
  delay(100);
   Serial.println ("begin");
Pressure2.setI2CAddress(0x77); // inlet pressure sensor
 Serial.println ("...");
Pressure.setI2CAddress(0x76); // outlet pressure sensor
 Serial.println ("...");
//Serial.println ("Address");
  while (Pressure.beginI2C()==false){
    delay(50);
    Serial.println("pressure sensor 1 false");
    delay(50);    
  }
  
  Serial.println ("pressure 1 okay");
    while (Pressure2.beginI2C()==false){
       delay(50);    
    Serial.println("pressure sensor 2 false");
     delay(50);
  }

  Serial.println ("pressure 2 okay");
  
  pinMode(lgr_relay, OUTPUT);
  pinMode(rst, OUTPUT);
  pinMode(mode_signal, INPUT);
  pinMode(lgr_control, INPUT);
  pinMode(lgr_on_led, OUTPUT);
  pinMode(lgr_off_led, OUTPUT);
  pinMode(rv, INPUT);
  pinMode(tmp, INPUT);
  pinMode(lgr_manual_switch, INPUT);

  digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);


//CHecking if rtc is okay

 tmElements_t tm;
     
     if (RTC.read(tm)) {  //if okay flash green led once for 1 second             
Serial.println("rtc okay");
     
                }

      else if (RTC.chipPresent()) { //if the chip needs resetting flash the red led 

        
                //delay(1000);
                for(;;){
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, HIGH);
   delay(1000);
   digitalWrite(lgr_off_led, LOW);
   digitalWrite(lgr_on_led, LOW);
   delay(1000);

   Serial.println(" no rtc 1");
                }}
      

      else {   //if the chip not detected at all then keep the LED on 
               for(;;){
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, HIGH);
   delay(250);
   digitalWrite(lgr_off_led, LOW);
   digitalWrite(lgr_on_led, LOW);
   delay(250);
 Serial.println("no rtc 2");

               }
      }


//sd card check

     pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin(4)) { //whilst the sd is not okay, the green and red led are turned on alternately
               
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, HIGH);
   //Serial.println("sd not okay");
 }
 Serial.println("sd okay");



 //system start sd write 

    //tmElements_t tm;
   RTC.read(tm);
   
char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("System ");
    myFile1.println("Restarted ");
    // close the file:
    myFile1.close();}
    delay(100);


/*

//while (digitalRead (valves_open_command) == LOW) {
    //waiting for okay call from valve module
 //   digitalWrite(relay, LOW);
//  }

//  digitalWrite(relay, HIGH);
 // delay(1000);

  //waiting for initial air flow to be detected 


    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);



    delay(200);

    TMP_Therm_ADunits2 = analogRead(analogPinForTMP);
    RV_Wind_ADunits2 = analogRead(analogPinForRV);
   RV_Wind_Volts2 = (RV_Wind_ADunits2 *  0.0048828125);
    delay(200);

    TMP_Therm_ADunits3 = analogRead(analogPinForTMP);
    RV_Wind_ADunits3 = analogRead(analogPinForRV);
   RV_Wind_Volts3 = (RV_Wind_ADunits3 *  0.0048828125);

    delay(200);

    TMP_Therm_ADunits4 = analogRead(analogPinForTMP);
    RV_Wind_ADunits4 = analogRead(analogPinForRV);
   RV_Wind_Volts4 = (RV_Wind_ADunits4 *  0.0048828125);
    delay(200);


    TMP_Therm_ADunits5 = analogRead(analogPinForTMP);
    RV_Wind_ADunits5 = analogRead(analogPinForRV);
    RV_Wind_Volts5 = (RV_Wind_ADunits5 *  0.0048828125);
    delay(200);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_2 = (0.005 *((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2)) - (16.862 * (float)TMP_Therm_ADunits2) + 9075.4;  

    zeroWind_ADunits_2 = -0.0006*((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2) + 1.0727 * (float)TMP_Therm_ADunits2 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_2 = (zeroWind_ADunits_2 * 0.0048828125) - zeroWindAdjustment;  



        TempCtimes100_3 = (0.005 *((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3)) - (16.862 * (float)TMP_Therm_ADunits3) + 9075.4;  

    zeroWind_ADunits_3 = -0.0006*((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3) + 1.0727 * (float)TMP_Therm_ADunits3 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_3 = (zeroWind_ADunits_3 * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_4 = (0.005 *((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4)) - (16.862 * (float)TMP_Therm_ADunits4) + 9075.4;  

    zeroWind_ADunits_4 = -0.0006*((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4) + 1.0727 * (float)TMP_Therm_ADunits4 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_4 = (zeroWind_ADunits_4* 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_5 = (0.005 *((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5)) - (16.862 * (float)TMP_Therm_ADunits5) + 9075.4;  

    zeroWind_ADunits_5 = -0.0006*((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5) + 1.0727 * (float)TMP_Therm_ADunits5 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_5 = (zeroWind_ADunits_5 * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);  
    WindSpeed_MPH2 =  pow(((RV_Wind_Volts2 - zeroWind_volts_2) /.2300) , 2.7265);  
     WindSpeed_MPH3 =  pow(((RV_Wind_Volts3 - zeroWind_volts_3) /.2300) , 2.7265);  
      WindSpeed_MPH4 =  pow(((RV_Wind_Volts4 - zeroWind_volts_4) /.2300) , 2.7265);  
       WindSpeed_MPH5 =  pow(((RV_Wind_Volts5 - zeroWind_volts_5) /.2300) , 2.7265);   



float   average_speed_per_second= ((WindSpeed_MPH + WindSpeed_MPH2 + WindSpeed_MPH3 + WindSpeed_MPH4 + WindSpeed_MPH5)/5);



int hhhh = (int) average_speed_per_second;
Serial.println(hhhh);

while (hhhh<12) {
  digitalWrite (lgr_off_led, HIGH);
  digitalWrite (lgr_on_led, HIGH);
  
  TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);



    delay(200);

    TMP_Therm_ADunits2 = analogRead(analogPinForTMP);
    RV_Wind_ADunits2 = analogRead(analogPinForRV);
   RV_Wind_Volts2 = (RV_Wind_ADunits2 *  0.0048828125);
    delay(200);

    TMP_Therm_ADunits3 = analogRead(analogPinForTMP);
    RV_Wind_ADunits3 = analogRead(analogPinForRV);
   RV_Wind_Volts3 = (RV_Wind_ADunits3 *  0.0048828125);

    delay(200);

    TMP_Therm_ADunits4 = analogRead(analogPinForTMP);
    RV_Wind_ADunits4 = analogRead(analogPinForRV);
   RV_Wind_Volts4 = (RV_Wind_ADunits4 *  0.0048828125);
    delay(200);


    TMP_Therm_ADunits5 = analogRead(analogPinForTMP);
    RV_Wind_ADunits5 = analogRead(analogPinForRV);
    RV_Wind_Volts5 = (RV_Wind_ADunits5 *  0.0048828125);
    delay(200);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_2 = (0.005 *((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2)) - (16.862 * (float)TMP_Therm_ADunits2) + 9075.4;  

    zeroWind_ADunits_2 = -0.0006*((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2) + 1.0727 * (float)TMP_Therm_ADunits2 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_2 = (zeroWind_ADunits_2 * 0.0048828125) - zeroWindAdjustment;  



        TempCtimes100_3 = (0.005 *((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3)) - (16.862 * (float)TMP_Therm_ADunits3) + 9075.4;  

    zeroWind_ADunits_3 = -0.0006*((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3) + 1.0727 * (float)TMP_Therm_ADunits3 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_3 = (zeroWind_ADunits_3 * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_4 = (0.005 *((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4)) - (16.862 * (float)TMP_Therm_ADunits4) + 9075.4;  

    zeroWind_ADunits_4 = -0.0006*((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4) + 1.0727 * (float)TMP_Therm_ADunits4 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_4 = (zeroWind_ADunits_4* 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_5 = (0.005 *((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5)) - (16.862 * (float)TMP_Therm_ADunits5) + 9075.4;  

    zeroWind_ADunits_5 = -0.0006*((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5) + 1.0727 * (float)TMP_Therm_ADunits5 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_5 = (zeroWind_ADunits_5 * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);  
    WindSpeed_MPH2 =  pow(((RV_Wind_Volts2 - zeroWind_volts_2) /.2300) , 2.7265);  
     WindSpeed_MPH3 =  pow(((RV_Wind_Volts3 - zeroWind_volts_3) /.2300) , 2.7265);  
      WindSpeed_MPH4 =  pow(((RV_Wind_Volts4 - zeroWind_volts_4) /.2300) , 2.7265);  
       WindSpeed_MPH5 =  pow(((RV_Wind_Volts5 - zeroWind_volts_5) /.2300) , 2.7265);   



average_speed_per_second= ((WindSpeed_MPH + WindSpeed_MPH2 + WindSpeed_MPH3 + WindSpeed_MPH4 + WindSpeed_MPH5)/5);



hhhh = (int) average_speed_per_second;
}

  digitalWrite (lgr_off_led, LOW);
  digitalWrite (lgr_on_led, HIGH);
  digitalWrite (lgr_relay, HIGH);*/

switch_state = digitalRead (lgr_manual_switch);
digitalWrite(rst, LOW);

} ////end of setup



void loop() {

  delay(3000);
int mode_call = digitalRead(mode_signal);
int GHG_call = digitalRead(lgr_control);

while((GHG_call > 0) && (mode_call >0) ){
digitalWrite(lgr_off_led, HIGH);
delay(500);
digitalWrite(lgr_off_led, LOW);
delay(500);
mode_call = digitalRead(mode_signal);
GHG_call = digitalRead(lgr_control);
}


  //start up sequence before entering switch off loop 

mode_call = digitalRead(mode_signal);
GHG_call = digitalRead(lgr_control);

if ((mode_call == 1) && (GHG_call == 0) ){
  manual_mode();
}


else if (mode_call == 0 && (GHG_call == 1) ){

  auto_mode();
}

}



void manual_mode(){
tmElements_t tm;
RTC.read(tm);
   
char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("Manual ");
    myFile1.print("Mode ");
    myFile1.println("Selected ");

    // close the file:
    myFile1.close();}
    delay(100);


//when the switch state changes the lgr is turned on 
int current_switch_state = digitalRead (lgr_manual_switch);
while(current_switch_state == switch_state){

    digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);
   current_switch_state = digitalRead (lgr_manual_switch);
 }


    digitalWrite(lgr_relay, HIGH);
   digitalWrite(lgr_off_led, LOW);
   digitalWrite(lgr_on_led, HIGH);

  
 
 


  delay(50);
switch_state = digitalRead(lgr_manual_switch);
  delay(50);

//pressure check sequence 

///confirm LGR is on safe

//when the switch state changes the pressure baseline is taken
current_switch_state = digitalRead (lgr_manual_switch);
while(current_switch_state == switch_state){
digitalWrite(lgr_on_led, HIGH);
digitalWrite(lgr_off_led, LOW);
delay(50);
digitalWrite(lgr_on_led, LOW);
delay(50);
digitalWrite(lgr_on_led, HIGH);
delay(50);
digitalWrite(lgr_on_led, LOW);
delay(250);
digitalWrite(lgr_on_led, LOW);
digitalWrite(lgr_off_led, HIGH);
delay(50);
digitalWrite(lgr_off_led, LOW);
delay(50);
digitalWrite(lgr_off_led, HIGH);
delay(50);
digitalWrite(lgr_off_led, LOW);

current_switch_state = digitalRead (lgr_manual_switch);
delay(250);

  }

  delay(50);
 switch_state = digitalRead(lgr_manual_switch);
  delay(50);

////////

//measure baseline pressures when the systen is okay 
data[12];
DAY = (tm.Day);
MONTH = (tm.Month);
YEAR = (tm.Year)+1970-2000;

data2[12];
seconds = (tm.Second);
minutes = (tm.Minute);
hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("Authorisation ");
    myFile1.print("recieved ");
    myFile1.print(": ");
    myFile1.print("System ");
    myFile1.print("is ");
    myFile1.println("safe ");
    // close the file:
    myFile1.close();}
    delay(100);

  digitalWrite(lgr_on_led, HIGH);
digitalWrite(lgr_off_led, HIGH);

int long pressure1 = (Pressure.readFloatPressure());
//Serial.print("Pressure1 ");
//Serial.println(pressure1);
delay(1000);
int long pressure2 = (Pressure.readFloatPressure());
//Serial.print("Pressure2 ");
//Serial.println(pressure2);
delay(1000);
int long pressure3 = (Pressure.readFloatPressure());
//Serial.print("Pressure3 ");
//Serial.println(pressure3);
delay(1000);
int long pressure4 = (Pressure.readFloatPressure());
//Serial.print("Pressure4 ");
//Serial.println(pressure4);
delay(1000);
int long pressure5 = (Pressure.readFloatPressure());
//Serial.print("Pressure5 ");
//Serial.println(pressure5);

int long average_speed_standard = ((pressure1+pressure2+pressure3+pressure4+pressure5)/5);
int long cut_off = (average_speed_standard+10000);
//Serial.print(average_speed_standard);

//Serial.print(", cut off: ");
//Serial.println(cut_off);


int long pressure1_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure1 ");
//Serial.println(pressure1_2);
delay(1000);
int long pressure2_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure2 ");
//Serial.println(pressure2_2);
delay(1000);
int long pressure3_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure3 ");
//Serial.println(pressure3_2);
delay(1000);
int long pressure4_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure4 ");
//Serial.println(pressure4_2);
delay(1000);
int long pressure5_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure5 ");
//Serial.println(pressure5_2);

int long average_speed_standard2 = ((pressure1_2+pressure2_2+pressure3_2+pressure4_2+pressure5_2)/5);
int long cut_off2 = (average_speed_standard2-10000);
//Serial.print(average_speed_standard2);
////////////


//main monitoring loop. If the pressure changes then the system just shuts down. If the switch state changes then the manual function starts again and waits for the analyser to be turned on again 

for(;;){

  digitalWrite (lgr_off_led, LOW);
  digitalWrite (lgr_on_led, HIGH);
  
 current_switch_state = digitalRead (lgr_manual_switch);

 
   if (current_switch_state != switch_state) {
    digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);
   switch_state = digitalRead(lgr_manual_switch);
   break;
 }


  int long current_pressure1 = (Pressure.readFloatPressure());
delay(300);
int long current_pressure2 = (Pressure.readFloatPressure());
delay(300);
int long current_pressure3 = (Pressure.readFloatPressure());
delay(300);

int long current_pressure_ave = ((current_pressure1+current_pressure2+current_pressure3)/3);
//Serial.println(current_pressure_ave);
//Serial.println(Pressure.readFloatPressure());
//int tmp_Press = (Pressure.readTempC());
//Serial.print(", ");
//Serial.println(tmp_Press);
//delay(500);


 current_switch_state = digitalRead (lgr_manual_switch);

 
   if (current_switch_state != switch_state) {
    digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);
   switch_state = digitalRead(lgr_manual_switch);
   break;
 }

int long current_pressure1_2 = (Pressure2.readFloatPressure());
delay(300);
int long current_pressure2_2 = (Pressure2.readFloatPressure());
delay(300);
int long current_pressure3_2 = (Pressure2.readFloatPressure());
delay(300);

int long current_pressure_ave2 = ((current_pressure1_2+current_pressure2_2+current_pressure3_2)/3);
//Serial.println(current_pressure_ave2);
//Serial.println(Pressure2.readFloatPressure());



   if (current_pressure_ave >= cut_off || current_pressure_ave2 <= cut_off2){ //less than equal to 35 for outlet, average normal around 38 when plugged in
                  //ave inlet = 20, less than or = 15 is blocked (need to add another port for seconds wind sensor)
  digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);

   digitalWrite(rst, HIGH);
   delay(1000);
   digitalWrite(rst, LOW);

   RTC.read(tm);
   
char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("Airflow ");
    myFile1.print("abnormal, ");
    myFile1.print("manual ");
    myFile1.print("mode ");
    myFile1.print("run ");
    myFile1.println("aborted ");

    // close the file:
    myFile1.close();}
    delay(100);
  
for(;;){}
   }

   else {
  digitalWrite (lgr_off_led, LOW);
  digitalWrite (lgr_on_led, HIGH);
  digitalWrite (lgr_relay, HIGH);

   }
   }

 
}


void auto_mode(){

tmElements_t tm;
RTC.read(tm);
   
char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("auto_mode ");
    myFile1.print("Began ");
    myFile1.print("Waiting ");
    myFile1.print("For ");
    myFile1.print("Authorisation ");
    myFile1.print("to ");
    myFile1.println("Start ");
    // close the file:
    myFile1.close();}
    delay(100);

int GHG_call = digitalRead(lgr_control);
  while(GHG_call == 0)
  {digitalWrite(lgr_off_led, HIGH);
  GHG_call = digitalRead(lgr_control);
  }

   digitalWrite (lgr_relay, HIGH);

int current_switch_state = digitalRead (lgr_manual_switch);
while(current_switch_state == switch_state){
digitalWrite(lgr_on_led, HIGH);
digitalWrite(lgr_off_led, LOW);
delay(250);
digitalWrite(lgr_on_led, LOW);
digitalWrite(lgr_off_led, HIGH);
current_switch_state = digitalRead (lgr_manual_switch);
delay(250);

  }
  

  RTC.read(tm);
   
data[12];
DAY = (tm.Day);
MONTH = (tm.Month);
YEAR = (tm.Year)+1970-2000;

data2[12];
seconds = (tm.Second);
minutes = (tm.Minute);
hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("Authorisation ");
    myFile1.print("recieved ");
    myFile1.print(": ");
    myFile1.print("System ");
    myFile1.print("is ");
    myFile1.println("safe ");
    // close the file:
    myFile1.close();}
    delay(100);
digitalWrite(lgr_on_led, HIGH);
digitalWrite(lgr_off_led, HIGH);

int long pressure1 = (Pressure.readFloatPressure());
//Serial.print("Pressure1 ");
//Serial.println(pressure1);
delay(1000);
int long pressure2 = (Pressure.readFloatPressure());
//Serial.print("Pressure2 ");
//Serial.println(pressure2);
delay(1000);
int long pressure3 = (Pressure.readFloatPressure());
//Serial.print("Pressure3 ");
//Serial.println(pressure3);
delay(1000);
int long pressure4 = (Pressure.readFloatPressure());
//Serial.print("Pressure4 ");
//Serial.println(pressure4);
delay(1000);
int long pressure5 = (Pressure.readFloatPressure());
//Serial.print("Pressure5 ");
//Serial.println(pressure5);

int long average_speed_standard = ((pressure1+pressure2+pressure3+pressure4+pressure5)/5);
int long cut_off = (average_speed_standard+10000);
//Serial.print(average_speed_standard);

//Serial.print(", cut off: ");
//Serial.println(cut_off);


int long pressure1_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure1 ");
//Serial.println(pressure1_2);
delay(1000);
int long pressure2_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure2 ");
//Serial.println(pressure2_2);
delay(1000);
int long pressure3_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure3 ");
//Serial.println(pressure3_2);
delay(1000);
int long pressure4_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure4 ");
//Serial.println(pressure4_2);
delay(1000);
int long pressure5_2 = (Pressure2.readFloatPressure());
//Serial.print("2Pressure5 ");
//Serial.println(pressure5_2);

int long average_speed_standard2 = ((pressure1_2+pressure2_2+pressure3_2+pressure4_2+pressure5_2)/5);
int long cut_off2 = (average_speed_standard2-10000);
//Serial.print(average_speed_standard2);

//Serial.print(", cut off: ");
//Serial.println(cut_off2);

for(;;){

  digitalWrite (lgr_off_led, LOW);
  digitalWrite (lgr_on_led, HIGH);

  
 


  //if (millis() - lastMillis > 200){      // read every 200 ms - printing slows this down further
   /* 
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);



    delay(200);

    TMP_Therm_ADunits2 = analogRead(analogPinForTMP);
    RV_Wind_ADunits2 = analogRead(analogPinForRV);
   RV_Wind_Volts2 = (RV_Wind_ADunits2 *  0.0048828125);
    delay(200);

    TMP_Therm_ADunits3 = analogRead(analogPinForTMP);
    RV_Wind_ADunits3 = analogRead(analogPinForRV);_2
   RV_Wind_Volts3 = (RV_Wind_ADunits3 *  0.0048828125);

    delay(200);

    TMP_Therm_ADunits4 = analogRead(analogPinForTMP);
    RV_Wind_ADunits4 = analogRead(analogPinForRV);
   RV_Wind_Volts4 = (RV_Wind_ADunits4 *  0.0048828125);
    delay(200);


    TMP_Therm_ADunits5 = analogRead(analogPinForTMP);
    RV_Wind_ADunits5 = analogRead(analogPinForRV);
    RV_Wind_Volts5 = (RV_Wind_ADunits5 *  0.0048828125);
    delay(200);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_2 = (0.005 *((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2)) - (16.862 * (float)TMP_Therm_ADunits2) + 9075.4;  

    zeroWind_ADunits_2 = -0.0006*((float)TMP_Therm_ADunits2 * (float)TMP_Therm_ADunits2) + 1.0727 * (float)TMP_Therm_ADunits2 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_2 = (zeroWind_ADunits_2 * 0.0048828125) - zeroWindAdjustment;  



        TempCtimes100_3 = (0.005 *((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3)) - (16.862 * (float)TMP_Therm_ADunits3) + 9075.4;  

    zeroWind_ADunits_3 = -0.0006*((float)TMP_Therm_ADunits3 * (float)TMP_Therm_ADunits3) + 1.0727 * (float)TMP_Therm_ADunits3 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_3 = (zeroWind_ADunits_3 * 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_4 = (0.005 *((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4)) - (16.862 * (float)TMP_Therm_ADunits4) + 9075.4;  

    zeroWind_ADunits_4 = -0.0006*((float)TMP_Therm_ADunits4 * (float)TMP_Therm_ADunits4) + 1.0727 * (float)TMP_Therm_ADunits4 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_4 = (zeroWind_ADunits_4* 0.0048828125) - zeroWindAdjustment;  


        TempCtimes100_5 = (0.005 *((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5)) - (16.862 * (float)TMP_Therm_ADunits5) + 9075.4;  

    zeroWind_ADunits_5 = -0.0006*((float)TMP_Therm_ADunits5 * (float)TMP_Therm_ADunits5) + 1.0727 * (float)TMP_Therm_ADunits5 + 47.172;  //  13.0C  553  482.39

    zeroWind_volts_5 = (zeroWind_ADunits_5 * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);  
    WindSpeed_MPH2 =  pow(((RV_Wind_Volts2 - zeroWind_volts_2) /.2300) , 2.7265);  
     WindSpeed_MPH3 =  pow(((RV_Wind_Volts3 - zeroWind_volts_3) /.2300) , 2.7265);  
      WindSpeed_MPH4 =  pow(((RV_Wind_Volts4 - zeroWind_volts_4) /.2300) , 2.7265);  
       WindSpeed_MPH5 =  pow(((RV_Wind_Volts5 - zeroWind_volts_5) /.2300) , 2.7265);   



float   average_speed_per_second= ((WindSpeed_MPH + WindSpeed_MPH2 + WindSpeed_MPH3 + WindSpeed_MPH4 + WindSpeed_MPH5)/5);
*/
int long current_pressure1 = (Pressure.readFloatPressure());
delay(300);
int long current_pressure2 = (Pressure.readFloatPressure());
delay(300);
int long current_pressure3 = (Pressure.readFloatPressure());
delay(300);

int long current_pressure_ave = ((current_pressure1+current_pressure2+current_pressure3)/3);
//Serial.println(current_pressure_ave);
//Serial.println(Pressure.readFloatPressure());
//int tmp_Press = (Pressure.readTempC());
//Serial.print(", ");
//Serial.println(tmp_Press);
//delay(500);

int long current_pressure1_2 = (Pressure2.readFloatPressure());
delay(300);
int long current_pressure2_2 = (Pressure2.readFloatPressure());
delay(300);
int long current_pressure3_2 = (Pressure2.readFloatPressure());
delay(300);

int long current_pressure_ave2 = ((current_pressure1_2+current_pressure2_2+current_pressure3_2)/3);
//Serial.println(current_pressure_ave2);
//Serial.println(Pressure2.readFloatPressure());



   if (current_pressure_ave >= cut_off || current_pressure_ave2 <= cut_off2){ //less than equal to 35 for outlet, average normal around 38 when plugged in
                  //ave inlet = 20, less than or = 15 is blocked (need to add another port for seconds wind sensor)
  digitalWrite(lgr_relay, LOW);
   digitalWrite(lgr_off_led, HIGH);
   digitalWrite(lgr_on_led, LOW);

   digitalWrite(rst, HIGH);
   delay(1000);
   digitalWrite(rst, LOW);

   RTC.read(tm);
   
char data[12];
int* DAY = (tm.Day);
int* MONTH = (tm.Month);
int* YEAR = (tm.Year)+1970-2000;

char data2[12];
int* seconds = (tm.Second);
int* minutes = (tm.Minute);
int* hours = (tm.Hour);

  sprintf(data, "%u_%u_%u", DAY,MONTH,YEAR);
  sprintf(data2, "%u:%u:%u", hours,minutes,seconds);
  
   myFile1 = SD.open("Status.txt", FILE_WRITE);
   if (myFile1) {
    //Serial.print("Writing to test.txt...");
    myFile1.print(data);
    myFile1.print(", ");
    myFile1.print(data2);
    myFile1.print(", ");
    myFile1.print("Airflow ");
    myFile1.print("abnormal, ");
    myFile1.print("auto ");
    myFile1.print("mode ");
    myFile1.print("run ");
    myFile1.println("aborted ");

    // close the file:
    myFile1.close();}
    delay(100);
  
for(;;){}
   }

   else {
  digitalWrite (lgr_off_led, LOW);
  digitalWrite (lgr_on_led, HIGH);
  digitalWrite (lgr_relay, HIGH);

   }
   }
}
 
