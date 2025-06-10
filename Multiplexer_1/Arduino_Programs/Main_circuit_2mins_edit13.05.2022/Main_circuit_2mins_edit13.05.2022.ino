/*
 * next things to do:
 * Add the pause time for the dessicant / runs based on the iteration 
 * Add the low memory string functions i.e F("hdhdhd")), see how much this lowers it by 
 * Then put them all together, and make comparisons with the pervious sketch to see if it is improved
 * 
 * trying the int8_t in teh array to save memory... will this measn that i in the loops needs to be int8_t as well?...first glance appears not
 * 
 * edit: 18.04.2021. changed the measurement time to 2 minutes in the middle sampling. left the start and end as they are to avoid messing around too much. 
 * this is assuming that the most recent edit of the valve changer is this main circuit valve testing edit sketch...I hope it is the right file...it is the right file 
 * 
 * next changing the 
 */

//int8_tmyArray[10]={1, 12, 6, 21, 20};
//int8_tmyArray1[10]={13, 10, 22, 5, 14};

//const int8_tmyArray[15]={1, 13, 12, 10, 6, 22, 21, 5, 20, 14, 3, 7, 11, 15, 19};
const int8_t myArray[15] PROGMEM ={1, 13, 12, 10, 6, 22, 21, 5, 20, 14, 7, 3, 11, 15, 19};

const int8_t Array2[6] PROGMEM ={2, 3, 6, A1, A0, 7};

int8_t option;
int8_t chamber_number = 0;

  
int8_t run_time = 0;

int8_t last_on = 0;
int8_t last_r = 0;
int8_t last_off =0;


#include <Time.h>
#include <DS1307RTC.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
//#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#define USE_PROGMEM true

AltSoftSerial mySerial; //TX, RX


#include <SPI.h>
#include <SD.h>
//File myFile;
File myFile1;



#define potpin A2  // analog pin used to connect the potentiometer
int8_t val; 
#define buttonPin A3   
int8_t buttonState = 0;         // current state of the button
int8_t lastButtonState = 0;
tmElements_t tm;

//lcd

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); 

//const char *monthName[12] = {
//  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
//  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
//};

//int8_t sd_card_reader = 10;



//HC-12 communication

//SoftwareSerial mySerial(8, 9); //TX, RX



//int8_t setPin = 7; old hc-12 set pin
boolean onOff = 0;

//boolean led_bool = false;

int8_t required_state = (""); 

int8_t input = 0; 




//shift registers

//intro code from sequence

#define SER_Pin 1   //pin 14 on the 75HC595
#define RCLK_Pin 5  //pin 12 on the 75HC595
#define SRCLK_Pin 4 //pin 11 on the 75HC595


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
//declare variables for the shift registers 
/*#define relay_2 3  // 
#define relay_1 7  // 
#define relay_3 11  // 
#define relay_4 15  // 
#define relay_5 19  // */
#define relay_6 23  //  












/*
#define L33 22  //  valve 3 off
#define L4 21  //   valve 4 on 
#define L5 20  //   valve 5 on 
#define L1 1  //  valve 1 on  
*/
#define L77 2  //    valve 6 (descant) off
/*#define L55 14  //  valve 5 off

#define L11 10  // valve 1 off
#define L2 12  //  valve 2 on   
#define L22 10  //  vave 2 off 
#define L3 6  //  valve 3 on 
#define L44 5  //  valve 4 off*/
#define L7 4  // valve 6 (desiccant) on 
#define HC_12 8  // valve 6 (desiccant) on 
#define mode_signal 16  // valve 6 (desiccant) on
#define lgr_control 0  


void setup() {



buttonState = digitalRead(buttonPin);
lastButtonState = buttonState;



pinMode(2, INPUT);
//digitalWrite(testpin1, HIGH);

pinMode(3, INPUT);
//digitalWrite(testpin2, HIGH);

pinMode(6, INPUT);
//digitalWrite(testpin3, HIGH);

pinMode(A1, INPUT);
//digitalWrite(testpin4, HIGH);

pinMode(A0, INPUT);
//digitalWrite(testpin5, HIGH);

pinMode(7, INPUT);
//digitalWrite(testpin6, HIGH);




pinMode(buttonPin, INPUT);

 lcd.begin (16,2);
 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("VALVE CHANGER 1.0"));
 //printProgStr (1);
 //printProgStr ((const char *) &descriptions [2]);
 
 //lcd.println();
  delay(2000);
 pinMode(buttonPin, INPUT);
 
 delay(2000);


  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);


  //reset all register pins
  clearRegisters();
  writeRegisters();

//set modes to high to keep control circuit inactive

     setRegisterPin(mode_signal, HIGH);
     setRegisterPin(lgr_control, HIGH);
     writeRegisters();

//HC-12 startup check 

//pinMode(setPin, OUTPUT);
 delay(500);
 mySerial.begin(9600);
 delay(500);

 
// digitalWrite(setPin, LOW);           // Set HC-12 into AT Command mode
setRegisterPin(HC_12, LOW);
                writeRegisters();
 delay(100);                          // Wait for the HC-12 to enter AT Command mode
 mySerial.print("AT");               // Send AT Command to HC-12 //making this an F() actually adds bytes!
 delay(100);
 //digitalWrite(setPin, HIGH);
 setRegisterPin(HC_12, LOW);
                writeRegisters();
 delay(200);
 
  if (mySerial.available()) {
 delay(100);


 
 String input1 = mySerial.readString();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(input1);
//lcd.print("              ");
 //printProgStr (1);
 delay(2000);

//WriteLcd(input1);
 
 }

 

else {

 
lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
//printProgStr (0);
 lcd.print(F("HC-12 not okay"));
 delay(2000);
 ///WriteLcd("HC-12 not okay");

//WriteLcd("HC-12 not okay");
 
 }


//CHecking if rtc is okay

 tmElements_t tm;
     
     if (RTC.read(tm)) {  //if okay flash green led once for 1 second             
                lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("RTC OK"));
 //lcd.print("              ");
 delay(2000);
                }

      else if (RTC.chipPresent()) { //if the chip needs resetting flash the red led 

        
                //delay(1000);
                for(;;){
                 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("RTC rst"));
// lcd.print("              ");
 }
                }
      

      else {   //if the chip not detected at all then keep the LED on 
               for(;;){

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("NO RTC"));
 //lcd.print("              ");

               }
      }


//sd card check

     pinMode(10, OUTPUT); //ss/ cs
       delay(500);  
       while (!SD.begin(4)) { //whilst the sd is not okay, the green and red led are turned on alternately
               
                 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("SD Error"));
//printProgStr (2);
 //lcd.print("              ");
 }

                lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("SD OK"));
 //printProgStr (3);
 //lcd.print("              ");
 delay(2000);
sd_write(111);

//sd system restarted write  
}

void loop() {

//int start_time = millis ();   



//while (millis () - start_time <= 180000){    //automatic timeout to auto mode


//val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)

option = map(analogRead(potpin), 0, 1020, 1, 2);  

lcd.home ();   
lcd.setCursor ( 0, 0 );
lcd.print(F("Mode Select"));
//lcd.print(0xDF);
lcd.print(":");

if (option==1) {
  lcd.setCursor ( 0, 1 );
  lcd.print(F("Automatic  "));
//lcd.println();
  }
 
else if(option ==2){
lcd.setCursor ( 0, 1 );  
lcd.print(F("Manual     "));
//lcd.println();
}

/*else if (option==3) {
lcd.setCursor ( 0, 1 );  
lcd.print("Startup    ");
//lcd.println();
}*/

buttonState = digitalRead(buttonPin);

if (buttonState != lastButtonState) {

  if (option==2) {
    lcd.clear();
    lcd.setCursor (0, 1);
    lcd.print(F("Manual Selected"));
    delay (2000); 
    //lcd.clear();
     setRegisterPin(mode_signal, HIGH);
     setRegisterPin(lgr_control, LOW);
                writeRegisters();
    manual_setting ();
  }

  if (option==1) {
    lcd.clear();
    lcd.setCursor (0, 1);
    lcd.print(F("Auto 1 Selected"));
     setRegisterPin(mode_signal, LOW);
     setRegisterPin(lgr_control, HIGH);
                writeRegisters();
    all_together_2();
    }

  /* if (option==3) {
    //lcd.clear();
    lcd.setCursor (0, 1);
    lcd.print("Trial Selected ");}   */


  
  delay (2000); 
  lcd.clear();
  }
   lastButtonState = buttonState;
   delay(40);
   //lcd.clear();
   
  
  /*
  lcd.clear();
    lcd.setCursor (0, 1);
    lcd.print("Time-Out Auto 1");
  Auto_1 ();*/
 


  
  // put your main code here, to run repeatedly:
//control_sequence();
//void all_together_2() __attribute__((optimize("-O0")));
//all_together_2();
//all_together_2();
//all_together();

//manual_setting ();
//manual_setting1 ();
}


void sd_write(int8_t x){

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
    myFile1.print(F("Status: "));
    myFile1.println(x);

    // close the file:
    myFile1.close();}
    delay(100);



}

/*
void printProgStr (const char * str)
{
  char c;
  if (!str) 
    return;
     
  while ((c = pgm_read_byte(str++)))
 

    lcd.print(c);
   
   
    
} // end of printProgStr

*/


void radio_loop () {

  
/*
//18318 bytes (56% prog) 1477 bytes (72%) dynamic memory 
//tested and it works
for (int8_t y = 1;  y <=  5; y++ ){
  
String stringOne = "AT+C00";
stringOne += y;


                lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(stringOne);
 //lcd.print("              ");
 delay(2000);

setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 mySerial.print(stringOne);               // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(1000);


 for(int8_t i=1; i<=5; i++){ 
    mySerial.println(2222);//chamber open or LED off code
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("code "));
lcd.print(y);
 lcd.print(F(" sent"));
    delay(100);}//delay little for better serial communication}
*/

/*
//this method uses 18408 bytes (57%) and 1471 bytes (71%) dynamic mem 
//doesnt work other than channel 1
for (int8_t y = 1;  y <=  5; y++ ){
                 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(y);
 lcd.print("              ");
 delay(2000);

setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 printProgStr ((const char *) &descriptions [y]);             // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(1000);

for(int8_t i=1; i<=5; i++){ 
     mySerial.println(2222);//chamber open or LED off code
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print("code ");
lcd.print(y);
 lcd.print(" sent");
    delay(100);}//delay little for better serial communication}
 
*/
 
//}
}



void control_sequence(){

//for(int8_t i=1; i<=5; i++){ 

//integer method



for(int8_t i=0; i<=4; i++){ 

  
  
  

/*
 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(myArray[i+i]);
 //lcd.print("              ");
 delay(1000);

  lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(myArray[i+(i+1)]);
 //lcd.print("              ");
 delay(1000);
*/



  /*
 setRegisterPin((myArray[i+i]), HIGH);
 //setRegisterPin((myArray[i+(i+1)]), LOW);

 setRegisterPin((myArray[i+10]), HIGH);
// setRegisterPin(L7, LOW);
 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000);

 setRegisterPin((myArray[i+i]), LOW);
 setRegisterPin((myArray[i+(i+1)]), HIGH);
 setRegisterPin((myArray[i+10]), HIGH);
// setRegisterPin(L7, LOW);
// setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

// setRegisterPin((myArray[i+i]), LOW);
 setRegisterPin((myArray[i+(i+1)]), LOW);
 setRegisterPin(L7, HIGH);
// setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

 //setRegisterPin((myArray[i+i]), LOW);
 //setRegisterPin((myArray[i+(i+1)]), LOW);
 setRegisterPin(L7, LOW);
 setRegisterPin(L77, HIGH);
                writeRegisters();


delay(1000);

*/

///////
/*
//pgm_read_byte_near(x_move + k);
 setRegisterPin(pgm_read_byte_near(myArray+ (i+i)), HIGH);
 //setRegisterPin((myArray[i+(i+1)]), LOW);

 setRegisterPin(pgm_read_byte_near(myArray+ (i+10)), HIGH);
// setRegisterPin(L7, LOW);
 setRegisterPin(L77, LOW);
                writeRegisters();*/

 setRegisterPin((myArray[i+i]), HIGH);
 //setRegisterPin((myArray[i+(i+1)]), LOW);

 setRegisterPin((myArray[i+10]), HIGH);
// setRegisterPin(L7, LOW);
 setRegisterPin(L77, LOW);
                writeRegisters();



}}






void timeWait (){
  //beginnign of 5 x measurements = 120 seconds 
  //measurement time = 60 seconds 
  //10 seconds dry for start of each measurement after chamber confirmed message recieved
  
  //20 seconds for chamber 1 marker end 
  //40 seconds chamebr 2 marker end 
  //60 secodns chamber 3 marker end 
  //80 seconds chamber 4 marker end 
  //100 seconds chamber 5 marker end 

  delay(100);
  
  delay(10);
  
  

}

//setRegisterPin(pgm_read_byte_near(myArray+ (i+i)), HIGH);

void all_together_2 () {
  lcd.clear ();
buttonState = digitalRead(buttonPin);
   delay(40);
   lastButtonState = buttonState;
   delay(40); 

for(;;){ 
   
//val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)

option = map(analogRead(potpin), 0, 1020, 1, 5); 

//lcd.clear ();
lcd.home ();   
lcd.setCursor ( 0, 0 );
lcd.print(F("Chamber Number: "));
//lcd.print(0xDF);
//lcd.print(":");
 

lcd.setCursor ( 0, 1 );  
lcd.print(option);
//lcd.print("               ");
//lcd.println();



buttonState = digitalRead(buttonPin);

if (buttonState != lastButtonState) {

  chamber_number = option;
  break;
}

   lastButtonState = buttonState;
   delay(40);}

for(;;){  
  buttonState = digitalRead(buttonPin);
   delay(40);
   lastButtonState = buttonState;
   delay(40);
//val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)

option = map(analogRead(potpin), 0, 1020, 1, 60);  

//lcd.clear ();
lcd.home ();   
lcd.setCursor ( 0, 0 );
lcd.print(F("Number of runs: "));
//lcd.print(0xDF);
//lcd.print(":");
 

lcd.setCursor ( 0, 1 );  
lcd.print(option);
lcd.print(" ");
//lcd.println();



buttonState = digitalRead(buttonPin);

if (buttonState != lastButtonState) {
  
 run_time = option;
 lcd.clear (); 
 lcd.home ();   
lcd.setCursor ( 0, 0 );
lcd.print(F("Auto Beginning"));
//lcd.print(0xDF);
//lcd.print(":");
  break;
}

   lastButtonState = buttonState;
   delay(40);
   }

  
//  int8_t chamber_number = 2;
 // int8_t run_time = 3;
for (int8_t i=0; i<4; i++){
 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), LOW);
 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), HIGH);
  writeRegisters();
}

setRegisterPin(L7, LOW);
setRegisterPin(L77, HIGH);

//valve 5 open incase no valves open
setRegisterPin(pgm_read_byte_near(myArray+8), HIGH);
setRegisterPin(pgm_read_byte_near(myArray+9), LOW);
setRegisterPin(pgm_read_byte_near(myArray+14), HIGH);
 setRegisterPin(lgr_control, HIGH);
                writeRegisters();


last_on = pgm_read_byte_near(myArray+8);
last_r = pgm_read_byte_near(myArray+14);//was 15
last_off = pgm_read_byte_near(myArray+9);



delay(10000);

for(;;){

for (int long i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print(" Start");
 delay(100);

/////////////////////   
//set HC-12 channel
  String stringOne = "AT+C00";
stringOne += (i+1);



setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 mySerial.print(stringOne);               // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(100);
 
//channel successfully closed
////////////////////////////////
//adding a timing loop to add this instead of an open for loop. If the time gets over the timeout then skip the measurements after commuication. 
//close chamber lid
//for(;;){ //the former for loop 

 unsigned long start_time = millis();
 unsigned long end_time = (start_time + 180000);
 unsigned long current_time = millis();

  
while (current_time <=end_time) { //time condition loop rather than infinate loop as before

required_state = 5678; //close chamber or light on



if(mySerial.available() > 1){


//read buffer

    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    

   // lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(input);
 lcd.print("            ");
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
 

    delay(100);

    
// writing chamber or light status with red yellow and green LEDs

    if (input == 4321){
 // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, LOW);
  //digitalWrite(led3, HIGH);
    }
    else if (input == 5678 ){  //digitalWrite(ledPin, HIGH);//turn LED on
 // digitalWrite(led2, LOW);
  //digitalWrite(led3, LOW);
  }

  else { // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);
  }


//choosing chamber command code

if (input==required_state) {chamber1(); 
break;}
  
else if (input!= required_state){chamber();}


    input = ("");
    delay(100);
    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);
    }

 else {//digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, HIGH);
 // digitalWrite(led3, LOW);
 // lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("buffer 1 empty  "));
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
 delay(100);
 chamber();}



//delay(1500);
delay(500);
//lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("waiting         "));
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
delay(500);

current_time = millis(); //set the time for the while loop conditions 
}

if (current_time >= end_time) { //if the system did not connect, and therefore completed the while loop. write the chamber number with an added 100, to mark that it was skipped in the sd card
  sd_write(i+100);}

else {                      //otherwise run the previus code, measuring the chamber

//lid closed

////////////////////

//mark measurement beginning with dessicant 


setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

//valve 5 close 
setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

if(i==0){delay(120000);}
else{delay(10000);}


 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH);
 //setRegisterPin((myArray[i+(i+1)]), LOW);

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH);
//next valve open 
 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay (120000); //chamber closed measurement time = was 30 seconds (30000), after testing changed this to 2 mins (120000)


//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), LOW); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), LOW); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 


 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
 
last_on = pgm_read_byte_near(myArray+(i+i));
last_r = pgm_read_byte_near(myArray+(i+10));
last_off = pgm_read_byte_near(myArray+(i+(i+1)));

sd_write(i);
delay(5000);
//chamber close cycle finished
}
}

////end of start
/////////////////////////////////////////

////////////mid sampling////////////
delay(5000);
for (int x=0; x<run_time; x++){ 
  sd_write(x+30);
for (int long i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 
lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print("       ");
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Middle "));
  lcd.print(x);
  lcd.print(F(" of "));
  lcd.print(run_time);
delay(500);



//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

delay (10000); //dessicant marking start 

 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay(120000); //measurement time changed to 2 minutes 

setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), LOW); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), LOW); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 



last_on = pgm_read_byte_near(myArray+(i+i));
last_r = pgm_read_byte_near(myArray+(i+10));
last_off = pgm_read_byte_near(myArray+(i+(i+1)));

sd_write(i+10);

}
}

 setRegisterPin((last_on), HIGH); //valve on light

 setRegisterPin((last_r), HIGH); //valve itself 

 setRegisterPin((last_off), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay(5000);
////end of mid sampling 
//////////////////////

//Chamber lid up sequence

for (int long i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print(" End  ");
 delay(100);

/////////////////////   
//set HC-12 channel
  String stringOne = "AT+C00";
stringOne += (i+1);



setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 mySerial.print(stringOne);               // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(100);
 
//channel successfully closed
////////////////////////////////

//end measurements


setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

//valve 5 close 
setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

delay(10000);


 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
  

delay(120000); //measurement time - was 30 seconds, changed to 2 mins after testing


//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), LOW); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), LOW); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 


 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
                
//measurements taken 
/////////////////////////////////////

//close signal to be sent


//adding a timing loop to add this instead of an open for loop. If the time gets over the timeout then skip the measurements after commuication. 
//close chamber lid
//for(;;){ //the former for loop 

 unsigned long start_time = millis();
 unsigned long end_time = (start_time + 180000);
 unsigned long current_time = millis();

  
while (current_time < end_time) { //time condition loop rather than infinate loop as before


required_state = 4321; //close chamber or light on



if(mySerial.available() > 1){


//read buffer

    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    


    
// writing chamber or light status with red yellow and green LEDs

    if (input == 4321){
 // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, LOW);
 // digitalWrite(led3, HIGH);
    }
    else if (input == 5678 ){  //digitalWrite(ledPin, HIGH);//turn LED on
  //digitalWrite(led2, LOW);
  //digitalWrite(led3, LOW);
  }

  else { // digitalWrite(ledPin, LOW);//turn LED on
  //digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);
  }


//choosing chamber command code

if (input==required_state) {chamber1();
 
break;}
  
else if (input!= required_state){chamber3();}


    input = ("");
    delay(100);
    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);
    }

 else {//digitalWrite(ledPin, LOW);//turn LED on
  //digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);

// lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("buffer 2 empty  "));
 delay(100);
  chamber3();}


delay(500);

//lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("waiting 2       "));
//delay(500);}

current_time = millis(); //set the time for the while loop conditions 
}

if (current_time >= end_time) { //if the system did not connect, and therefore completed the while loop. write the chamber number with an added 50, to mark that it was skipped in the sd card
  sd_write(i+50);}

else {                      //otherwise run the previus code, measuring the chamber



sd_write(i+20);

}

last_on = pgm_read_byte_near(myArray+(i+i));
last_r = pgm_read_byte_near(myArray+(i+10));
last_off = pgm_read_byte_near(myArray+(i+(i+1)));
}
delay(180000); //delay for ventilation when open 

}
}







void all_together () {

  int8_t chamber_number = 2;
  int8_t run_time = 3;
for (int8_t i=0; i<4; i++){
 setRegisterPin((myArray[i+i]), LOW);
 setRegisterPin((myArray[i+(i+1)]), HIGH);
  writeRegisters();
}

setRegisterPin(L7, LOW);
setRegisterPin(L77, HIGH);

//valve 5 open incase no valves open
setRegisterPin((myArray[8]), HIGH);
setRegisterPin((myArray[9]), LOW);
setRegisterPin((myArray[14]), HIGH);
                writeRegisters();

last_on = (myArray[8]);
last_r = (myArray[15]);
last_off = (myArray[9]);



delay(10000);

for(;;){

for (int8_t i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print(" Start");
 delay(100);

/////////////////////   
//set HC-12 channel
  String stringOne = "AT+C00";
stringOne += (i+1);



setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 mySerial.print(stringOne);               // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(100);
 
//channel successfully closed
////////////////////////////////

//close chamber lid
for(;;){


required_state = 5678; //close chamber or light on



if(mySerial.available() > 1){


//read buffer

    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    

   // lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(input);
 lcd.print("            ");
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
 

    delay(100);

    
// writing chamber or light status with red yellow and green LEDs

    if (input == 4321){
 // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, LOW);
  //digitalWrite(led3, HIGH);
    }
    else if (input == 5678 ){  //digitalWrite(ledPin, HIGH);//turn LED on
 // digitalWrite(led2, LOW);
  //digitalWrite(led3, LOW);
  }

  else { // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);
  }


//choosing chamber command code

if (input==required_state) {chamber1(); 
break;}
  
else if (input!= required_state){chamber();}


    input = ("");
    delay(100);
    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);
    }

 else {//digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, HIGH);
 // digitalWrite(led3, LOW);
 // lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("buffer 1 empty  "));
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
 delay(100);
 chamber();}



//delay(1500);
delay(500);
//lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("waiting         "));
 // lcd.setCursor ( 0, 1 );
 //lcd.print(F("Chamber "));
delay(500);
}

//lid closed

////////////////////

//mark measurement beginning with dessicant 


setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

//valve 5 close 
setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

if(i==0){delay(120000);}
else{delay(10000);}


 setRegisterPin((myArray[i+i]), HIGH);
 //setRegisterPin((myArray[i+(i+1)]), LOW);

 setRegisterPin((myArray[i+10]), HIGH);
//next valve open 
 setRegisterPin((myArray[i+i]), HIGH); //valve on light

 setRegisterPin((myArray[i+10]), HIGH); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay (30000); //chamber closed measurement time


//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin((myArray[i+i]), LOW); //valve on light

 setRegisterPin((myArray[i+10]), LOW); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 


 setRegisterPin((myArray[i+i]), HIGH); //valve on light

 setRegisterPin((myArray[i+10]), HIGH); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
 
last_on = (myArray[i+i]);
last_r = (myArray[i+10]);
last_off = (myArray[i+(i+1)]);

sd_write(i);
delay(5000);
//chamber close cycle finished
}

////end of start
/////////////////////////////////////////

////////////mid sampling////////////
delay(5000);
for (int8_t i=0; i<run_time; i++){ 
  sd_write(i+30);
for (int8_t i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 
lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print("       ");
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Mid-Sampling "));

delay(500);



//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

delay (10000); //dessicant marking start 

 setRegisterPin((myArray[i+i]), HIGH); //valve on light

 setRegisterPin((myArray[i+10]), HIGH); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay(30000); //measurement time

setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin((myArray[i+i]), LOW); //valve on light

 setRegisterPin((myArray[i+10]), LOW); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 



last_on = (myArray[i+i]);
last_r = (myArray[i+10]);
last_off = (myArray[i+(i+1)]);

sd_write(i+10);

}
}

 setRegisterPin((last_on), HIGH); //valve on light

 setRegisterPin((last_r), HIGH); //valve itself 

 setRegisterPin((last_off), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();

delay(5000);
////end of mid sampling 
//////////////////////

//Chamber lid up sequence

for (int8_t i=0; i<chamber_number; i++){    //loop for the 5 chambers closing 

 lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 1 );
 lcd.print(F("Chamber "));
 lcd.print(i+1);
 lcd.print(" End  ");
 delay(100);

/////////////////////   
//set HC-12 channel
  String stringOne = "AT+C00";
stringOne += (i+1);



setRegisterPin(HC_12, LOW);
                writeRegisters();
           // Set HC-12 into AT Command mode
 delay(100);

 mySerial.print(stringOne);               // using this channel for chamber 1
 delay(100);
 
 setRegisterPin(HC_12, HIGH);
                writeRegisters();
 delay(100);
 
//channel successfully closed
////////////////////////////////

//end measurements


setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();


delay(1000);

//valve 5 close 
setRegisterPin(last_on, LOW);

setRegisterPin(last_r, LOW);

setRegisterPin(last_off, HIGH);
                writeRegisters();

delay(10000);


 setRegisterPin((myArray[i+i]), HIGH); //valve on light

 setRegisterPin((myArray[i+10]), HIGH); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
  

delay(30000); //measurement time


//valve close and dessicant open again
setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();

delay(1000); //overlap valve time

 setRegisterPin((myArray[i+i]), LOW); //valve on light

 setRegisterPin((myArray[i+10]), LOW); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), HIGH); //valve off light
                writeRegisters();

if(i==0){delay(20000);}//dessicant measurement time
else{delay(i*20000);} 


 setRegisterPin((myArray[i+i]), HIGH); //valve on light

 setRegisterPin((myArray[i+10]), HIGH); //valve itself 

 setRegisterPin((myArray[i+(i+1)]), LOW); //valve off light
                writeRegisters();

delay(1000); //overlap valve time

setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

 setRegisterPin(L77, HIGH);
                writeRegisters();
                
//measurements taken 
/////////////////////////////////////

//close signal to be sent


for(;;){


required_state = 4321; //close chamber or light on



if(mySerial.available() > 1){


//read buffer

    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    


    
// writing chamber or light status with red yellow and green LEDs

    if (input == 4321){
 // digitalWrite(ledPin, LOW);//turn LED on
 // digitalWrite(led2, LOW);
 // digitalWrite(led3, HIGH);
    }
    else if (input == 5678 ){  //digitalWrite(ledPin, HIGH);//turn LED on
  //digitalWrite(led2, LOW);
  //digitalWrite(led3, LOW);
  }

  else { // digitalWrite(ledPin, LOW);//turn LED on
  //digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);
  }


//choosing chamber command code

if (input==required_state) {chamber1();
 
break;}
  
else if (input!= required_state){chamber3();}


    input = ("");
    delay(100);
    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);
    }

 else {//digitalWrite(ledPin, LOW);//turn LED on
  //digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);

// lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("buffer 2 empty  "));
 delay(100);
  chamber3();}


delay(500);

//lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("waiting 2       "));
//delay(500);}

}

last_on = (myArray[i+i]);
last_r = (myArray[i+10]);
last_off = (myArray[i+(i+1)]);

sd_write(i+20);

}
}
}












 void chamber (){
  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(2321);//chamber close or LED on code
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);

// lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("code 1 sent     "));
    delay(100);}//delay little for better serial communication}
/*delay(2000);
      for(int8_t i=1; i<=1; i++){ 
    mySerial.println(1234);//end transmission code
    onOff = 1;//set boolean to 1
    delay(100);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}*/
 }




void chamber1 (){
required_state = 7689;
delay(300);
  for(;;){

 //lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("not wanted loop"));
 delay(100);
  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(5412);//end transmission code only 
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}

    input = ("");
    delay(100);
    mySerial.flush();//clear the serial buffer for unwanted inputs     
    delay(100);
    mySerial.flush();
    delay(100);


    delay(1000);

 //radio acknowledge transmission end

if(mySerial.available() > 1){


//read buffer

    input = mySerial.parseInt();//read serial input and convert to integer (-32,768 to 32,767)
    delay(100);    

 //lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(input);
 lcd.print("            ");
    delay(100);}

    


  else {//digitalWrite(ledPin, LOW);//turn LED on
  //digitalWrite(led2, HIGH);
  //digitalWrite(led3, LOW);
  }
  

  if (input==required_state) {break;}

  else if (input!=required_state){
    
    }

 }
 }



 void chamber3(){
  for(int8_t i=1; i<=1; i++){ 
    mySerial.println(4536);//chamber open or LED off code
    onOff = 1;//set boolean to 1
    delay(200);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);

 //lcd.clear();
 lcd.home ();   
 lcd.setCursor ( 0, 0 );
 lcd.print(F("code 2 sent     "));
    delay(100);}//delay little for better serial communication}
/*delay(2000);
      for(int8_t i=1; i<=1; i++){ 
    mySerial.println(1234);//end transmission code
    onOff = 1;//set boolean to 1
    delay(100);
    mySerial.println("");
    onOff = 1;//set boolean to 1
    delay(100);}//delay little for better serial communication}*/
 }





 void manual_setting (){

    // sd_write(2);

for (;;){



for(int8_t i=0; i<=5; i++){

int8_t x = (digitalRead(pgm_read_byte_near(Array2+(i))));
  //delay(500);
if(i==5) {


  
  if (x == 1) {
 setRegisterPin(L7, HIGH);

setRegisterPin(relay_6, HIGH);

 setRegisterPin(L77, LOW);
                writeRegisters();
}

else {
setRegisterPin(L7, LOW);

setRegisterPin(relay_6, LOW);

setRegisterPin(L77, HIGH);
                writeRegisters();
}
}
                
else{   
if (x == 1) {
 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), HIGH); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), HIGH); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), LOW); //valve off light
                writeRegisters();
}

else {
 setRegisterPin(pgm_read_byte_near(myArray+(i+i)), LOW); //valve on light

 setRegisterPin(pgm_read_byte_near(myArray+(i+10)), LOW); //valve itself 

 setRegisterPin(pgm_read_byte_near(myArray+(i+(i+1))), HIGH); //valve off light
                writeRegisters();
}}
}}}

/*
if( one == 1){
  
 setRegisterPin(led1, HIGH);

 setRegisterPin(relay_1, HIGH);

 setRegisterPin(led2, LOW);
                writeRegisters();
                
}

if( one == 0){

 setRegisterPin(led2, HIGH);

 setRegisterPin(relay_1, LOW);

 setRegisterPin(led1, LOW);
                writeRegisters();
}

if( two == 1){

 setRegisterPin(led3, HIGH);

 setRegisterPin(relay_2, HIGH);

 setRegisterPin(led4, LOW);
                writeRegisters();
}

if( two == 0){

 setRegisterPin(led4, HIGH);

 setRegisterPin(relay_2, LOW);

 setRegisterPin(led3, LOW);
                writeRegisters();
}

if( three == 1){

 setRegisterPin(led5, HIGH);

 setRegisterPin(relay_3, HIGH);

 setRegisterPin(led6, LOW);
                writeRegisters();
}

if( three == 0){

 setRegisterPin(led6, HIGH);

 setRegisterPin(relay_3, LOW);

 setRegisterPin(led5, LOW);
                writeRegisters();
}

if( four == 1){

 setRegisterPin(led7, HIGH);

 setRegisterPin(relay_4, HIGH);

 setRegisterPin(led8, LOW);
                writeRegisters();
}

if( four == 0){

 setRegisterPin(led8, HIGH);

 setRegisterPin(relay_4, LOW);

 setRegisterPin(led7, LOW);
                writeRegisters();
}

if( five == 1){
 setRegisterPin(led9, HIGH);

 setRegisterPin(relay_5, HIGH);

 setRegisterPin(led10, LOW);
                writeRegisters();
}

if( five == 0){

 setRegisterPin(led10, HIGH);

 setRegisterPin(relay_5, LOW);

 setRegisterPin(led9, LOW);
                writeRegisters();
}

if( six == 1){
 setRegisterPin(led11, HIGH);

 setRegisterPin(relay_6, HIGH);

 setRegisterPin(led12, LOW);
                writeRegisters();
  
}

if( six == 0){

 setRegisterPin(led12, HIGH);

 setRegisterPin(relay_6, LOW);

 setRegisterPin(led11, LOW);
                writeRegisters();
}




  

}}*/
