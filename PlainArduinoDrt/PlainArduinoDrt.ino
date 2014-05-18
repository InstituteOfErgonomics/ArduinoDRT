//------------------------------------------------------
//Revision History 'Arduino-DRT-Plain'
//------------------------------------------------------
//Version  Date		Author		  Mod
//1        Mar, 2014	Michael Krause	  initial
//1.1      Mai, 2014    Michael Krause    gRootNumberOfFiles++ bug
//
//------------------------------------------------------
/*
  GPL due to the use of SD libs.

  Copyright (C) 2014  Michael Krause (krause@tum.de), Institute of Ergonomics, Technische Universität München
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


 
#include <SD.h>
#include <EEPROM.h>
#include <SPI.h>

//---------------------------------------------------------------------------
//CONST
//ethernetshield uses 10,11,12,13
const int SD_CARD_CS_PIN = 10;//chip select for SD-card (if ethernet shield is used, sd chip select = 4)

const int REACT_BUTTON_PIN = 2; //reaction button (ext-IRQ 0)
const int START_STOP_BUTTON_PIN = 3;//to start/stop experiment
const int EXP_RUNNING_LED_PIN = 5; // show if experiment is running
const int BUTTON_CLOSED_LED_PIN = 6;// show if button closed/open
const int SD_CARD_LED_PIN_L = 7;//duoColorLED for SD-card status low pin
const int SD_CARD_LED_PIN_H = 8;//duoColorLED for SD-card status high pin
const int STIMULUS_LED_PIN = 9; // stimulus led / tactile tactor

const int DUO_COLOR_LED_OFF = 0; 
const int DUO_COLOR_LED_GREEN = 1; 
const int DUO_COLOR_LED_RED = 2; 

const String HEADER = "count;stimulusT;onsetDelay;soa;soaNext;rt;result;marker;edges;edgesDebounced;hold;buttonDownCount;pwm;";
const String VERSION = "V1.1-plain";//plain, without Ethernet. version number is logged to result header

const unsigned long CHEAT = 100000;//lower 100000 micro seconds = cheat
const unsigned long MISS = 2500000;//greater 2500000 micro seconds = miss

const unsigned long STIM_MIN = 3000000;//next stimulus min after x micro seconds
const unsigned long STIM_MAX = 5000000;//next stimulus max after x micro seconds

const byte MARKER_DEFAULT = '-';

//global VARs
volatile byte gExpRunningF = false; //flag, set by startExperiment, cleared by stopExperiment
volatile byte gUnhandeledPushEventF = false; //flag set by ISR, cleared by program loop
volatile byte gUnhandeledReleaseEventF = false; //flag set by ISR, cleared by program loop
volatile unsigned long gButtonDownT;//start of button down, in uptime micro sec
volatile unsigned long gHoldStartT;//start of button hold, in uptime micro sec
volatile unsigned long gHoldStopT; //stop of button hold, in uptime micro sec
volatile byte gResponseWindowF= false;//true during 'responseWindow'; from 0 micro sec to MISS micrro sec after stimulus 
unsigned long gExpStartT;//start of experiment in uptime, micro sec
byte gCalculateHoldF=false;
byte gSdCardAvailableF=false;//is sd card available
unsigned int gCurFileNumber;//current file-number for saving
int gRootNumberOfFiles;//how many files in root; important we assure that it is <500
byte gMarker = MARKER_DEFAULT;//if we get a marker/trigger we set gMarker to this trigger number
byte gReadablePacketSendF=false;//true: sendPacket will transmit readable format. false: binary format
byte gStimulsOnF = false;//is stimuls on
byte gStimulusStrength=255;//PWM stimulus strength
const byte STIMULUS_PWM_EEPROM = 2;//location in EEPROM to save pwm stimulus signal strength
//----------
#pragma pack(1)
typedef struct myDrtPacket{
  unsigned int count; //stimuli/packet count
  unsigned long stimulusT; //stimulus showen microsec (us) after start of experiment
  unsigned long onsetDelay;//error in us of onset compared to schedule, always positive
  unsigned long soa;//random stimulus onset asynchrony in us; current
  unsigned long soaNext;//random stimulus onset asynchrony in us between current and next
  unsigned long rt;// reaction time in us; in case of miss, rt is set to 0
  byte result; //'H' hit, 'M' miss or 'C' cheat. status message 'R' ready to start, '$' experiment started, '#' experimentz stopped, 'N' no sd card, 'E' error while logging
  byte marker; //received marker '0' to '9'
  unsigned int edges; //edge count to detect hardware malfunctions;
  unsigned int edgesDebounced; //edge count to detect hardware malfunctions; edges after debounce
  unsigned long hold;// button hold time at previous stimuli
  unsigned int buttonDownCount; //how often button is pessed down during one experiment  
  unsigned int fileNumber; //to which file we are currently logging
  byte stimulusStrength;//pwm signal strength of stimulus
} sDrtPacket, *pDrtPacket;
#pragma pack()

sDrtPacket gPacket;
pDrtPacket gpPacket = &gPacket;//pointer to gPacket

//---------------------------------------------------------------------------
//helper
void duoLed(int state){

     switch( state )
     {
	case DUO_COLOR_LED_OFF :
           digitalWrite(SD_CARD_LED_PIN_H, LOW);
           digitalWrite(SD_CARD_LED_PIN_L, LOW);
          break;

        case DUO_COLOR_LED_GREEN :
           digitalWrite(SD_CARD_LED_PIN_H, HIGH);
           digitalWrite(SD_CARD_LED_PIN_L, LOW);
          break;

        case DUO_COLOR_LED_RED :
           digitalWrite(SD_CARD_LED_PIN_H, LOW);
           digitalWrite(SD_CARD_LED_PIN_L, HIGH);
          break;


	default  : //default is off
           digitalWrite(SD_CARD_LED_PIN_H, LOW);
           digitalWrite(SD_CARD_LED_PIN_L, LOW);
     }  
  
}
//---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);  
  pinMode(REACT_BUTTON_PIN, INPUT); 
  pinMode(START_STOP_BUTTON_PIN, INPUT); 
  pinMode(STIMULUS_LED_PIN, OUTPUT); 
  pinMode(EXP_RUNNING_LED_PIN, OUTPUT);
  pinMode(BUTTON_CLOSED_LED_PIN, OUTPUT); 
  pinMode(SD_CARD_LED_PIN_L, OUTPUT); 
  pinMode(SD_CARD_LED_PIN_H, OUTPUT); 

  pinMode(SD_CARD_CS_PIN, OUTPUT);

  digitalWrite(REACT_BUTTON_PIN, HIGH); // pullUp on  
  attachInterrupt(0, buttonISR, CHANGE);

  digitalWrite(START_STOP_BUTTON_PIN, HIGH); // pullUp on  

  //reset packet 
  memset((byte*)gpPacket,0, sizeof(sDrtPacket));
  
  //---SD------- 
   //Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CARD_CS_PIN)) {
        Serial.println("SD initialization failed!");
        //reset packet 
        memset((byte*)gpPacket,0, sizeof(sDrtPacket));
        gPacket.result = 'N'; // 'N' SD not available
        sendPacket();//send packet
        gSdCardAvailableF = false;
        //blink two times red
        duoLed(DUO_COLOR_LED_RED);//duoColorLed red
        delay(250);      
        duoLed(DUO_COLOR_LED_OFF);//duoColorLed off
        delay(250);      
        duoLed(DUO_COLOR_LED_RED);//duoColorLed red
        delay(250);      
        duoLed(DUO_COLOR_LED_OFF);//duoColorLed off
        delay(250);      
  }else{
    //Serial.println("initialization done.");
        gSdCardAvailableF = true;
    
       //duoLed(DUO_COLOR_LED_GREEN);//set duoColorLed to green
  }
  gRootNumberOfFiles = getRootNumberOfFiles();//we need this later to assure that we are not over the limit of 512; to seek the directory before every experiment need to much time
  
  modEpromNumber();//fileNumber for logging is set to next hundred on power up
  
  if (gSdCardAvailableF) duoLed(DUO_COLOR_LED_GREEN);//set duoColorLed to green, turn it on here after, all setup is done
  
  //load PWM signalStrength from EEPROM 
  gStimulusStrength = EEPROM.read(STIMULUS_PWM_EEPROM);
  if (gStimulusStrength == 0){//if 0 set to 255, 0 is likely due to first use or clear of EEPROM
    gStimulusStrength = 255;
    EEPROM.write(STIMULUS_PWM_EEPROM, gStimulusStrength);
  }
}
//-------------------------------------------------------------------------------------
int getRootNumberOfFiles(){
  //get number of fles in root--------
  int numberOfFiles = 0;
    
    File root = SD.open("/");
    root.seek(0);
    while(true){
      numberOfFiles++;
      File entry = root.openNextFile();
      if (!entry) {
         entry.close();
         break;
      }
      //Serial.println(entry.name());
      entry.close();
    }
   //root.rewindDirectory();
   root.close();
  
  return numberOfFiles;
}

//-------------------------------------------------------------------------------------
void modEpromNumber(){//set the eprom to the next hundred number

  //load from eeprom
  int lowB  = EEPROM.read(0);  
  int highB = EEPROM.read(1);
  unsigned int temp = lowB + (highB << 8);

 gPacket.fileNumber = temp;//transmit and save old filenumber
 
  if (temp % 100 > 0){
    temp += 100 -(temp % 100); 
  }
  
  //save to eeprom
  EEPROM.write(0,  lowByte(temp)); 
  EEPROM.write(1, highByte(temp));
}
//-------------------------------------------------------------------------------------
void incCurFileNumber(){

  if (!gSdCardAvailableF) return;

  //load from eeprom
  int lowB  = EEPROM.read(0);  
  int highB = EEPROM.read(1);
  gCurFileNumber = lowB + (highB << 8);

  gCurFileNumber++;
  
  gRootNumberOfFiles++;//important so we can assure that we not over the limit of file count of max fiels in root
  
  //save to eeprom
  EEPROM.write(0,  lowByte(gCurFileNumber)); 
  EEPROM.write(1, highByte(gCurFileNumber));

   if (gCurFileNumber > 65000){//hang forever limit of unsigned int is 65535
     Serial.println("CurFileNumber > 65000. Reset EEPROM.");
     while(1){
       //duoColorLed red blinking slow
       duoLed(DUO_COLOR_LED_RED);
       delay(1000);
       duoLed(DUO_COLOR_LED_OFF);
       delay(1000);
     }
   }          
   if (gRootNumberOfFiles > 500){//hang forever limit of files in root folder is 512;
     Serial.println("More than 500 files in rrot dir. Empty SD card");
     while(1){
       //duoColorLed red blinking
       duoLed(DUO_COLOR_LED_RED);
       delay(250);
       duoLed(DUO_COLOR_LED_OFF);
       delay(250);
     }
   } 
}
//-------------------------------------------------------------------------------------
void handleStartStopButton() {//called in every loop
  
  const unsigned int SSCOUNT_ACTION_AT = 500;
  const unsigned int SSCOUNT_PRESSED_AND_HANDLED = 501;
  
  static unsigned long ssDownOld; 
  static unsigned long ssCount; 
  
  int ssButton = digitalRead(START_STOP_BUTTON_PIN);
  if (ssButton == LOW){//if pressed
     if (millis() != ssDownOld){
       ssCount++;
       ssDownOld = millis();
     }
     if (ssCount == SSCOUNT_ACTION_AT){//after the button is pressed a while
       if (gExpRunningF){//toggle; if running => stop, if stopped => start experiment
         stopExp();
       }else{
         startExp(); 
       }
       ssCount = SSCOUNT_PRESSED_AND_HANDLED;
     }  
     if (ssCount > SSCOUNT_ACTION_AT) ssCount = SSCOUNT_PRESSED_AND_HANDLED;//no integer overflow of ssCount, ssCount is catched at SSCOUNT_PRESSED_AND_HANDLED;

   }else{//if not pressed
     ssCount=0;
  }
}
//-------------------------------------------------------------------------------------
void loop() {

   //listen on serial line ------------------------------
  int inByte = 0; //reset in every loop
  if (Serial.available() > 0) inByte = Serial.read();//read in
  
  if (((inByte == '#')||(inByte == 32)) && (!gExpRunningF)) startExp();//start experiment with '#' or 32 (=SPACE)
  if (((inByte == '$')||(inByte == 27)) &&  (gExpRunningF)) stopExp();//stop experiment with '$' or 27 (=ESC)
  if (inByte == 'b') gReadablePacketSendF = false;//binary send format
  if (inByte == 'r') gReadablePacketSendF = true;//readable send format
  //if ((inByte == '?') && (!gExpRunningF)) sendCardDataLastFile();//send last logged data from card over seriall  
  //if ((inByte == '*') && (!gExpRunningF)) sendCardDataAllFiles();//send all card data over serial
  if ((inByte >= 48) && (inByte <= 57)) gMarker = inByte;//set marker '0' to '9'
  if ((inByte == '+') && (!gExpRunningF) &&  (gStimulusStrength < 255)) {
    EEPROM.write(STIMULUS_PWM_EEPROM, ++gStimulusStrength);//increase PWM strength and save
    if (gStimulsOnF) setStimulus(gStimulusStrength);//if stimulus is on, refresh it with new value
  }
  if ((inByte == '-') && (!gExpRunningF) &&  (gStimulusStrength > 1)){
    EEPROM.write(STIMULUS_PWM_EEPROM, --gStimulusStrength); //decrease PWM strength and save. 0 is not possible because it is converted to 255 in setup
    if (gStimulsOnF) setStimulus(gStimulusStrength);//if stimulus is on, refresh it with new value
  }
  if ((inByte == 't') && (!gExpRunningF)) {//switch stimuls on/off for 't'esting
    gStimulsOnF = !gStimulsOnF; //toggle
    if (gStimulsOnF) setStimulus(gStimulusStrength);
    else digitalWrite(STIMULUS_LED_PIN,LOW);
  }
  //----------------------------------------------------

  handleStartStopButton();

//refresh buttonState LED
  int buttonState = digitalRead(REACT_BUTTON_PIN);
  digitalWrite(BUTTON_CLOSED_LED_PIN,!buttonState); 


  if (gExpRunningF){
    handleDRT();
  }else{
    unsigned long now = millis();//millis not micros!
    static unsigned long last;
    if ((now - last) > 1000){ //if expriment not running, send every second a "R" ready packet
      last = now;
        unsigned int temp1 = gPacket.edges;//we save edges over the packet reset
        unsigned int temp2 = gPacket.edgesDebounced;//we save edgesDebounced over the packet reset
        //reset packet 
        memset((byte*)gpPacket,0, sizeof(sDrtPacket));
        gPacket.result = 'R'; // 'R' Ready to start
        gPacket.edges = temp1;
        gPacket.edgesDebounced = temp2;
        sendPacket();//send empty packet as ready message
    }
  } 

}
//-------------------------------------------------------------------------------------
/*
void sendCardDataLastFile(){
  if (!gSdCardAvailableF){
    Serial.println("SD card not initialised");
    return;
  }
  
  int lowB  = EEPROM.read(0);  
  int highB = EEPROM.read(1);
  int number = lowB + (highB << 8);
  File file;
  char fileName[16];
  do{
      sprintf(fileName, "%08d.txt", number);
      number--;
  }while(!SD.exists(fileName) && (number > -1));
  
  Serial.println("");
  Serial.println("---------------------");
  
    // open file for reading:
    file = SD.open(fileName);
    if (file) {
      Serial.println(fileName);
      
      // read from the file
      while (file.available()) {
      	Serial.write(file.read());
      }
      file.close();
    } else {
      Serial.print("error opening file: ");
      Serial.println(fileName);
      //duoColorLed red 
      duoLed(DUO_COLOR_LED_RED); 
    }
    
      Serial.println("---------------------"); 
}
*/
//-------------------------------------------------------------------------------------
/*
void sendCardDataAllFiles(){
  File file;
  
  if (!gSdCardAvailableF){
    Serial.println("SD card not initialised");
    return;
  }
  
  Serial.println("");
  Serial.println("---------------------");
    
  char fileName[16];
  int lowB  = EEPROM.read(0);  
  int highB = EEPROM.read(1);
  int number = lowB + (highB << 8);

  for (unsigned int i = 0; i<=number;i++){
   sprintf(fileName, "%08d.txt", i); 
    if (!SD.exists(fileName)) continue;
    
    //if (Serial.available() > 0) break;//break if new command on serial
    
    // open file for reading:
    file = SD.open(fileName);
    if (file) {
      Serial.println(fileName);
      
      // read from the file
      while (file.available()) {
      	Serial.write(file.read());
      }
      file.close();
    } else {
      Serial.print("error opening file: ");
      Serial.println(fileName);
      //duoColorLed red 
       duoLed(DUO_COLOR_LED_RED);    
    }
    
      Serial.println("---------------------");

  }  
}
*/
//-------------------------------------------------------------------------------------
void writeHeader(){//helper stub
  writeHeaderOrData(true);
}
//-------------------------------------------------------------------------------------
void writeData(){//helper stub
  writeHeaderOrData(false);
}
//-------------------------------------------------------------------------------------
void writeHeaderOrData(byte writeHeader){//true: writeHeader, false: data
  
  File file;
  
  if (!gSdCardAvailableF) return;   

  char fileName[16];//actual file for saving
  sprintf(fileName, "%08d.txt", gCurFileNumber);
  file = SD.open(fileName, FILE_WRITE);
  
  //Serial.println(actFileName);
  // if the file opened okay, write to it:
  if (file) {
      if (writeHeader){//write header
        file.println(HEADER+VERSION);   
      }
      else{//write data
        //Serial.println("logging");  
        file.print(gPacket.count);
        file.print(";");
        file.print(gPacket.stimulusT);
        file.print(";");
        file.print(gPacket.onsetDelay);
        file.print(";");
        file.print(gPacket.soa);
        file.print(";");
        file.print(gPacket.soaNext);
        file.print(";");
        file.print(gPacket.rt);
        file.print(";");
        file.print(char(gPacket.result));
        file.print(";");
        file.print(char(gPacket.marker));
        file.print(";");
        file.print(gPacket.edges);
        file.print(";");
        file.print(gPacket.edgesDebounced);
        file.print(";");
        file.print(gPacket.hold);
        file.print(";");
        file.print(gPacket.buttonDownCount);
        file.print(";");
        file.println(gPacket.stimulusStrength);
      }  
  
    file.close();

  } else {
    // if the file didn't open, print an error:
    //Serial.println("error opening file");
    //duoColorLed red 
      duoLed(DUO_COLOR_LED_RED);
      gPacket.result = 'E'; // 'E' error while logging
      sendPacket();//send packet
     
  }
} 
//-------------------------------------------------------------------------------------
void stopExp(){
    gExpRunningF = false; 
    //we discard any started trial; so do nothing but switch led off
    digitalWrite(STIMULUS_LED_PIN,LOW);//led off
    digitalWrite(EXP_RUNNING_LED_PIN,LOW);//led off

    gResponseWindowF = false;    
    
    gPacket.result = '$'; // '$' stop of experiment; send pending data like edge counts
    sendPacket();//send packet

}
//-------------------------------------------------------------------------------------
unsigned long getRandomStimulusOnset(){
  return STIM_MIN + random(STIM_MAX - STIM_MIN);
}
//-------------------------------------------------------------------------------------
unsigned long setStimulus(byte value){
  if (value < 255){
        analogWrite(STIMULUS_LED_PIN,value);//stimulus on with PWM
  }else{
        digitalWrite(STIMULUS_LED_PIN,HIGH);//stimulus full on without PWM
  }
}
//-------------------------------------------------------------------------------------
void startExp(){

    gExpRunningF = true;
      
    incCurFileNumber(); //set global file number to a new value

    writeHeader();
    
    digitalWrite(EXP_RUNNING_LED_PIN,HIGH);//led on
    
    digitalWrite(STIMULUS_LED_PIN,LOW);//stimulus off
    gStimulsOnF = false;  

    randomSeed(analogRead(3));//read floating A3 for seed
    //randomSeed(77); //pseudo random

    gExpStartT = micros();
    
    memset((byte*)gpPacket,0, sizeof(sDrtPacket));//reset packet and experiment data
    //implicitly done by reset packet
    //gPacket.stimulusT = 0;
    gPacket.soaNext = getRandomStimulusOnset();
    
    gPacket.result = '#'; // '#' start of experiment    
    sendPacket();//send packet
    
    //now we set count to 1 after '#' packet is sent, so '#' gets count 0
    gPacket.count = 1; 
    
    gResponseWindowF = false;
    
    //reset marker
    //gMarker = MARKER_DEFAULT;
}
//-------------------------------------------------------------------------------------
void handleDRT(){

  unsigned long experimentT = (micros() - gExpStartT);

  //set stimuli--------------------------------------
  if ((experimentT - gPacket.stimulusT) >= gPacket.soaNext){
    setStimulus(gStimulusStrength);
    gStimulsOnF = true;
    experimentT = (micros() - gExpStartT);//stop time again, maybe more accurate
    gPacket.onsetDelay =  experimentT - (gPacket.stimulusT + gPacket.soaNext);
    gPacket.stimulusT = experimentT;
    gPacket.soa = gPacket.soaNext;
    gPacket.soaNext = getRandomStimulusOnset();
    gUnhandeledPushEventF = false;
    gResponseWindowF = true;     
  }
  //----------------------------------------------------

  //cheat/hit/miss/--------------------------------------
  
  if ((gStimulsOnF)&& (experimentT - gPacket.stimulusT >= 1000000)){//switch of stimulus after 1 000 000us 
      digitalWrite(STIMULUS_LED_PIN,LOW);//stimulus off
      gStimulsOnF = false;
  }
  
  
  if (gResponseWindowF){
    if (experimentT - gPacket.stimulusT >= MISS){//timeout, user didnt react
      gCalculateHoldF = false;
      gPacket.rt = 0;
      gPacket.result = 'M';//miss
      logging();
      gResponseWindowF = false;  
    } 
    else{

      if (gUnhandeledPushEventF){
        digitalWrite(STIMULUS_LED_PIN,LOW);//stimulus off
        gStimulsOnF = false;
        gCalculateHoldF=true;
        gHoldStartT = gButtonDownT;

        gPacket.rt = gButtonDownT - gExpStartT- gPacket.stimulusT;//gButtonDownT is in uptime micros so it is converted to 'experiment time' by subtract gExpStartT
        if (gPacket.rt < CHEAT){
          gPacket.result = 'C';//cheat
        }else{
          gPacket.result = 'H'; //hit
        }  
        logging();
        gResponseWindowF = false;  
        gUnhandeledPushEventF = false;//reset flag
      }   
    }
  } 
  //----------------------------------------------------

  //holdtime
  if (gUnhandeledReleaseEventF){
    if (gCalculateHoldF){
      gPacket.hold = gHoldStopT - gHoldStartT;
      gCalculateHoldF = false;
    }
    gUnhandeledReleaseEventF = false;
  }

}
//-------------------------------------------------------------------------------------
void logging(){
    
  sendPacket();//send packet, first send packet so SD writing wont delay further
  writeData();//write data to SDcard
  
  gPacket.count++; // increment packet count
  gPacket.edges = 0; // reset edge count
  gPacket.edgesDebounced = 0; // reset edge count

}
//-------------------------------------------------------------------------------------
void sendPacket(){
   gPacket.stimulusStrength = gStimulusStrength;//set current PWM signal strength in packet
   gPacket.fileNumber = gCurFileNumber;//set current logging file in packet
   gPacket.marker = gMarker;//set marker in packet before send and write

  if(gReadablePacketSendF){  
    //readable    
    Serial.print("count;");
    Serial.print(gPacket.count);
    Serial.print(";stimulusT:");
    Serial.print(gPacket.stimulusT);
    Serial.print(";onsetDelay:");
    Serial.print(gPacket.onsetDelay);
    Serial.print(";soa:");
    Serial.print(gPacket.soa);
    Serial.print(";soaNext:");
    Serial.print(gPacket.soaNext);
    Serial.print(";rt:");
    Serial.print(gPacket.rt);
    Serial.print(";result:");
    Serial.print(char(gPacket.result));
    Serial.print(";marker:");
    Serial.print(char(gPacket.marker));
    Serial.print(";edges:");
    Serial.print(gPacket.edges);
    Serial.print(";edgesDebounced:");
    Serial.print(gPacket.edgesDebounced);
    Serial.print(";hold:");
    Serial.print(gPacket.hold);
    Serial.print(";buttonDownCount:");
    Serial.print(gPacket.buttonDownCount);
    Serial.print(";fileNumber:");
    Serial.print(gPacket.fileNumber);   
    Serial.print(";pwm:");
    Serial.println(gPacket.stimulusStrength);   
  }else{
    //byte array
    Serial.write((byte*)gpPacket,sizeof(sDrtPacket));//send oevr serial com/usb
  }


 
}
//-------------------------------------------------------------------------------------
volatile unsigned long gLastEdge = 0;
const unsigned long BOUNCING = 15000;//edges within BOUNCING micro secs after last handeled edge are discarded

void buttonISR(){
    
    unsigned long now = micros();
    int buttonState = digitalRead(REACT_BUTTON_PIN);
  
    //count edges
    gPacket.edges++;
    
    if (now - gLastEdge < BOUNCING){
      return;
    }

    gLastEdge = now;
    gPacket.edgesDebounced++;

    if (gExpRunningF){

     if (buttonState==HIGH){//this is a button release
        if (!gUnhandeledReleaseEventF){  
          gHoldStopT = now;
          gUnhandeledReleaseEventF = true;
        }  
      }
      else{ //this is a button press 
       gPacket.buttonDownCount++;

        if (!gUnhandeledPushEventF){  
          gButtonDownT = now;
          gUnhandeledPushEventF = true;
        }  
      }

    } //expRunning
}
//-------------------------------------------------------------------------------------


