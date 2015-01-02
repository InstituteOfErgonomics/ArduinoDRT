//------------------------------------------------------
//Revision History 'Arduino-DRT-Plain'
//------------------------------------------------------
//Version  Date		Author		  Mod
//1        Mar, 2014	Michael Krause	  initial
//1.1      Mai, 2014    Michael Krause    gRootNumberOfFiles++ bug
//1.2      Aug, 2014    Michael Krause    meanRt and hitRate
//1.3      Nov, 2014    Michael Krause    shortened error messages, this removes non logging on SDcard bug. Important note: SRAM (string const, etc) was full, 1.2.1 sketch was compiled without error but failed during operation 
//2.0      Jan, 2015    Michael Krause    part. refactored (const EEPROM and handleCommand()) & added measurement() for piezo & improved ISR
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

const int PIEZO = 0;//piezo connected to analog0
const int ERR = -1;//error in piezo measurement

const int DUO_COLOR_LED_OFF = 0; 
const int DUO_COLOR_LED_GREEN = 1; 
const int DUO_COLOR_LED_RED = 2; 

const String HEADER = "count;stimulusT;onsetDelay;soa;soaNext;rt;result;marker;edges;edgesDebounced;hold;buttonDownCount;pwm;";
const String VERSION = "V2.0-plain";//plain, without Ethernet. version number is logged to result header
const String LINE = "----------";
const String SEP = ";";

const unsigned long CHEAT = 100000;//lower 100000 micro seconds = cheat
const unsigned long MISS = 2500000;//greater 2500000 micro seconds = miss

const unsigned long STIM_MIN = 3000000;//next stimulus min after x micro seconds
const unsigned long STIM_MAX = 5000000;//next stimulus max after x micro seconds

const byte EEPROM_FILENUM_L = 0x00;//location in EEPROM to file number low byte
const byte EEPROM_FILENUM_H = 0x01;//location in EEPROM to file number high byte
const byte EEPROM_STIMULUS_PWM = 0x02;//location in EEPROM to save pwm stimulus signal strength

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
unsigned long gRtSum=0;//sum of all reaction times during one experiment, to calculate meanRt
int gButtonState;//button state of reaction button

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
  unsigned long meanRt;// mean rt in this experiement, up to now
  unsigned long hitCount;//count hits in this experiment 
  unsigned long missCount;//count miss in this experiment 
  unsigned long cheatCount;//count cheat in this experiment  
  byte hitRate;//hit rate in this experiment, up to now (0-100) 
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
//helper
void duoLedBlink(int howOften, int halfcycle, int color){
       for(int i=0;i <howOften;i++){
         duoLed(color);
         delay(halfcycle);
         duoLed(DUO_COLOR_LED_OFF);
         delay(halfcycle);
       }  
}
//---------------------------------------------------------------------------
void sdInit(){
   //---SD-init-------------------------------------- 
   //Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CARD_CS_PIN)) {
        Serial.println("SD init failed");
        //reset packet 
        memset((byte*)gpPacket,0, sizeof(sDrtPacket));
        gPacket.result = 'N'; // 'N' SD not available
        sendPacket();//send packet
        gSdCardAvailableF = false;
        //blink two times red
        duoLedBlink(2, 250, DUO_COLOR_LED_RED);
  }else{
    //Serial.println("initialization done.");
        gSdCardAvailableF = true;
    
       //duoLed(DUO_COLOR_LED_GREEN);//set duoColorLed to green
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


  //init gButtonState global var
  gButtonState = digitalRead(REACT_BUTTON_PIN);
  
  digitalWrite(REACT_BUTTON_PIN, HIGH); // pullUp on  
  attachInterrupt(0, buttonISR, CHANGE);

  digitalWrite(START_STOP_BUTTON_PIN, HIGH); // pullUp on  

  //reset packet 
  memset((byte*)gpPacket,0, sizeof(sDrtPacket));

  sdInit();
  
  gRootNumberOfFiles = getRootNumberOfFiles();//we need this later to assure that we are not over the limit of 512; to seek the directory before every experiment need to much time
  
  modEpromNumber();//fileNumber for logging is set to next hundred on power up
  
  if (gSdCardAvailableF){
    duoLed(DUO_COLOR_LED_GREEN);//set duoColorLed to green, turn it on here after, all setup is done
  }
  else{
    //while(1); //hang forever if you dont want that someone can start without a SD card
  } 

  //load PWM signalStrength from EEPROM 
  gStimulusStrength = EEPROM.read(EEPROM_STIMULUS_PWM);
  if (gStimulusStrength == 0){//if 0 set to 255, 0 is likely due to first use or clear of EEPROM
    gStimulusStrength = 255;
    EEPROM.write(EEPROM_STIMULUS_PWM, gStimulusStrength);
  }
  
   //TCCR1B = TCCR1B & 0b11111000 | 0x01;//setting timer1 divisor to 1 => 31250Hz on Pin 9 & 10 for Arduino >>Uno<< so we can use TI DRV2603 & LRA  

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
  int lowB  = EEPROM.read(EEPROM_FILENUM_L);  
  int highB = EEPROM.read(EEPROM_FILENUM_H);
  unsigned int temp = lowB + (highB << 8);

 gPacket.fileNumber = temp;//transmit and save old filenumber
 
  if (temp % 100 > 0){
    temp += 100 -(temp % 100); 
  }
  
  //save to eeprom
  EEPROM.write(EEPROM_FILENUM_L,  lowByte(temp)); 
  EEPROM.write(EEPROM_FILENUM_H, highByte(temp));
}
//-------------------------------------------------------------------------------------
void incCurFileNumber(){

  if (!gSdCardAvailableF) return;

  //load from eeprom
  int lowB  = EEPROM.read(EEPROM_FILENUM_L);  
  int highB = EEPROM.read(EEPROM_FILENUM_H);
  gCurFileNumber = lowB + (highB << 8);

  gCurFileNumber++;
  
  gRootNumberOfFiles++;//important so we can assure that we not over the limit of file count of max fiels in root
  
  //save to eeprom
  EEPROM.write(EEPROM_FILENUM_L,  lowByte(gCurFileNumber)); 
  EEPROM.write(EEPROM_FILENUM_H, highByte(gCurFileNumber));

   if (gCurFileNumber > 65000){//hang forever limit of unsigned int is 65535
     Serial.println("E65000. Reset EEPROM.");
     while(1){
       //duoColorLed red blinking slow
       duoLedBlink(1, 1000, DUO_COLOR_LED_RED);
     }
   }          
   if (gRootNumberOfFiles > 500){//hang forever limit of files in root folder is 512;
     Serial.println("E500. Empty SD card");
     while(1){
       //duoColorLed red blinking
       duoLedBlink(1, 250, DUO_COLOR_LED_RED);
     }
   } 
}
//-------------------------------------------------------------------------------------
void handleStartStopButton() {//called in every loop
  
  const unsigned int SSCOUNT_ACTION_AT = 300;
  const unsigned int SSCOUNT_PRESSED_AND_HANDLED = 301;
  
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
void handleCommand(byte command){
  
  if(!gExpRunningF){//commands which are only handled when experiment is NOT running
  
      switch( command ){
        
    	case '#':
    	case 32://space
              startExp();//start experiment with '#' or 32 (=SPACE)
              break;
            /* 
    	case '?':
              sendCardDataLastFile();//send last logged data from card over serial  
              break;
    	case '*':
              sendCardDataAllFiles();//send all card data over serial
              break;
            */
     	case '+'://careful eeprom write cycles are limited!
              setPWM(++gStimulusStrength);
              break;       
     	case '-'://careful eeprom write cycles are limited!
             setPWM(--gStimulusStrength);
             break;       
  
      	case 't'://toggle stimulus on/off
            gStimulsOnF = !gStimulsOnF; //toggle
            if (gStimulsOnF) setStimulus(gStimulusStrength);
            else digitalWrite(STIMULUS_LED_PIN,LOW);
          break; 
          
      	case 'm'://measurement via piezo
          measurement();
          break;

/*       
    //used during an experiment with different PWMs
      	case 's'://measurement
          gMarker = command;
          setPWM(48);
          break;
         
      	case 'd'://measurement
          gMarker = command;
          setPWM(73);
          break;
         
      	case 'f'://measurement
          gMarker = command;
          setPWM(255);
          break;
         
*/         
         
         default: 
          break;
   
       }  
    
  }else{//commands which are only handled when experiment IS running
  
       switch( command )
       {
  	case '$':
  	case 27://ESC
            stopExp();//stop experiment with '$' or 27 (=ESC)
            break;
         default: 
          break;
       }  
  
  }
  
  //commands which are independant handled of experiment running/not running
  if (command == 'b') gReadablePacketSendF = false;//binary send format
  if (command == 'r') gReadablePacketSendF = true;//readable send format
 
  if ((command >= 48) && (command <= 57)) gMarker = command;//set marker '0' to '9'
  
}
//-------------------------------------------------------------------------------------
void setPWM(byte pwm){
  if (pwm > 0){
    EEPROM.write(EEPROM_STIMULUS_PWM, pwm);//store in EEPROM
    gStimulusStrength = pwm;//set global variable
    if (gStimulsOnF) setStimulus(gStimulusStrength);//if stimulus is on, refresh it with new value
  }
}
//-------------------------------------------------------------------------------------
void loop() {

   //listen on serial line ------------------------------
  int inByte = 0; //reset in every loop
  if (Serial.available() > 0) inByte = Serial.read();//read in
  
  handleCommand(inByte);

  handleStartStopButton();

 //continiously refresh buttonState & LED
  gButtonState = digitalRead(REACT_BUTTON_PIN);
  digitalWrite(BUTTON_CLOSED_LED_PIN,!gButtonState); 


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
    Serial.println("noSD");
    return;
  }
  
  int lowB  = EEPROM.read(EEPROM_FILENUM_L);  
  int highB = EEPROM.read(EEPROM_FILENUM_H);
  int number = lowB + (highB << 8);
  File file;
  char fileName[16];
  do{
      sprintf(fileName, "%08d.txt", number);
      number--;
  }while(!SD.exists(fileName) && (number > -1));
  
  Serial.println("");
  Serial.println(LINE);
  
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
      Serial.print("err");      
      //Serial.print("error opening file: ");
      Serial.println(fileName);
      //duoColorLed red 
      duoLed(DUO_COLOR_LED_RED); 
    }
    
      Serial.println(LINE); 
}
*/
//-------------------------------------------------------------------------------------
/*
void sendCardDataAllFiles(){
  File file;
  
  if (!gSdCardAvailableF){
    Serial.println("noSD");
    return;
  }
  
  Serial.println("");
  Serial.println(LINE);
    
  char fileName[16];
  int lowB  = EEPROM.read(EEPROM_FILENUM_L);  
  int highB = EEPROM.read(EEPROM_FILENUM_H);
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
      Serial.print("err");
      //Serial.print("error opening file: ");
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
        file.print(SEP);
        file.print(gPacket.stimulusT);
        file.print(SEP);
        file.print(gPacket.onsetDelay);
        file.print(SEP);
        file.print(gPacket.soa);
        file.print(SEP);
        file.print(gPacket.soaNext);
        file.print(SEP);
        file.print(gPacket.rt);
        file.print(SEP);
        file.print(char(gPacket.result));
        file.print(SEP);
        file.print(char(gPacket.marker));
        file.print(SEP);
        file.print(gPacket.edges);
        file.print(SEP);
        file.print(gPacket.edgesDebounced);
        file.print(SEP);
        file.print(gPacket.hold);
        file.print(SEP);
        file.print(gPacket.buttonDownCount);
        file.print(SEP);
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
      while(1);//hang forever, we dont continue whithout sd card logging, if sd card was originaly present on start up     
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

    gRtSum=0; //reset sum
            
    incCurFileNumber(); //set global file number to a new value

    writeHeader();//write header to SD file
    
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
      gPacket.missCount++;      
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
          gPacket.cheatCount++;          
        }else{
          gPacket.result = 'H'; //hit
          gPacket.hitCount++;
          gRtSum += gPacket.rt;//add to sum          
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
    //calculate some values
  gPacket.meanRt = gRtSum / gPacket.hitCount;
  gPacket.hitRate = (gPacket.hitCount * 100) / (gPacket.hitCount + gPacket.missCount);
    
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
   //NOTE: this needs a lot of SRAM; if you dont need it comment it out, delete or just transmit what is interesting to you.  
    Serial.print("cnt:");
    Serial.print(gPacket.count);
    Serial.print(SEP);
    Serial.print("stimuT:");
    Serial.print(gPacket.stimulusT);
    Serial.print(SEP);
    Serial.print("onsetDelay:");
    Serial.print(gPacket.onsetDelay);
    Serial.print(SEP);
    Serial.print("soa:");
    Serial.print(gPacket.soa);
    Serial.print(SEP);
    Serial.print("soaNxt:");
    Serial.print(gPacket.soaNext);
    Serial.print(SEP);    
    Serial.print("rt:");
    Serial.print(gPacket.rt);
    Serial.print(SEP);
    Serial.print("rslt:");
    Serial.print(char(gPacket.result));
    Serial.print(SEP);
    Serial.print("meanRt:");
    Serial.print(gPacket.meanRt);
    Serial.print(SEP);
    Serial.print("hitCnt:");
    Serial.print(gPacket.hitCount);
    Serial.print(SEP);
    Serial.print("missCnt:");
    Serial.print(gPacket.missCount);
    Serial.print(SEP);
    Serial.print("cheatCnt:");
    Serial.print(gPacket.cheatCount);
    Serial.print(SEP);
    Serial.print("hitRate:");
    Serial.print(gPacket.hitRate);    
    Serial.print(SEP);
    Serial.print("marker:");
    Serial.print(char(gPacket.marker));
    Serial.print(SEP);
    Serial.print("edg:");
    Serial.print(gPacket.edges);
    Serial.print(SEP);
    Serial.print("edgDebncd:");
    Serial.print(gPacket.edgesDebounced);
    Serial.print(SEP);
    Serial.print("hold:");
    Serial.print(gPacket.hold);
    Serial.print(SEP);
    Serial.print("btnDwnCnt:");
    Serial.print(gPacket.buttonDownCount);
    Serial.print(SEP);
    Serial.print("fileNr:");
    Serial.print(gPacket.fileNumber);
    Serial.print(SEP);   
    Serial.print("pwm:");
    Serial.println(gPacket.stimulusStrength);   
  }else{
    //byte array
    Serial.write((byte*)gpPacket,sizeof(sDrtPacket));//send oevr serial com/usb
  }


 
}
//-------------------------------------------------------------------------------------
volatile unsigned long gLastEdge = 0;
const unsigned long BOUNCING = 15000;//edges within BOUNCING micro secs after last edge are discarded

void buttonISR(){
    
    unsigned long now = micros();
    //int buttonState = digitalRead(REACT_BUTTON_PIN); //this simple statement was used before to detect later if falling or rising edge and sometimes got wrong values! impressive fast bouncing spikes.
    //so we do it below and with a little more insight
  
    //count edges
    gPacket.edges++;
    
    if (now - gLastEdge < BOUNCING){
      gLastEdge = now;
      return; //discard
    }

    gLastEdge = now;
    gPacket.edgesDebounced++;
    
    //workaround for buttonState problem
    int isrButtonState = digitalRead(REACT_BUTTON_PIN);
    if (isrButtonState == gButtonState){ // new == old?
      //here is something wrong, the ISR is issued by a change and now we detect no change (new value == old value)?! 
      if (gButtonState==HIGH){ 
        gPacket.edgesDebounced += 100;//we add 100 to log this strange event with a strange value for falling edge (push event)
      }
      else{
        gPacket.edgesDebounced += 1000;//we add 1000 to log this strange event with a strange value for rising edge (release event)
      }
      //we trust the gButtonState (old stored value before the ISR) more and invert isrButtonState
      if(isrButtonState==HIGH){
        isrButtonState = LOW;
      }else{
        isrButtonState=HIGH;
      }
    }

    if (gExpRunningF){

     if (isrButtonState==HIGH){//this is a button release
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
//below this line piezo measurement. to measure some indicational data from the vibro tactor via a piezo element on A0
//-------------------------------------------------------------------------------------
const String RAMP = "ramp";
const String LAG = "lag";
const String FREQ = "freq";
const String US = " [us]:";
const String HZ = " [Hz]:";

// use external piezo to measure some tech details from vibrator motor (start lag, ramp up, rotation frequency)
void measurement(){
  
    const int REPEATED_MEASUREMENT = 10;
    long int lags[REPEATED_MEASUREMENT];
    long int ramps[REPEATED_MEASUREMENT];
    long int freqs[REPEATED_MEASUREMENT];
    
    ADCSRA = ADCSRA & 0b11111000 | 0x04;//setting adc prescaler to 16 => adc sampling > ~60kHz
  
    for(int i = 0; i<REPEATED_MEASUREMENT; i++){//measure 10 times
       Serial.println(LINE);
       Serial.print(i+1);
       Serial.print("/");
       Serial.println(REPEATED_MEASUREMENT);
      //order important!:
      //   measure_lag() relies that motor was switched off; 
      //   measure_ramp() relies that measure_lag() was performed before
      //   measure_freq() relies that measure_ramp() was performed before
      digitalWrite(STIMULUS_LED_PIN,LOW);//switch off
      delay(500);//wait 500ms
     
      lags[i] = measure_lag();
    
      if (lags[i] != ERR){
        ramps[i] = measure_ramp();
      }else{
        ramps[i] = ERR;
      }
      freqs[i] = measure_freq();
      
    }   
    
       digitalWrite(STIMULUS_LED_PIN,LOW);//switch off
       Serial.println(LINE);
       Serial.print(" PWM ");
       Serial.println(gStimulusStrength);
       
      int length;
      
      Serial.println("Medians");
      length = discardErrors(lags, REPEATED_MEASUREMENT);
      Serial.print(LAG);
      Serial.print(US);
      Serial.println(median(lags, length));
      length = discardErrors(ramps, REPEATED_MEASUREMENT);
      Serial.print(RAMP);
      Serial.print(US);
      Serial.println(median(ramps, length));
      length = discardErrors(freqs, REPEATED_MEASUREMENT);
      Serial.print(FREQ);
      Serial.print(HZ);
      Serial.println(median(freqs, length));

       Serial.println(LINE); 
       
}

//-------------------------------------------------------------------------------------
long int measure_lag(){
  
    //measure for 100000us what is the avg&max noise while 'quiet'=motor off
    Serial.println(LAG);

    unsigned int noiseMax =0;
    unsigned long int avgNoiseSum =0;
    unsigned long int avgNoiseCount =0;
    int avgNoise;
    int analogIn;
    unsigned long startQuietT = micros();
    unsigned long nowT = micros();
    boolean error = false;
    
    
    //do{//read until low noise or timeout
      while(1){
        nowT = micros();
        if ((nowT - startQuietT) > 100000){break;}//measure for 50ms
        analogIn = analogRead(PIEZO);
        avgNoiseSum += analogIn;
        avgNoiseCount++;
        if (analogIn > noiseMax){ noiseMax = analogIn;}
      }//end while
      avgNoise = avgNoiseSum / avgNoiseCount;
    //  if ((nowT - startQuietT) > 1000000){error = true; break;}//max 1 sec
    //}while(avgNoise > 20);
    
    //logic: switch motor on and stop time when analogValue is several samples higher than a threshold
    unsigned long startOfMotorT =  micros();
    setStimulus(gStimulusStrength);//switch motor on
    int threshold = avgNoise + noiseMax/2 +2; //+2 prevent threshold=0 in realy quiet environment
    
    Serial.print(" thrs [0-1024]: ");
    Serial.println(threshold);
    
    int overThresholdCount = 0;
    
    while(1){
      nowT = micros();
      analogIn = analogRead(PIEZO);
      if (analogIn > threshold){
        overThresholdCount++;
         if (overThresholdCount > 30) {break;}
      }else{
        overThresholdCount = 0;
      }  
      if ((nowT - startOfMotorT) > 250000){error = true; break;}//maximum lag 250000 uSec
    }//end while
    long int lagT = (nowT- startOfMotorT);//save lag time
    
    if (error){
      Serial.println("ERR");//timeout 
      lagT = ERR;
    }
    
    Serial.print(LAG);
    Serial.print(US);
    Serial.println(lagT);
    
    return lagT;
    
}


//-------------------------------------------------------------------------------------
long int measure_ramp(){
  
    //measure ramp up of motor
    //logic: save the time everytime the input value level gets 10% higher;
    //       the last saved time should be the end of the ramp 
    Serial.println(RAMP);
    int analogIn;
    unsigned long nowT = micros();
    unsigned long startOfRampT =  nowT;
    unsigned long lastIntensityIncrementT =  nowT;
    int intensityThreshold = 0;
    int intensityThresholdCount = 0;
    
    while(1){
      nowT = micros();
      analogIn = analogRead(PIEZO);
      if (analogIn > intensityThreshold) {
        intensityThresholdCount++;
        if (intensityThresholdCount > 15){// are more than 15 sequential measurement values above the threshold?
          //set new threshold and save time
          intensityThreshold = analogIn + analogIn/10;//set new intensityThreshold 10% higher
          lastIntensityIncrementT = micros();
        }
      }else{
        intensityThresholdCount=0;
      }  
  
      if ((nowT - startOfRampT) > 1000000){ break;}//measure for max 1 000 000 uSec
    }//end while
    long int rampT = (lastIntensityIncrementT- startOfRampT);
    
    Serial.print(" thrs [0-1024]: ");
    Serial.println(intensityThreshold);   
    Serial.print(RAMP);
    Serial.print(US);
    Serial.println(rampT);
    
    return rampT;
    
}

//-------------------------------------------------------------------------------------
long int measure_freq(){
  
    //measure frequency  
    //logic: the piezo generates a AC signal; part of it is discarded due to internal clamp diode (rectification); we measure from start of zeroPeriod to zeroPeriod to get frequency
    Serial.println(FREQ);
        
    int analogIn;
    unsigned long nowT = micros();
    
    unsigned long startFreqMeasurementT =  micros();
    unsigned long zeroT;//start of last zero period (note: is not init! we do it with the firstFlag construct)
    int oldAnalogIn = 1;
    unsigned long int tempSum=0;//sum all diffT to average
    unsigned long int tempCount=0;//
    unsigned long int diffT;
    boolean firstF = true;//flag if zeroT is init
    long int freq;
    
    while(1){
      nowT = micros();
      diffT = nowT - zeroT;
      analogIn = analogRead(PIEZO);
    
      if ((oldAnalogIn > 0) && (analogIn==0)){//start of zero period
        if(((diffT > 2000) && (diffT < 20000))||(firstF)){//between 2000uSec and 20000uSec; 50Hz-500Hz; or zeroT is not init(firstF=true)
          zeroT = nowT;
          if(!firstF){ 
            tempSum += diffT;
            tempCount++;
          }
          firstF = false;  
        }//50Hz-500Hz
      }//zero period
      oldAnalogIn = analogIn;
    
      if ((nowT - startFreqMeasurementT) > 500000){break;}//measure for 500 000 uSec
    }//end while
    
    
    if (tempCount>0){
      
      freq = (1000000 * tempCount) / (tempSum +1); //+1us prevent div0
      Serial.print(FREQ);
      Serial.print(HZ);
      Serial.println(freq);
      
    }else{
      Serial.println("ERR");
      freq = ERR;
    }
    
  return freq;

  
}

//-------------------------------------------------------------------------------------
int discardErrors(long int *array, int length){//adjust length so errors '-1' in sorted array are discarded

  bubblesort(array, length);//sort ascending
  
  int countErrors = 0;
  

  for(int i=0; i<length; i++){
    if (array[i] == ERR) countErrors++;
  }
  
  for(int i=0; i+countErrors < length; i++){
    array[i] = array[i+countErrors];//overwrite array fields which contains errors (-1), due to sorting they are on low fields. after that valid values are in [0] to [x]
  }

  return length - countErrors;
  
}
//-------------------------------------------------------------------------------------
long int median(long int *array, int length){
  long int median;
  int m;
  //bubblesort(array, length);//already sorted by discardErrors
  if (length % 2 == 0)
    median = array[length/2];
  else
    median = (array[length/2] + array[(length/2)+1]) / 2;

  return median;
}
//-------------------------------------------------------------------------------------
 void bubblesort(long int *array, int length)//from wikibooks
 {
     int i, j;
     for (i = 0; i < length -1; ++i) 
     {
 
 	for (j = 0; j < length - i - 1; ++j) 
        {
 	    if (array[j] > array[j + 1]) 
            {
 		long int tmp = array[j];
 		array[j] = array[j + 1];
 		array[j + 1] = tmp;
 	    }
 	}
     }
 }
//-------------------------------------------------------------------------------------


