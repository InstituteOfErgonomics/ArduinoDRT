//------------------------------------------------------
//Revision History 'Arduino-DRT-Ethernet'
//------------------------------------------------------
//Version  Date		Author		  Mod
//1        Mar, 2014	Michael Krause	  initial
//1.1      Mai, 2014    Michael Krause    gRootNumberOfFiles++ bug
//1.2      July, 2014   Michael Krause    meanRt and hitRate
//1.2.1    Aug, 2014    Michael Krause    added error msg
//1.3      Nov, 2014    Michael Krause    shortened error messages, this removes non logging on SDcard bug. Important note: SRAM (string const, etc) was full, 1.2.1 sketch was compiled without error but failed during operation 
//2.0      Jan, 2015    Michael Krause    part. refactored (const EEPROM and handleCommand()) & improved ISR
//2.1      Feb, 2015    Michael Krause    improved pwm+/-;
//2.2      Mar, 2015    Michael Krause    same readable statements in plain/ethernet/mega; added reset eeprom command for file number
//
//VERSION const
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
#include <Ethernet.h>

//---------------------------------------------------------------------------
//CONST
//ethernetshield uses 10,11,12,13 for the network chip and 4 for chip select SD card
const int SD_CARD_CS_PIN = 4;//chip select for SD-card is 4 on ethernet shield

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

const String HEADER = "cnt;stimT;onsetDly;soa;soaNxt;rt;rslt;marker;edgs;edgsDbncd;hld;btnDwnCnt;pwm;";
const String VERSION = "V2.2-e";//with 'e'thernet. version number is logged to result header
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

  // Enter a MAC address for your controller below.
  // Newer Ethernet shields have a MAC address printed on a sticker on the shield
  byte gMac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02 };
  IPAddress gIp(192,168,1,15);
  //IPAddress gIp(10,152,238,77);
  
  // the router's gateway address:
  //byte gGateway[] = { 192, 168, 2, 1 };
  // the subnet:
  //byte gSubnet[] = { 255, 255, 255, 0 };
  
  EthernetServer gServer(7008);//enter PORT number
  EthernetClient gClient;




//----------
#pragma pack(1)
typedef struct myDrtPacket{
  unsigned int count; //stimuli/packet count
  unsigned long stimulusT; //stimulus showen microsec (us) after start of experiment
  unsigned long onsetDelay;//error in us of onset compared to schedule, always positive
  unsigned long soa;//random stimulus onset asynchrony in us; current
  unsigned long soaNext;//random stimulus onset asynchrony in us between current and next
  unsigned long rt;// reaction time in us; in case of miss, rt is set to 0
  byte result; //'H' hit, 'M' miss or 'C' cheat. status message 'R' ready to start, '#' experiment started, '$' experimentz stopped, 'N' no sd card, 'E' error while logging
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
        //Serial.println("initialization failed!");
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
  //!!! disable SD card SPI while starting ethernet
  digitalWrite(SD_CARD_CS_PIN, HIGH);
  
  digitalWrite(REACT_BUTTON_PIN, HIGH); // pullUp on  
  attachInterrupt(0, buttonISR, CHANGE);

  digitalWrite(START_STOP_BUTTON_PIN, HIGH); // pullUp on  

  memset((byte*)gpPacket,0, sizeof(sDrtPacket));//reset packet
  

     Ethernet.begin(gMac,gIp);
     //Ethernet.begin(mac, ip, gateway, subnet);  
   gServer.begin();
  
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

  gCurFileNumber = temp;
 
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
     Serial.println("E65000 Reset EEPROM");
     while(1){
       //duoColorLed red blinking slow
       duoLedBlink(1, 1000, DUO_COLOR_LED_RED);
     }
   }          
   if (gRootNumberOfFiles > 500){//hang forever limit of files in root folder is 512;
     Serial.println("E500 Empty SD card");
     while(1){
       //duoColorLed red blinking
       duoLedBlink(1, 250, DUO_COLOR_LED_RED);
     }
   } 
}
//-------------------------------------------------------------------------------------
void handleStartStopButton() {//called in every loop
  
  const unsigned int SSCOUNT_ACTION_AT = 15;
  const unsigned int SSCOUNT_PRESSED_AND_HANDLED = SSCOUNT_ACTION_AT+1;

  
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
              if (gStimulusStrength < 255) setPWM(++gStimulusStrength);
              break;       
     	case '-'://careful eeprom write cycles are limited!
             if (gStimulusStrength > 1) setPWM(--gStimulusStrength);
             break;       
  
      	case 't'://toggle stimulus on/off
            gStimulsOnF = !gStimulsOnF; //toggle
            if (gStimulsOnF) setStimulus(gStimulusStrength);
            else digitalWrite(STIMULUS_LED_PIN,LOW);
          break; 
          
      	case 'p'://'p' ping
            gPacket.result = 'P'; // 'P' ping back
            sendPacket();//send ping packet as message
          break; 

      	case '~'://reset file number in eeprom
            EEPROM.write(EEPROM_FILENUM_L, 0); 
            EEPROM.write(EEPROM_FILENUM_H, 0);
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
    
  }else{//commands wich are only handled when experiment IS running
  
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

  int inByte = 0; //reset in every loop
  
    if ((gClient) && (!gClient.connected())) {//stop and null if disconnected
      gClient.stop();
    }
    
    if ((gClient) && (gClient.available())){inByte = gClient.read();}
    
   //listen on serial line ------------------------------
  if (Serial.available() > 0) inByte = Serial.read();//read in

  handleCommand(inByte);  
  
  handleStartStopButton();

 //continiously refresh buttonState & LED
  gButtonState = digitalRead(REACT_BUTTON_PIN);
  digitalWrite(BUTTON_CLOSED_LED_PIN,!gButtonState); 


  if (gExpRunningF){
    handleDRT();
  }else{
    
     // listen for incoming client; this needs some uSecs so we do it only if experiment is not running or in handleDRT() when we are far away (before and after) a stimulus
    gClient = gServer.available();
   
    unsigned long now = millis();//millis not micros!
    static unsigned long last;
    if ((now - last) > 1000){ //if expriment not running, send every second a "R" ready packet
      last = now;
        //reset packet 
        memset((byte*)gpPacket,0, sizeof(sDrtPacket));
        gPacket.result = 'R'; // 'R' Ready to start
        gPacket.fileNumber = gCurFileNumber;
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
    // if the file is not open, print an error:
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

  //----------------------------------------------------
  long diff =  gPacket.soaNext - (experimentT - gPacket.stimulusT);
  if ((diff > 5000) || (diff < -5000)){
      // listen for incoming client; this needs some uSecs so we do it only if experiment is not running or in handleDRT() when we are far away (before and after) a stimulus
    gClient = gServer.available();
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
  
  //first send than write
  sendPacket();//send packet, first send packet so SD writing wont delay further
  writeData();//write data to SDcard
  
  gPacket.count++; // increment packet count
  gPacket.edges = 0; // reset edge count
  gPacket.edgesDebounced = 0; // reset edge count

}
//-------------------------------------------------------------------------------------
void sendPacket(){
  
   gPacket.fileNumber = gCurFileNumber;//set current logging file in packet
   gPacket.marker = gMarker;//set marker in packet before send and write
   gPacket.stimulusStrength = gStimulusStrength;

  if(gReadablePacketSendF){ 

      //readable over ethernet 
      gServer.print("cnt:");
      gServer.print(gPacket.count);
      gServer.print(";stmT:");
      gServer.print(gPacket.stimulusT);
      gServer.print(";onstDly:");
      gServer.print(gPacket.onsetDelay);
      gServer.print(";soa:");
      gServer.print(gPacket.soa);
      gServer.print(";soaNxt:");
      gServer.print(gPacket.soaNext);
      gServer.print(";rt:");
      gServer.print(gPacket.rt);
      gServer.print(";rslt:");
      gServer.print(char(gPacket.result));
      gServer.print(";meanRt:");
      gServer.print(gPacket.meanRt);
      gServer.print(";hCnt:");
      gServer.print(gPacket.hitCount);
      gServer.print(";mCnt:");
      gServer.print(gPacket.missCount);
      gServer.print(";cCnt:");
      gServer.print(gPacket.cheatCount);
      gServer.print(";hitRate:");
      gServer.print(gPacket.hitRate);
      gServer.print(";marker:");
      gServer.print(char(gPacket.marker));
      gServer.print(SEP);
      gServer.print(";edgs:");
      gServer.print(gPacket.edges);
      gServer.print(SEP);
      gServer.print(";edgsDbncd:");
      gServer.print(gPacket.edgesDebounced);
      gServer.print(SEP);
      gServer.print(";hld:");
      gServer.print(gPacket.hold);
      gServer.print(SEP);
      gServer.print(";btnDwnC:");
      gServer.print(gPacket.buttonDownCount);
      gServer.print(SEP);
      gServer.print(";fileN:");
      gServer.print(gPacket.fileNumber);  
      gServer.print(SEP);
      gServer.print(";pwm:");
      gServer.println(gPacket.stimulusStrength);  


    //readable over serial
    
    Serial.print("cnt:");
    Serial.print(gPacket.count);
    Serial.print(";stmT:");
    Serial.print(gPacket.stimulusT);
    Serial.print(";onstDly:");
    Serial.print(gPacket.onsetDelay);
    Serial.print(";soa:");
    Serial.print(gPacket.soa);
    Serial.print(";soaNxt:");
    Serial.print(gPacket.soaNext);
    Serial.print(";rt:");
    Serial.print(gPacket.rt);
    Serial.print(";rslt:");
    Serial.print(char(gPacket.result));
    Serial.print(";meanRt:");
    Serial.print(gPacket.meanRt);
    Serial.print(";hCnt:");
    Serial.print(gPacket.hitCount);
    Serial.print(";mCnt:");
    Serial.print(gPacket.missCount);
    Serial.print(";cCnt:");
    Serial.print(gPacket.cheatCount);
    Serial.print(";hitRate:");
    Serial.print(gPacket.hitRate);
    Serial.print(";marker:");
    Serial.print(char(gPacket.marker));
    Serial.print(";edgs:");
    Serial.print(gPacket.edges);
    Serial.print(";edgsDbncd:");
    Serial.print(gPacket.edgesDebounced);
    Serial.print(";hold:");
    Serial.print(gPacket.hold);
    Serial.print(";btnDwnC:");
    Serial.print(gPacket.buttonDownCount);
    Serial.print(";fileN:");
    Serial.print(gPacket.fileNumber); 
    Serial.print(";pwm:");
    Serial.println(gPacket.stimulusStrength);       
  }else{

      //byte array ethernet
      gServer.write((byte*)gpPacket,sizeof(sDrtPacket)); //send over ethernet
     //byte array serial
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


