#define TINY_GSM_MODEM_SIM800

#include "esp32-hal-timer.h"
#include <TinyGsmClient.h>
#include "Persistence.tpp"
#include <string>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TaskScheduler.h>


#include <vector>
using namespace std;



#define FRAMESIZE 256
#define DHTPIN 27
#define DHTTYPE DHT21
#define SERIAL_SIZE_RX 1024
#define SerialMon Serial
#define SerialAT Serial1
#define MODEM_RX 16
#define MODEM_TX 17
#define Power_Key 26
#define BUZZER_PIN 23
#define ACT_PIN 19
#define INTERVAL 1000
#define REL1_PIN 32
#define REL2_PIN 33
#define REL3_PIN 25
#define In1_PIN 34
#define In2_PIN 39
#define In3_PIN 36
#define checkval 1
#define Software_version "5.8"
#define Hardware_version "5.1"



PersistenceClass persistence;

hw_timer_t *timer = NULL;

string tempString;
string ATbuffer;
string tempString1;
string SMbuffer;
string tempString3;
string subscriberNumber;
string adminNumber;
string userNumber;
string destinationNumber;
string tempBuffer;
string commandString;
string Msg;
string executeResult;
string dateString;
string timeString;
string moduleId;
string moduleName;
string hardwareVersion;
string softwareVersion;
string smsIndex;
string webUrl;
uint8_t smsIndex2;
string localSenderNumber;
string localTempString;
string stringArray[10];
uint8_t arrayCount;
string sensorData;
string temperatureString = "00.0";
string humidString = "00.0";
string voltageString = "0000";
string responseString = "";
string atCmdString = "";
uint8_t CRC;
uint8_t parity;
uint16_t hum;
int32_t humValue;
int32_t temperatureValue;
uint16_t hValue;
uint16_t lValue;
uint8_t sensorSourceFlag = 0;
uint16_t wordTempValue;
int32_t gradeCelsius;
bool cmdSource;
bool bootFlag = false;
uint8_t soundFlag = 0;
uint8_t inStatus;
uint8_t outStatus;
uint8_t tempInStatus;
uint8_t ioSetFlag;

uint8_t gsmReady;
uint8_t stringCheckResult = 0;
uint8_t resultNumber;

uint8_t K;
uint8_t I;
uint8_t J;

uint8_t protectFlag;
uint8_t adminFlag = 0;
uint8_t signalLevel;
uint8_t leftCharPos;
uint8_t rightCharPos;

uint8_t senderCheckResult;
uint8_t echoFlag;
uint8_t userIndex;
uint8_t tempValue;
uint8_t tempValue2;
uint8_t stringLength;

uint16_t restartTime;
uint16_t loopCounter;
uint16_t waitValue;


uint16_t epromIndex;
uint16_t recCounter;
uint8_t chainFlag;






uint16_t delayValue;
uint16_t logSavePeriod = 0;
uint16_t logSaveDelay = 0;
uint16_t rel1Delay = 0;
uint16_t rel2Delay = 0;
uint16_t rel3Delay = 0;
uint8_t busyFlag = 0;

uint8_t temperatureReadInterval = 0;
uint8_t gsmCheckInterval = 0;
uint8_t ioCheckInterval = 0;
uint8_t updateTimeInterval = 0;

uint8_t flagByte = 0;
uint8_t treshCounter = 0;
bool alertSendFlag = false;

uint16_t resetPeriod;
uint16_t resetInterval = 0;
uint8_t resetFlag;

char tmp;
char tmp2;
uint8_t tmp3;
string date;
string timE;
vector<string> usersSet;


uint8_t debugFlag = 1;

void debugFunc(string toPrint){
  if(debugFlag == 1){
    SerialMon.println(toPrint.c_str());
  }
}

void debugFunc(char toPrint){
  if(debugFlag == 1){
    SerialAT.write(toPrint);
    SerialAT.write(char(13));
    SerialAT.write(char(10));
  }
}

void debugFunc(char toPrint[]){
  if(debugFlag == 1){
    SerialAT.write(toPrint);
    SerialAT.write(char(13));
    SerialAT.write(char(10));
  }
}


void onreceivefunc()
{  
  tmp = (char)SerialAT.read();
  tempString1 += tmp;
  if(tmp==13){
    ATbuffer += tempString1;
    tempString1.clear();
  }
  recCounter++;
  //gsmCheckInterval = 0;
}

void onreceiveMonitor(){
  K = SerialMon.read();
  if(K != 0){
    tempString3 += char(K);
    if(K==9){
      SerialMon.print("bm.cmd:");
      tempString3.assign("bm.cmd",6);
    }
    if((echoFlag == 1) && (K != 9)) SerialMon.print(K);
    if(K==13){
        SMbuffer += tempString3;
        if(echoFlag == 1) SerialMon.print(13);
        tempString3.clear();
    }
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    K=0;

  }
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
}

void IRAM_ATTR onTimer()
{
  resetInterval++;
  checkRelays();
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  gsmCheckInterval++;
  temperatureReadInterval++;
  ioCheckInterval++;
  updateTimeInterval++;

}


void setup() {
  
  

  moduleName.assign("ABM Module",10);
  moduleId.assign("BM990804181048",14);
  subscriberNumber.assign("+989123654705",13);
  hardwareVersion.assign("5.1",3);
  softwareVersion.assign("5.0",3);
  commandString.assign("BM.CMD:Empty;",13);

  //initialize Gsm and serial monitor port
  SerialMon.begin(9600);
  SerialMon.onReceive(onreceiveMonitor);
  SerialAT.onReceive(onreceivefunc);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);

  //initialize eeprom handler
  

  //initialize timer
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, INTERVAL * 1000, true);
  timerAlarmEnable(timer);

  //CONFIG PINMODES
  pinMode(In1_PIN, INPUT);
  pinMode(REL1_PIN, OUTPUT);
  pinMode(In2_PIN, INPUT);
  pinMode(REL2_PIN, OUTPUT);
  pinMode(In3_PIN, INPUT);
  pinMode(REL3_PIN, OUTPUT);
  pinMode(ACT_PIN, OUTPUT);
  pinMode(Power_Key, OUTPUT);

  persistence.init();
  initModule();
  initGsm();
  loopCounter =0;
  bootFlag = 0;
  delay(100);

}

void loop() {


  


  if(ATbuffer != ""){
    debugFunc(ATbuffer.c_str());
    stringCheckResult =ATbuffer.find("MTI:");
    debugFunc(stringCheckResult);
    if(stringCheckResult != 255){
      SerialMon.print("one message received");
      debugFunc(ATbuffer.c_str());
      leftCharPos = ATbuffer.find(",");
      leftCharPos++;
      smsIndex = ATbuffer.substr(leftCharPos,2);
      debugFunc(smsIndex.c_str());
      delay(20);
      if(smsIndex != ""){
        smsIndex2 = static_cast<uint8_t>(atoi(smsIndex.c_str()));
        debugFunc(smsIndex2);
        if(smsIndex2 > 0) readSms();
      }
      debugFunc("sms have been read!");
    }

    stringCheckResult = ATbuffer.find("RING");
    if(stringCheckResult != 255){
      atCMD("AT+CLCC");
      
      delay(500);
    }

    stringCheckResult = ATbuffer.find("+CLCC:");
    if(stringCheckResult != 255){
      leftCharPos = ATbuffer.find("+CLCC:");
      leftCharPos += 8;
      localSenderNumber = ATbuffer.substr(leftCharPos - 1, 13);
      if(protectFlag == 1){
        senderCheckResult = checkSender(localSenderNumber);
        SerialMon.print("Caller ID Is :");
        SerialMon.print(localSenderNumber.c_str());
        SerialMon.print("---");
        SerialMon.print(senderCheckResult);
      }else{
        senderCheckResult = 100;
      }
      if(senderCheckResult = 100){
        atCMD("ATA");
        delay(200);
        senderCheckResult = 0;
        Buzz(700,900);
        ATbuffer = "";
      }else{
        atCMD("ATH");
        delay(200);
      }
    }

    stringCheckResult = ATbuffer.find("+DTMF:");
    if(stringCheckResult != 255){
      tempBuffer = ATbuffer.substr(1,8);
      commandString = "";
      Buzz(400,800);
      SerialMon.print("tempbuffer is :  ");
      SerialMon.print(tempBuffer.c_str());
      if(tempBuffer == "+DTMF: 1"){
        commandString = "rel1=on;";
      }else if(tempBuffer == "+DTMF: 2"){
        commandString = "rel1=off;";
      }else if(tempBuffer == "+DTMF: 3"){
        commandString = "rel2=on;";
      }else if(tempBuffer == "+DTMF: 4"){
        commandString = "rel2=off;";
      }else if(tempBuffer == "+DTMF: 5"){
        commandString = "rel3=on;";
      }else if(tempBuffer == "+DTMF: 6"){
        commandString = "rel3=off;";
      }else if(tempBuffer == "+DTMF: 7"){
        commandString = "rel1=reset;";
      }else if(tempBuffer == "+DTMF: 8"){
        commandString = "rel2=reset;";
      }else if(tempBuffer == "+DTMF: 9"){
        commandString = "rel3=reset;";
      }else{
        debugFunc("Not Detected");
        tempBuffer = "";
      }
      if(commandString != ""){
        SerialMon.print("DTMF code :");
        SerialMon.print(tempBuffer.c_str());
        SerialMon.print("bm.cmd:");
        debugFunc(commandString.c_str());

        resultNumber = executeCommand(commandString);
      }
    }

    gsmCheckInterval = 0;
    ATbuffer = "";

  }


  if(SMbuffer != ""){
    if(echoFlag == 0) SerialMon.print(SMbuffer.c_str());
    tempBuffer = SMbuffer;
    cmdSource = 0;
    splitCommands();
    Buzz(200,750);
    cmdSource = 0;
    SMbuffer = "";
  }


  if(resetInterval > resetPeriod && resetFlag == 1){
    Buzz(2000, 1000);
    ESP.restart();
  }

  

  if(gsmCheckInterval >= 300 && busyFlag == 0) checkGsm();
  if(ioCheckInterval >= 3 && busyFlag == 0){ 
    //debugFunc("check io");
    Check_io();
    
  }
  if(updateTimeInterval >= 1500 && busyFlag == 0){
    checkSms();
    getDateTime();
    updateTimeInterval = 0;
  }

}



void append(char* s, char c) {
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}



void checkRelays(){


  if(rel1Delay > 0){
    rel1Delay--;
    if(rel1Delay==0) digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));
  }

  if(rel2Delay > 0){
    rel2Delay--;
    if(rel2Delay==0) digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));
  }

  if(rel3Delay > 0){
    rel3Delay--;
    if(rel3Delay==0) digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));
  }

}



void Buzz(uint16_t puls, uint16_t period){

  if(soundFlag==1) tone(BUZZER_PIN,puls,period);

}


void initModule(){

  debugFunc("Initializing Module ...");
  if(persistence.checkExistence("resetflag") && (resetFlag <= 1)){
    resetFlag = static_cast<uint8_t>(atoi(persistence.getValue("resetflag")->c_str()));  
  }else{
      resetFlag = 0;
      persistence.put("resetflag","0",true);
  }
  if(persistence.checkExistence("resetperiod")){
    resetPeriod = static_cast<uint16_t>(atoi(persistence.getValue("resetperiod")->c_str()));  
  }
  if((resetPeriod > 65000) || (resetPeriod < 10)){
      resetPeriod = 600;
      persistence.put("resetperiod","600",true);
  }
  if(persistence.checkExistence("restarttime")){
    restartTime = static_cast<uint16_t>(atoi(persistence.getValue("restarttime")->c_str()));
  }
  if(restartTime > 50000){
    persistence.put("restarttime","1",true);
  }else{
    restartTime++;
    persistence.put("restarttime",to_string(restartTime),true);
  }
  
  debugFunc("Checking inputs Status ...");
  Buzz(400,750);

  if(persistence.checkExistence("instatus")){
    inStatus = static_cast<uint8_t>(atoi(persistence.getValue("instatus")->c_str()));
  }
  if(inStatus == 255){
    inStatus = 0;
    persistence.put("instatus","0",true);
  }

  debugFunc("Checking Outputs Status ...");
  if(persistence.checkExistence("outstatus")){
    outStatus = static_cast<uint8_t>(atoi(persistence.getValue("outstatus")->c_str()));
  }
  if(outStatus == 255){
    outStatus = 0;
    persistence.put("outstatus","0",true);
  }
  outStatus = outStatus & ~((uint8_t)1 << 0);
  if((outStatus >> 1) & (uint8_t)1){
    digitalWrite(REL1_PIN, HIGH);
  }else{
    digitalWrite(REL1_PIN, LOW);
  }
  if((outStatus >> 2) & (uint8_t)1){
    digitalWrite(REL2_PIN, HIGH);
  }else{
    digitalWrite(REL2_PIN, LOW);
  }
  if((outStatus >> 3) & (uint8_t)1){
    digitalWrite(REL3_PIN, HIGH);
  }else{
    digitalWrite(REL3_PIN, LOW);
  }

  if(persistence.checkExistence("sensorsourceflag")){
    sensorSourceFlag = static_cast<uint8_t>(atoi(persistence.getValue("sensorsourceflag")->c_str()));  
  }
  if(sensorSourceFlag != 1){
    sensorSourceFlag  =0;
    persistence.put("sensorsourceflag","0",true);
  }

  if(persistence.checkExistence("debugflag")){
    debugFlag = static_cast<uint8_t>(atoi(persistence.getValue("debugflag")->c_str()));  
  }
  if(debugFlag != 1){
    debugFlag  =0;
    persistence.put("debugflag","0",true);
  }


  if(persistence.checkExistence("soundflag")){
    soundFlag = static_cast<uint8_t>(atoi(persistence.getValue("soundflag")->c_str()));
  }
  if(soundFlag > 0){
    soundFlag = 1;
    persistence.put("soundflag","1",true);
  }
  else{
    persistence.put("soundflag","0",true);
  }
  if(persistence.checkExistence("protectflag")){
    protectFlag = static_cast<uint8_t>(atoi(persistence.getValue("protectflag")->c_str()));
  }
  if(protectFlag > 1){
    protectFlag  = 0;
    persistence.put("protectflag","0",true);
  }
  if(protectFlag == 0){
    persistence.put("protectflag","0",true);
  }
  if(persistence.checkExistence("delayvalue")){
    delayValue = static_cast<uint16_t>(atoi(persistence.getValue("delayvalue")->c_str()));
  }else{
    delayValue = 10;
    persistence.put("delayvalue","10",true);
  }
  if(delayValue > 65000){
    delayValue = 10;
    persistence.put("delayvalue","10",true);
  }
  if(persistence.checkExistence("modulename")){
    moduleName = persistence.getValue("modulename")->c_str();
  }
  if(restartTime < 2 || (!persistence.checkExistence("modulename"))){
    moduleName = "ABM Module";
    persistence.put("modulename",moduleName,true);
  }
  if(persistence.checkExistence("adminnumber")){
    adminNumber = persistence.getValue("adminnumber")->c_str();
  }else{
    adminNumber = "";
  }
  stringLength = adminNumber.length();
  if((adminNumber == "")||(stringLength != 10)){
    protectFlag = 0;
    stringLength = 0;
  }
  if(persistence.checkExistence("chainflag")){
    chainFlag = static_cast<uint8_t>(atoi(persistence.getValue("chainflag")->c_str()));
  }
  if((chainFlag > 2) || (chainFlag == 0)){
    chainFlag = 0;
    persistence.put("chainflag","0",true);
  }

  int i = 1;
  while (persistence.checkExistence(string("user") + to_string(i)))
  {
    string temp = persistence.getValue(string("user") +to_string(i))->c_str();
    insertUsersSet(temp);
    debugFunc(temp.c_str());
    i++;
  }

  userIndex = usersSet.size();
  if(userIndex >= 100){
    userIndex =1;
  }
  if(persistence.checkExistence("echoflag")){
    echoFlag = static_cast<uint8_t>(atoi(persistence.getValue("echoflag")->c_str()));
  }else{
    echoFlag  = 1;
    persistence.put("echoflag","1",true);
  }

  if(echoFlag > 1){
    echoFlag = 1;
    persistence.put("echoflag","1",true);
  }
  if(persistence.checkExistence("iosetflag")){
    ioSetFlag = static_cast<uint8_t>(atoi(persistence.getValue("iosetflag")->c_str()));
  }else{
    ioSetFlag  = 10;
    persistence.put("iosetflag","10",true);
  }
  if(ioSetFlag > 60){
    ioSetFlag  = 10;
    persistence.put("iosetflag","10",true);
  }

  digitalWrite(ACT_PIN, HIGH);
  SerialMon.print("Module ");
  SerialMon.print(moduleName.c_str());
  SerialMon.print(" Initialized!");
  Buzz(400, 950);
}



uint8_t checkSender(string senderNumber){
  tmp3 = 0;
  debugFunc(userIndex);
  if(userIndex > 0){
    tempValue = 0;

    userIndex = usersSet.size();
    tempValue2 = userIndex ;
    for(tempValue = 0;tempValue < tempValue2;tempValue++){
      userNumber = usersSet[tempValue];
      stringLength = userNumber.length();
      if(stringLength > 10){
        userNumber = userNumber.substr(0,10);
      }

      if(userNumber != ""){
          if(senderNumber == userNumber){
            adminFlag = 0;
            tmp3 = 100;
            break;
          }
      }
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN)); 
    }


  }

  stringLength = senderNumber.length();
  if(stringLength > 10){
    senderNumber = senderNumber.substr(stringLength - 10,10);
  }
  if(persistence.checkExistence("adminnumber")){
    adminNumber = persistence.getValue("adminnumber")->c_str();
  }
  if(adminNumber == senderNumber){
    tmp3 = 100;
    adminFlag = 100;
  }else{
    subscriberNumber = adminNumber;
    if((subscriberNumber == senderNumber)||(senderNumber == "9123654705")){
      tmp3 = 100;
      adminFlag = 100;
    }

  }

  return tmp3;

}

string retriveString(string commandString,uint8_t stringLength){

  leftCharPos = commandString.find("=");
  rightCharPos = commandString.find(";");
  if((leftCharPos > 0) && (rightCharPos > 0)){
    rightCharPos = rightCharPos - leftCharPos;
    rightCharPos--;
    return commandString.substr(stringLength,rightCharPos);
  }
  else{
    return "";
  }
}



string retriveGsmString(string atCommandString, uint8_t startAddress, uint8_t endAddress){

  ATbuffer = "";
  atCMD(atCommandString.c_str());
  delay(1000);
  debugFunc(ATbuffer.c_str());
  if(ATbuffer != ""){
    trim(ATbuffer);
    debugFunc(ATbuffer.c_str());
    debugFunc(ATbuffer.length());
    int t1 = ATbuffer.find("+");
    if(t1 != 255){
      startAddress += t1;
    }
    int q = ATbuffer.find("+CCLK");
    if(q == 255){
      return "";
    }
    SerialMon.println("line 710");
    string t = ATbuffer.substr(startAddress,endAddress+4);
    SerialMon.println("line 712");
    debugFunc(t.c_str());
    ATbuffer = "";
    return t;
  }
  return "";
}


void getDateTime(){
  if(!bootFlag){
    debugFunc("Initiliazing Date and Time ...");
  }
  if(gsmReady == 10){
    tempString = retriveGsmString("AT+CCLK?", 8, 18);
    SerialMon.println("after the retrive gsm");
    debugFunc(tempString.c_str());
    if(tempString != ""){
      dateString = tempString.substr(0,8);
      date = dateString;
      timeString = tempString.substr(9,8);
      timE = timeString;
      if(!bootFlag){
        SerialMon.print("Date and Time initialized: ");
        SerialMon.print(dateString.c_str());
        SerialMon.print("    ");
        debugFunc(timeString.c_str()); 
      }
    Buzz(600, 950);
    }
  }
}


void getGsmImei(){
  moduleId = retriveGsmString("AT+GSN", 2, 15);
  if(moduleId != ""){
    SerialMon.print("Module ID/IMEI: ");
    SerialMon.print(moduleId.c_str());
    debugFunc(" ");
    Buzz(600,950);
  }
}

//+CSQ: 22,99
void getSignalLevel(){
  smsIndex = retriveGsmString("AT+CSQ",6,2);
  if(smsIndex != ""){
    signalLevel = static_cast<uint8_t>(atoi(smsIndex.c_str()));
  }
}


void trim(string& str){
  while(str[0] == ' ') str.erase(str.begin());
  while(str[str.size() - 1] == ' ') str.pop_back();
}


void sendSms(){

  if(cmdSource && (gsmReady == 10)){

    atCMD("AT");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(1000);

    atCMD("AT+CMGF=1");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(1000);
    subscriberNumber = string("+98") + subscriberNumber;
    /*tmp2 = 34;
    atCMD("AT+CMGS=");
    atCMD(tmp2);
    atCMD(subscriberNumber.c_str());
    atCMD(tmp2);
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    atCMD(Msg.c_str());
    tmp2 = 26;
    atCMD(tmp2);
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(3000);*/

    SerialAT.println((string("AT+CMGS=\"") + subscriberNumber +string("\"")).c_str());
    SerialAT.print(Msg.c_str());
    SerialAT.write(26);



    tmp2 = 0;
    if(restartTime > 2){
      SerialMon.print("Message : ");
      SerialMon.print(Msg.c_str());
      SerialMon.print("  was sent to :");
      debugFunc(subscriberNumber.c_str());
    }
    atCMD("AT");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(1000);
    Buzz(900,860);
    if(subscriberNumber.length() > 10){
      subscriberNumber = subscriberNumber.substr(subscriberNumber.length()-10);
    }
    
  }else{
    SerialMon.print(Msg.c_str());
  }
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  Msg = "";

}


void insertUsersSet(string phoneNumber){
    usersSet.push_back(phoneNumber);
}



bool isUser(string phoneNumber) {
    for (int i = 0; i < usersSet.size(); i++) {
        if (usersSet[i] == phoneNumber){
            return true;
        }
    }
    return false; 
}


void initGsm(){

  SerialMon.print("Initializing Gsm ...");
  Buzz(500,980);
  gsmReady = 0;
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  ATbuffer = "";
  atCMD("AT");
  delay(1000);
  if(ATbuffer == ""){
    digitalWrite(Power_Key,LOW);
    delay(1500);
    digitalWrite(Power_Key,HIGH);
    Buzz(500,980);
    delay(5500);
  }
  ATbuffer = "";
  atCMD("AT");
  delay(500);
  loopCounter = 0;
  while(1){
    stringCheckResult = ATbuffer.find("OK");
    if(stringCheckResult != -1){
      stringCheckResult = 0;
      ATbuffer = "";
      gsmReady = 10;
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
      break;
    }else{
      digitalWrite(Power_Key,LOW);
      delay(1000);
      digitalWrite(Power_Key,HIGH);
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
      delay(1500);
      atCMD("AT");
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
      delay(500);
      loopCounter++;
    }
    if(loopCounter > 15){
      ESP.restart();
    }


  }
  atCMD("ATE0");
  delay(50);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  loopCounter = 0;
  ATbuffer = "";
  delay(50);
  atCMD("AT+COPS?");
  delay(1500);
  while(1){
    if(ATbuffer != ""){
      SerialMon.print("Searching Network Operators... ");
      debugFunc(ATbuffer.c_str());
      stringCheckResult = ATbuffer.find("OK");
      if(stringCheckResult != -1){
        stringCheckResult = 0;
        ATbuffer = "";
        gsmReady = 10;
        break;
      }
    }else{
      atCMD("AT+COPS?");
      delay(100);
    }
    if(loopCounter > 25){
      SerialMon.print("Error: Network Operator Not Found or SIM Module Not Responde!");
      gsmReady = 0;
      break;
    }
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    loopCounter++;
  }

  ATbuffer = "";
  if(gsmReady == 10){
    atCMD("AT+CSPN?");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(3000);
    SerialMon.print("Registered On: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+GSV");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    SerialMon.print("GSM Module:");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    ATbuffer = "";

    atCMD("AT+CMGF=1");
    atCMD(char(13));
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Config SMS Text Mode: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CMGDA=\"DEL ALL\"");
    /*atCMD(char(34));
    atCMD("DEL ALL");
    atCMD(char(34));*/
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    SerialMon.print("Delete Old SMS's: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CNMI=2,1,2,0,0");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Config New SMS indication: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CPMS=\"ME\",\"ME\",\"ME\"");
    /*atCMD(char(34));
    atCMD("ME");
    atCMD(char(34));
    atCMD(",");
    atCMD(char(34));
    atCMD("ME");
    atCMD(char(34));
    atCMD(",");
    atCMD(char(34));
    atCMD("ME");
    atCMD(char(34));*/
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    SerialMon.print("Config SMS message storage: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";
    

    atCMD("AT+CSMP=17,167,0,0");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    SerialMon.print("irancell config: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CMEE=1" );
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(500);
    SerialMon.print("Config Error Report Mode: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CMTE=1");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Enable Internal Temp Sensor:");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+CLTS=1");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(600);
    SerialMon.print("Enable Network Clock: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT+DDET=1,0,0,0");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Enable DTMF Detecting: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";


    atCMD("AT+CLIP=1");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Enable Caller ID Detecting: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";

    atCMD("AT&W");
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    delay(200);
    SerialMon.print("Save Config: ");
    debugFunc(ATbuffer.c_str());
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    ATbuffer = "";
  }
  if(gsmReady == 10){
    Serial.println("before data and time");
    getDateTime();
    Serial.println("after data and time");    
    getGsmImei();
    Buzz(500,700);
    delay(100);
    Buzz(500,800);
    delay(100);
    Buzz(500,900);
    getSignalLevel();
    if(bootFlag == 0 && restartTime <3){
      cmdSource = 1;
      Msg = "ABM Mini Module: " + moduleId + " Started Up With Sig.Level: " + to_string(signalLevel) + "!";
      sendSms();
      cmdSource = 0;
    }
    debugFunc("GSM Initialized!");

  }else{
    if(gsmReady == 0){
      SerialMon.print("GSM Failed! ");
      debugFunc(ATbuffer.c_str());
      Buzz(1500,980);
    }
  }

  ATbuffer = "";


}


void readSms(){
  ATbuffer = "";
  senderCheckResult = 0;
  recCounter = 0;
  atCMD(string("AT+CMGR=") + to_string(smsIndex2));
  //atCMD(to_string(smsIndex2));
  delay(1);
  waitValue = 0;
  while(1){
    if(waitValue > 250) break;
    waitValue++;
    delay(21);
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));

  }
  SerialMon.print( "Recived Chars: ");
  SerialMon.print(recCounter);
  SerialMon.print(" String: ");
  debugFunc(ATbuffer.c_str());

  recCounter = 0;
  stringCheckResult = ATbuffer.find("UNREAD");

  if(stringCheckResult != 255){
    localSenderNumber = ATbuffer.substr(27, 10);
    dateString = ATbuffer.substr(43, 21);
    dateString = string("\"") + dateString;
    




    timeString = ATbuffer.substr(51, 13);
    ///////////////////////////////////////////////////////////
    cmdSource = 1;

    debugFunc(localSenderNumber.c_str());
    debugFunc("this are data and time string :");
    debugFunc(dateString.c_str());
    debugFunc(timeString.c_str());

    if(protectFlag == 1){
      localTempString = localSenderNumber;
      senderCheckResult = checkSender(localTempString);
    }else{
      senderCheckResult = 100;
    }
    debugFunc(senderCheckResult);
    if(senderCheckResult == 100){
      subscriberNumber = localSenderNumber;
      ///////////////////////////////////
      delay(10);
      tempBuffer = ATbuffer;
      splitCommands();
      debugFunc(Msg.c_str());
      if(echoFlag == 1 && Msg != ""){
        sendSms();
      }
      delay(1000);
    }else{
      Msg = "You are unauthorized! Please contact: www.abm.co.ir";
      subscriberNumber = localSenderNumber;
      sendSms();
      delay(1000);
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    }

  }

  atCMD(string("AT+CCLK=") + dateString);
  //atCMD("AT+CMGD=");
  //atCMD(to_string(smsIndex2));
  atCMD("AT+CMGDA=\"DEL READ\"");
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));


  delay(1000);
  cmdSource = 0;


}


void getHelp(){
  Msg = "bm.cmd:{sound=on/off;echo=on/off;sensor=in/out;ioset=noact/report/direct/invers/reset;relX=on/off;relX=reset;}";
  sendSms();
  Msg = "bm.cmd:{read=alert/all/io/info;showusers;deluser=x/all;unprotect;protect;resetme;resetgsm;callme;admin=9xxxxxxxxx;adduser=9xxxxxxxxx;setname=xxx;}";
  sendSms();
  Msg = "bm.cmd:{dest=9xxxxxxxxx;msg=xxx;selfreset=on/off;chainmode=off/on/booth;setdelay=[2~65000s];rstperiod=[10~65000m];}";
  sendSms();
}


void getVoltage(){
  voltageString = retriveGsmString("AT+CBC", 11,4);
}

void applyATCmd(){
  if(gsmReady == 10){
    if(ATbuffer != "" && echoFlag==1){
      debugFunc(ATbuffer.c_str());
    }
    delay(100);
    ATbuffer = "";
    atCMD(atCmdString.c_str());
    delay(100);
    if(echoFlag == 1){
      debugFunc(atCmdString.c_str());
    }
    delay(1600);
    while(1){
        if(ATbuffer != ""){
          stringCheckResult = ATbuffer.find("OK");
          if(stringCheckResult != -1){
            responseString = "OK";
            break;
          }else{
            responseString = "ERROR";
            break;
          }
        }
    }
    if(ATbuffer != "" && echoFlag == 1){
      debugFunc(ATbuffer.c_str());
    }
  }
}


void checkSms(){

  atCMD("+CMGL=\"REC UNREAD\"");
  /*atCMD("AT+CMGL=");
  atCMD(char(34));
  atCMD("REC UNREAD");
  atCMD(char(34));
  atCMD(",1");*/
  delay(200);
  debugFunc(ATbuffer.c_str());
  if(ATbuffer != ""){
    smsIndex = char(13);
    arrayCount = split(ATbuffer, stringArray,0,char(13),10);
    for(I=1;I<=arrayCount;I++){
      stringCheckResult = stringArray[I].find("+CMGL:");
      if(stringCheckResult != -1){
        leftCharPos = stringArray[I].find(" ");
        rightCharPos = stringArray[I].find(",");
        if((leftCharPos != -1)&&(rightCharPos != -1)){
          leftCharPos++;
          rightCharPos = rightCharPos - leftCharPos;
          smsIndex = stringArray[I].substr(leftCharPos-1,rightCharPos);
          smsIndex2 = static_cast<uint8_t>(atoi(smsIndex.c_str()));
          if(smsIndex2 > 0) readSms();
        }
      }
    }

  }
  ATbuffer = "";
}

void checkGsm(){
  //Buzz(500,500);
  ATbuffer = "";
  atCMD("AT+CREG?");
  atCMD(char(13));
  atCMD(char(10));
  delay(1000);
  SerialMon.print("Check Network Operator... ");
  debugFunc(ATbuffer.c_str());
  if(ATbuffer != ""){
    
    stringCheckResult = ATbuffer.find("1");
    debugFunc("check result is:");
    debugFunc(stringCheckResult);
    if(stringCheckResult != 255){
      ATbuffer = "";
      gsmReady = 10;
      J = 0;
      Buzz(300,750);
    }else{
      debugFunc("gsm interval is");
      debugFunc(gsmCheckInterval);  
    if(gsmCheckInterval >= 30){
      gsmCheckInterval = 0;
      SerialMon.print("Re-Init->GSM ");
      debugFunc(ATbuffer.c_str());    
      initGsm();
    }
    }
  }else{
    if(gsmCheckInterval >= 40){
      gsmCheckInterval = 0;
      SerialMon.print("Re-Init->GSM ");
      debugFunc(ATbuffer.c_str());    
      initGsm();
    } 
  }
  gsmCheckInterval = 0;
}


void makeCall(){
  if(gsmReady == 10){
    subscriberNumber = string("+98") + subscriberNumber;
    debugFunc(subscriberNumber.c_str());

    SerialAT.write("ATD");
    SerialAT.write(subscriberNumber.c_str());
    atCMD(";");
    delay(200);
    if(subscriberNumber == "+989123654705"){
      debugFunc("Dialing ...");
    }else{
      SerialMon.print("Dialing ");
      SerialMon.print(subscriberNumber.c_str());
      debugFunc("...");
    }
    for(J=1;J<61;J++){
      if(SMbuffer != "") break;
      if(ATbuffer != ""){
        debugFunc(ATbuffer.c_str());
        stringCheckResult = ATbuffer.find("BUSY");
        if(stringCheckResult != -1) break;
        ATbuffer = "";
      }
      delay(500);
    }
    Buzz(500,870);
    debugFunc("Calling Terminated!");
  }
}

void sendAlert(){
  userIndex = usersSet.size();
  writeIoStatus();
  if(0 <= userIndex){
    tempValue = 0;
    tempValue2 = userIndex-1;
    if(tempValue2 >= 6) tempValue2 = 6;
    for(tempValue =0; tempValue <= tempValue2;tempValue++){
        userNumber = usersSet[tempValue];
        stringLength = userNumber.length();
        if(10 < stringLength){
          userNumber =userNumber.substr(0,10);
        }
        if(userNumber != ""){
          subscriberNumber = userNumber;
          resultNumber = executeCommand(commandString);
        }  
    }
    if(protectFlag == 1){
      adminNumber = persistence.getValue("adminnumber")->c_str();
    }
    stringLength = adminNumber.length();
    debugFunc(adminNumber.c_str());

    if(stringLength > 10){
      adminNumber = adminNumber.substr(0,10);
      if(adminNumber != ""){
        subscriberNumber  =  adminNumber;
        makeCall();
      }
    }
  }else{
    debugFunc("Not any user found!");
  }
}

int split(const string& source, string fillArray[], int startIdx, char delimiter, int maxSize) {
    int count = 0;
    size_t startPos = 0;
    size_t delimiterPos;

    while ((delimiterPos = source.find(delimiter, startPos)) != std::string::npos) {
      
        if (count + startIdx < maxSize) {
            fillArray[startIdx + count] = source.substr(startPos, delimiterPos - startPos);
            debugFunc((source.substr(startPos, delimiterPos - startPos)).c_str());
            count++;
        } else {
            
            break;
        }
        startPos = delimiterPos + 1;
    }

    if (count + startIdx < maxSize) {
        fillArray[startIdx + count] = source.substr(startPos);
        count++;
    }

    return count;  
}

/*void getTemperature(){
  gradeCelsius = 0;
  humValue = 0;
  if(sensorSourceFlag == 0){
    temperatureValue = 0;
  }else{
    waitValue = 0;
    sensorData = "";

  }
}*/

void writeIoStatus(){

  tempInStatus = 0;
  uint8_t In1 = digitalRead(In1_PIN) == HIGH ? 1 : 0;
  uint8_t In2 = digitalRead(In2_PIN) == HIGH ? 1 : 0;
  uint8_t In3 = digitalRead(In3_PIN) == HIGH ? 1 : 0;
  tempInStatus |= (In1 << 1);
  tempInStatus |= (In2 << 2);
  tempInStatus |= (In3 << 3);
  persistence.put("instatus",to_string(tempInStatus),true);


  outStatus = 0;
  uint8_t Rel1 = digitalRead(REL1_PIN) == HIGH ? 1 : 0;
  uint8_t Rel2 = digitalRead(REL2_PIN) == HIGH ? 1 : 0;
  uint8_t Rel3 = digitalRead(REL3_PIN) == HIGH ? 1 : 0;
  outStatus |= (Rel1 << 1);
  outStatus |= (Rel2 << 2);
  outStatus |= (Rel3 << 3);
  persistence.put("outstatus",to_string(outStatus),true);

  
}


uint8_t executeCommand(string cmdString){
  uint8_t  eC = 2;
  executeResult = "OK !";

  if(cmdString == "selfreset=on;"){
    resetFlag = 1;
    persistence.put("resetflag","1",true);
  }else if(cmdString == "selfreset=off;"){
    resetFlag = 0;
    persistence.put("resetflag","0",true);
  }else if(cmdString == "sensor=in;"){
      sensorSourceFlag = 0;
      persistence.put("sensorsourceflag","0",true);
  }else if(cmdString == "sensor=out;"){
      sensorSourceFlag = 1;
      persistence.put("sensorsourceflag","1",true);
  }else if(cmdString == "chainmode=off;"){
    chainFlag = 0;
    persistence.put("chainflag","0",true);
  }else if(cmdString == "chainmode=on;"){
    chainFlag = 1;
    persistence.put("chainflag","1",true);
  }else if(cmdString == "chainmode=booth;"){
    chainFlag = 2;
    persistence.put("chainflag","2",true);
  }else if(cmdString == "rstcounter;"){
    restartTime = 0;
    persistence.put("restarttime","0",true);
  }else if(cmdString == "sound=on;"){
    soundFlag =1;
    persistence.put("soundflag","1",true);
  }else if(cmdString == "sound=off;"){
    soundFlag =0;
    persistence.put("soundflag","0",true);
  }else if(cmdString == "echo=on;"){
    echoFlag =1;
    persistence.put("echoflag","1",true);
  }else if(cmdString == "echo=off;"){
    echoFlag =0;
    persistence.put("echoflag","0",true);
  }else if(cmdString == "ioset=noact;"){
    ioSetFlag = 10;
    persistence.put("iosetflag","10",true);
  }else if(cmdString == "ioset=report;"){
    ioSetFlag = 20;
    persistence.put("iosetflag","20",true);
  }else if(cmdString == "ioset=direct;"){
    ioSetFlag = 30;
    persistence.put("iosetflag","30",true);
  }else if(cmdString == "ioset=invers;"){
    ioSetFlag = 40;
    persistence.put("iosetflag","40",true);
  }else if(cmdString == "ioset=reset;"){
    ioSetFlag = 50;
    persistence.put("iosetflag","50",true);
  }else if(cmdString == "rel1=on;"){
    digitalWrite(REL1_PIN, HIGH);
    writeIoStatus();
  }else if(cmdString == "rel1=off;"){
    digitalWrite(REL1_PIN, LOW);
    writeIoStatus();
  }else if(cmdString == "rel2=on;"){
    digitalWrite(REL2_PIN, HIGH);
    writeIoStatus();
  }else if(cmdString == "rel2=off;"){
    digitalWrite(REL2_PIN, LOW);
    writeIoStatus();
  }else if(cmdString == "rel3=on;"){
    digitalWrite(REL3_PIN, HIGH);
    writeIoStatus();
  }else if(cmdString == "rel3=off;"){
    digitalWrite(REL3_PIN, LOW);
    writeIoStatus();
  }else if(cmdString == "rel1=reset;"){
    digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));
    rel1Delay = delayValue;
  }else if(cmdString == "rel2=reset;"){
    digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));
    rel2Delay = delayValue;
  }else if(cmdString == "rel3=reset;"){
    digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));
    rel3Delay = delayValue;
  }else if(cmdString == "unprotect;"){
    if(adminFlag == 100 || cmdSource == 0){
      protectFlag = 0;
      persistence.put("protectflag","0",true);
    }else{
      protectFlag = 1;
      persistence.put("protectflag","1",true);
      executeResult = "Error: Access Denied2!";
    }
  }else if(cmdString == "protect;"){
    if(adminFlag == 100 || cmdSource == 0){
      protectFlag = 1;
      persistence.put("protectflag","1",true);
    }else{
      executeResult = "Error: Access Denied3!";
    }
  }else if (cmdString == "debug=on;"){
    debugFlag = 1;
    persistence.put("debugflag","1",true);
  }else if(cmdString == "debug=off;"){
    debugFlag = 0;
    persistence.put("debugflag","0",true);
  }else if (cmdString == "resetme;"){
    Buzz(2000, 1000);
    ESP.restart();
  }else if(cmdString == "resetgsm;"){
    Buzz(2000, 1000);
    initGsm();
  }else if(cmdString == "callme;"){
    makeCall();
  }else if(cmdString == "help;"){
    getHelp();
    executeResult = "";
  }else if(cmdString == "read=alert;"){
    //
    Msg = string("Alert(20")+ date + string("-") + timE  + string(") Inputs:") + toBinary(tempInStatus) + string(" Outputs:") + toBinary(outStatus);
    if(Msg != "") sendSms();
    executeResult = "";
  }else if(cmdString == "read=all;"){
    getSignalLevel();
    getVoltage();
    writeIoStatus();
    Msg = string("Inputs:") + toBinary(tempInStatus) + string(" Outputs:") + toBinary(outStatus) + string(" Sig.Level:") + to_string(signalLevel) +
            string(" IO.Flag:") + to_string(ioSetFlag) + string(" Sound:") + to_string(soundFlag) +
            string(" Echo:") + to_string(echoFlag) + string(" Delay:") + to_string(delayValue) + " VCC:" + voltageString +"mV";
    if(Msg != "") sendSms();
    executeResult = "";
  }else if(cmdString == "read=io;"){
    writeIoStatus();
    Msg = string("Inputs:") + toBinary(tempInStatus) + string(" Outputs:") + toBinary(outStatus) + string(" Chain:") + to_string(chainFlag) + string(" Delay:") + to_string(delayValue);
    if(Msg != "") sendSms();
    executeResult = "";
  }else if(cmdString == "read=info;"){
    Msg = string("Module Name:") + moduleName + string(" Type:Mini ID:") + moduleId + string(" Hw.V:") + string(Hardware_version) + string(" Sw.V:") + string(Software_version) + string(" Protect:") + to_string(protectFlag) + string(" Rst.Count:") + to_string(restartTime) + string(" Self.Rst:") + to_string(resetFlag) + string(" Rst.Period:") + to_string(resetPeriod) + string("(") + to_string(resetInterval) + string(")");
    if(Msg != "") sendSms();
    executeResult = "";
  }else if(cmdString == "showusers;"){

    if(userIndex > 0){
      tempValue = 1;
      userIndex = usersSet.size();
      SerialMon.print("userindex is :");
      debugFunc(userIndex);
      if(persistence.checkExistence("adminnumber")){
        adminNumber = persistence.getValue("adminnumber")->c_str();
      }
      Msg = string("Admin:") + adminNumber + string("; Users=> ");
      for(tempValue =0;tempValue < userIndex;tempValue++){
        userNumber = usersSet[tempValue];
        debugFunc(userNumber.c_str());
        stringLength = userNumber.length();
        if(stringLength > 10) userNumber = userNumber.substr(0,10);
        if(userNumber != ""){
          Msg = Msg + to_string(tempValue + 1) + string(":") + userNumber + ";";
        }
        tempValue2 = (tempValue + 1) % 10;
        if(tempValue2 == 0){
          sendSms();
          Msg = "";
          ATbuffer = "";

        }
      }
      if(Msg != "") sendSms();
    }else{
      Msg = "Not any user found!";
      if(Msg != "") sendSms();
    }
    executeResult = "";

  }else{
    executeResult = string("Unknown Command: bm.cmd:") + cmdString;
    if((adminFlag == 100)||(protectFlag != 1)||(cmdSource == 0)){
      tempValue = cmdString.find("adduser=");
      if(tempValue != 255){
        userNumber = retriveString(cmdString, 8);
        debugFunc(userNumber.c_str());
        stringLength = userNumber.length();
        if(stringLength == 10){
          userIndex = usersSet.size();
          debugFunc(userIndex);
          delay(10);
          if(100 <= userIndex) userIndex = 0;
          if(userIndex ==0 && protectFlag != 1){
            adminNumber = userNumber;
            persistence.put("adminNumber",adminNumber,true);
            protectFlag = 1;
            persistence.put("protectflag","1",true);
          }
          userIndex++;
          insertUsersSet(userNumber);
          debugFunc(userIndex);
          debugFunc(userNumber.c_str());
          delay(10);
          userIndex = usersSet.size();
          delay(10);
          persistence.put(string("user") + to_string(usersSet.size()),userNumber,true);
          executeResult = string("User: ") + userNumber + string(" Saved at index: ") + to_string(userIndex);
          
        }else{
          executeResult = "Error: Incorrect Value!";
        }
      }
      tempValue = cmdString.find("deluser=");
      if(tempValue != 255){
        tempString = retriveString(cmdString, 8);
        debugFunc(tempString.c_str());
        userIndex = usersSet.size();
        if(tempString == "all" || userIndex <= 1){
          usersSet.clear();
          for (int i = 1; i <= usersSet.size(); ++i) {
            persistence.removeKey(string("user") + to_string(i),true);
          }
          protectFlag = 0;
          persistence.put("protectflag","0",true);
          executeResult = "All Users Deleted!";
        }else{
          tempValue2 =static_cast<uint8_t>(atoi(tempString.c_str()));
          if(tempValue2 > 0 && tempValue2 < 100){
            userNumber = usersSet[tempValue2-1];
            if (tempValue2 > 0 && tempValue2 <= usersSet.size()) {
                usersSet.erase(usersSet.begin() + (tempValue2 - 1));
                persistence.removeKey(string("user") + to_string(tempValue2),true);
            }
            for (int i = tempValue2 + 1; i <= usersSet.size() + 1; ++i) {
                persistence.removeKey(string("user") + to_string(i),true);
            }
            for (int i = tempValue2 - 1; i < usersSet.size(); ++i) {
                persistence.put(string("user") + to_string(i+1),usersSet[i],true);
            }

            executeResult = string("User: ") + userNumber + string(" at Index: ") + to_string(tempValue2) + string(" Deleted!");
          }else{
            executeResult =  "Error: Incorrect Value!";
          }
        }
      }
      tempValue = cmdString.find("setdelay=");
      if(tempValue != 255){
        tempString = retriveString(cmdString,9);
        debugFunc(tempString.c_str());
        delayValue = static_cast<uint16_t>(atoi(tempString.c_str()));
        if(2 <= delayValue){
          persistence.put("delayvalue",to_string(delayValue),true);
          executeResult = string("Value: ") + to_string(delayValue) + string(" Saved!");
        }else{
          executeResult = "Error: Incorrect Value!";
        }
      }
      tempValue = cmdString.find("rstperiod=");
      if(tempValue != 255){
        tempString = retriveString(cmdString,10);
        debugFunc(tempString.c_str());
        resetPeriod = static_cast<uint16_t>(atoi(tempString.c_str()));
        if(10 <= resetPeriod){
           persistence.put("resetperiod",to_string(resetPeriod),true);
           executeResult = string("Value: ") + to_string(resetPeriod) + string(" Saved!");
        }else{
          resetPeriod = static_cast<uint16_t>(atoi(persistence.getValue("resetperiod")->c_str()));  
          executeResult = "Error: Incorrect Value! ";
        }
      }
      tempValue = cmdString.find("setname=");
      if(tempValue != 255){
        moduleName = retriveString(cmdString, 8);
        persistence.put("modulename",moduleName,true);
        executeResult = string("Name: ") + moduleName + string(" Saved!");
      }
      tempValue = cmdString.find("admin=");
      if(tempValue != 255){
        adminNumber = retriveString(cmdString, 6);
        stringLength = adminNumber.length();
        if(stringLength == 10){
          persistence.put("adminnumber",adminNumber,true);
          protectFlag = 1;
          adminFlag = 100;
          persistence.put("protectflag","1",true);
          executeResult = string("Number: ") + adminNumber + string(" Saved As Admin");
        }else{
          protectFlag = 0;
          executeResult = "Error: Incorrect Value!";
        }
      }
      

    }else{
       executeResult = "Error: Access Denied4!";
    }
    Msg = "";
  }
  Msg = "";
  return eC;

}


void splitCommands(){
  toLowerCase(tempBuffer);
  trim(tempBuffer);
  debugFunc(tempBuffer.c_str());

  arrayCount = split(tempBuffer,stringArray,0,char(59),10);
  
  for(I=0;I<arrayCount;I++){
    
    tempBuffer = stringArray[I] + string(";");
    debugFunc(tempBuffer.c_str());
    stringCheckResult = tempBuffer.find("bm.");
    if(stringCheckResult != 255){
      
      leftCharPos = tempBuffer.find("bm.cmd:");
      rightCharPos = tempBuffer.find(";");
      if((leftCharPos != 255)&&(rightCharPos != 255)){
        rightCharPos = rightCharPos - leftCharPos;
        rightCharPos++;
        leftCharPos += 7;
        
        commandString = tempBuffer.substr(leftCharPos,rightCharPos);
        
        debugFunc(commandString.c_str());
        resultNumber = executeCommand(commandString);
      }
    }else{
      stringCheckResult = tempBuffer.find("help");
      if(stringCheckResult != 255){
        getHelp();
      }else{
        atCMD(SMbuffer.c_str());
        atCMD(char(13));
      }
      Buzz(50,900);
    }
    
    debugFunc(Msg.c_str());
    debugFunc(executeResult.c_str());
    debugFunc(echoFlag);
    debugFunc(cmdSource);
    if((executeResult != "")&&(echoFlag == 1)){
      Msg = executeResult;
      sendSms();
    }
    executeResult = "";
    ATbuffer = "";
    Buzz(500,850);
    
  }
}



void toLowerCase(string &str){
  for(char &c : str){
    c = tolower(c);
  }
}

void Check_io() {
  tempInStatus = 0;
  uint8_t In1 = digitalRead(In1_PIN) == HIGH ? 1 : 0;
  uint8_t In2 = digitalRead(In2_PIN) == HIGH ? 1 : 0;
  uint8_t In3 = digitalRead(In3_PIN) == HIGH ? 1 : 0;

  tempInStatus |= In1 << 1; 
  tempInStatus |= In2 << 2; 
  tempInStatus |= In3 << 3; 
  
  cmdSource = 0;
  
  /*SerialMon.print( );
  debugFunc(toBinary(tempInStatus).c_str());

  SerialMon.print("in:");
  debugFunc(toBinary(inStatus).c_str());

  SerialMon.print("outstatus:");
  debugFunc(toBinary(outStatus).c_str());*/


  if (tempInStatus != inStatus) {
    
    
    Msg = "";
    
    if (ioSetFlag == 30) {
      if ((tempInStatus & 0x02) != (inStatus & 0x02)) {
        Msg += (tempInStatus & 0x02) ? "bm.cmd:rel1=on;" : "bm.cmd:rel1=off;";
      }
      if ((tempInStatus & 0x04) != (inStatus & 0x04)) {
        Msg += (tempInStatus & 0x04) ? "bm.cmd:rel2=on;" : "bm.cmd:rel2=off;";
      }
      if ((tempInStatus & 0x08) != (inStatus & 0x08)) {
        Msg += (tempInStatus & 0x08) ? "bm.cmd:rel3=on;" : "bm.cmd:rel3=off;";
      }
    }
    
    if (ioSetFlag == 40) {
      if ((tempInStatus & 0x02) != (inStatus & 0x02)) {
        Msg += (tempInStatus & 0x02) ? "bm.cmd:rel1=off;" : "bm.cmd:rel1=on;";
      }
      if ((tempInStatus & 0x04) != (inStatus & 0x04)) {
        Msg += (tempInStatus & 0x04) ? "bm.cmd:rel2=off;" : "bm.cmd:rel2=on;";
      }
      if ((tempInStatus & 0x08) != (inStatus & 0x08)) {
        Msg += (tempInStatus & 0x08) ? "bm.cmd:rel3=off;" : "bm.cmd:rel3=on;";
      }
    }
    
    if (ioSetFlag == 50) {
      if ((tempInStatus & 0x02) != (inStatus & 0x02)) Msg += "bm.cmd:rel1=reset;";
      if ((tempInStatus & 0x04) != (inStatus & 0x04)) Msg += "bm.cmd:rel2=reset;";
      if ((tempInStatus & 0x08) != (inStatus & 0x08)) Msg += "bm.cmd:rel3=reset;";
    }
    debugFunc(Msg.c_str());
  
    if (Msg != "") {
      tempBuffer = Msg;
      debugFunc(tempBuffer.c_str());
      if (chainFlag == 0) {
        splitCommands();
      }

      SerialMon.print("msg is :");
      SerialMon.print(Msg.c_str());
      
      if (chainFlag >= 1) {
  
        userNumber = usersSet[0];
        stringLength = userNumber.length();
        if (stringLength > 10) {
          userNumber = userNumber.substr(0, 10);
        }

        if (userNumber != "") {
          subscriberNumber = userNumber;
          cmdSource = 1;
          sendSms();
        }



        if (chainFlag == 2) {
          cmdSource = 0;
          SerialMon.print("temp buffer is :");
          debugFunc(tempBuffer.c_str());
          splitCommands();
        }
      }
    }
    
    userIndex = usersSet.size();
    if (userIndex != 0 && ioSetFlag >= 20) {
      cmdSource = 1;
      commandString = "read=alert;";
      
      sendAlert();
      
      subscriberNumber  =  adminNumber;
      makeCall();

    }
    
    
    inStatus = tempInStatus;
    
    
    Buzz(500, 930); 
    
    persistence.put("instatus",to_string(tempInStatus),true);
  }

  
  outStatus = 0;

  uint8_t Rel1 = digitalRead(REL1_PIN) == HIGH ? 1 : 0;
  uint8_t Rel2 = digitalRead(REL2_PIN) == HIGH ? 1 : 0;
  uint8_t Rel3 = digitalRead(REL3_PIN) == HIGH ? 1 : 0;

  outStatus |= Rel1 << 1; 
  outStatus |= Rel2 << 2; 
  outStatus |= Rel3 << 3; 
  
  persistence.put("outstatus",to_string(outStatus),true);
  ioCheckInterval = 0;
  cmdSource = 0;
}

void atCMD(char cmdAT[]){
  SerialAT.write(cmdAT);
  SerialAT.write(char(13));
  SerialAT.write(char(10));
}

void atCMD(char cmdAT){
  SerialAT.write(cmdAT);
  SerialAT.write(char(13));
  SerialAT.write(char(10));
}

void atCMD(string cmdAT){
  SerialAT.write(cmdAT.c_str());
  SerialAT.write(char(13));
  SerialAT.write(char(10));
}


string toBinary(uint8_t value){

  string binaryStr(8, '0');

  for(int i = 7;i >= 0;--i){
    if(value & (1<<i)){
      binaryStr[7-i] = '1';
    }
  }

  return binaryStr;  

}