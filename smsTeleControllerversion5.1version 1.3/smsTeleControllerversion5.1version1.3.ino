#define TINY_GSM_MODEM_SIM800 

#include <TinyGsmClient.h>
#include <StreamString.h>
#include "CommandControl.tpp"
#include "Persistence.tpp"
#include <iostream>
#include <string>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TaskScheduler.h>




#define DHTPIN 27     
#define DHTTYPE    DHT21     

#define SERIAL_SIZE_RX  1024
#define SerialMon Serial
#define SerialAT  Serial1

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


void checkSMS();
void checkGsm();
void checkTemperature();

Scheduler scheduler;

Task taskSMS(110000, TASK_FOREVER, &checkSMS); 
Task taskTemperature(30000, TASK_FOREVER, &checkTemperature); 
Task taskGsm(50000, TASK_FOREVER, &checkGsm); 




DHT_Unified dht(DHTPIN, DHTTYPE);
sensor_t sensor;


bool readSerial = true;
String bufferAT = "";
TinyGsm modem(SerialAT);



PersistenceClass persistence;
CommandControl commandcontrol(&persistence);

hw_timer_t * timer = NULL;
const int debounceDelay = 3000;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;

unsigned long int resetInterval; 

String ioSetMessage = "";
int ioSetNumber = 0;




bool alarmFlag = true;
bool sensorError = false;

void onreceivefunc(){
  bufferAT += readSerial ? (char)SerialAT.read() : '\0';
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
}

void IRAM_ATTR onTimer() {
  
  resetInterval++;
  commandcontrol.checkRelays();
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  
}

bool In1;
bool In2;
bool In3;

String temc = "";

//Process Messages in Buffer of GSM
void processMessages(String& bufferAT) {
    String delimiter = "\r";
    int start = 0, end = 0;
    while ((end = bufferAT.indexOf(delimiter, start)) != -1) {
        
        String tempMsg = bufferAT.substring(start, end + delimiter.length());
        SerialMon.println(tempMsg);
        if(tempMsg.indexOf("+CMTI") != -1){
          int idx = tempMsg.indexOf(",");
          idx = tempMsg.substring(idx+1,idx+3).toInt();
          handleSMS(idx);   
        }else if(tempMsg.indexOf("+CLIP:") != -1){
          commandcontrol.buzz(500,1000);
          
          int tempStart = bufferAT.lastIndexOf("+CLIP: ");
          String response = bufferAT.substring(tempStart);
          tempStart = response.indexOf("\"");
          int tempEnd = response.indexOf("\",",tempStart);
          String callerID = response.substring(tempStart+3,tempEnd);
          temc = callerID;
          if(commandcontrol.isAuthorized(callerID, "")){
            //tone(BUZZER_PIN,1000,5000);
            bufferAT = "";
            modem.sendAT(GF("A"));
            modem.waitResponse();
            commandcontrol.buzz(700,900);
          }else{
            modem.sendAT(GF("H"));
            modem.waitResponse();
            bufferAT = "";
          }

        }else if(tempMsg.indexOf("NO CARRIER") != -1){
          
        }else if(tempMsg.indexOf("+DTMF:")){

          commandcontrol.dtmfHandler(tempMsg);
          commandcontrol.buzz(400,600);
        }
        bufferAT.remove(0, end + delimiter.length());
    }
}





void setup() {

  SerialAT.setRxBufferSize(SERIAL_SIZE_RX);
  
  timer = timerBegin(0, 80, true); 
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, INTERVAL * 1000, true);
  timerAlarmEnable(timer);

  resetInterval =0;
  
  dht.begin();
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  persistence.init();

  SerialMon.begin(9600);
  SerialAT.onReceive(onreceivefunc);
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  
  initializeGSM();
  SerialMon.println("modem has been initialzed");

  pinMode(In1_PIN, INPUT);
  pinMode(REL1_PIN, OUTPUT);
  

  pinMode(In2_PIN, INPUT);
  pinMode(REL2_PIN, OUTPUT);
  

  pinMode(In3_PIN, INPUT);
  pinMode(REL3_PIN, OUTPUT);
  

  configEEPROM();

  pinMode(ACT_PIN,OUTPUT);

  scheduler.addTask(taskSMS);
  scheduler.addTask(taskTemperature);
  scheduler.addTask(taskGsm);

  scheduler.enableAll();


  In1 = digitalRead(In1_PIN) == HIGH ? true : false;
  In2 = digitalRead(In2_PIN) == HIGH ? true : false;
  In3 = digitalRead(In3_PIN) == HIGH ? true : false;


  

}

void loop() {

  //SerialMon.println("in loop");
  SerialMon.println(temc);

  bool tempIn1 = digitalRead(In1_PIN) == HIGH ? true : false;
  bool tempIn2 = digitalRead(In2_PIN) == HIGH ? true : false;
  bool tempIn3 = digitalRead(In3_PIN) == HIGH ? true : false;

  if(tempIn1 != In1){
    unsigned long currentMillis = millis();    
    if (currentMillis - lastDebounceTime1 < debounceDelay) {
        if (currentMillis < lastDebounceTime1) {
            lastDebounceTime1 = currentMillis;
        } else {
            return;
        }
    }

    ioSetNumber = 1;
    lastDebounceTime1 = currentMillis;
    In1 = digitalRead(In1_PIN) == HIGH ? true : false;
  }

  if(tempIn2 != In2){
    unsigned long currentMillis = millis();    
    if (currentMillis - lastDebounceTime2 < debounceDelay) {
        if (currentMillis < lastDebounceTime2) {
            lastDebounceTime2 = currentMillis;
        } else {
            return;
        }
    }

    ioSetNumber = 2;
    lastDebounceTime2 = currentMillis;
    In2 = digitalRead(In2_PIN) == HIGH ? true : false;

  }
  
  if(tempIn3 != In3){

    unsigned long currentMillis = millis();    
    if (currentMillis - lastDebounceTime3 < debounceDelay) {
        if (currentMillis < lastDebounceTime3) {
            lastDebounceTime1 = currentMillis;
        } else {
            return;
        }
    }

    ioSetNumber = 3;
    lastDebounceTime3 = currentMillis;
    In3 = digitalRead(In3_PIN) == HIGH ? true : false;


  }
  


  


  //Check Self Reset Condition
  if(resetInterval > (commandcontrol.resetPeriod * 60)){
     resetInterval =0;
     if(commandcontrol.selfreset) resetModule();
  }  

  scheduler.execute();
  
  //Handle IOset and Chain mode instructions in case of interupt
  if(ioSetNumber != 0){

    ioSetMessage = commandcontrol.handleIoSet(ioSetNumber);
    //Report to Admin
    if(commandcontrol.getIoSetFlag() == "20"){
       ioSetMessage = commandcontrol.handleIoSet(ioSetNumber);
       sendingSMS(String(commandcontrol.admin.c_str()), ioSetMessage);
       ioSetMessage = "";
    }
    ioSetNumber = 0;
  }
  checkIoSetMessage();


  processMessages(bufferAT);
  updateSerial();

}

void updateSerial(){
  delay(400);
  String readBuffer = "";

  while (SerialMon.available()) {
    //commandcontrol.buzz(200,50);
    char c = SerialMon.read();
    readBuffer += c;             
  }
  //Serial Port Debug
  if (readBuffer.indexOf("DEBUG:") != -1) {
    String tempMsg = readBuffer.substring(6,readBuffer.indexOf(";")+1);
    handleSerialCommand(tempMsg);

  }else { // Send AT Command to GSM
      SerialAT.write(readBuffer.c_str(), readBuffer.length());
  }
}

String getSignalLevel(){

  modem.sendAT(GF("+CSQ"));
  modem.waitResponse();
  int tempStart = bufferAT.lastIndexOf("+CSQ: ");
  String signalLevel = bufferAT.substring(tempStart+6,tempStart+8);
  bufferAT.remove(tempStart,12);
  if(isNumeric(signalLevel)) return signalLevel;
  else return "";

}

void getTimeAndDate(String& time,String& date){

  modem.sendAT("+CCLK?");
  modem.waitResponse();
  int tempStart = bufferAT.lastIndexOf("+CCLK: ") + 7;
  int tempEnd = bufferAT.indexOf("\"",tempStart+1);
  String payload = bufferAT.substring(tempStart + 1,tempEnd);
  
  date = payload.substring(0,payload.indexOf("\,"));
  time = payload.substring(payload.indexOf("\,") + 1,payload.length()-3);
  bufferAT.remove(tempStart-7,tempEnd-tempStart-7);
  commandcontrol.buzz(600,950);

}

void makeCall(String phoneNumber){
  
  readSerial = false;
  modem.sendAT(GF("D"),phoneNumber, ";");
  readSerial = true;
  commandcontrol.buzz(500,870);
}

//Checks for Unprocess Messages in Inbox
void checkSMS(){

  commandcontrol.setBuzzer(false);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  modem.sendAT(GF("+CMGF=1"));
  modem.waitResponse();
  modem.sendAT(GF("+CMGL=\"REC UNREAD\",1"));
  modem.waitResponse();
  int tempStart = bufferAT.indexOf("+CMGL:");
  if(tempStart == -1) return;
  int tempEnd = bufferAT.indexOf("OK",tempStart);
  String response = bufferAT.substring(tempStart, tempEnd);

  bufferAT.remove(tempStart,tempEnd - tempStart);

  int start = 0;
  while((start = response.indexOf("+CMGL:")) != -1){

    int end = response.indexOf("+CMGL:",1);
    String payload = (end != -1) ? response.substring(start,end) : response.substring(start);
    int tempStart = payload.indexOf(":");
    String idx = payload.substring(tempStart+2,tempStart+4);
    if(isNumeric(idx)) handleSMS(idx.toInt());
    if(end != -1) response.remove(start,end - start);
    else break;
  }
  commandcontrol.setBuzzer(true);

}

String getImei(){
  modem.sendAT(GF("+GSN"));
  modem.waitResponse();
  int tempStart = bufferAT.indexOf("86"); // Type Allocation Code (TAC) by GSMA Approval Group, for China is 86
  if(tempStart == -1) return "";
  String imei = bufferAT.substring(tempStart,tempStart + 15);
  bufferAT.remove(tempStart,15);
  commandcontrol.buzz(600,950);
  return imei;
}

String getVoltage(){

  modem.sendAT(GF("+CBC"));
  modem.waitResponse();
  int tempStart = bufferAT.lastIndexOf("+CBC:");
  int tempEnd = bufferAT.indexOf("OK",tempStart);
  if(tempStart == -1 || tempEnd == -1) return "";
  String response = bufferAT.substring(tempStart,tempEnd);
  bufferAT.remove(tempStart-3,tempEnd - tempStart+5);
  tempStart = response.lastIndexOf(",");
  tempEnd = response.indexOf("\n");
  if(tempStart == -1 || tempEnd == -1) return "";
  return response.substring(tempStart + 1,tempEnd);
}

bool initializeGSM(){
  
  commandcontrol.buzz(500,980);
  pinMode(Power_Key, OUTPUT);
  SerialMon.println("Initializing GSM ...");
  bufferAT = "";
  modem.sendAT(GF("AT"));
  delay(1000);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  if(bufferAT == ""){
    commandcontrol.buzz(500,980);
    digitalWrite(Power_Key, HIGH);
    delay(1000);
    digitalWrite(Power_Key, LOW);
    SerialMon.println("Initializing modem->..");
    modem.restart();
    SerialMon.println("restart done!");
    modem.init();
    SerialMon.println("init done!");
  }

  bufferAT = "";
  modem.sendAT(GF("AT"));
  delay(500);
  int loopCounter = 0;

  while(1){

      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
      if(bufferAT.indexOf("OK")>0){
        bufferAT = "";
        break;
      } else{       
        digitalWrite(Power_Key, HIGH);
        delay(1000);
        digitalWrite(Power_Key, LOW);
        digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
        SerialMon.println("Initializing modem->..");
        modem.restart();
        SerialMon.println("restart done!");
        modem.init();
        SerialMon.println("init done!");
        delay(1500);
        modem.sendAT(GF("AT"));
        delay(500);
        digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
        loopCounter += 1;
      }
      if(loopCounter>15) return false;    
  }

  modem.sendAT(GF("ATE0"));
  delay(50);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  bufferAT = "";
  delay(50);
  modem.sendAT(GF("+COPS?"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(1500);
  loopCounter = 0;

  digitalWrite(Power_Key, HIGH);
  delay(1000);
  digitalWrite(Power_Key, LOW);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  SerialMon.println("Initializing modem->..");
  modem.restart();
  SerialMon.println("restart done!");
  modem.init();
  SerialMon.println("init done!");
  delay(5000);


  while(1){
    digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
    if(bufferAT != ""){
      SerialMon.print("Searching Network Operators... " + bufferAT);
      if(bufferAT.indexOf("OK")>0){
        bufferAT = "";
        commandcontrol.buzz(500,2400);
        break;
      }
      loopCounter +=1; 
    } else{
      
      modem.sendAT(GF("+COPS?"));
      digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
      delay(10000);
      
    }
    if(loopCounter > 25){
      SerialMon.println("Error: Network Operator Not Found or SIM Module Not Responde!");
      commandcontrol.buzz(1500,980);
      return false;
      break;
    }
  }
  


  bufferAT = "";
  modem.sendAT(GF("+CLIP=1"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(3000);
  bufferAT = "";
  modem.sendAT(GF("+CSPN?"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(3000);
  SerialMon.println("Registered On: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+GSV"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(500);
  SerialMon.println("GSM Module: " + bufferAT);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(500);
  bufferAT = "";
  modem.sendAT(GF("+CMGF=1"),char(13));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Config SMS Text Mode: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CMGDA="),char(34),"DEL ALL", char(34));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(500);
  SerialMon.println("Delete Old SMS's: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CNMI=2,1,2,0,0"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Config New SMS indication: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CSMP=17,167,0,0"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Config New SMS CREDICT: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CPMS="),char(34),"ME",char(34),",",char(34),"ME",char(34),",",char(34),"ME",char(34));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(500);
  
  SerialMon.println("Config SMS message storage: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CMEE=1"));
  delay(500);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  SerialMon.println("Config Error Report Mode: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CMTE=1"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Enable Internal Temp Sensor: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+CLTS=1"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(600);
  SerialMon.println("Enable Network Clock: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+DDET=1,0,0,0"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Enable DTMF Detecting: " + bufferAT);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  bufferAT = "";
  modem.sendAT(GF("+CLIP=1"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  SerialMon.println("Enable Caller ID Detecting: " + bufferAT);
  bufferAT = "";
  modem.sendAT(GF("+DDET=1"));
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(200);
  modem.sendAT("AT&W");
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  delay(300);
  SerialMon.println("Save Config: " + bufferAT);
  bufferAT="";
  return true;

}
  
boolean isNumeric(String str) {

        unsigned int stringLength = str.length();
 
    if (stringLength == 0) {
        return false;
    }
 
    bool seenDecimal = false;
    bool seenNegative = false; // Track if a negative sign '-' is encountered
 
    for(unsigned int i = 0; i < stringLength; ++i) {
        if (isdigit(str.charAt(i))) {
            continue;
        }
 
        if (str.charAt(i) == '.') {
            if (seenDecimal) {
                return false;
            }
            seenDecimal = true;
            continue;
        }

        if (str.charAt(i) == '-' && i == 0) {
            // Negative sign '-' is allowed only at the beginning of the string
            seenNegative = true;
            continue;
        }
        
        return false;
    }
    
    // The string should not contain only a negative sign
    if (seenNegative && stringLength == 1) {
        return false;
    }

    return true;
    
}

void handleSMS(int idx){

  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  modem.sendAT(GF("+CMGR="), idx);
  modem.waitResponse();
        
  int temp_start = bufferAT.lastIndexOf("+CMGR: \"REC UNREAD\",");
  int temp_end = bufferAT.indexOf("\n\r",temp_start);
  String response = bufferAT.substring(temp_start,temp_end);
  bufferAT.remove(temp_start,temp_end - temp_start);
  response.toLowerCase();
  String ID,message;
  String feedback = "";
  processSms(response, ID, message);
   
  if((response.indexOf("help") != -1) || (response.indexOf("?") != -1)){
    if(commandcontrol.isAuthorized(ID,"")) getHelp(ID);
    return;
  }
  
  if(message == "WRONG"){
    sendingSMS(ID,"Command Format Incorrect!");
    return;
  }
 
  commandcontrol.commandHandler(ID, message, modem, feedback);
  //commandcontrol.buzz(900,860);
  if(feedback == "ALERT"){
    String temperature,humid;
    String time,date;

    getTemperature(temperature,humid);
    getTimeAndDate(time, date);

    String payload = "Alert(" + date + " " + time + ")=> Temp:" + temperature + " Hum:" + humid;
    payload += " Inputs:0000";
    payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload += "Outputs:0000";
    payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
    payload += "0";
    sendingSMS(ID, payload);

  }else if(feedback =="ALL"){
    String temperature,humid;
    getTemperature(temperature,humid);

    String payload = "Inputs:0000";
    payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload += "Outputs:0000";
    payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
    payload += "0";
    payload +=  " Sig.Level:" + getSignalLevel();
    payload += "IO.Flag:" + commandcontrol.getIoSetFlag();
    String temp = commandcontrol.getBuzzer() ? "1" : "0";
    payload += " Sound:" + temp;
    temp = commandcontrol.getEcho() ? "1" : "0";
    payload += " Echo:" + temp;
    payload += " Log:0 log.Period:0m";
    payload += " Delay:" + String(commandcontrol.delayOfReset);
    payload += "s Temp:" + temperature;
    payload += " Hum:" + humid;
    payload += " VCC:" + getVoltage() + "mV";

    sendingSMS(ID,payload);
    
  }else if(feedback == "INFO"){
    String payload = "Modele Name:ABM Module Type:Mini ";
    payload += "ID:" + getImei();
    payload += " Hw.V:5.1 Sw.V:1.0";
    payload += "Protect:";
    payload += commandcontrol.protectionEnable ? "1" : "0";
    payload += "Rst.Count:";
    payload += persistence.getValue("resetcount")->c_str();
    payload += " Self Rst:";
    payload += commandcontrol.selfreset ? "1": "0";
    payload += "Rst Period:";
    payload += String(commandcontrol.resetPeriod) + "m";
    sendingSMS(ID, payload);



  }else if(feedback == "RESET"){
    resetModule();
  }else if(feedback == "RESETGSM"){
    resetGsm();
  }else if(feedback == "CALLME"){
    if(commandcontrol.isAuthorized(ID, "")){
      makeCall("+98" + ID);
    }else{
      sendingSMS(ID,"You are unauthorized! Please contact: www.abm.co.ir");

    }
  }else if(feedback !=""){
    sendingSMS(ID,feedback);
  }

}

void getTemperature(String& temperature, String& humid){

  
  if(commandcontrol.sensorOut){
    dht.begin();
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    uint32_t delayMS = sensor.min_delay / 1000;
    delay(delayMS);
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      temperature = "Error reading temperature!";
    }
    else {

      temperature = event.temperature;
      temperature += "c";
      
    }
    
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      humid = "Error reading humidity!";
    }
    else {

      humid = event.relative_humidity;
      humid += "%";
    }
  }else{
    //commandcontrol.setBuzzer(false);
    modem.sendAT(GF("+CMTE?"));
    modem.waitResponse();
    int tempStart = bufferAT.indexOf("+CMTE:");
    int tempEnd = bufferAT.indexOf("OK",tempStart);
    String response = bufferAT.substring(tempStart,tempEnd);
    bufferAT.remove(tempStart,tempEnd-tempStart);
    tempStart = response.indexOf(",");
    tempEnd = response.indexOf("\r");
    
    temperature = response.substring(tempStart+1,tempEnd);
    temperature += "c";
    //commandcontrol.setBuzzer(true);
    humid = "1";

   
  }

}

void checkTemperature(){

  
  //commandcontrol.setBuzzer(false);
  String temperature;
  if(commandcontrol.sensorOut){  
    dht.begin();
    dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);
    uint32_t delayMS = sensor.min_delay / 1000;
    delay(delayMS);
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    temperature = event.temperature;
  }else{
    /*modem.sendAT(GF("+CMTE?"));
    modem.waitResponse();
    int tempStart = bufferAT.indexOf("+CMTE:");
    int tempEnd = bufferAT.indexOf("OK",tempStart);
    String response = bufferAT.substring(tempStart,tempEnd);
    bufferAT.remove(tempStart,tempEnd-tempStart);
    temperature = response.substring(response.indexOf(",")+1,response.indexOf("\r"));*/
    temperature = 2;
  }

  if(!isNumeric(temperature) && (!sensorError)){
    sensorError = true;
    sendingSMS("9145148413","sensor error");
    return;
  }else if(isNumeric(temperature)){

    sensorError = false;

  }


  int tmp = temperature.toInt();

  SerialMon.println(tmp);
  SerialMon.println(commandcontrol.maxtemp.toInt());
  SerialMon.println(commandcontrol.mintemp.toInt());
  SerialMon.println(alarmFlag);


  if((tmp > commandcontrol.maxtemp.toInt() || tmp < commandcontrol.mintemp.toInt()) && alarmFlag){
    

    String temperature,humid;
    String time,date;

    getTemperature(temperature,humid);
    getTimeAndDate(time, date);

    String payload = "Alert(" + date + " " + time + ")=> Temp:" + temperature + " Hum:" + humid;
    payload += " Inputs:0000";
    payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload += "Outputs:0000";
    payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
    payload += "0";

    
    sendingSMS(String(commandcontrol.admin.c_str()),payload);

    int i = 1;
    while(persistence.checkExistence( "user" + String(i))){
      String temp = persistence.getValue("user" + String(i))->c_str();
      sendingSMS(temp, payload);
      i++;
    }

    if(commandcontrol.callEnable) makeCall("+98" + String(commandcontrol.admin.c_str()));
    alarmFlag = false;



  }else if((tmp < commandcontrol.maxtemp.toInt() && tmp > commandcontrol.mintemp.toInt())){
    alarmFlag = true;
  }

  //commandcontrol.setBuzzer(true);
  return;
}
void sendingSMS(String ID, String message){

  unsigned int tempStart = bufferAT.length();
  readSerial = false;
  modem.sendSMS("+98" + ID, message);
  readSerial = true;
  modem.sendAT(GF("AT"));
  modem.waitResponse();
  unsigned int tempEnd = bufferAT.length();
  bufferAT.remove(tempStart,tempEnd-tempStart);

}

void getHelp(String ID){

  String helpMessage = "bm.cmd:{sound=on/off;echo=on/off;battery=on/off;sensor=in/out;ioset=noact/report/direct/invers/reset;webreport=on/off;relX=on/off;relX=reset;}";
  sendingSMS(ID, helpMessage);

  helpMessage = "bm.cmd:{read=alert/all/io/info;showusers;deluser=x/all;unprotect;protect;resetme;resetgsm;callme;admin=9xxxxxxxxx;adduser=9xxxxxxxxx;setname=xxx;}";
  sendingSMS(ID, helpMessage);

  helpMessage = "bm.cmd:{dest=9xxxxxxxxx;msg=xxx;selfreset=on/off;chainmode=off/on/booth;setdelay=[2~65000s];logperiod=[3~65000m];rstperiod=[10~65000m];weburl=xxx;}";
  sendingSMS(ID, helpMessage);

  commandcontrol.buzz(50,900);

}

void checkGsm(){
  //commandcontrol.setBuzzer(false);
  //commandcontrol.buzz(300,750);
  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  modem.sendAT(GF("+CREG?"));
  modem.waitResponse();
  int tempStart = bufferAT.indexOf("+CREG:");
  int tempEnd = bufferAT.indexOf("OK",tempStart);
  String response = bufferAT.substring(tempStart,tempEnd);
  bufferAT.remove(tempStart,tempEnd-tempStart);
  int networkStatus = response.substring(response.indexOf(",")+1,response.indexOf("\r")).toInt();
  //commandcontrol.setBuzzer(true);
  if(networkStatus != 1) resetGsm();
  return;

}

void resetModule(){
  ESP.restart();
}

void resetGsm(){

  initializeGSM();

}

void configEEPROM(){

  if(persistence.checkExistence("echo")){
    bool echoState = String(persistence.getValue("echo")->c_str()) == "ON" ? true : false;
    commandcontrol.setEcho(echoState);
    SerialMon.println(persistence.getValue("echo")->c_str());
  }
  if(persistence.checkExistence("callenable")){
    bool callState = String(persistence.getValue("callenable")->c_str()) == "ON" ? true : false;
    commandcontrol.callEnable = callState;
    SerialMon.println(persistence.getValue("echo")->c_str());
  }
  if(persistence.checkExistence("sound")){
    bool soundState = String(persistence.getValue("sound")->c_str()) == "ON" ? true : false;
    commandcontrol.setBuzzer(soundState);
    SerialMon.println(persistence.getValue("sound")->c_str());
  }
  if(persistence.checkExistence("rel1")){
    String relState = persistence.getValue("rel1")->c_str();
    pinMode(REL1_PIN, OUTPUT);
    if(relState == "ON") digitalWrite(REL1_PIN, HIGH);
    else if(relState == "OFF") digitalWrite(REL1_PIN, LOW);
    SerialMon.println(persistence.getValue("rel1")->c_str());
  }
  if(persistence.checkExistence("rel2")){
    String relState = persistence.getValue("rel2")->c_str();
    pinMode(REL2_PIN, OUTPUT);
    if(relState == "ON") digitalWrite(REL2_PIN, HIGH);
    else if(relState == "OFF") digitalWrite(REL2_PIN, LOW);
    SerialMon.println(persistence.getValue("rel2")->c_str());
  }
  if(persistence.checkExistence("rel3")){
    String relState = persistence.getValue("rel3")->c_str();
    pinMode(REL3_PIN, OUTPUT);
    if(relState == "ON") digitalWrite(REL3_PIN, HIGH);
    else if(relState == "OFF") digitalWrite(REL3_PIN, LOW);
    SerialMon.println(persistence.getValue("rel3")->c_str());
  }
  if(persistence.checkExistence("sensor")){
    bool sensorState = String(persistence.getValue("sensor")->c_str()) == "OUT" ? true : false;
    commandcontrol.sensorOut = sensorState;
    SerialMon.println(persistence.getValue("sensor")->c_str());
  }
  if(persistence.checkExistence("protection")){
    bool protectionState = String(persistence.getValue("protection")->c_str()) == "ON" ? true : false;
    commandcontrol.protectionEnable = protectionState;
    SerialMon.println(persistence.getValue("protection")->c_str());
  }
  if(persistence.checkExistence("admin")){
    String adminNumber = persistence.getValue("admin")->c_str();
    commandcontrol.admin = adminNumber.c_str();
    SerialMon.println(adminNumber);
  }
  if(persistence.checkExistence("devicename")){
    String devicename = persistence.getValue("devicename")->c_str();
    commandcontrol.deviceName = devicename.c_str();
    SerialMon.println(devicename);

  }
  if(persistence.checkExistence("ioset")){
    String temp = persistence.getValue("ioset")->c_str();
    commandcontrol.setIoState(temp);
    SerialMon.println(temp);
  }
  if(persistence.checkExistence("chainmode")){
    String temp = persistence.getValue("chainmode")->c_str();
    commandcontrol.setChainMode(temp);
    SerialMon.println(temp);
  }
  if(persistence.checkExistence("delay")){
    String temp = persistence.getValue("delay")->c_str();
    commandcontrol.delayOfReset = std::stoul(temp.c_str());
    SerialMon.println(temp);
  }
  int i = 1;
  while(persistence.checkExistence( "user" + String(i))){
    String temp = persistence.getValue("user" + String(i))->c_str();
    commandcontrol.insertUsersSet(temp);
    SerialMon.println(temp);
    i++;
  }
  if(persistence.checkExistence("maxtemp")){
    commandcontrol.maxtemp = String(persistence.getValue("maxtemp")->c_str());
  }else commandcontrol.maxtemp = 50;
  if(persistence.checkExistence("mintemp")){
    commandcontrol.mintemp = String(persistence.getValue("mintemp")->c_str());
  }else commandcontrol.mintemp = 10;
  
  if(persistence.checkExistence("resetperiod")){

    String temp = persistence.getValue("resetperiod")->c_str();
    unsigned long int rstprd = std::stoul(temp.c_str());
    commandcontrol.resetPeriod = rstprd;
  }else{
    commandcontrol.resetPeriod =1440;
  }
  if(persistence.checkExistence("resetcount")){
    String temp = persistence.getValue("resetcount")->c_str();
    int rst = temp.toInt() + 1;
    persistence.put("resetcount",String(rst),true);
  }else{
    persistence.put("resetcount","0",true);
  }

  digitalWrite(ACT_PIN, !digitalRead(ACT_PIN));
  commandcontrol.buzz(400,950);

}

void handleSerialCommand(String payload){

  payload.toLowerCase();
  String feedback = "";
  commandcontrol.commandHandler("DEBUG", payload, modem, feedback);
  commandcontrol.buzz(900,860);
  if(feedback == "ALERT"){
    String temperature,humid;
    String time,date;

    getTemperature(temperature,humid);
    getTimeAndDate(time, date);

    String payload = "Alert(" + date + " " + time + ")=> Temp:" + temperature + "C Hum:" + humid + "   ";
    payload += " Inputs:0000";
    payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload += "Outputs:0000";
    payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
    SerialMon.println(payload);

  }else if(feedback =="ALL"){
    String temperature,humid;
    getTemperature(temperature,humid);

    String payload = "Inputs : 0000";
    payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload += "Outputs : 0000";
    payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
    payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
    payload += "0 ";
    payload +=  "   Sig.Level:" + getSignalLevel();
    payload += " IO.Flag:" + commandcontrol.getIoSetFlag();
    String temp = commandcontrol.getBuzzer() ? "ON" : "OFF";
    payload += " Sound:" + temp;
    temp = commandcontrol.getEcho() ? "ON" : "OFF";
    payload += " Echo:" + temp;
    payload += "  Delay:" + String(commandcontrol.delayOfReset);
    payload += "ms Temp:" + temperature;
    payload += "c Hum:" + humid;
    payload += "% VCC:" + getVoltage() + "mV";
    SerialMon.println(payload);
    
  }else if(feedback == "RESET"){
    resetModule();
  }else if(feedback == "RESETGSM"){
    resetGsm();
  }else if(feedback == "INFO"){
    String payload = "Modele Name:ABM Module Type:Mini ";
    payload += " ID:" + getImei();
    payload += " Hw.V:5.1 Sw.V:1.0";
    payload += " Protect:";
    payload += commandcontrol.protectionEnable ? "1" : "0";
    payload += " Rst.Count:";
    payload += persistence.getValue("resetcount")->c_str();
    payload += " Self Rst:";
    payload += commandcontrol.selfreset ? "1": "0";
    payload += " Rst Period:";
    payload += String(commandcontrol.resetPeriod) + "m";
    SerialMon.println(payload);

  }else if(feedback !=""){
    SerialMon.println(feedback);
  }
}

void checkIoSetMessage(){
  if(ioSetMessage != "" && ioSetMessage != "OK!"){

    String destination = commandcontrol.usersSet[0];
    sendingSMS(destination,ioSetMessage);   

  }
  ioSetMessage = "";
  
}

//get ID and Payload of a SMS 
void processSms(String payload,String& ID, String& message){

  int eol = payload.indexOf("\n",2);
  message = payload.substring(eol);

  //Detecting Wrong Command Format
  if((message.indexOf("bm.cmd:") == -1) || (message.indexOf(";") == -1)) message ="WRONG";
  else message = message.substring(message.indexOf("bm"),message.indexOf(";") + 1);

  int first = payload.indexOf(",");
  int second = payload.indexOf(",", first + 1);
  ID = payload.substring(first+5, second-1);

}
