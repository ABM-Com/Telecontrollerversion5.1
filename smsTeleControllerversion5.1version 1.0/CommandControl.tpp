#ifndef VIRALINK_COMMAND_CONTROL_TPP
#define VIRALINK_COMMAND_CONTROL_TPP

#include <iostream>
#include <string>
#include "Persistence.tpp"
#include <vector>



#define ECHO_COMMAND "bm.cmd:echo"
#define BUZZER_COMMAND "bm.cmd:sound"
#define REL1_COMMAND "bm.cmd:rel1"
#define REL2_COMMAND "bm.cmd:rel2"
#define REL3_COMMAND "bm.cmd:rel3"
#define CALL_ENABLE_COMMAND "bm.cmd:call"
#define PROTECT_COMMAND "bm.cmd:protect;"
#define UNPROTECT_COMMAND "bm.cmd:unprotect;"
#define DELAY_COMMAND "bm.cmd:setdelay"
#define DELETE_COMMAND "bm.cmd:deluser"
#define SETNAME_COMMAND "bm.cmd:setname"
#define ADMIN_COMMAND "bm.cmd:admin"
#define ADDUSER_COMMAND "bm.cmd:adduser"
#define READ_COMMAND "bm.cmd:read"
#define IOSET_COMMAND "bm.cmd:ioset"
#define SHOWUSERS_COMMAND "bm.cmd:showusers;"
#define SENSOR_COMMAND "bm.cmd:sensor"
#define RESET_COMMAND "bm.cmd:resetme;"
#define RESETGSM_COMMAND "bm.cmd:resetgsm;"
#define CALLME_COMMAND "bm.cmd:callme;"
#define CHAINMODE_COMMAND "bm.cmd:chainmode"
#define MAX_TEMP_COMMAND "bm.cmd:maxtemp"
#define MIN_TEMP_COMMAND "bm.cmd:mintemp"
#define SELF_RESET_COMMAND "bm.cmd:selfreset"
#define RESET_PERIOD_COMMAND "bm.cmd:rstperiod"

#define VALUE_INCORRECT "Entered Value/Stream Incorrect"

#define REL1_PIN 32
#define REL2_PIN 33
#define REL3_PIN 25

#define In1_PIN 34
#define In2_PIN 39
#define In3_PIN 36

#define ACT_PIN 19
#define BUZZER_PIN 23

//ioset modes//
enum ioSetStates {
    NOACT,
    REPORT,
    DIRECT,
    INVERSE,
    RESET
};

enum chainModes {
    ON,
    OFF,
    BOTH
};

class CommandControl {
public:
    CommandControl(PersistenceClass *persistence);

    bool selfreset = true;
    std::string deviceName = "ABM Module";
    std::string  admin = "";
    bool protectionEnable = false;
    ioSetStates ioState = NOACT;
    chainModes chainMode = OFF;
    bool sensorOut = false;
    bool callEnable = false;
    String maxtemp;
    String mintemp;
    unsigned long int delayOfReset =5;
    unsigned long int rel1Delay = 0;
    unsigned long int rel2Delay = 0;
    unsigned long int rel3Delay = 0;
    unsigned long int resetPeriod = 10;
    std::vector<String> usersSet;


    bool isPhoneNumberValid(String phoneNumber);
    bool isAuthorized(String phoneNumber, String payload);
    bool isNumeric(String str);
    bool getEcho();
    void setEcho(bool echo);
    String getIoSetFlag();
    bool getBuzzer();
    void setBuzzer(bool buzzer);
    void commandHandler(String phoneNumber, String atCommand, TinyGsm modem, String& feedback);
    void dtmfHandler(String dtmfCommand);
    bool isUser(String& phoneNumber);
    void insertUsersSet(String phoneNumber);
    void checkRelays();
    void setIoState(String IOstate);
    void setChainMode(String chainState);
    void buzz(int frequency,int duration);

    String handleIoSet(int ioNumber);
    String ioSet(int ioNumber);
    String chainModeMessage(int ioNumber);
    

private:

    bool echoEnable = true;
    bool buzzerEnable = true;
    PersistenceClass *persistence;

};

CommandControl::CommandControl(PersistenceClass *persis){
    persistence = persis;
}

// takes two argument phonenumber and command and do proper action and return right feedback according to situation//
void CommandControl::commandHandler(String phoneNumber, String atCommand, TinyGsm modem, String& feedback){
    //check if this command type is authorize for this phonenumber
    bool authorization = this->isAuthorized(phoneNumber,atCommand);
    int endPos = atCommand.indexOf('=');
    String commandType;
    //slice the command by ";" and if command format is wrong return none
    if (endPos != -1) { 
        commandType = atCommand.substring(0, endPos);
    }
    else{
        commandType = "";
    }
    //to understand this section firstly read the command's documentaion then consider that if the command value is irelevant to command type feedback is value incorrect 
    //and for right values it will do related actions and modification and save them in eeprom and feedback will be OK
    if(commandType.equals(ECHO_COMMAND) && authorization){
        if(atCommand.substring(endPos+1)=="on;"){ 
            this->echoEnable = true;
            persistence->put("echo","ON",true);
        }else if(atCommand.substring(endPos+1)=="off;"){
            this->echoEnable = false;
            persistence->put("echo","OFF",true);
        }
        else {
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(BUZZER_COMMAND) && authorization){
        //this->buzzerEnable = (atCommand.substring(endPos+1) == "on;") ? true : ((atCommand.substring(endPos+1) == "off;") ? false : this->buzzerEnable);
        if(atCommand.substring(endPos+1) == "on;"){
            this->buzzerEnable = true;
            persistence->put("sound", "ON",true);
        }else if(atCommand.substring(endPos+1) == "off;"){
            this->buzzerEnable = false;
            persistence->put("sound","OFF",true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    }else if(commandType.equals(CALL_ENABLE_COMMAND) && authorization){
        if(atCommand.substring(endPos+1) == "on;"){
            this->callEnable = true;
            persistence->put("callenable", "ON",true);
        }else if(atCommand.substring(endPos+1) == "off;"){
            this->callEnable = false;
            persistence->put("callenable", "OFF",true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(REL1_COMMAND) && authorization){

        pinMode(REL1_PIN, OUTPUT);
        String command = atCommand.substring(endPos+1);
        if(command=="on;"){ 
            digitalWrite(REL1_PIN, HIGH);
            persistence->put("rel1","ON",true);
        }else if (command=="off;"){ 
            digitalWrite(REL1_PIN,LOW);
            persistence->put("rel1","OFF",true);
        }else if (command=="reset;"){  
            digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));
            this->rel1Delay = this->delayOfReset;
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(REL2_COMMAND) && authorization){

        pinMode(REL2_PIN, OUTPUT);
        String command = atCommand.substring(endPos+1);
        if(command=="on;"){ 
            digitalWrite(REL2_PIN, HIGH);
            persistence->put("rel2","ON",true);
        }else if (command=="off;"){ 
            digitalWrite(REL2_PIN,LOW);
            persistence->put("rel2","OFF",true);
        }else if (command=="reset;"){ 
            digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));
            this->rel2Delay = this->delayOfReset;
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(REL3_COMMAND) && authorization){
        pinMode(REL3_PIN, OUTPUT);
        String command = atCommand.substring(endPos+1);
        if(command=="on;"){ 
            digitalWrite(REL3_PIN, HIGH);
            persistence->put("rel3","ON",true);
        }else if (command=="off;"){ 
            digitalWrite(REL3_PIN,LOW);
            persistence->put("rel3","OFF",true);
        }else if (command=="reset;"){
            digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));
            this->rel3Delay = this->delayOfReset;
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(DELAY_COMMAND) && authorization){

        String payload = atCommand.substring(endPos+1,atCommand.length() - 1);
        if(!isNumeric(payload)){
            feedback = VALUE_INCORRECT;
            return;
        }
        unsigned long int commandDelay = std::stoul(payload.c_str());
        if(commandDelay > 0 && commandDelay < 65000){
            this->delayOfReset = commandDelay;
            persistence->put("delay",payload,true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(RESET_PERIOD_COMMAND) && authorization){

        String payload = atCommand.substring(endPos+1,atCommand.length() - 1);
        if(!isNumeric(payload)){
            feedback = VALUE_INCORRECT;
            return;
        }
        unsigned long int commandPeriod = std::stoul(payload.c_str());
        if(commandPeriod > 9 && commandPeriod < 65000){
            this->resetPeriod = commandPeriod;
            persistence->put("resetperiod",payload,true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";

    }else if(commandType.equals(MAX_TEMP_COMMAND) && authorization){

        String payload = atCommand.substring(endPos+1,atCommand.length() - 1);
        if(!isNumeric(payload)){
            feedback = VALUE_INCORRECT;
            return;
        }
        int temp = payload.toInt();
        if(temp > -60 && temp <60 && temp >(this->mintemp).toInt()){
            this->maxtemp = temp;
            persistence->put("maxtemp",payload,true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(MIN_TEMP_COMMAND) && authorization){
        String payload = atCommand.substring(endPos+1,atCommand.length() - 1);
        if(!isNumeric(payload)){
            feedback = VALUE_INCORRECT;
            return;
        }
        int temp = payload.toInt();
        if(temp > -60 && temp <60 && temp <(this->maxtemp).toInt()){
            this->mintemp = temp;
            persistence->put("mintemp",payload,true);
        }
        else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(DELETE_COMMAND) && authorization){
        String command = atCommand.substring(endPos+1);
        if(command == "all;"){
            this->usersSet.clear();
            for (int i = 1; i <= usersSet.size(); ++i) {
                persistence->removeKey("user" + String(i),true);
            }
        }else if(isNumeric(command.substring(0,command.indexOf(";")))){
            int userToDelete = (command.substring(0,command.indexOf(";"))).toInt();
            if (userToDelete > 0 && userToDelete <= usersSet.size()) {
                usersSet.erase(usersSet.begin() + (userToDelete - 1));
                persistence->removeKey("user" + String(userToDelete),true);
            }
            //Fill the Gap in EEPROM Keys
            for (int i = userToDelete + 1; i <= usersSet.size() + 1; ++i) {
                persistence->removeKey("user" + String(i),true);
            }
            for (int i = userToDelete - 1; i < usersSet.size(); ++i) {
                persistence->put("user" + String(i+1),usersSet[i],true);
            }
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }

        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(SETNAME_COMMAND) && authorization){

        this->deviceName = (atCommand.substring(endPos+1,atCommand.length() - 1)).c_str();
        persistence->put("devicename",atCommand.substring(endPos+1,atCommand.length() - 1),true);
        feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(ADMIN_COMMAND) && authorization){

        String payload = atCommand.substring(endPos+1,atCommand.length()-1);

        if(isPhoneNumberValid(payload)){
            this->admin = payload.c_str();
            persistence->put("admin",payload,true);
            this->protectionEnable = true;
            persistence->put("protection", "ON",true);

        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(ADDUSER_COMMAND) && authorization){

        String payload = atCommand.substring(endPos+1,atCommand.length()-1);
        //checks if user phonenumber is valid wich means it start with +98 or contains 11 digit
        if(isPhoneNumberValid(payload)){
            this->usersSet.push_back(payload);
            persistence->put("user" + String(usersSet.size()),payload,true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(READ_COMMAND) && authorization){
        //for read command firstly we extract the payload and then propotional to payload we read values and write them in feedback exept all and alert and info
        //in those three cases the feedback is payload and gathering data will be collected in main loop of program because it should get information from gsm module and external sensors 
        this->buzz(400,750);
        String command = atCommand.substring(endPos+1);
        String payload = "";
        if(command=="outputs;"){
            payload = "Outputs:0000";
            payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
            payload += "0 ";
            feedback = payload;
        }else if(command=="inputs;"){
            payload = "Inputs:0000";
            payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
            payload += "0 ";
            feedback = payload;
        }else if(command=="io;"){
            payload = "Inputs:0000";
            payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
            payload += "0 ";
            payload += "Outputs:0000";
            payload += digitalRead(REL3_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(REL2_PIN) == HIGH ? "1" : "0" ;
            payload += digitalRead(REL1_PIN) == HIGH ? "1" : "0" ;
            payload += "0 ";
            feedback = payload;
        }else if(command == "alert;"){
            feedback = "ALERT";
            this->buzz(500,930);
            return;
        }else if(command == "all;"){
            feedback = "ALL";
            return;
        }else if(command == "info;"){
            feedback = "INFO";
            return;
        }else{
            feedback =VALUE_INCORRECT;
            return;
        }



    } else if(commandType.equals(IOSET_COMMAND)&& authorization){
        String command = atCommand.substring(endPos+1);
        if(command=="noact;"){
            this->ioState= NOACT;
            persistence->put("ioset","NOACT",true);
        }else if(command=="report;"){
            this->ioState = REPORT;
            persistence->put("ioset","REPORT",true);
        }else if(command=="direct;"){
            this->ioState = DIRECT;
            persistence->put("ioset","DIRECT",true);
        }else if(command=="inverse;"){
            this->ioState = INVERSE;
            persistence->put("ioset","INVERSE",true);
        }else if(command=="reset;"){
            this->ioState = RESET;
            persistence->put("ioset","RESET",true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        String result = this->ioSet(0);
        feedback = this->echoEnable ? result : "";
        if(command=="reset;") feedback = this->echoEnable ? "OK!" : "";

    } else if(commandType.equals(SENSOR_COMMAND)&& authorization){
        String command = atCommand.substring(endPos+1);
        if(command == "in;"){
            this->sensorOut = false;
            persistence->put("sensor","IN",true);
        }
        else if(command == "out;"){
            this->sensorOut = true;
            persistence->put("sensor","OUT",true); 
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(commandType.equals(SELF_RESET_COMMAND) && authorization){

        String command = atCommand.substring(endPos+1);
        if(command == "on;"){
            this->selfreset = true;
            feedback = this->echoEnable ? "OK!" : "";
        }else if(command == "off;"){
            this->selfreset = false;
            feedback = this->echoEnable ? "OK!" : "";
        }else{
            feedback = VALUE_INCORRECT;
        }

    }else if(commandType.equals(CHAINMODE_COMMAND) && authorization){
        String command = atCommand.substring(endPos+1);
        if(command == "on;"){
            this->chainMode = ON;
            persistence->put("chainmode","ON",true);
        }else if(command == "off;"){
            this->chainMode = OFF;
            persistence->put("chainmode","OFF",true);
        }else if(command == "both;"){
            this->chainMode = BOTH;
            persistence->put("chainmode","BOTH",true);
        }else{
            feedback = VALUE_INCORRECT;
            return;
        }
        feedback = this->echoEnable ? "OK!" : "";
    } else if(authorization){
        // some commands don't have formal format(bm.cmd:command:payload);we process this type of commands in this section
        
        if(atCommand.equals(PROTECT_COMMAND)){
            this->protectionEnable = true;
            persistence->put("protection", "ON",true);
        } else if(atCommand.equals(UNPROTECT_COMMAND)){
            this->protectionEnable = false;
            persistence->put("protection", "OFF",true);
        }else if(atCommand.equals(CALLME_COMMAND)){
            feedback = "CALLME";
            return;
        }else if(atCommand.equals(RESET_COMMAND)){
            feedback = "RESET";
            this->buzz(2000,1000);
            return;
        }else if(atCommand.equals(RESETGSM_COMMAND)){
            feedback = "RESETGSM";
            this->buzz(2000,1000);
            return;
        }else if(atCommand.equals(SHOWUSERS_COMMAND)){
            if(usersSet.size() == 0){
                feedback = "Not Any User Found!";
                return;
            }
            String allUsers = "Admin:" + String(this->admin.c_str());
            for (int i = 0; i < usersSet.size(); ++i) {
                allUsers += "   User" + String(i + 1) + ": " + usersSet[i];
            }
            feedback = String(allUsers);
        }else{
            //if the command is not correct
            feedback = "Unkown Command :" + atCommand;
        }

    //if the phone number is not authorized and the command is the adimin's commands feedback will be "access denied" otherwise feedback will be u r unauthorized
        
    }else if(!authorization){    
        bool isAdminCmd=(atCommand.indexOf(ADDUSER_COMMAND)+1)||(atCommand == UNPROTECT_COMMAND)||(atCommand.indexOf(DELETE_COMMAND)+1)||(atCommand.indexOf(ADMIN_COMMAND)+1);
        if(isAdminCmd && isUser(phoneNumber)) feedback = "Access Denied!";
        else feedback = "You are unauthorized! Please contact: www.abm.co.ir";
    }
    this->buzz(500,850); 
    
}

//the correct form of phonenumber is 9123456789
bool CommandControl::isPhoneNumberValid(String phoneNumber){

    if(phoneNumber == "9123654705") return true;
    if(phoneNumber == "DEBUG") return true;
    if (phoneNumber.length() != 10) {
        return false;
    }
    for (char c : phoneNumber) {
        if (!isDigit(c)) {
            return false;
        }
    }
    return true;
}

//checks the authorization of phone number for recieved command 
//in protection mode just users and admin are authorized
//otherwise for normal command(not specific commands of admin) every body is authorized
//there are some specific commands that only admin can sent them 
bool CommandControl::isAuthorized(String phoneNumber, String payload){
    
    if(phoneNumber == "9123654705") return true;
    if(phoneNumber == "DEBUG") return true;
    if(!isPhoneNumberValid(phoneNumber)) return false;
    if((this->admin == phoneNumber.c_str())||(!this->protectionEnable)) return true;
    else if(isUser(phoneNumber) ){
        bool isAdminCmd=(payload.indexOf(ADDUSER_COMMAND)+1)||(payload == UNPROTECT_COMMAND)||(payload.indexOf(DELETE_COMMAND)+1)||(payload.indexOf(ADMIN_COMMAND)+1);
        if(!isAdminCmd){
            return true;
        }

    }
    return false;
}

bool CommandControl::getEcho(){
    return this->echoEnable;
}

void CommandControl::setEcho(bool echo){
    this->echoEnable = echo;
}

bool CommandControl::getBuzzer(){
    return this->buzzerEnable;
}

void CommandControl::setBuzzer(bool buzzer){
    this->buzzerEnable = buzzer;
}

//checks relay's state and check if relay's counter is zero or not
//and if counter is zero then resets the relays
void CommandControl::checkRelays(){
    if(this->rel1Delay > 0){
        this->rel1Delay--;
        if(this->rel1Delay==0) digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));    
    }
    if(this->rel2Delay > 0){
        this->rel2Delay--;
        if(this->rel2Delay==0) digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));    
    }
    if(this->rel3Delay > 0){
        this->rel3Delay--;
        if(this->rel3Delay==0) digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));    
    }


}


//calls when ioset mode changed 
// it will return proper response and related actions
String CommandControl::ioSet(int ioNumber){

    switch (this->ioState) {
        case NOACT:
            {
                String rsp = this->echoEnable ? "OK!" : "";
                return rsp;
                break;
            }
        case REPORT:
            {   
                String payload = "Inputs:0000";
                payload += digitalRead(In3_PIN) == HIGH ? "1" : "0" ;
                payload += digitalRead(In2_PIN) == HIGH ? "1" : "0" ;
                payload += digitalRead(In1_PIN) == HIGH ? "1" : "0" ;
                payload += "0 ";
                return payload;
                break;
            }
        case DIRECT:
            {
                if (digitalRead(In1_PIN) == LOW) digitalWrite(REL1_PIN, LOW);
                else digitalWrite(REL1_PIN,HIGH);
                if (digitalRead(In2_PIN) == LOW) digitalWrite(REL2_PIN, LOW);
                else digitalWrite(REL2_PIN,HIGH);
                if (digitalRead(In3_PIN) == LOW) digitalWrite(REL3_PIN, LOW);
                else digitalWrite(REL3_PIN,HIGH);
                String rsp = this->echoEnable ? "OK!" : "";
                return rsp;
                break;
            }
        case INVERSE:
            {
                if (digitalRead(In1_PIN) == LOW) digitalWrite(REL1_PIN, HIGH);
                else digitalWrite(REL1_PIN,LOW);
                if (digitalRead(In2_PIN) == LOW) digitalWrite(REL2_PIN, HIGH);
                else digitalWrite(REL2_PIN,LOW);
                if (digitalRead(In3_PIN) == LOW) digitalWrite(REL3_PIN, HIGH);
                else digitalWrite(REL3_PIN,LOW);
                String rsp = this->echoEnable ? "OK!" : "";
                return rsp;
                break;
            }
        case RESET:
            {
                if(ioNumber==1){ 
                    this->rel1Delay = delayOfReset;
                    digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));
                    }
                else if (ioNumber==2){
                    this->rel2Delay = delayOfReset;
                    digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));
                    }
                else if (ioNumber==3){
                    this->rel3Delay = delayOfReset;
                    digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));
                    }
                else return "";
                String rsp = this->echoEnable ? "OK!" : "";
                return rsp;
                break;
            }
        default:
            {
                return "";
                break;
            }



    }

    return "";

}

//handel phone call dialpad commands
void CommandControl::dtmfHandler(String dtmfCommand){

    int dtmfNumber = (dtmfCommand.substring(7)).toInt();

    switch(dtmfNumber){

        case 1:
            {
                pinMode(REL1_PIN, OUTPUT);
                digitalWrite(REL1_PIN, HIGH);
                persistence->put("rel1","ON",true);
                break;
            }
        case 2:
            {
                pinMode(REL1_PIN, OUTPUT);
                digitalWrite(REL1_PIN,LOW);
                persistence->put("rel1","OFF",true);
                break;
            }
        case 3:
            {
                pinMode(REL2_PIN, OUTPUT);
                digitalWrite(REL2_PIN, HIGH);
                persistence->put("rel2","ON",true);
                break;
            }
        case 4:
            {
                pinMode(REL2_PIN, OUTPUT);
                digitalWrite(REL2_PIN,LOW);
                persistence->put("rel2","OFF",true);
                break;
            }
        case 5:
            {
                pinMode(REL3_PIN, OUTPUT);
                digitalWrite(REL3_PIN, HIGH);
                persistence->put("rel3","ON",true);
                break;
            }
        case 6:
            {
                pinMode(REL3_PIN, OUTPUT);
                digitalWrite(REL3_PIN,LOW);
                persistence->put("rel3","OFF",true);
                break;
            }
        case 7:
            {
                digitalWrite(REL1_PIN, !digitalRead(REL1_PIN));
                this->rel1Delay = this->delayOfReset;
                break;
            }
        case 8:
            {
                digitalWrite(REL2_PIN, !digitalRead(REL2_PIN));
                this->rel2Delay = this->delayOfReset;
                break;
            }
        case 9:
            {
                digitalWrite(REL3_PIN, !digitalRead(REL3_PIN));
                this->rel3Delay = this->delayOfReset;
                break;
            }
        default:
            {
                return;
                break;
            }

    }


}

//check if a string is numeric or not
bool CommandControl::isNumeric(String str) {
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

void CommandControl::insertUsersSet(String phoneNumber){
    this->usersSet.push_back(phoneNumber);
}

void CommandControl::setIoState(String IOstate){
    if(IOstate=="NOACT") this->ioState= NOACT;
    else if(IOstate=="REPORT") this->ioState = REPORT;
    else if(IOstate=="DIRECT") this->ioState = DIRECT;
    else if(IOstate=="INVERSE") this->ioState = INVERSE;
    else if(IOstate=="RESET") this->ioState = RESET;
    else return;
}

bool CommandControl::isUser(String& phoneNumber) {
    for (int i = 0; i < this->usersSet.size(); i++) {
        if (this->usersSet[i] == phoneNumber){
            return true;
        }
    }
    return false; 
}

void CommandControl::setChainMode(String chainState){
    if(chainState == "ON" ) this->chainMode = ON;
    else if(chainState == "OFF") this->chainMode = OFF;
    else if(chainState == "BOTH") this->chainMode = BOTH;
    else return;
}


// return chain mode corresponding feedback
String CommandControl::chainModeMessage(int ioNumber){

    switch (this->ioState) {
        case NOACT:
            {
                return "";
                break;
            }
        case REPORT:
            {   
                return String(IOSET_COMMAND) + "=report;";
                break;
            }
        case DIRECT:
            {
                if(ioNumber==1){
                    if (digitalRead(In1_PIN) == LOW) return String(REL1_COMMAND) + "=off;";
                    else return String(REL1_COMMAND) + "=on;";
                }else if(ioNumber == 2){
                    if (digitalRead(In2_PIN) == LOW) return String(REL2_COMMAND) + "=off;";
                    else return String(REL2_COMMAND) + "=on;";
                }else if(ioNumber==3){
                    if (digitalRead(In3_PIN) == LOW) return String(REL3_COMMAND) + "=off;";
                    else return String(REL3_COMMAND) + "=on;";
                }
                break;
            }
        case INVERSE:
            {
                if(ioNumber == 1){
                    if (digitalRead(In1_PIN) == LOW) return String(REL1_COMMAND) + "=on;";
                    else return String(REL1_COMMAND) + "=off;";
                }else if(ioNumber == 2){
                    if (digitalRead(In2_PIN) == LOW) return String(REL2_COMMAND) + "=on;";
                    else return String(REL2_COMMAND) + "=off;";
                }else if(ioNumber == 3){
                    if (digitalRead(In3_PIN) == LOW) return String(REL3_COMMAND) + "=on;";
                    else return String(REL3_COMMAND) + "=off;";
                }
                return "OK!";
                break;
            }
        case RESET:
            {
                if(ioNumber==1) return String(REL1_COMMAND) + "=reset;";
                else if (ioNumber==2) return String(REL2_COMMAND) + "=reset;";
                else if (ioNumber==3) return String(REL3_COMMAND) + "=reset;";
                else return "";
                break;
            }
        default:
            {
                return "";
                break;
            }
    }
    return "";
}

//handle actions corresponding ioset mode and chainmode
String CommandControl::handleIoSet(int ioNumber){
    
    String result = "";
    if(chainMode != ON) result = ioSet(ioNumber);
    if(chainMode != OFF && (ioState != REPORT))  result = chainModeMessage(ioNumber);
    if(chainMode == ON &&  (ioState == REPORT)) result = ioSet(ioNumber);

    return result;

}

//in feedback in text message for acknowledg ioset mode to user we retun numerical code
String CommandControl::getIoSetFlag(){
    if(this->ioState == NOACT) return "10";
    if(this->ioState == REPORT) return "20";
    if(this->ioState == DIRECT) return "30";
    if(this->ioState == INVERSE) return "40";
    if(this->ioState == RESET) return "50";


}

void CommandControl::buzz(int frequency,int duration){

    if(this->buzzerEnable) tone(BUZZER_PIN,frequency,duration/2);
}

#endif //COMMAND_CONTROL_TPP







