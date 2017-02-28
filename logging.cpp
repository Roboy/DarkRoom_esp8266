#include "logging.h"

static const char * LEVEL_STRING[] = {
    FOREACH_LEVEL(GENERATE_STRING)
}; 

static LogString * logString; 
static TLogLevel currentLoglevel = logINFO;

static void printWiFiUDP()
{
    Serial.print("WiFi print:"); 
    Serial.println(logString->buff); 
}

static void printSerial()
{
    Serial.println(logString->buff); 
}

static void setLoggingLevel(TLogLevel newLevel)
{
    currentLoglevel = newLevel; 
}

static void setFlushInterface(FlushInterface in)
{
    switch(in){
        case F_SERIAL:
            (logString->flushI) = printSerial;
            break;
        case F_WIFI:
            (logString->flushI) = printWiFiUDP;
            break; 
    }
}

static LogString * createLogString(char * input, TLogLevel level)
{
    LogString * thisLog; 
    thisLog = static_cast<LogString*>(malloc( sizeof *thisLog ));
    if (NULL != thisLog)
    { 
        thisLog->length = strlen(input) + 1; 
        thisLog->messageLevel = level; 

        thisLog->buff = static_cast<char*>(malloc(thisLog->length));

        if( NULL != thisLog->buff)    
        {
            strncpy(thisLog->buff, input, thisLog->length); 
        }else{
            free(thisLog);
            thisLog = NULL; 
        }
    }
    return thisLog;
}

static void deleteLogString()
{
    free(logString->buff); 
    free(logString); 
    logString = NULL; 
}

static void appendLogString(const char * const append)
{
    char baseString[logString->length + 1];
    strncpy(baseString, logString->buff, logString->length +1);  
    logString->length = strlen(logString->buff)  + strlen(append) + 1;  
    free(logString->buff); 
    logString->buff= static_cast<char*>(malloc (logString->length)); 
    if( NULL != logString->buff){
        logString->buff[0] = '\0';
        strcat(logString->buff, baseString); 
        strcat(logString->buff, append); 
    }
}

static TLogLevel GetReportingLevel(void)
{
    return currentLoglevel; 
}

static void GetLogString(TLogLevel level, const char * msg)
{
    char time[20];
    snprintf(time, 20, "%ld", micros()); 

    char logMessage [strlen(msg)+1]; 
    strncpy(logMessage, msg, strlen(msg)+1); 

    logString = createLogString(time, logDEBUG); 
    appendLogString("\t"); 
    appendLogString(LEVEL_STRING[level]); 
    appendLogString("\t"); 
    appendLogString(logMessage); 
    appendLogString("\t"); 

    setFlushInterface(F_SERIAL); 
    logString->flushI(); 
    deleteLogString(); 
}

static void MakeldLogString(TLogLevel level, const char * msg, long int number)
{
    char time[20];
    snprintf(time, 20, "%ld", micros()); 

    char logMessage [strlen(msg)+1]; 
    strncpy(logMessage, msg, strlen(msg)+1); 

    char logNumber [20]; 
    snprintf(logNumber, 20, "%ld", number); 

    logString = createLogString(time, logDEBUG); 
    appendLogString("\t"); 
    appendLogString(LEVEL_STRING[level]); 
    appendLogString("\t"); 
    appendLogString(logMessage); 
    appendLogString("\t"); 
    appendLogString(logNumber); 

    setFlushInterface(F_SERIAL); 
    logString->flushI(); 
    deleteLogString(); 
}

static void MakedLogString(TLogLevel level,  const char * msg, int number)
{
    char time[20];
    snprintf(time, 20, "%ld", micros()); 

    char logMessage [strlen(msg)+1]; 
    strncpy(logMessage, msg, strlen(msg)+1); 

    char logNumber [20]; 
    snprintf(logNumber, 20, "%d", number); 

    logString = createLogString(time, logDEBUG); 
    appendLogString("\t"); 
    appendLogString(LEVEL_STRING[level]); 
    appendLogString("\t"); 
    appendLogString(logMessage); 
    appendLogString("\t"); 
    appendLogString(logNumber); 

    setFlushInterface(F_SERIAL); 
    logString->flushI(); 
    deleteLogString(); 
}

static void MakefLogString(TLogLevel level, const char * msg, float number)
{
    char time[20];
    snprintf(time, 20, "%ld", micros()); 

    char logMessage [strlen(msg)+1]; 
    strncpy(logMessage, msg, strlen(msg)+1); 

    char logNumber [20]; 
    snprintf(logNumber, 20, "%f", number); 

    logString = createLogString(time, logDEBUG); 
    appendLogString("\t"); 
    appendLogString(LEVEL_STRING[level]); 
    appendLogString("\t"); 
    appendLogString(logMessage); 
    appendLogString("\t"); 
    appendLogString(logNumber); 

    setFlushInterface(F_SERIAL); 
    logString->flushI(); 
    deleteLogString(); 
}

LogI const logi = {GetLogString, MakefLogString, MakeldLogString, MakedLogString, GetReportingLevel, setFlushInterface, setLoggingLevel }; 
