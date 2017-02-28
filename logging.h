#ifndef LOGGING_H
#define LOGGING_H

#include "Arduino.h"

static bool enableLogging = true; 

#define FOREACH_LEVEL(LEVEL) \
    LEVEL(logERROR) \
    LEVEL(logWARNING) \
    LEVEL(logDEBUG) \
    LEVEL(logINFO) \
    LEVEL(logVERBOSE) \
    LEVEL(logVERBOSE_1) \
    LEVEL(logVERBOSE_2) \
    LEVEL(logVERBOSE_3) \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

typedef enum TLogLevel 
{
    FOREACH_LEVEL(GENERATE_ENUM)
}TLogLevel; 

typedef enum FlushInterface{
    F_SERIAL  = 1, 
    F_WIFI    = 2, 
}FlushInterface;

typedef struct LogI
{
    void                    ( * const GetLogString)(TLogLevel, const char *); 
    void                    ( * const MakefLogString)(TLogLevel, const char *, float); 
    void                    ( * const MakeldLogString)(TLogLevel, const char *, long int ); 
    void                    ( * const MakedLogString)(TLogLevel,  const char *, int ); 
    TLogLevel               ( * const GetReportingLevel)(void); 
    void                    ( * const setFlushInterface)(FlushInterface);
    void                    ( * const setLoggingLevel)(TLogLevel); 
}LogI; 

extern LogI const logi; 

typedef void (* FlushI)(void); 

typedef struct LogString
{
    size_t length; 
    char * buff; 
    TLogLevel messageLevel;  
    FlushI flushI; 
}LogString; 

#define LOG(level, message) \
    if (level > (logi.GetReportingLevel()) || (false == enableLogging)); \
    else logi.GetLogString(level, message)

#define LOG_ld(level, message, number) \
    if (level > (logi.GetReportingLevel()) || (false == enableLogging)); \
    else logi.MakeldLogString(level, message, number)

#define LOG_d(level, message, number ) \
    if (level > (logi.GetReportingLevel()) || (false == enableLogging)); \
    else logi.MakedLogString(level, (message), (number))

#define LOG_f(level, message, number ) \
    if (level > (logi.GetReportingLevel()) || (false == enableLogging)); \
    else logi.MakefLogString(level, message, number)


#endif
