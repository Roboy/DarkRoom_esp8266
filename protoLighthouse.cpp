#include "protoLighthouse.h"

PROTO_LOVE::PROTO_LOVE()
{
    clearProtos(); 
}   

void PROTO_LOVE::clearProtos()
{
    configObjMsg        = mkr1000_lighthouse_configObject_init_zero; 
    commandObjMsg       = mkr1000_lighthouse_commandObject_init_zero; 
    configObjMsg        = mkr1000_lighthouse_configObject_init_zero; 
} 

bool PROTO_LOVE::decode_config_Proto(pb_byte_t * buffer, size_t rcvd_msg_len)
{
    LOG(logINFO, "decode config Message Proto"); 

    pb_istream_t stream = pb_istream_from_buffer(buffer, rcvd_msg_len); 
    bool status = pb_decode(&stream, mkr1000_lighthouse_configObject_fields, &configObjMsg);  

    if(status){
        LOG(logINFO, "decoded Config Message Proto: "); 
    }else{
        LOG(logERROR, "decoding failed"); 
    }
    return status; 
}

bool PROTO_LOVE::encode_trackedObjConfig(uint32_t ip, uint16_t cmndPort_l, pb_byte_t *buffer, size_t &msg_len )
{
    mkr1000_lighthouse_trackedObjectConfig trackedObjConfMsg; 
    LOG(logVERBOSE, "Encode Tracked Object Config"); 
    LOG_d(logVERBOSE, "Local IP Address of the MKR: ", ip); 
    LOG_d(logVERBOSE, "Local Command Port from the MKR: ", cmndPort_l); 

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, 512); 
  
    trackedObjConfMsg.ip = ip;  
    trackedObjConfMsg.command_port = cmndPort_l; 
    bool status = pb_encode(&stream, mkr1000_lighthouse_trackedObjectConfig_fields, &trackedObjConfMsg); 
    msg_len = stream.bytes_written; 
    LOG_d(logVERBOSE, "Message len encode protobuf: ", msg_len);  
 
    if(!status)
    {
        LOG(logERROR, "Encoding failed!"); 
        Serial.println(PB_GET_ERROR(&stream)); 
    }
    return status;
}

bool PROTO_LOVE::encode_loggingObject(const char * msg, pb_byte_t *buffer, size_t &msg_len )
{
    LOG(logINFO, "Encode Logging Object"); 

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, 512); 
  
    strncpy(loggingObjMsg.message, msg, sizeof loggingObjMsg.message);    
    bool status = pb_encode(&stream, mkr1000_lighthouse_loggingObject_fields, &loggingObjMsg); 
    msg_len = stream.bytes_written; 

    if(!status)
    {
        LOG(logERROR, "Encoding failed!"); 
        Serial.println(PB_GET_ERROR(&stream)); 
    }
    return status;
}
