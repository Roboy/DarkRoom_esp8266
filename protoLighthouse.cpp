#include "protoLighthouse.h"

PROTO_LOVE::PROTO_LOVE()
{
    clearProtos(); 
}   

void PROTO_LOVE::clearProtos()
{
    configObjMsg        = DarkRoomProtobuf_configObject_init_zero; 
    commandObjMsg       = DarkRoomProtobuf_commandObject_init_zero; 
    configObjMsg        = DarkRoomProtobuf_configObject_init_zero; 
    imuObjMsg           = DarkRoomProtobuf_imuObject_init_zero; 
} 

bool PROTO_LOVE::decode_config_Proto(pb_byte_t * buffer, size_t rcvd_msg_len)
{
    LOG(logDEBUG, "decode config Message Proto"); 

    pb_istream_t stream = pb_istream_from_buffer(buffer, rcvd_msg_len); 
    bool status = pb_decode(&stream, DarkRoomProtobuf_configObject_fields, &configObjMsg);  

    if(status){
        LOG(logDEBUG, "decoded Config Message Proto"); 
    }else{
        LOG(logERROR, "decoding failed"); 
    }
    return status; 
}

bool PROTO_LOVE::decode_command_Proto(pb_byte_t * buffer, size_t rcvd_msg_len)
{
    LOG(logDEBUG, "decode command Message Proto"); 

    pb_istream_t stream = pb_istream_from_buffer(buffer, rcvd_msg_len); 
    bool status = pb_decode(&stream, DarkRoomProtobuf_commandObject_fields, &commandObjMsg);  

    if(status){
        LOG(logDEBUG, "decoded command Message Proto"); 
    }else{
        LOG(logERROR, "decoding failed"); 
    }
    return status; 
}

bool PROTO_LOVE::encode_trackedObjConfig(uint32_t ip, uint16_t cmndPort_l, pb_byte_t *buffer, size_t &msg_len )
{
    DarkRoomProtobuf_trackedObjectConfig trackedObjConfMsg; 
    LOG(logVERBOSE, "Encode Tracked Object Config"); 
    LOG_d(logVERBOSE, "Local IP Address of the MKR: ", ip); 
    LOG_d(logVERBOSE, "Local Command Port from the MKR: ", cmndPort_l); 

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, 512); 
  
    trackedObjConfMsg.ip = ip;  
    trackedObjConfMsg.command_port = cmndPort_l; 
    bool status = pb_encode(&stream, DarkRoomProtobuf_trackedObjectConfig_fields, &trackedObjConfMsg); 
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
    bool status = pb_encode(&stream, DarkRoomProtobuf_loggingObject_fields, &loggingObjMsg); 
    msg_len = stream.bytes_written; 

    if(!status)
    {
        LOG(logERROR, "Encoding failed!"); 
        Serial.println(PB_GET_ERROR(&stream)); 
    }
    return status;
}

bool PROTO_LOVE::encode_imuObjConfig(Quaternion &q, VectorInt16 &acc, VectorFloat &gravity, pb_byte_t *buffer, size_t &msg_len ){
    imuObjMsg = DarkRoomProtobuf_imuObject_init_zero; 
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, 512); 
  
    imuObjMsg.quaternion[0]=q.x;
    imuObjMsg.quaternion[1]=q.y;
    imuObjMsg.quaternion[2]=q.z;
    imuObjMsg.quaternion[3]=q.w;
    imuObjMsg.quaternion_count = 4;
    imuObjMsg.acc[0]=acc.x;
    imuObjMsg.acc[1]=acc.y;
    imuObjMsg.acc[2]=acc.z;
    imuObjMsg.acc_count = 3;
    imuObjMsg.gravity[0]=gravity.x;
    imuObjMsg.gravity[1]=gravity.y;
    imuObjMsg.gravity[2]=gravity.z;
    imuObjMsg.gravity_count = 3;
    bool status = pb_encode(&stream, DarkRoomProtobuf_imuObject_fields, &imuObjMsg); 
    msg_len = stream.bytes_written; 
    LOG_d(logVERBOSE, "Message len encode protobuf: ", msg_len);  
 
    if(!status)
    {
        LOG(logERROR, "Encoding failed!"); 
        Serial.println(PB_GET_ERROR(&stream)); 
    }
    return status;
}

