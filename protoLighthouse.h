#ifndef PROTOLIGHTHOUSE_H
#define PROTOLIGHTHOUSE_H

#include "logging.h"
#include "pb.h"
#include "lighthouse.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "helper_3dmath.h"

typedef enum _ES{
    ES_WIFI_FAIL_INIT_NO_SHIELD = 1,
    ES_WIFI_FAIL_INIT_CANNOT_CONNECT,
    ES_WIFI_FAIL_UDP_SOCKET,
    ES_PROTO_FAIL_INIT,
    ES_PROTO_FAIL_ENCODE,
    ES_PROTO_FAIL_DECODE,
    ES_PROTO_ERROR,
    ES_PROTO_SUCCESS,
    ES_WIFI_SUCCESS, 
    ES_WIFI_ERROR,
}ES_PROTO;

class PROTO_LOVE{
public:
    PROTO_LOVE(); 
    void clearProtos();
    bool decode_config_Proto(pb_byte_t * buffer, size_t rcvd_msg_len);    
    bool decode_command_Proto(pb_byte_t * buffer, size_t rcvd_msg_len);
    bool encode_trackedObjConfig(uint32_t ip, uint16_t cmndPort_l, pb_byte_t *buffer, size_t &msg_len );
    bool encode_loggingObject(const char *msg, pb_byte_t *buffer, size_t &msg_len);
    bool encode_imuObjConfig(Quaternion &q, VectorInt16 &acc, VectorFloat &gravity, pb_byte_t *buffer, size_t &msg_len );
    bool enable_logging = true;
	DarkRoomProtobuf_loggingObject            loggingObjMsg; 
	DarkRoomProtobuf_commandObject            commandObjMsg; 
	DarkRoomProtobuf_configObject             configObjMsg; 
  DarkRoomProtobuf_imuObject                imuObjMsg; 
};

#endif
