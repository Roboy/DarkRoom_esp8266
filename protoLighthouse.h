#ifndef PROTOLIGHTHOUSE_H
#define PROTOLIGHTHOUSE_H

#include "logging.h"
#include "pb.h"
#include "lighthouse_sensor.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

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
    bool encode_trackedObjConfig(uint32_t ip, uint16_t cmndPort_l, pb_byte_t *buffer, size_t &msg_len );
    bool encode_loggingObject(const char *msg, pb_byte_t *buffer, size_t &msg_len);
    bool enable_logging = true;
	mkr1000_lighthouse_loggingObject            loggingObjMsg; 
	mkr1000_lighthouse_commandObject            commandObjMsg; 
	mkr1000_lighthouse_configObject             configObjMsg; 
};

#endif
