#ifndef _ATHRILL_COMM_H_
#define _ATHRILL_COMM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "athrill_comm_types.h"
#include "athrill_comm_error.h"


extern acomm_rtype athrill_comm_init(void);

/*
 * QueueOperations
 */
extern acomm_rtype athrill_comm_send(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_send_force(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_send_uniq(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_recv(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_peek(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_is_exist(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_bool athrill_comm_is_arrived(acomm_busid busid, acomm_elmid elmid);

/*
 * DataOperations
 */
extern acomm_rtype athrill_comm_write(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);
extern acomm_rtype athrill_comm_read(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size);

#ifdef __cplusplus
}
#endif
#endif /* _ATHRILL_COMM_H_ */
