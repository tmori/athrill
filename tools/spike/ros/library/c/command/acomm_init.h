#ifndef _ACOMM_INIT_H_
#define _ACOMM_INIT_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "athrill_comm_config.h"

extern acomm_bus_metadata_type *acomm_open(char *path);
extern void acomm_close(acomm_bus_metadata_type *p);

#ifdef __cplusplus
}
#endif

#endif /* _ACOMM_INIT_H_ */