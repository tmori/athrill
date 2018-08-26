#ifndef _ACOMM_INIT_H_
#define _ACOMM_INIT_H_

extern acomm_bus_metadata_type *acomm_open(char *path);
extern void acomm_close(acomm_bus_metadata_type *p);

#endif /* _ACOMM_INIT_H_ */