#ifndef _ATHRILL_COMM_IMAGE_H_
#define _ATHRILL_COMM_IMAGE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "athrill_comm_types.h"
#include "athrill_comm_error.h"

extern void athrill_comm_make_image(void);
extern int athrill_comm_generate_image(const char *generate_path);

#ifdef __cplusplus
}
#endif

#endif /* _ATHRILL_COMM_IMAGE_H_ */