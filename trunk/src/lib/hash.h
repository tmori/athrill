#ifndef _HASH_H_
#define _HASH_H_

#include "std_types.h"

#define HASH_VALUE_CONTAINER_SIZE	4
typedef struct {
	uint32 rawdata;
	uint8  data[HASH_VALUE_CONTAINER_SIZE];
} HashValueType;
typedef uint32 HashIdType;

#define HASHID_MIN	0
#define HASHID_MAX	0
#define HASHID_NUM	1
#define HASHSIZE	119U

extern void hash_init(void);
extern HashValueType* hash_search(HashIdType id, HashValueType *value);
extern void hash_add(HashIdType id, HashValueType *value);

extern HashValueType* hash_first(HashIdType id);
extern HashValueType* hash_next(HashIdType id);


#endif /* _HASH_H_ */
