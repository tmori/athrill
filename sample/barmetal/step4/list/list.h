#ifndef _LIST_H_
#define _LIST_H_

#ifndef NULL_PTR
#define NULL_PTR    ((void*)0)
#endif /* NULL_PTR */

#define LIST_ENTRY_NUM          3
#define LIST_ENTRY_DATA_SIZE    16

extern void list_init(void);


/*
 * return allocated data from free list.
 * The returned entry is not on the free list, but on the allocated list.
 * 
 * if free list is empty, then return NULL_PTR
 */ 
extern void* my_alloc(void);


/*
 * add entryp on the free list.
 * 
 * if entryp is valid, return 0.
 * if entryp is not valid, return -1 and free list is not changed.
 */
extern int my_free(void *entryp);


#endif /* _LIST_H_ */