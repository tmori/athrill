#ifndef _LIST_PRIVATE_H_
#define _LIST_PRIVATE_H_

#include "list.h"

typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;

typedef enum {
    ListEntryStatus_FREE = 0,
    ListEntryStatus_ALLOCATED,
} ListEntryStatusType;

typedef struct ListEntry {
    void *next;
    void *prev;
    void *data;
    ListEntryStatusType status;
} ListEntryType;

/*
 * free list header
 */
extern ListEntryType *free_headp;

/*
 * allocated list header
 */
extern ListEntryType *allocated_headp;

/*
 * list entry buffer
 */ 
extern ListEntryType ListEntry_Buffer[LIST_ENTRY_NUM];

/*
 * list entry data buffer
 */
extern char ListEntry_DataBuffer[LIST_ENTRY_NUM][LIST_ENTRY_DATA_SIZE];
#define LIST_ENTRY_DATA_BUFFER_SIZE             ((uint32)(LIST_ENTRY_NUM * LIST_ENTRY_DATA_SIZE))
#define LIST_ENTRY_DATA_BUFFER_START_ADDR       ((uint32)(&ListEntry_DataBuffer[0][0]))
#define LIST_ENTRY_DATA_BUFFER_END_ADDR         ((uint32)(LIST_ENTRY_DATA_BUFFER_START_ADDR + LIST_ENTRY_DATA_BUFFER_SIZE))

/*
 * initialize list
 * 
 * free_headp ==> first entry
 * allocated_headp ==> NULL_PTR
 * 
 * ListEntry_Buffer[i].next = next entry address
 * ListEntry_Buffer[i].prev = prev entry address
 * ListEntry_Buffer[i].data = &ListEntry_DataBuffer[i][0]
 * ListEntry_Buffer[i].status = ListEntryStatus_FREE
 */
extern void list_private_init(void);

/*
 * remove the top entry from headp
 * 
 * returned entry's next/prev must point the entry itself.
 */
extern ListEntryType *list_private_remove_head(ListEntryType **headpp);
/*
 * add entryp on the top of headp
 * 
 * caller must gurantee entryp's next/prev point the entryp itself
 */
extern void list_private_add_head(ListEntryType **headpp, ListEntryType *entryp);
/*
 * remove the entryp from headp
 * 
 * returned entry's next/prev must point the entry itself.
 */
extern void list_private_remove(ListEntryType **headpp, ListEntryType *entryp);
/*
 * add entryp on the last of headp
 * 
 * caller must gurantee entryp's next/prev point the entryp itself
 */
extern void list_private_add_tail(ListEntryType **headpp, ListEntryType *entryp);

/*
 *                     entryp
 *                       |
 *                       v
 * <target_prev> <target> <target_next>
 */
extern void list_private_add_next(ListEntryType **headpp, ListEntryType *target, ListEntryType *entryp);


/*
 *            entryp
 *              |
 *              v
 * <target_prev> <target> <target_next>
 */
extern void list_private_add_prev(ListEntryType **headpp, ListEntryType *target, ListEntryType *entryp);

#endif /* _LIST_PRIVATE_H_ */