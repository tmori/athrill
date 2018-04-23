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

extern ListEntryType *free_headp;
extern ListEntryType *allocated_headp;
extern ListEntryType ListEntry_Buffer[LIST_ENTRY_NUM];
extern char ListEntry_DataBuffer[LIST_ENTRY_NUM][LIST_ENTRY_DATA_SIZE];
#define LIST_ENTRY_DATA_BUFFER_SIZE             ((uint32)(LIST_ENTRY_NUM * LIST_ENTRY_DATA_SIZE))
#define LIST_ENTRY_DATA_BUFFER_START_ADDR       ((uint32)(&ListEntry_DataBuffer[0][0]))
#define LIST_ENTRY_DATA_BUFFER_END_ADDR         ((uint32)(LIST_ENTRY_DATA_BUFFER_START_ADDR + LIST_ENTRY_DATA_BUFFER_SIZE))

extern void list_private_init(void);
extern ListEntryType *list_private_remove_head(ListEntryType *headp);
extern void list_private_add_head(ListEntryType *headp, ListEntryType *entryp);
extern void list_private_remove(ListEntryType *headp, ListEntryType *entryp);
extern void list_private_add_tail(ListEntryType *headp, ListEntryType *entryp);

#endif /* _LIST_PRIVATE_H_ */