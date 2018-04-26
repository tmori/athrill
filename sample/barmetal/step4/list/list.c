#include "list_private.h"

void list_init(void)
{
    list_private_init();
}
void* my_alloc(void)
{
    ListEntryType *p;
    if (free_headp == NULL_PTR) {
        return NULL_PTR;
    }
    p = list_private_remove_head(&free_headp);
    list_private_add_head(&allocated_headp, p);
    p->status = ListEntryStatus_ALLOCATED;

    return p->data;
}
int my_free(void *data)
{
    ListEntryType *p;
    uint32 rel_off;
    uint32 rel_inx;

    if (( ((uint32)data) < LIST_ENTRY_DATA_BUFFER_START_ADDR)
            && ( ((uint32)data) >= LIST_ENTRY_DATA_BUFFER_END_ADDR)) {
        return -1;
    }
    rel_off = ((uint32)data) - LIST_ENTRY_DATA_BUFFER_START_ADDR;
    if ((rel_off % LIST_ENTRY_DATA_SIZE) != 0) {
        return -1;
    }
    rel_inx = rel_off / LIST_ENTRY_DATA_SIZE;
    p = &ListEntry_Buffer[rel_inx];
    if (p->status != ListEntryStatus_ALLOCATED) {
        return -1;
    }
    list_private_remove(&allocated_headp, p);
    list_private_add_tail(&free_headp, p);
    p->status = ListEntryStatus_FREE;

    return 0;
}