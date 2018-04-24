#include "list_private.h"
#include "list_test.h"

ListEntryType ListEntry_Buffer[LIST_ENTRY_NUM];
ListEntryType *free_headp = NULL_PTR;
ListEntryType *allocated_headp = NULL_PTR;
char ListEntry_DataBuffer[LIST_ENTRY_NUM][LIST_ENTRY_DATA_SIZE];

void list_private_init(void)
{
    int i;
    ListEntryType *p = &ListEntry_Buffer[0];

    for (i = 0; i < LIST_ENTRY_NUM; i++) {
        if (i == 0) {
            p[i].next = &p[i + 1];
            p[i].prev = &p[LIST_ENTRY_NUM - 1];
        }
        else if (i == (LIST_ENTRY_NUM - 1)) {
            p[i].next = &p[0];
            p[i].prev = &p[i -1];
        }
        else {
            p[i].next = &p[i + 1];
            p[i].prev = &p[i -1];
        }
        p[i].status = ListEntryStatus_FREE;
        p[i].data = &ListEntry_DataBuffer[i][0];
    }
    free_headp = &p[0];
    allocated_headp = NULL_PTR;
    return;
}

#define ENTRY_NEXT(entryp)    ((ListEntryType*)(entryp)->next)
#define ENTRY_PREV(entryp)    ((ListEntryType*)(entryp)->prev)
#define HEAD_PTR(headpp)        (*(headpp))


/*
 * <victim_prev> <victim> <victim_next>
 *                  |
 *                  v
 */
void list_private_remove(ListEntryType **headpp, ListEntryType *entryp) 
{
    ListEntryType *victim;
    ListEntryType *victim_next;
    ListEntryType *victim_prev;

    if (HEAD_PTR(headpp) != NULL_PTR) {
        victim = entryp;
        victim_next = ENTRY_NEXT(victim);
        victim_prev = ENTRY_PREV(victim);

        victim_prev->next = victim_next;
        victim_next->prev = victim_prev;

        if (HEAD_PTR(headpp) == victim) {
            if (victim != victim_next) {
                HEAD_PTR(headpp) = victim_next;
            }
            else {
               /*
                * last entry
                */
                HEAD_PTR(headpp) = NULL_PTR;
            }
        }
        victim->next = victim;
        victim->prev = victim;
    }
    return;
}

ListEntryType *list_private_remove_head(ListEntryType **headpp) 
{
    ListEntryType *victim;

    victim = HEAD_PTR(headpp);
    if (victim != NULL_PTR) {
        list_private_remove(headpp, victim);
    }
    return victim;
}


/*
 *                     entryp
 *                       |
 *                       v
 * <target_prev> <target> <target_next>
 */
void list_private_add_next(ListEntryType **headpp, ListEntryType *target, ListEntryType *entryp) 
{
    entryp->next = target->next;
    entryp->prev = target;

    ENTRY_NEXT(target)->prev = entryp;
    target->next = entryp;
    return;
}

/*
 *            entryp
 *              |
 *              v
 * <target_prev> <target> <target_next>
 */
void list_private_add_prev(ListEntryType **headpp, ListEntryType *target, ListEntryType *entryp) 
{
    entryp->next = target;
    entryp->prev = target->prev;

    ENTRY_PREV(target)->next = entryp;
    target->prev = entryp;
    return;
}

void list_private_add_head(ListEntryType **headpp, ListEntryType *entryp) 
{
    ASSERT(entryp->next == entryp);
    ASSERT(entryp->prev == entryp);

    if (HEAD_PTR(headpp) != NULL_PTR) {
        list_private_add_prev(headpp, HEAD_PTR(headpp), entryp);
    }
    HEAD_PTR(headpp) = entryp;
    return;
}


void list_private_add_tail(ListEntryType **headpp, ListEntryType *entryp) 
{
    ASSERT(entryp->next == entryp);
    ASSERT(entryp->prev == entryp);

    if (HEAD_PTR(headpp) != NULL_PTR) {
        list_private_add_prev(headpp, HEAD_PTR(headpp), entryp);
    }
    else {
        HEAD_PTR(headpp) = entryp;
    }
    return;
}
