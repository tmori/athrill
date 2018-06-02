#include "list_private.h"
#include "list_test.h"

ListEntryType ListEntry_Buffer[LIST_ENTRY_NUM];
ListEntryType *free_headp = NULL_PTR;
ListEntryType *allocated_headp;
char ListEntry_DataBuffer[LIST_ENTRY_NUM][LIST_ENTRY_DATA_SIZE];

void list_private_init(void){
    
    int e_num;
	free_headp = &ListEntry_Buffer[0];

    for(e_num=0; e_num < LIST_ENTRY_NUM; e_num++){

       if (e_num == 0) {
           free_headp[e_num].next   = &free_headp[e_num+1]; 
           free_headp[e_num].prev   = &free_headp[LIST_ENTRY_NUM - 1];
       }
       else if (e_num == (LIST_ENTRY_NUM - 1)) {
           free_headp[e_num].next   = &free_headp[0];
           free_headp[e_num].prev   = &free_headp[e_num - 1];
       }
       else {
           free_headp[e_num].next   = &free_headp[e_num+1];
           free_headp[e_num].prev   = &free_headp[e_num - 1];
       } 
       free_headp[e_num].data   = &ListEntry_DataBuffer[e_num][0];
       free_headp[e_num].status = ListEntryStatus_FREE;
    }
    allocated_headp = NULL_PTR;
}

#define ENTRY_NEXT(entryp)    ((ListEntryType*)(entryp)->next)
#define ENTRY_PREV(entryp)    ((ListEntryType*)(entryp)->prev)
#define HEAD_PTR(headpp)        (*(headpp))


/*
 * victim 犠牲者って意味
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

   /* if((*headpp) == NULL_PTR){
        return 0;
    }*/

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

   /* if((*headpp) == NULL_PTR || target==NULL_PTR ||entryp == NULL_PTR){
        return;
    }*/

    entryp->next = target;
    entryp->prev = target->prev;

    ENTRY_PREV(target)->next = entryp;
    target->prev = entryp;
    return;
}

void list_private_add_head(ListEntryType **headpp, ListEntryType *entryp) 
{
    /*if((*headpp) == NULL_PTR ){
        return;
    }*/

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

   /* if((*headpp) == NULL_PTR || entryp == NULL_PTR){
        return;
    }*/

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

ListEntryType *GetlistEntry(ListEntryType *data)
{
    uint32 top;
    uint32 bottom;
    uint32 rela;
    uint32 line,row;

    top    = (uint32)(&ListEntry_DataBuffer[0][0]);
    bottom = (uint32)(&ListEntry_DataBuffer[LIST_ENTRY_NUM-1][LIST_ENTRY_DATA_SIZE-1]);



    if((uint32)(data) >= top && (uint32)(data) <= bottom){
           
        rela = ((uint32)(data) - top);
        line = rela / LIST_ENTRY_DATA_SIZE;
        row  = rela % LIST_ENTRY_DATA_SIZE;  

        if(row>=0 && row<=15){
            if(line == 0 && row>=0 && row<=15){
                return &ListEntry_Buffer[0];
            }
            else if(line == 1 && row>=0 && row<=15){
                return &ListEntry_Buffer[1];
            }
            else if(line == 2 && row>=0 && row<=15){
                    return &ListEntry_Buffer[2];
                }
                else{
                    return NULL_PTR;    
                }
            }
            else{
                return NULL_PTR;
            }
        }
        else{
            return NULL_PTR;
        }
 }
