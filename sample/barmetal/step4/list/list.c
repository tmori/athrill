 #include "list.h"
 #include "list_private.h"

void list_init(void)
{
    list_private_init();
}

void* my_alloc(void)
{    
    ListEntryType *free_box;
	
	if (free_headp == NULL_PTR) {
		return NULL_PTR;
	}

    free_box = list_private_remove_head(&free_headp);
    if(free_box == NULL_PTR){
        return NULL_PTR;
    }
    list_private_add_head(&allocated_headp, free_box);
    return free_box->data;

}

int my_free(void *data)
{
    ListEntryType *free_box;
    free_box = GetlistEntry(data);
    
    if (free_box->data != data) {
        return -1;
    }

    if (free_box == NULL_PTR){
        return -1;
    }
    else {
        list_private_remove(&allocated_headp,free_box);
        list_private_add_head(&free_headp, free_box);
        return 0;
    } 
}