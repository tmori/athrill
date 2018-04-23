#include "list_test.h"
#include "list.h"

void list_test_ERR_1(void)
{
    int ret;
    char *data;
    data = my_alloc();
    ret = my_free(data);

    ASSERT(data == NULL_PTR);
    ASSERT(ret == -1);

    return;
}

void list_test_ERR_2(void)
{
    int ret;
    char *data = (char*)&ret;

    ret = my_free(data);
    ASSERT(ret == -1);
    
    return;
}
void list_test_ERR_3(void)
{
    int ret;
    char *data;
    list_init();
    data = my_alloc();

    ASSERT(data != NULL_PTR);

    ret = my_free(data + 1);
    ASSERT(ret == -1);

    ret = my_free(data - 1024);
    ASSERT(ret == -1);
    ret = my_free(data + 1024);
    ASSERT(ret == -1);
    return;
}

void list_test_OK_1(void)
{
    char *data;
    list_init();
    data = my_alloc();

    ASSERT(data != NULL_PTR);
    return;
}

void list_test_OK_2(void)
{
    int ret;
    char *data;
    list_init();
    data = my_alloc();

    ASSERT(data != NULL_PTR);

    ret = my_free(data);
    ASSERT(ret == 0);
    return;
}

void list_test_OK_3(void)
{
    int ret;
    char *data[LIST_ENTRY_NUM];
    char *err_data;
    int i;

    list_init();

    for (i = 0; i < LIST_ENTRY_NUM; i++) {
        data[i] = my_alloc();
        ASSERT(data[i] != NULL_PTR);
    }

    err_data = my_alloc();
    ASSERT(err_data == NULL_PTR);

    for (i = 0; i < LIST_ENTRY_NUM; i++) {
        ret = my_free(data[i]);
        ASSERT(ret == 0);
    }

    return;
}
