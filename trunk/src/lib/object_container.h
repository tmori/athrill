#ifndef _OBJECT_CONTAINER_H_
#define _OBJECT_CONTAINER_H_

#include "std_types.h"
#include "assert.h"
#include <stdlib.h>


#define OBJECT_CONTAINER_OBJ_DEFAULT_SIZE	4U
typedef struct {
	char 	data[OBJECT_CONTAINER_OBJ_DEFAULT_SIZE];
} ObjectType;

typedef struct {
	void 	*p;
} ObjectSortElementType;

typedef struct {
	/*
	 * num of using objects
	 */
	uint32		num_using_objects;
	/*
	 * num of objects including preallocation
	 */
	uint32		num_total_objects;
	/*
	 * num of append objects if no data.
	 */
	uint32		append_size;
	/*
	 * object size
	 */
	uint32		object_size;
	/*
	 * head
	 */
	ObjectType	*head;
} ObjectContainerType;


/*********************************************
 * private
 *********************************************/
static inline uint32 object_container_get_element_size(ObjectContainerType *container)
{
	uint32 obj_size = sizeof(ObjectType);

	if (container->object_size > OBJECT_CONTAINER_OBJ_DEFAULT_SIZE) {
		obj_size += (container->object_size) - OBJECT_CONTAINER_OBJ_DEFAULT_SIZE;
	}
	return obj_size;
}

static inline void object_container_expand(ObjectContainerType *container)
{
	void *new_ptr;
	uint32 obj_size = object_container_get_element_size(container);

	container->num_total_objects += container->append_size;
	new_ptr = realloc(container->head, (container->num_total_objects) * obj_size);
	ASSERT(new_ptr != NULL);
	container->head = new_ptr;

	return;
}

/*********************************************
 * public
 *********************************************/
static inline ObjectContainerType *object_container_create(uint32 object_size, uint32 append_array_size)
{
	ObjectContainerType *p = (ObjectContainerType*)malloc(sizeof(ObjectContainerType));
	ASSERT(p != NULL);
	p->num_using_objects = 0;
	p->num_total_objects = 0;
	p->object_size = object_size;
	p->append_size = append_array_size;
	p->head = NULL;
	return p;
}

static inline void *object_container_get_element(ObjectContainerType *container, uint32 index)
{
	char *p = (char*)(container->head);
	ObjectType *obj;
	uint32 obj_size = object_container_get_element_size(container);
	if (index >= container->num_using_objects) {
		return NULL;
	}
	p += (index * obj_size);
	obj = (ObjectType*)p;

	return &obj->data[0];
}

static inline void *object_container_create_element(ObjectContainerType *container)
{
	if (container->num_using_objects < container->num_total_objects) {
		container->num_using_objects++;
		return object_container_get_element(container, container->num_using_objects - 1);
	}
	object_container_expand(container);
	container->num_using_objects++;
	return object_container_get_element(container, container->num_using_objects - 1);
}

static inline ObjectContainerType *object_container_sort(ObjectContainerType *container, bool (*object_compare) (const ObjectSortElementType *obj1, const ObjectSortElementType *obj2))
{
	uint32 index;
	bool isSwap = FALSE;
	ObjectContainerType *sort_array;

	sort_array = object_container_create(sizeof(ObjectSortElementType), 4U);

	//copy org container ==> sort_array
	for (index = 0; index < container->num_using_objects; index++) {
		void *p = object_container_get_element(container, index);
		ObjectSortElementType *obj = (ObjectSortElementType*)object_container_create_element(sort_array);
		obj->p = p;
	}

	//do bubble sort
	do {
		isSwap = FALSE;
		for (index = 0; index < sort_array->num_using_objects - 1; index++) {
			ObjectSortElementType *obj1 = (ObjectSortElementType*)object_container_get_element(sort_array, index);
			ObjectSortElementType *obj2 = (ObjectSortElementType*)object_container_get_element(sort_array, index + 1);
			if (object_compare((const ObjectSortElementType *)obj1, (const ObjectSortElementType *)obj2)) {
				void *org1 = obj1->p;
				void *org2 = obj2->p;
				obj1->p = org2;
				obj2->p = org1;
				isSwap = TRUE;
			}
		}
	} while (isSwap == TRUE);

	return sort_array;
}

static inline void object_container_foreach(ObjectContainerType *container, void (*object_action) (const void *p))
{
	uint32 index;
	for (index = 0; index < container->num_using_objects; index++) {
		void *p = object_container_get_element(container, index);
		object_action(p);
	}
	return;
}

static inline void *object_container_find_first(ObjectContainerType *container, bool (*object_filter) (const void *p))
{
	uint32 index;
	for (index = 0; index < container->num_using_objects; index++) {
		void *p = object_container_get_element(container, index);
		if (object_filter(p) == TRUE) {
			return p;
		}
	}
	return NULL;
}


static inline void object_container_delete(ObjectContainerType *container)
{
	if (container == NULL) {
		return;
	}
	if (container->head != NULL) {
		free(container->head);
		container->head = NULL;
	}
	free(container);
}

#endif /* _OBJECT_CONTAINER_H_ */
