#include "hash.h"
#include <stdlib.h>
#include <stdio.h>

struct HashEntry;
typedef struct HashEntry {
	struct HashEntry 	*list;
	struct HashEntry 	*next;
	HashValueType		value;
} HashEntryType;

typedef struct  {
	struct HashEntry 	*next;
	struct HashEntry 	*last;
} HashHeadType;

typedef struct {
	struct HashEntry 	*ref;
	struct HashEntry 	*next;
	struct HashEntry 	*last;
} HashListCtrlType;
static HashListCtrlType HashListCtrl[HASHID_NUM];
static HashHeadType HashTable[HASHID_NUM][HASHSIZE];

static uint32 hashValue(uint32 rawdata)
{
	return rawdata % HASHSIZE;
}
static HashEntryType *getEntry(void)
{
	return malloc(sizeof(HashEntryType));
}

void hash_init(void)
{
	int i;
	int k;

	for (k = 0; k < HASHID_NUM; k++) {
		HashListCtrl[k].ref = NULL;
		HashListCtrl[k].next = NULL;
		HashListCtrl[k].last = NULL;
		for (i = 0; i < HASHSIZE; i++) {
			HashTable[k][i].next = NULL;
			HashTable[k][i].last = NULL;
		}
	}
	return;
}

HashValueType* hash_search(HashIdType id, HashValueType *value)
{
	HashEntryType *entry;
	HashHeadType *head;
	uint32 hvalue;

	hvalue = hashValue(value->rawdata);
	head = &HashTable[id][hvalue];

	entry = head->next;
	while (entry != NULL) {
		if (value->rawdata == entry->value.rawdata) {
			return &entry->value;
		}
		entry = entry->next;
	}
	return NULL;
}

void hash_add(HashIdType id, HashValueType *value)
{
	HashValueType *tmp;
	HashEntryType *entry;
	HashHeadType *head;
	uint32 hvalue;

	tmp = hash_search(id, value);
	if (tmp != NULL) {
		return;
	}
	//printf("hash_add:0x%x\n", value->rawdata);
	//fflush(stdout);

	hvalue = hashValue(value->rawdata);
	head = &HashTable[id][hvalue];
	entry = getEntry();
	entry->value = *value;
	entry->next = NULL;
	entry->list = NULL;

	/*
	 * ハッシュリスト登録
	 */
	if (head->last == NULL) {
		head->next = head->last = entry;
	}
	else {
		head->last->next = entry;
		head->last = entry;
	}

	/*
	 * 全エントリ管理用リスト登録
	 */
	if (HashListCtrl[id].last == NULL) {
		HashListCtrl[id].next = HashListCtrl[id].last = entry;
	}
	else {
		HashListCtrl[id].last->list = entry;
		HashListCtrl[id].last = entry;
	}

	return;
}

HashValueType* hash_first(HashIdType id)
{
	HashListCtrl[id].ref = HashListCtrl[id].next;
	if (HashListCtrl[id].ref == NULL) {
		return NULL;
	}
	return &HashListCtrl[id].ref->value;
}

HashValueType* hash_next(HashIdType id)
{
	if (HashListCtrl[id].ref == NULL) {
		return NULL;
	}
	if (HashListCtrl[id].ref->list == NULL) {
		return NULL;
	}
	HashListCtrl[id].ref = HashListCtrl[id].ref->list;
	return &HashListCtrl[id].ref->value;
}
