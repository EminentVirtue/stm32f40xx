/*
 * list.h
 *
 *  Created on: Jul 5, 2024
 *      Author: Andrew Streng
 *
 *  Description: Very basic list to be used for transfer data that is larger than 1 byte
 *  with SPI or I2C
 *
 *
 */

#ifndef LIST_H_
#define LIST_H_

#define LIST_EMPTY	0

struct node {
	struct node *next;
	uint8_t data;
};

struct list {
	struct node *head;
	struct node *tail;
};

/* The head's next node is initially the tail and the tail
 * is equal to itself  */
#define LIST_HEAD_INIT(name)  &(name)
#define LIST_TAIL_INIT(head)  &(head)
#define LIST_INIT_NODES(name) \
		{ \
			LIST_HEAD_INIT(name) \
			LIST_TAIL_INIT(LIST_HEAD_INIT) \
		}

#define LIST_CREATE(name)	\
		struct list name = LIST_INIT_NODES(name)

/**
 * @list_for_each 	- iterates over a list
 * @pos:	the &struct node to use a cursor to the current node of the iteration
 * @head:	the head of the list
 */
#define list_for_each(pos,head) \
	for(pos = (head)->next; prefetch(pos->next), pos != NULL; \
		pos = pos->next)

static inline void INIT_LIST_HEAD(struct node *list)
{
	list->next = list;
}

/* Internal use only */
static inline void __list_add(struct node *new, struct node *tail)
{
	new->next = tail;
	tail = new;
	tail->next = NULL;
}

/**
 * @list_add - adds a new data item to the list
 * @new:  the new entry to be added
 * @list: the list to insert the item
 */
static inline void list_add(struct node *new, struct list *pList)
{
	__list_add(new, pList->tail);
}

/**
 * @list_empty - checks to see if a given list is empty
 * @pList - list pointer
 * @return - 1 or 0
 */
static inline uint8_t list_empty(const struct list *pList)
{
	return pList->head == pList->tail;
}

/**
 * @list_size - returns the size of the list
 * @pList - list pointer
 */
static inline uint32_t list_size(const struct list *pList)
{
	uint32_t __size = 0;
	node curr;

	list_for_each(&curr, pList->head)
	{
		__size++;
	}
	return size;
}

#endif /* LIST_H_ */
