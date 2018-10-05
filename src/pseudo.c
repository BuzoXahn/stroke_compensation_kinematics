#include <stdio.h>
#include <stdlib.h>

#ifndef __func__
#define __func__	"ethiel"
#endif

struct list 
{
	int data;
	struct list *next;
	struct list *prev;
};

void list_init(struct list *head)
{
	if (!head)
		printf("%s: head is NULL\n", __func__);

	head->data = 0;
	head->next = NULL;
	head->prev = NULL;
}


int list_get_pos(struct list *head, int pos)
{
	int i;
	struct list *l;

	if (!head) {
		printf("%s: head is NULL\n", __func__);
		return -1;
	}

	l = head;
	for (i = 0; i < pos && l; i++, l = l->next);

	if (l)
		return l->data;
	else {
		printf("%s: there is no element at the position %d\n", __func__, pos);
		return -1;
	}
}

void list_del(struct list *l)
{
	if (!l) {
		printf("%s: list is NULL\n", __func__);
		return;
	}

	if (l->prev) l->prev->next = l->next;
	if (l->next) l->next->prev = l->prev;

	free(l);
	l = NULL;
}

void list_del_pos(struct list *head, int pos)
{
	int i;
	struct list *l;

	if (!head) {
		printf("%s: head is NULL\n", __func__);
		return;
	}

	l = head;
	for (i = 0; i < pos && l; i++, l = l->next);

	if (l)
		list_del(l);
	else
		printf("%s: there is no element at the position %d\n", __func__, pos);
}

void list_add(struct list *head, struct list *l, int d)
{
	struct list *tail;

	if (!head) {
		printf("%s: head is NULL\n", __func__);
		return;
	}
	if (!l) {
		printf("%s: list to be added is NULL\n", __func__);
		return;
	}

	for (tail = head; tail->next; tail = tail->next);

	l->data = d;
	l->next = tail->next;
	l->prev = tail;

	tail->next = l;
}

void list_free(struct list *head)
{
	if (!head) {
		printf("%s: list is NULL\n", __func__);
		return;
	}

	for (; head->next; list_del(head->next));

	free(head);
	head = NULL;
}

int main()
{
	int a[50], i, r;
	struct list *head = NULL;
	struct list *l = NULL;

	/* initialize a random function */
	srand(1);

	/* make a list which has elements from 1 to 50 */
	head = (struct list *)malloc(sizeof(struct list));
	list_init(head);
	for (i = 1; i <= 50; i++) {
		l = (struct list *)malloc(sizeof(struct list));
		list_add(head, l, i);
	}

	/* print elements of the list */
	printf("\n____________ list elements ___________\n");
	for (l = head->next, i=0; l; l = l->next, i++) {
		printf("(%2d) %2d\t", i+1, l->data);
		if (!((i+1)%5))
			printf("\n");
	}


	/* generate 50 random elements */
	for (i = 50;i > 0; i--) {
		r = 1+(int)(i*(float)rand()/(RAND_MAX+1.0));
		a[i-1] = list_get_pos(head, r);
		list_del_pos(head, r);
	}

	/* print the result */
	printf("\n________ generated random number ______\n");
	for (i = 0; i < 50; i++) {
		printf("(%2d) %2d\t", i+1,a[i]);
		if (!((i+1)%5))
			printf("\n");
	}

	/* free the list */
	list_free(head);

	return 0;
}
