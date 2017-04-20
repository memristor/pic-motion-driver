#ifndef QUEUE_H
#define QUEUE_H


typedef struct tagQueue {
	int head;
	int tail;
	int size;
	int capacity;
	void** data;
} Queue;

void queue_init(Queue* q, void** data, int capacity);
void* queue_pop(Queue* q);
int queue_full(Queue* q);
void queue_clear(Queue* q);
int queue_empty(Queue* q);
void* queue_top(Queue* q);
int queue_push(Queue* q, void* val);

#endif
