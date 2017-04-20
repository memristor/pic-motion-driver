#include "queue.h"

// 0  1 2 3 4 5
// th          
void queue_init(Queue* q, void** data, int capacity) {
	q->head = 0;
	q->tail = 0;
	q->data = data;
	q->size = 0;
	q->capacity = capacity;
}
void* queue_pop(Queue* q) {
	if(q->size <= 0) return 0;
	void* r = q->data[q->tail++];
	q->tail %= q->capacity;
	q->size--;
	return r;
}

int queue_push(Queue* q, void* val) {
	if(q->size >= q->capacity) return 0;
	q->data[q->head++] = val;
	q->head %= q->capacity;
	q->size++;
	return 1;
}

int queue_full(Queue* q) {
	return q->size >= q->capacity;
}
void queue_clear(Queue* q) {
	q->head = 0;
	q->tail = 0;
	q->size = 0;
}
int queue_empty(Queue* q) {
	return q->size <= 0;
}
void* queue_top(Queue* q) {
	return q->data[q->size];
}
