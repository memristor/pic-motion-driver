#include "queue.h"
#include <stdio.h>
int main() {
	int d[20];
	void* data[10];
	Queue q;
	queue_init(&q, data, 10);
	int i;
	for(i=0; i < 20; i++) {
		d[i] = i;
		queue_push(&q, &d[i]);
	}
	
	for(i=0; i < 20; i++) {
		if(!queue_empty(&q)) {
			
			int* p = queue_pop(&q);
			printf("%d\n", *p);
		}
	}	
}
