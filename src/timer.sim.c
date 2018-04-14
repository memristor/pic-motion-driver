#include "timer.h"
#include <unistd.h>
#include <pthread.h>
void timer_deinit(void)
{
}

void _T1Interrupt();

static void* timer_thread() {
	while(1) {
		_T1Interrupt();
		usleep(1000); // 1ms timer
		// printf("tick\n");
	}
}

void timer_init(void)
{
	pthread_t thread;
	pthread_create(&thread, NULL, timer_thread, 0);
	pthread_detach(thread);
}
