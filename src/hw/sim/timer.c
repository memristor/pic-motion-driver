#include "../timer.h"
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
void timer_deinit(void)
{
}

void _T1Interrupt();


#include <time.h>
struct timespec diff(struct timespec start, struct timespec end)
{
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}
extern int avg2;
static void* timer_thread() {
	unsigned int d_calc = 1000000-5000;
	int t=0;
	while(1) {
		// _T1Interrupt();
		regulator_interrupt();
		
		struct timespec past,now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		past=now;
		
		// printf("time passed: %ld\n", tmp.tv_nsec);
		past=now;
		
		usleep(800); // 1ms timer
		// printf("tick\n");
		// while(diff(past,now).tv_nsec < 1000000-5000) {
		clock_gettime(CLOCK_MONOTONIC, &now);
		unsigned int d = diff(past,now).tv_nsec;
		// while(d < 1000000-25000) {
		// while(d < 1000000-15000) {
		
		
		while(d < d_calc) {
		// while(d < 1000000) {
			clock_gettime(CLOCK_MONOTONIC, &now);
			d = diff(past,now).tv_nsec;
		}
		if(++t > 1000) {
			if(avg2 < 1500000) {
			d_calc += (1000000 - avg2) / 10;
			// printf("fix with: %u %u\n", d_calc, avg2);
			}
			t=0;
		}
	}
}

void timer_init(void) {
	pthread_t thread;
	pthread_create(&thread, NULL, timer_thread, 0);
	pthread_detach(thread);
}
